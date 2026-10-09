/*
 * module.c - ADR-084 forward-only A/B kernel module roll-on.
 */
#include "module.h"
#include "pix.h"
#include "platform.h"
#include "mmu.h"
#include "crypto.h"
#include "simd.h"
#include "timer.h"
#include "core.h"

extern const u8 module_proof_v1_start[];
extern const u8 module_proof_v1_end[];
extern const u8 module_proof_v2_start[];
extern const u8 module_proof_v2_end[];

struct module_slot {
    u32 state;
    u32 artifact_generation;
    u32 code_bytes;
    u32 entry_offset;
    u32 arena_schema;
    u32 capabilities;
    u64 entry;
    u64 dispatch_epoch;
    u64 retain_until_ms;
    u64 reserved[2];
} ALIGNED(64);

struct module_control {
    volatile u64 dispatch_epoch;
    volatile u64 entry;
    volatile u32 active_slot;
    volatile u32 switching;
    u32 staged_slot;
    u32 retained_slot;
    u32 arena_schema;
    u32 capabilities;
    u32 artifact_generation;
    u32 initialized;
    u64 reserved[2];
} ALIGNED(64);

struct module_diag {
    u64 roll_count;
    u64 rollback_count;
    u64 reclaim_count;
    u64 reserved[5];
} ALIGNED(64);

struct module_call_record {
    volatile u64 epoch;
    volatile u32 depth;
    u32 reserved0;
    u64 calls;
    u64 reserved[5];
} ALIGNED(64);

_Static_assert(sizeof(struct module_slot) == 64U, "module slot cache line");
_Static_assert(sizeof(struct module_control) == 64U,
               "module control cache line");
_Static_assert(sizeof(struct module_diag) == 64U,
               "module diagnostics cache line");
_Static_assert(sizeof(struct module_call_record) == 64U,
               "module call record cache line");
_Static_assert(MODULE_MAX_IDENTITIES * 2U * MODULE_SLOT_BYTES ==
               PIOS_MODULE_CODE_SIZE || !PIOS_HAS_HOT_MODULES,
               "module A/B slots must fill code reservation");
_Static_assert(MODULE_MAX_IDENTITIES * MODULE_ARENA_BYTES ==
               PIOS_MODULE_STATE_SIZE || !PIOS_HAS_HOT_MODULES,
               "module arenas must fill state reservation");

static struct module_slot module_slots[MODULE_MAX_IDENTITIES][2] ALIGNED(64);
static struct module_control module_controls[MODULE_MAX_IDENTITIES] ALIGNED(64);
static struct module_diag module_diags[MODULE_MAX_IDENTITIES] ALIGNED(64);
static struct module_call_record
    module_calls[MODULE_MAX_IDENTITIES][PIOS_PLATFORM_CORE_COUNT] ALIGNED(64);

static const struct module_services services = {
    .abi_version = MODULE_ABI_VERSION,
    .bytes = sizeof(struct module_services),
    .memset_fn = memset,
    .memcpy_fn = memcpy,
    .crc32c_fn = hw_crc32c
};

static const u32 allowed_caps[MODULE_MAX_IDENTITIES] = {
    [MODULE_ID_B50] = MODULE_CAP_PCIE | MODULE_CAP_DMA | MODULE_CAP_IRQ |
                      MODULE_CAP_FIFO,
    [MODULE_ID_TENSOR] = MODULE_CAP_DMA | MODULE_CAP_FIFO,
    [MODULE_ID_MAC] = MODULE_CAP_DMA | MODULE_CAP_IRQ | MODULE_CAP_FIFO |
                      MODULE_CAP_NETWORK,
    [MODULE_ID_IP] = MODULE_CAP_FIFO | MODULE_CAP_NETWORK,
    [MODULE_ID_TCP] = MODULE_CAP_FIFO | MODULE_CAP_NETWORK,
    [MODULE_ID_CAPSULE] = MODULE_CAP_FIFO | MODULE_CAP_PICOSCRIPT,
    [MODULE_ID_FILESYSTEM] = MODULE_CAP_FIFO | MODULE_CAP_STORAGE,
    [MODULE_ID_PROOF] = 0U
};

static u64 slot_base(u32 module_id, u32 slot)
{
    return PIOS_MODULE_CODE_BASE +
           ((u64)module_id * 2U + slot) * MODULE_SLOT_BYTES;
}

static void *arena_base(u32 module_id)
{
    return (void *)(usize)(PIOS_MODULE_STATE_BASE +
                           (u64)module_id * MODULE_ARENA_BYTES);
}

static u32 next_generation(u32 value)
{
    value++;
    return value ? value : 1U;
}

static bool digest_equal(const u8 *a, const u8 *b)
{
    u8 different = 0U;
    for (u32 i = 0U; i < 32U; i++)
        different |= a[i] ^ b[i];
    return different == 0U;
}

static bool slot_has_callers(u32 module_id, u64 epoch)
{
    if (!epoch)
        return false;
    for (u32 core = 0U; core < PIOS_PLATFORM_CORE_COUNT; core++) {
        const struct module_call_record *record =
            &module_calls[module_id][core];
        dmb();
        if (record->depth && record->epoch == epoch)
            return true;
    }
    return false;
}

static void slot_poison(u32 module_id, u32 slot)
{
#if PIOS_HAS_HOT_MODULES
    u64 base = slot_base(module_id, slot);
    if (!mmu_module_code_set_rx(base, 0U, MODULE_SLOT_BYTES, false))
        return;
    memset((void *)(usize)base, 0xA5, MODULE_SLOT_BYTES);
    dcache_clean_range(base, MODULE_SLOT_BYTES);
#else
    (void)module_id;
    (void)slot;
#endif
    memset(&module_slots[module_id][slot], 0,
           sizeof(module_slots[module_id][slot]));
}

void module_init(void)
{
    memset(module_slots, 0, sizeof(module_slots));
    memset(module_controls, 0, sizeof(module_controls));
    memset(module_diags, 0, sizeof(module_diags));
    memset(module_calls, 0, sizeof(module_calls));
#if PIOS_HAS_HOT_MODULES
    memset((void *)(usize)PIOS_MODULE_STATE_BASE, 0,
           PIOS_MODULE_STATE_SIZE);
    for (u32 id = 0U; id < MODULE_MAX_IDENTITIES; id++) {
        module_controls[id].active_slot = ~0U;
        module_controls[id].staged_slot = ~0U;
        module_controls[id].retained_slot = ~0U;
        module_controls[id].dispatch_epoch = 1U;
        module_controls[id].initialized = 1U;
        for (u32 slot = 0U; slot < 2U; slot++)
            (void)mmu_module_code_set_rx(
                slot_base(id, slot), 0U, MODULE_SLOT_BYTES, false);
    }
#endif
}

bool module_stage_image(const u8 *image, u32 image_bytes)
{
#if !PIOS_HAS_HOT_MODULES
    (void)image;
    (void)image_bytes;
    return false;
#else
    if (core_id() != CORE_NET || !image ||
        image_bytes < sizeof(struct module_image_header))
        return false;
    const struct module_image_header *header =
        (const struct module_image_header *)(const void *)image;
    if (header->magic != MODULE_IMAGE_MAGIC ||
        header->version != MODULE_IMAGE_VERSION ||
        header->header_bytes != sizeof(*header) ||
        header->module_id >= MODULE_MAX_IDENTITIES ||
        header->abi_version != MODULE_ABI_VERSION ||
        header->code_bytes == 0U ||
        header->code_bytes > MODULE_SLOT_BYTES ||
        header->entry_offset >= header->code_bytes ||
        header->header_bytes > image_bytes ||
        header->code_bytes > image_bytes - header->header_bytes ||
        (header->capabilities & ~allowed_caps[header->module_id]))
        return false;
    u8 digest[32];
    sha256(image + header->header_bytes, header->code_bytes, digest);
    if (!digest_equal(digest, header->sha256))
        return false;
    struct module_control *control = &module_controls[header->module_id];
    if (!control->initialized || control->switching ||
        (control->arena_schema &&
         control->arena_schema != header->arena_schema) ||
        header->artifact_generation <= control->artifact_generation)
        return false;
    u32 slot = control->active_slot == 0U ? 1U : 0U;
    if (module_slots[header->module_id][slot].state != MODULE_SLOT_FREE)
        return false;
    u64 base = slot_base(header->module_id, slot);
    if (!mmu_module_code_set_rx(base, 0U, MODULE_SLOT_BYTES, false))
        return false;
    memset((void *)(usize)base, 0, MODULE_SLOT_BYTES);
    memcpy((void *)(usize)base, image + header->header_bytes,
           header->code_bytes);
    dcache_clean_range(base, header->code_bytes);
    if (!mmu_module_code_set_rx(base, header->code_bytes,
                                MODULE_SLOT_BYTES, true))
        return false;
    module_entry_fn entry =
        (module_entry_fn)(usize)(base + header->entry_offset);
    if (entry(MODULE_OP_VALIDATE, 0U, 0U,
              arena_base(header->module_id), &services) !=
        MODULE_VALIDATE_TOKEN) {
        slot_poison(header->module_id, slot);
        return false;
    }
    struct module_slot *record = &module_slots[header->module_id][slot];
    record->state = MODULE_SLOT_STAGED;
    record->artifact_generation = header->artifact_generation;
    record->code_bytes = header->code_bytes;
    record->entry_offset = header->entry_offset;
    record->arena_schema = header->arena_schema;
    record->capabilities = header->capabilities;
    record->entry = (u64)(usize)entry;
    control->staged_slot = slot;
    return true;
#endif
}

static bool publish_slot(u32 module_id, u32 slot, bool rollback)
{
#if !PIOS_HAS_HOT_MODULES
    (void)module_id;
    (void)slot;
    (void)rollback;
    return false;
#else
    struct module_control *control = &module_controls[module_id];
    struct module_slot *next = &module_slots[module_id][slot];
    if (core_id() != CORE_NET || control->switching ||
        next->state == MODULE_SLOT_FREE)
        return false;
    control->switching = 1U;
    dmb();
    u32 old_slot = control->active_slot;
    if (old_slot < 2U) {
        module_entry_fn old =
            (module_entry_fn)(usize)module_slots[module_id][old_slot].entry;
        if (old)
            (void)old(MODULE_OP_QUIESCE, 0U, 0U, arena_base(module_id),
                      &services);
    }
    module_entry_fn entry = (module_entry_fn)(usize)next->entry;
    if (entry(MODULE_OP_ADOPT, 0U, 0U, arena_base(module_id), &services) !=
        0U) {
        control->switching = 0U;
        return false;
    }
    u64 epoch = control->dispatch_epoch + 1U;
    if (!epoch)
        epoch = 1U;
    next->state = MODULE_SLOT_ACTIVE;
    next->dispatch_epoch = epoch;
    control->entry = next->entry;
    control->active_slot = slot;
    control->dispatch_epoch = epoch;
    control->arena_schema = next->arena_schema;
    control->capabilities = next->capabilities;
    control->artifact_generation = next->artifact_generation;
    control->staged_slot = ~0U;
    if (old_slot < 2U && old_slot != slot) {
        struct module_slot *old = &module_slots[module_id][old_slot];
        old->state = MODULE_SLOT_RETAINED;
        old->retain_until_ms =
            timer_monotonic_ms() + MODULE_DRAIN_TARGET_MS;
        control->retained_slot = old_slot;
    }
    if (rollback)
        module_diags[module_id].rollback_count++;
    else
        module_diags[module_id].roll_count++;
    dmb();
    control->switching = 0U;
    return true;
#endif
}

bool module_activate(u32 module_id)
{
    if (module_id >= MODULE_MAX_IDENTITIES)
        return false;
    u32 slot = module_controls[module_id].staged_slot;
    return slot < 2U && publish_slot(module_id, slot, false);
}

bool module_rollback(u32 module_id)
{
    if (module_id >= MODULE_MAX_IDENTITIES)
        return false;
    struct module_control *control = &module_controls[module_id];
    u32 slot = control->retained_slot;
    if (slot < 2U)
        return publish_slot(module_id, slot, true);
#if PIOS_HAS_HOT_MODULES
    if (core_id() != CORE_NET || control->switching ||
        control->active_slot >= 2U)
        return false;
    control->switching = 1U;
    dmb();
    u32 active = control->active_slot;
    struct module_slot *record = &module_slots[module_id][active];
    module_entry_fn entry = (module_entry_fn)(usize)record->entry;
    if (entry)
        (void)entry(MODULE_OP_QUIESCE, 0U, 0U, arena_base(module_id),
                    &services);
    u64 epoch = control->dispatch_epoch + 1U;
    if (!epoch)
        epoch = 1U;
    record->state = MODULE_SLOT_RETAINED;
    record->retain_until_ms =
        timer_monotonic_ms() + MODULE_DRAIN_TARGET_MS;
    control->retained_slot = active;
    control->active_slot = ~0U;
    control->entry = 0U;
    control->artifact_generation = 0U;
    control->capabilities = 0U;
    control->dispatch_epoch = epoch;
    module_diags[module_id].rollback_count++;
    dmb();
    control->switching = 0U;
    return true;
#else
    return false;
#endif
}

u64 module_dispatch_call(u32 module_id, u32 op, u64 a0, u64 a1,
                         module_fallback_fn fallback)
{
    if (module_id >= MODULE_MAX_IDENTITIES)
        return fallback ? fallback(op, a0, a1) : 0U;
#if !PIOS_HAS_HOT_MODULES
    return fallback ? fallback(op, a0, a1) : 0U;
#else
    struct module_control *control = &module_controls[module_id];
    u32 core = core_id();
    if (core >= PIOS_PLATFORM_CORE_COUNT || control->switching ||
        control->active_slot >= 2U || !control->entry)
        return fallback ? fallback(op, a0, a1) : 0U;
    struct module_call_record *record = &module_calls[module_id][core];
    for (u32 attempt = 0U; attempt < 2U; attempt++) {
        u64 epoch = control->dispatch_epoch;
        u64 entry = control->entry;
        if (record->depth && record->epoch != epoch)
            return fallback ? fallback(op, a0, a1) : 0U;
        record->epoch = epoch;
        record->depth++;
        dmb();
        if (epoch != control->dispatch_epoch || entry != control->entry ||
            control->switching) {
            record->depth--;
            dmb();
            continue;
        }
        module_entry_fn fn = (module_entry_fn)(usize)entry;
        u64 result = fn(op, a0, a1, arena_base(module_id), &services);
        record->calls++;
        record->depth--;
        dmb();
        return result;
    }
    return fallback ? fallback(op, a0, a1) : 0U;
#endif
}

void module_service(void)
{
#if PIOS_HAS_HOT_MODULES
    if (core_id() != CORE_NET)
        return;
    u64 now = timer_monotonic_ms();
    for (u32 id = 0U; id < MODULE_MAX_IDENTITIES; id++) {
        struct module_control *control = &module_controls[id];
        u32 slot = control->retained_slot;
        if (slot >= 2U)
            continue;
        struct module_slot *record = &module_slots[id][slot];
        if (record->state != MODULE_SLOT_RETAINED ||
            now < record->retain_until_ms ||
            slot_has_callers(id, record->dispatch_epoch))
            continue;
        module_entry_fn old = (module_entry_fn)(usize)record->entry;
        if (old)
            (void)old(MODULE_OP_CLEANUP, 0U, 0U, arena_base(id),
                      &services);
        slot_poison(id, slot);
        control->retained_slot = ~0U;
        module_diags[id].reclaim_count++;
    }
#endif
}

void module_status_get(u32 module_id, struct module_status *out)
{
    if (!out)
        return;
    memset(out, 0, sizeof(*out));
    out->module_id = module_id;
    out->supported = PIOS_HAS_HOT_MODULES != 0;
    if (module_id >= MODULE_MAX_IDENTITIES)
        return;
    const struct module_control *control = &module_controls[module_id];
    out->switching = control->switching != 0U;
    out->active_slot = control->active_slot;
    out->retained_slot = control->retained_slot;
    out->staged_slot = control->staged_slot;
    out->artifact_generation = control->artifact_generation;
    out->arena_schema = control->arena_schema;
    out->capabilities = control->capabilities;
    out->dispatch_epoch = control->dispatch_epoch;
    out->roll_count = module_diags[module_id].roll_count;
    out->rollback_count = module_diags[module_id].rollback_count;
    out->reclaim_count = module_diags[module_id].reclaim_count;
    for (u32 core = 0U; core < PIOS_PLATFORM_CORE_COUNT; core++)
        out->active_calls += module_calls[module_id][core].depth;
}

static bool proof_image(const u8 *start, const u8 *end, u32 generation,
                        u8 *image, u32 capacity, u32 *bytes_out)
{
    u32 code_bytes = (u32)(end - start);
    u32 total = sizeof(struct module_image_header) + code_bytes;
    if (!image || !bytes_out || total > capacity)
        return false;
    struct module_image_header *header =
        (struct module_image_header *)(void *)image;
    memset(image, 0, total);
    header->magic = MODULE_IMAGE_MAGIC;
    header->version = MODULE_IMAGE_VERSION;
    header->header_bytes = sizeof(*header);
    header->module_id = MODULE_ID_PROOF;
    header->abi_version = MODULE_ABI_VERSION;
    header->arena_schema = 1U;
    header->artifact_generation = generation;
    header->code_bytes = code_bytes;
    memcpy(image + sizeof(*header), start, code_bytes);
    sha256(image + sizeof(*header), code_bytes, header->sha256);
    *bytes_out = total;
    return true;
}

static bool run_proof(void)
{
#if !PIOS_HAS_HOT_MODULES
    return false;
#else
    u8 image[256];
    u32 bytes;
    if (!proof_image(module_proof_v1_start, module_proof_v1_end,
                     1U, image, sizeof(image), &bytes) ||
        !module_stage_image(image, bytes) ||
        !module_activate(MODULE_ID_PROOF) ||
        module_dispatch_call(MODULE_ID_PROOF, MODULE_OP_CALL,
                             10U, 0U, NULL) != 11U)
        return false;
    if (!proof_image(module_proof_v2_start, module_proof_v2_end,
                     2U, image, sizeof(image), &bytes) ||
        !module_stage_image(image, bytes) ||
        !module_activate(MODULE_ID_PROOF) ||
        module_dispatch_call(MODULE_ID_PROOF, MODULE_OP_CALL,
                             10U, 0U, NULL) != 1012U)
        return false;
    if (!module_rollback(MODULE_ID_PROOF) ||
        module_dispatch_call(MODULE_ID_PROOF, MODULE_OP_CALL,
                             10U, 0U, NULL) != 13U)
        return false;
    return true;
#endif
}

const char *module_command(u32 operation)
{
    if (operation == MODULE_COMMAND_PROOF)
        return run_proof() ?
            "module A/B roll+rollback PASS; stable arena preserved\n" :
            "module proof FAILED\n";
    if (operation == MODULE_COMMAND_ROLLBACK)
        return module_rollback(MODULE_ID_PROOF) ?
            "module proof rollback published\n" :
            "module proof rollback unavailable\n";
    return PIOS_HAS_HOT_MODULES ?
        "module loader ready; A/B RX slots, stable arenas, drain+rollback active\n" :
        "module loader unavailable on this platform\n";
}

/* Legacy compatibility. New modules use module_stage_image/module_activate. */
bool module_load(const u8 *file, u32 file_size)
{
    return module_stage_image(file, file_size);
}

void module_call_hooks(u32 hook_type, void *arg)
{
    (void)hook_type;
    (void)arg;
}
