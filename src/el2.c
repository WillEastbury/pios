#include "el2.h"
#include "simd.h"
#include "proc.h"
#include "core_env.h"
#include "uart.h"

__asm__(".global __el2_integrity_start\n__el2_integrity_start:");

volatile u32 el2_boot_el_state = 1U;

static bool g_el2_active;
static u32 g_boot_el;

/* Per-core stage-2 activation/fault bookkeeping. VTTBR_EL2/HCR_EL2 are
 * themselves banked per-PE in hardware, so each core already runs its own
 * independent stage-2 translation -- but the fixed core-assignment table
 * (kernel.c) runs application processes concurrently on cores 2 AND 3, and
 * the software tracking of "which capsule is active"/fault attribution was a
 * single global here, racing across cores exactly like the packed
 * loops[4]/stage[4]/... arrays proc.c had to split into proc_rwake_percore
 * (see the comment there). Each core gets its OWN 64-byte-aligned owner
 * record so a core's activate/deactivate/fault write never shares a cache
 * line with another core's, per the hard-invariant ban on packed per-core
 * arrays of mutable state. */
struct el2_stage2_core_state {
    volatile u32 active_capsule;      /* EL2_CAPSULE_MAX == none active on this core */
    volatile u32 fault_count;         /* lifetime stage-2 faults taken on this core */
    volatile u64 last_esr;
    volatile u64 last_elr;
    volatile u64 last_far_ipa;        /* full faulting IPA: HPFAR_EL2[39:4]<<12 | FAR_EL2[11:0] */
    volatile u64 last_sp;             /* faulting context's SP: SP_EL0 or SP_EL1 as appropriate */
    volatile bool last_faulted_el0;   /* true if the faulting context was EL0, false if EL1 */
    volatile u32 last_fault_capsule;  /* EL2_CAPSULE_MAX == no fault recorded yet */
    u32 _pad[4];                      /* fill out the 64-byte line */
} ALIGNED(64);
_Static_assert(sizeof(struct el2_stage2_core_state) == 64,
               "el2_stage2_core_state must be exactly one cache line");

#define EL2_STAGE2_CORE_INIT \
    { .active_capsule = EL2_CAPSULE_MAX, .last_fault_capsule = EL2_CAPSULE_MAX }
static struct el2_stage2_core_state g_stage2_core[4] ALIGNED(64) = {
    EL2_STAGE2_CORE_INIT, EL2_STAGE2_CORE_INIT,
    EL2_STAGE2_CORE_INIT, EL2_STAGE2_CORE_INIT
};

static inline struct el2_stage2_core_state *el2_core_state(u32 core)
{
    return &g_stage2_core[core & 3U];
}

static u32 g_el2_integrity_baseline;
static bool g_boot_integrity_armed;
static u64 g_boot_el1_text_start;
static u32 g_boot_el1_text_len;
static u32 g_boot_el1_hash;
static u32 g_boot_el2_hash;
struct el2_port_owner {
    bool used;
    u32 generation;
    u16 port;
    u32 owner_pid;
    u32 capsule_hash;
    u32 _pad[11];
} ALIGNED(64);
static struct el2_port_owner g_port_owner[128];
_Static_assert(sizeof(struct el2_port_owner) == 64,
               "EL2 port-owner descriptors must be one cache line");

#define EL2_PORT_POISON_U32 0xDEAD2E12U
#define EL2_PORT_POISON_U16 0xDEADU

static u32 el2_bump_generation(u32 old)
{
    u32 g = old + 1U;
    return g ? g : 1U;
}

static void el2_port_owner_poison(struct el2_port_owner *o)
{
    if (!o)
        return;
    u32 g = el2_bump_generation(o->generation);
    simd_zero(o, sizeof(*o));
    o->used = false;
    o->generation = g;
    o->port = EL2_PORT_POISON_U16;
    o->owner_pid = EL2_PORT_POISON_U32;
    o->capsule_hash = EL2_PORT_POISON_U32;
}
static struct el2_capsule_desc g_capsules[EL2_CAPSULE_MAX];
static struct el2_stage2_plan g_stage2[EL2_CAPSULE_MAX];
static u64 g_stage2_root[EL2_CAPSULE_MAX][512] ALIGNED(4096);
static u64 g_stage2_l2[EL2_CAPSULE_MAX][512] ALIGNED(4096);

#define S2_L1_BLOCK_SIZE   (1UL << 30)
#define S2_L2_BLOCK_SIZE   (1UL << 21)
#define S2_PTE_VALID       (1UL << 0)
#define S2_PTE_TABLE       (1UL << 1)
#define S2_PTE_BLOCK       (0UL << 1)
#define S2_PTE_AF          (1UL << 10)
#define S2_MEMATTR_NORMAL  (0xFUL << 2)
#define S2_SH_INNER        (3UL << 8)
#define S2_S2AP_RW         (0UL << 6)

static u32 read_current_el(void)
{
    u64 v = 0;
    __asm__ volatile("mrs %0, CurrentEL" : "=r"(v));
    return (u32)((v >> 2) & 0x3U);
}

extern u8 __el2_integrity_start;
extern u8 __el2_integrity_end;

static u32 el2_integrity_hash_now(void)
{
    u64 s = (u64)(usize)&__el2_integrity_start;
    u64 e = (u64)(usize)&__el2_integrity_end;
    if (e <= s)
        return 0;
    if (e - s > (256UL * 1024UL))
        return 0;
    return hw_crc32c((const void *)(usize)s, (u32)(e - s));
}

static void el2_stage2_build_table(u32 id)
{
    struct el2_stage2_plan *p = &g_stage2[id];
    simd_zero(g_stage2_root[id], sizeof(g_stage2_root[id]));
    simd_zero(g_stage2_l2[id], sizeof(g_stage2_l2[id]));

    u64 *l1 = g_stage2_root[id];
    u64 *l2 = g_stage2_l2[id];
    /* The capsule's IPA range is identity-mapped from its PA (see
     * el2_capsule_bind_slot: ipa_base == pa_base always), and physical RAM
     * does not universally sit below 1GB -- e.g. QEMU_VIRT's per-core RAM
     * lives at 0x42000000+ (~1.03GB), which is already past a hardcoded
     * L1 slot 0. Index the L1 entry that actually covers ipa_base instead
     * of assuming slot 0; el2_stage2_plan_set() guarantees the whole range
     * fits within this single 1GB-aligned L1 block before we get here. */
    u32 l1_idx = (u32)(p->ipa_base / S2_L1_BLOCK_SIZE);
    u64 l1_block_base = (u64)l1_idx * S2_L1_BLOCK_SIZE;
    l1[l1_idx] = ((u64)(usize)l2) | S2_PTE_VALID | S2_PTE_TABLE;

    u64 blocks = p->ipa_size / S2_L2_BLOCK_SIZE;
    u64 ipa = p->ipa_base;
    u64 pa = p->pa_base;
    for (u64 i = 0; i < blocks; i++, ipa += S2_L2_BLOCK_SIZE, pa += S2_L2_BLOCK_SIZE) {
        /* el2_stage2_plan_set() already rejects any range that would reach
         * here, since this single L2 table only covers one 1GB-aligned L1
         * block. This is a belt-and-braces bound, not the primary
         * enforcement -- a config that needed a second L1 entry must be
         * rejected at plan_set() time (fail closed), not silently
         * truncated here. */
        if (ipa - l1_block_base >= S2_L1_BLOCK_SIZE)
            break;
        u32 idx = (u32)((ipa - l1_block_base) / S2_L2_BLOCK_SIZE);
        l2[idx] = (pa & ~(S2_L2_BLOCK_SIZE - 1)) |
                  S2_PTE_VALID | S2_PTE_BLOCK | S2_PTE_AF |
                  S2_MEMATTR_NORMAL | S2_SH_INNER | S2_S2AP_RW;
    }

}

static bool el2_stage2_pa_range_is_normal_wb(u64 pa_base, u64 size)
{
    if (size == 0 || pa_base + size < pa_base)
        return false;
    /* Stage-2 descriptors currently encode only Normal cacheable memory. Do not
     * map any PA that is NC/device in stage-1, or the same physical page would be
     * visible with conflicting attributes. For now capsules may map only the
     * per-core private RAM window, which the kernel maps WB everywhere. */
    return pa_base >= CORE0_RAM_BASE && pa_base + size <= SHARED_FIFO_BASE;
}

static bool el2_stage2_program_hw(u32 id)
{
    if (read_current_el() != 2U)
        return false;
    struct el2_stage2_plan *p = &g_stage2[id];
    if (!p->configured)
        return false;

    u64 vtcr = (16UL << 0) |   /* T0SZ=16 => 48-bit IPA */
               (0UL  << 6)  |  /* SL0 level 1 */
               (0UL  << 8)  |  /* IRGN0 WB WA */
               (0UL  << 10) |  /* ORGN0 WB WA */
               (3UL  << 12) |  /* SH0 inner */
               (2UL  << 16) |  /* TG0 4KB */
               (2UL  << 32);   /* PS 40-bit PA */
    u64 vttbr = ((u64)p->vmid << 48) |
                (((u64)(usize)g_stage2_root[id]) & 0x0000FFFFFFFFFFFFULL);

    __asm__ volatile("msr vtcr_el2, %0" :: "r"(vtcr));
    __asm__ volatile("msr vttbr_el2, %0" :: "r"(vttbr));
    __asm__ volatile("dsb sy\nisb");
    p->programmed = true;

    if ((p->flags & EL2_STAGE2_F_ACTIVATE_VM) != 0) {
        u64 hcr = 0;
        __asm__ volatile("mrs %0, hcr_el2" : "=r"(hcr));
        hcr |= (1UL << 0); /* VM */
        __asm__ volatile("msr hcr_el2, %0\nisb" :: "r"(hcr));
    }
    return true;
}

static bool el2_stage2_deactivate_hw(void)
{
    if (read_current_el() != 2U)
        return false;
    __asm__ volatile("msr vttbr_el2, xzr\n"
                     "dsb sy\n"
                     "isb");
    u64 hcr = 0;
    __asm__ volatile("mrs %0, hcr_el2" : "=r"(hcr));
    hcr &= ~(1UL << 0); /* VM */
    __asm__ volatile("msr hcr_el2, %0\nisb" :: "r"(hcr));
    return true;
}

void el2_init(void)
{
    u32 now = read_current_el();
    g_boot_el = el2_boot_el_state ? el2_boot_el_state : now;
    if (g_boot_el > 3U) g_boot_el = now;
    g_el2_active = (g_boot_el == 2U);
    for (u32 i = 0; i < 4U; i++) {
        g_stage2_core[i].active_capsule = EL2_CAPSULE_MAX;
        g_stage2_core[i].fault_count = 0;
        g_stage2_core[i].last_esr = 0;
        g_stage2_core[i].last_elr = 0;
        g_stage2_core[i].last_far_ipa = 0;
        g_stage2_core[i].last_sp = 0;
        g_stage2_core[i].last_faulted_el0 = false;
        g_stage2_core[i].last_fault_capsule = EL2_CAPSULE_MAX;
    }
    g_el2_integrity_baseline = el2_integrity_hash_now();
    g_boot_integrity_armed = false;
    g_boot_el1_text_start = 0;
    g_boot_el1_text_len = 0;
    g_boot_el1_hash = 0;
    g_boot_el2_hash = 0;
    for (u32 i = 0; i < (u32)(sizeof(g_port_owner) / sizeof(g_port_owner[0])); i++)
        el2_port_owner_poison(&g_port_owner[i]);
    simd_zero(g_capsules, sizeof(g_capsules));
    simd_zero(g_stage2, sizeof(g_stage2));
    simd_zero(g_stage2_root, sizeof(g_stage2_root));
    simd_zero(g_stage2_l2, sizeof(g_stage2_l2));
}

bool el2_active(void)
{
    return g_el2_active;
}

u32 el2_boot_el(void)
{
    return g_boot_el;
}

i32 el2_capsule_register(u32 id, u32 owner_principal, u32 manifest_hash,
                         u64 el1_entry, u64 el0_slot_base, u64 el0_slot_size)
{
    if (id >= EL2_CAPSULE_MAX || el1_entry == 0 || el0_slot_size == 0)
        return -1;
    struct el2_capsule_desc *c = &g_capsules[id];
    c->used = true;
    c->id = id;
    c->owner_principal = owner_principal;
    c->manifest_hash = manifest_hash;
    c->el1_entry = el1_entry;
    c->el0_slot_base = el0_slot_base;
    c->el0_slot_size = el0_slot_size;
    return 0;
}

u32 el2_capsule_count(void)
{
    u32 n = 0;
    for (u32 i = 0; i < EL2_CAPSULE_MAX; i++)
        if (g_capsules[i].used) n++;
    return n;
}

bool el2_capsule_get(u32 id, struct el2_capsule_desc *out)
{
    if (!out || id >= EL2_CAPSULE_MAX) return false;
    if (!g_capsules[id].used) return false;
    simd_memcpy(out, &g_capsules[id], sizeof(*out));
    return true;
}

i32 el2_stage2_plan_set(u32 id, u64 ipa_base, u64 ipa_size, u64 pa_base, u64 flags)
{
    if (id >= EL2_CAPSULE_MAX || !g_capsules[id].used || ipa_size == 0)
        return -1;
    if ((ipa_base & ((1UL << 21) - 1)) != 0 || (pa_base & ((1UL << 21) - 1)) != 0 ||
        (ipa_size & ((1UL << 21) - 1)) != 0)
        return -1;
    /* el2_stage2_build_table() only populates a single L2 table under one
     * L1 entry -- one 1GB-aligned block of IPA space, wherever ipa_base
     * falls (physical RAM is not universally below 1GB -- e.g. QEMU_VIRT's
     * per-core RAM sits at 0x42000000+). Reject any range that would cross
     * into a second 1GB-aligned L1 block, or that would index past the
     * fixed 512-entry L1 table, instead of silently building a table that
     * maps only the leading portion and leaves the remainder untranslated;
     * malformed/oversized descriptors must fail closed at configuration
     * time, not surface later as an unexpected stage-2 fault deep into the
     * capsule's execution. */
    {
        u64 l1_idx_base = ipa_base / S2_L1_BLOCK_SIZE;
        u64 l1_idx_end = (ipa_base + ipa_size - 1) / S2_L1_BLOCK_SIZE;
        if (l1_idx_base != l1_idx_end || l1_idx_base >= 512UL)
            return -1;
    }
    if (!el2_stage2_pa_range_is_normal_wb(pa_base, ipa_size))
        return -1;
    /* Defense in depth: reject a plan whose PA range intersects any other
     * currently-configured capsule's PA range. A capsule is normally bound
     * to its own private-RAM slot (el2_capsule_bind_slot), so this should
     * never trigger in practice -- but a duplicate or malicious
     * registration must fail closed here rather than rely solely on the
     * slot-allocation caller getting it right. */
    {
        u64 end = pa_base + ipa_size;
        for (u32 other = 0; other < EL2_CAPSULE_MAX; other++) {
            if (other == id || !g_stage2[other].configured)
                continue;
            u64 other_base = g_stage2[other].pa_base;
            u64 other_end = other_base + g_stage2[other].ipa_size;
            if (pa_base < other_end && other_base < end)
                return -1;
        }
    }
    struct el2_stage2_plan *p = &g_stage2[id];
    p->configured = true;
    p->enabled = false;
    p->programmed = false;
    p->vmid = (u16)(id + 1U);
    p->ipa_base = ipa_base;
    p->ipa_size = ipa_size;
    p->pa_base = pa_base;
    p->flags = flags;
    p->root_table_pa = (u64)(usize)g_stage2_root[id];
    el2_stage2_build_table(id);
    return 0;
}

i32 el2_stage2_enable(u32 id, bool enable)
{
    if (id >= EL2_CAPSULE_MAX || !g_stage2[id].configured)
        return -1;
    g_stage2[id].enabled = enable;
    if (enable)
        (void)el2_stage2_program_hw(id);
    return 0;
}

bool el2_stage2_status(u32 id, struct el2_stage2_plan *out)
{
    if (!out || id >= EL2_CAPSULE_MAX || !g_stage2[id].configured)
        return false;
    simd_memcpy(out, &g_stage2[id], sizeof(*out));
    return true;
}

i32 el2_stage2_activate(u32 id)
{
    u64 out = 0;
    if (el2_hvc_call(EL2_HVC_STAGE2_ACTIVATE, id, 0, 0, 0, &out) != 0)
        return -1;
    return (out == 0) ? 0 : -1;
}

i32 el2_capsule_bind_slot(u32 owner_principal, u32 manifest_hash, u64 el0_slot_base,
                          u64 el0_slot_size, u32 *id_out)
{
    if (!id_out || manifest_hash == 0 || el0_slot_size == 0)
        return -1;
    for (u32 i = 0; i < EL2_CAPSULE_MAX; i++) {
        if (!g_capsules[i].used) continue;
        if (g_capsules[i].owner_principal == owner_principal &&
            g_capsules[i].manifest_hash == manifest_hash) {
            *id_out = i;
            return 0;
        }
    }
    for (u32 i = 0; i < EL2_CAPSULE_MAX; i++) {
        if (g_capsules[i].used) continue;
        if (el2_capsule_register(i, owner_principal, manifest_hash, el0_slot_base,
                                 el0_slot_base, el0_slot_size) != 0)
            return -1;
        if (el2_stage2_plan_set(i, el0_slot_base, el0_slot_size, el0_slot_base,
                                EL2_STAGE2_F_ACTIVATE_VM) != 0)
            return -1;
        if (el2_stage2_enable(i, true) != 0)
            return -1;
        *id_out = i;
        return 0;
    }
    return -1;
}

i32 el2_hvc_dispatch(u32 fid, u64 x1, u64 x2, u64 x3, u64 x4, u64 *ret0)
{
    if (!ret0) return -1;
    if (fid == EL2_HVC_GET_EL) {
        *ret0 = (u64)g_boot_el;
        return 0;
    }
    if (fid == EL2_HVC_CAPSULE_COUNT) {
        *ret0 = (u64)el2_capsule_count();
        return 0;
    }
    if (fid == EL2_HVC_STAGE2_PLAN) {
        if (el2_stage2_plan_set((u32)x1, x2, x3, x4, 0) != 0)
            return -1;
        *ret0 = 0;
        return 0;
    }
    if (fid == EL2_HVC_STAGE2_ENABLE) {
        if (el2_stage2_enable((u32)x1, x2 != 0) != 0)
            return -1;
        *ret0 = 0;
        return 0;
    }
    if (fid == EL2_HVC_STAGE2_STATUS) {
        struct el2_stage2_plan st;
        if (!el2_stage2_status((u32)x1, &st))
            return -1;
        *ret0 = ((u64)(st.configured ? 1U : 0U) << 63) |
                ((u64)(st.enabled ? 1U : 0U) << 62) |
                ((u64)(st.programmed ? 1U : 0U) << 61) |
                ((u64)st.vmid << 48) |
                (st.ipa_size & 0x0000FFFFFFFFFFFFULL);
        return 0;
    }
    if (fid == EL2_HVC_STAGE2_ACTIVATE) {
        u32 id = (u32)x1;
        struct el2_stage2_core_state *cs = el2_core_state(core_id());
        if (id >= EL2_CAPSULE_MAX) {
            (void)el2_stage2_deactivate_hw();
            cs->active_capsule = EL2_CAPSULE_MAX;
            *ret0 = 0;
            return 0;
        }
        if (!g_stage2[id].configured || !g_stage2[id].enabled)
            return -1;
        (void)el2_stage2_program_hw(id);
        cs->active_capsule = id;
        *ret0 = 0;
        return 0;
    }
    if (fid == EL2_HVC_STAGE2_FAULTS) {
        /* System-wide aggregate view (external callers pass x1=0 and expect
         * one packed summary, not a per-core drill-down): total fault count
         * summed across all cores, with the ESR/active/last-fault fields
         * taken from whichever core has accumulated the most faults (ties
         * broken toward the lowest core id). Preserves the exact external
         * call signature/behavior of kernel.c's existing callers while
         * fixing the underlying per-core race. */
        u64 total = 0;
        u32 pick = 0;
        for (u32 c = 1; c < 4U; c++)
            if (g_stage2_core[c].fault_count > g_stage2_core[pick].fault_count)
                pick = c;
        for (u32 c = 0; c < 4U; c++)
            total += g_stage2_core[c].fault_count;
        struct el2_stage2_core_state *cs = &g_stage2_core[pick];
        *ret0 = (total & 0xFFFFFFFFULL) |
                ((cs->last_esr & 0xFFFFULL) << 32) |
                ((cs->active_capsule & 0xFFULL) << 48) |
                ((cs->last_fault_capsule & 0xFFULL) << 56);
        return 0;
    }
    if (fid == EL2_HVC_INTEGRITY_CHECK) {
        u32 el2h = el2_integrity_hash_now();
        u32 expect_el2 = g_boot_integrity_armed ? g_boot_el2_hash : g_el2_integrity_baseline;
        if (expect_el2 != 0 && el2h != expect_el2) {
            *ret0 = EL2_INTEGRITY_EL2_CHANGED;
            return 0;
        }
        if (g_boot_integrity_armed && g_boot_el1_text_start != 0 && g_boot_el1_text_len != 0) {
            u32 el1h = hw_crc32c((const void *)(usize)g_boot_el1_text_start, g_boot_el1_text_len);
            if (el1h != g_boot_el1_hash) {
                *ret0 = EL2_INTEGRITY_EL1_CHANGED;
                return 0;
            }
        }
        if (x1 == 0 || x2 == 0 || x2 > PROC_SLOT_SIZE) {
            *ret0 = ~0ULL;
            return 0;
        }
        u32 ph = hw_crc32c((const void *)(usize)x1, (u32)x2);
        *ret0 = (ph == (u32)x3) ? 0ULL : 1ULL;
        return 0;
    }
    if (fid == EL2_HVC_BOOT_INTEGRITY_SET) {
        if (x1 == 0 || x2 == 0 || x2 > (4UL * 1024UL * 1024UL) || x3 == 0 || x4 == 0)
            return -1;
        g_boot_el1_text_start = x1;
        g_boot_el1_text_len = (u32)x2;
        g_boot_el1_hash = (u32)x3;
        g_boot_el2_hash = (u32)x4;
        g_boot_integrity_armed = true;
        *ret0 = 0;
        return 0;
    }
    if (fid == EL2_HVC_PORT_BIND) {
        u16 port = (u16)x1;
        u32 pid = (u32)x2;
        u32 cap = (u32)x3;
        if (port == 0 || pid == 0) return -1;
        for (u32 i = 0; i < (u32)(sizeof(g_port_owner) / sizeof(g_port_owner[0])); i++) {
            if (!g_port_owner[i].used) continue;
            if (g_port_owner[i].port != port) continue;
            if (g_port_owner[i].owner_pid == pid && g_port_owner[i].capsule_hash == cap) {
                *ret0 = 0;
                return 0;
            }
            return -1;
        }
        for (u32 i = 0; i < (u32)(sizeof(g_port_owner) / sizeof(g_port_owner[0])); i++) {
            if (g_port_owner[i].used) continue;
            u32 gen = el2_bump_generation(g_port_owner[i].generation);
            simd_zero(&g_port_owner[i], sizeof(g_port_owner[i]));
            g_port_owner[i].used = true;
            g_port_owner[i].generation = gen;
            g_port_owner[i].port = port;
            g_port_owner[i].owner_pid = pid;
            g_port_owner[i].capsule_hash = cap;
            *ret0 = 0;
            return 0;
        }
        return -1;
    }
    if (fid == EL2_HVC_PORT_UNBIND) {
        u16 port = (u16)x1;
        u32 pid = (u32)x2;
        for (u32 i = 0; i < (u32)(sizeof(g_port_owner) / sizeof(g_port_owner[0])); i++) {
            if (!g_port_owner[i].used) continue;
            if (g_port_owner[i].port == port && g_port_owner[i].owner_pid == pid) {
                el2_port_owner_poison(&g_port_owner[i]);
                *ret0 = 0;
                return 0;
            }
        }
        *ret0 = 0;
        return 0;
    }
    if (fid == EL2_HVC_PORT_CHECK) {
        u16 port = (u16)x1;
        u32 pid = (u32)x2;
        u32 cap = (u32)x3;
        if (port == 0 || pid == 0) return -1;
        for (u32 i = 0; i < (u32)(sizeof(g_port_owner) / sizeof(g_port_owner[0])); i++) {
            if (!g_port_owner[i].used) continue;
            if (g_port_owner[i].port != port) continue;
            if (g_port_owner[i].owner_pid == pid && g_port_owner[i].capsule_hash == cap) {
                *ret0 = 0;
                return 0;
            }
            *ret0 = 1;
            return 0;
        }
        *ret0 = 0;
        return 0;
    }
    if (fid == EL2_HVC_PORT_UNBIND_ALL) {
        u32 pid = (u32)x1;
        if (pid == 0) return -1;
        for (u32 i = 0; i < (u32)(sizeof(g_port_owner) / sizeof(g_port_owner[0])); i++) {
            if (g_port_owner[i].used && g_port_owner[i].owner_pid == pid)
                el2_port_owner_poison(&g_port_owner[i]);
        }
        *ret0 = 0;
        return 0;
    }
    return -1;
}

u64 el2_hvc_trap(u32 fid, u64 x1, u64 x2, u64 x3, u64 x4)
{
    u64 r0 = 0;
    if (el2_hvc_dispatch(fid, x1, x2, x3, x4, &r0) != 0)
        return ~0ULL;
    return r0;
}

u64 el2_sync_fault_trap(u64 esr, u64 elr)
{
    /* Still executing at EL2 here (this is the EL2 sync-exception handler's
     * C callee, not a dropped-to-EL1 context), so HPFAR_EL2/FAR_EL2 (faulting
     * IPA for a stage-2 abort), SPSR_EL2 (to tell which EL faulted), and
     * SP_EL0/SP_EL1 are all readable directly -- no vectors.S plumbing
     * needed. Captures the fuller trap context the hard invariant asks for
     * (core, capsule, syndrome, PC, SP, faulting address) beyond just
     * esr/elr.
     *
     * CORRECTED (post rubber-duck review): two prior inaccuracies fixed
     * here. (1) HPFAR_EL2 alone is NOT the full faulting IPA -- per the ARM
     * ARM, HPFAR_EL2<39:4> holds IPA<39:12>; the low 12 bits (page offset)
     * come from FAR_EL2<11:0>, so they must be combined. (2) the faulting
     * context's relevant stack pointer is SP_EL0 when the fault came from a
     * guest running at EL0, not unconditionally SP_EL1 -- determined here
     * from SPSR_EL2.M[3:2] (0b00 == EL0). This kernel always runs EL1 code
     * in EL1h mode (SPSel=1, see start.S), so the EL1 case is always
     * SP_EL1. */
    u64 hpfar = 0, far = 0, spsr = 0, sp_el0 = 0, sp_el1 = 0;
    __asm__ volatile("mrs %0, hpfar_el2" : "=r"(hpfar));
    __asm__ volatile("mrs %0, far_el2"   : "=r"(far));
    __asm__ volatile("mrs %0, spsr_el2"  : "=r"(spsr));
    __asm__ volatile("mrs %0, sp_el0"    : "=r"(sp_el0));
    __asm__ volatile("mrs %0, sp_el1"    : "=r"(sp_el1));
    u64 ipa = ((hpfar >> 4) << 12) | (far & 0xFFFULL);
    bool from_el0 = ((spsr >> 2) & 0x3ULL) == 0;

    struct el2_stage2_core_state *cs = el2_core_state(core_id());
    cs->fault_count++;
    cs->last_esr = esr;
    cs->last_elr = elr;
    cs->last_far_ipa = ipa;
    cs->last_sp = from_el0 ? sp_el0 : sp_el1;
    cs->last_faulted_el0 = from_el0;
    cs->last_fault_capsule = cs->active_capsule;
    if (cs->active_capsule < EL2_CAPSULE_MAX) {
        u32 id = cs->active_capsule;
        g_stage2[id].enabled = false;
        g_stage2[id].programmed = false;
        (void)el2_stage2_deactivate_hw();
        cs->active_capsule = EL2_CAPSULE_MAX;
    }
    return ~0ULL;
}

/* Direct (non-HVC) diagnostic accessor for the fuller per-core fault
 * context above -- this is plain kernel memory (both EL1 and EL2 share the
 * same .data/.bss in this single-binary kernel), so unlike el2_stage2_
 * activate/enable (which must reach real EL2 system registers), reading it
 * needs no hypercall, matching the existing el2_stage2_status()/
 * el2_capsule_get() plain-accessor pattern. */
bool el2_stage2_fault_detail(u32 core, u32 *fault_count, u64 *esr, u64 *elr,
                              u64 *far_ipa, u64 *sp, bool *faulted_el0,
                              u32 *active_capsule, u32 *last_fault_capsule)
{
    if (core >= 4U)
        return false;
    struct el2_stage2_core_state *cs = el2_core_state(core);
    if (fault_count) *fault_count = cs->fault_count;
    if (esr) *esr = cs->last_esr;
    if (elr) *elr = cs->last_elr;
    if (far_ipa) *far_ipa = cs->last_far_ipa;
    if (sp) *sp = cs->last_sp;
    if (faulted_el0) *faulted_el0 = cs->last_faulted_el0;
    if (active_capsule) *active_capsule = cs->active_capsule;
    if (last_fault_capsule) *last_fault_capsule = cs->last_fault_capsule;
    return true;
}

i32 el2_hvc_call(u32 fid, u64 x1, u64 x2, u64 x3, u64 x4, u64 *ret0)
{
    u64 r0;
    if (g_el2_active) {
        register u64 a0 __asm__("x0") = fid;
        register u64 a1 __asm__("x1") = x1;
        register u64 a2 __asm__("x2") = x2;
        register u64 a3 __asm__("x3") = x3;
        register u64 a4 __asm__("x4") = x4;
        __asm__ volatile("hvc #0"
                         : "+r"(a0)
                         : "r"(a1), "r"(a2), "r"(a3), "r"(a4)
                         : "x5", "x6", "x7", "x8", "x9", "x10", "x11",
                           "x12", "x13", "x14", "x15", "x16", "x17", "memory");
        r0 = a0;
    } else {
        r0 = el2_hvc_trap(fid, x1, x2, x3, x4);
    }
    if (ret0) *ret0 = r0;
    return (r0 == ~0ULL) ? -1 : 0;
}

/* Pure-logic validation of the three correctness properties fixed in this
 * branch: per-core activation state, fail-closed IPA bounds, and cross-
 * capsule PA overlap rejection. Deliberately does NOT dereference any of
 * the dummy PA ranges it plans -- el2_stage2_plan_set()/build_table() only
 * ever encode a PA into a descriptor word, never read/write through it --
 * so this is safe to run against live physical-RAM addresses without
 * touching whatever is actually resident there. Uses three reserved
 * capsule ids at the top of the id space, snapshotting and restoring their
 * prior state so a real production capsule that happens to occupy one of
 * those ids is left exactly as it was. Registered in the QEMU-safe
 * selftest battery (kernel.c). */
bool el2_stage2_selftest(void)
{
    const u32 IDA = EL2_CAPSULE_MAX - 1U; /* 7 */
    const u32 IDB = EL2_CAPSULE_MAX - 2U; /* 6 */
    const u32 IDC = EL2_CAPSULE_MAX - 3U; /* 5: left unconfigured, "never armed" case */

    struct el2_capsule_desc save_ca, save_cb, save_cc;
    struct el2_stage2_plan save_pa, save_pb, save_pc;
    static u64 save_l1a[512], save_l2a[512], save_l1b[512], save_l2b[512];
    simd_memcpy(&save_ca, &g_capsules[IDA], sizeof(save_ca));
    simd_memcpy(&save_cb, &g_capsules[IDB], sizeof(save_cb));
    simd_memcpy(&save_cc, &g_capsules[IDC], sizeof(save_cc));
    simd_memcpy(&save_pa, &g_stage2[IDA], sizeof(save_pa));
    simd_memcpy(&save_pb, &g_stage2[IDB], sizeof(save_pb));
    simd_memcpy(&save_pc, &g_stage2[IDC], sizeof(save_pc));
    simd_memcpy(save_l1a, g_stage2_root[IDA], sizeof(save_l1a));
    simd_memcpy(save_l2a, g_stage2_l2[IDA], sizeof(save_l2a));
    simd_memcpy(save_l1b, g_stage2_root[IDB], sizeof(save_l1b));
    simd_memcpy(save_l2b, g_stage2_l2[IDB], sizeof(save_l2b));
    bool ca_was_configured = g_stage2[IDC].configured;
    bool ca_was_enabled = g_stage2[IDC].enabled;

    bool ok = true;
    /* Use the free tail of core2/core3 private RAM as dummy PA windows:
     * slots occupy [core_base+PROC_SLOT_OFFSET, core_base+PROC_SLOT_OFFSET+
     * MAX_PROCS_PER_CORE*PROC_SLOT_SIZE) = [+1MB, +13MB) on each core, so
     * +14MB is guaranteed clear of any real, currently-configured capsule
     * (now that isolation is mandatory by default, real boot processes do
     * occupy real stage-2 ranges at the low end of each core's RAM -- an
     * earlier version of this test collided with one of those and the new
     * overlap check correctly rejected it). Using two different cores'
     * RAM keeps the two windows trivially disjoint from each other too. */
    u64 pa_a = CORE2_RAM_BASE + (14UL << 20);
    u64 pa_b = CORE3_RAM_BASE + (14UL << 20);

    if (el2_capsule_register(IDA, 0xFFFFFFFEU, 0xE1517E57U, 1, pa_a, (2UL << 20)) != 0) { ok = false; uart_puts("[e2t] fail: register A\n"); }
    if (el2_capsule_register(IDB, 0xFFFFFFFEU, 0xE1517E58U, 1, pa_b, (2UL << 20)) != 0) { ok = false; uart_puts("[e2t] fail: register B\n"); }
    g_stage2[IDC].configured = false;
    g_stage2[IDC].enabled = false;

    /* Disjoint ranges: both must be accepted. */
    if (el2_stage2_plan_set(IDA, pa_a, (2UL << 20), pa_a, 0) != 0) { ok = false; uart_puts("[e2t] fail: plan A disjoint\n"); }
    if (el2_stage2_plan_set(IDB, pa_b, (2UL << 20), pa_b, 0) != 0) { ok = false; uart_puts("[e2t] fail: plan B disjoint\n"); }

    /* Overlapping range must be rejected (cross-capsule PA overlap check). */
    if (el2_stage2_plan_set(IDB, pa_a, (2UL << 20), pa_a, 0) == 0) { ok = false; uart_puts("[e2t] fail: overlap not rejected\n"); }
    /* Restore IDB to its disjoint range for the rest of the test. */
    if (el2_stage2_plan_set(IDB, pa_b, (2UL << 20), pa_b, 0) != 0) { ok = false; uart_puts("[e2t] fail: plan B restore\n"); }

    /* Oversized IPA range (would need a second L1 entry) must be rejected. */
    if (el2_stage2_plan_set(IDA, pa_a, (2UL << 30), pa_a, 0) == 0) { ok = false; uart_puts("[e2t] fail: oversized not rejected\n"); }
    /* Restore IDA's valid plan after the rejected oversized attempt. */
    if (el2_stage2_plan_set(IDA, pa_a, (2UL << 20), pa_a, 0) != 0) { ok = false; uart_puts("[e2t] fail: plan A restore\n"); }

    if (el2_stage2_enable(IDA, true) != 0) { ok = false; uart_puts("[e2t] fail: enable A\n"); }
    if (el2_stage2_enable(IDB, true) != 0) { ok = false; uart_puts("[e2t] fail: enable B\n"); }

    /* Per-core activation bookkeeping: valid enabled capsules must activate,
     * and an id that was never configured/enabled must be rejected. */
    if (el2_stage2_activate(IDA) != 0) { ok = false; uart_puts("[e2t] fail: activate A\n"); }
    if (el2_stage2_activate(IDB) != 0) { ok = false; uart_puts("[e2t] fail: activate B\n"); }
    if (el2_stage2_activate(IDC) == 0) { ok = false; uart_puts("[e2t] fail: activate C not rejected\n"); }
    (void)el2_stage2_activate(PROC_CAPSULE_ID_NONE); /* leave this core clean */

    simd_memcpy(&g_capsules[IDA], &save_ca, sizeof(save_ca));
    simd_memcpy(&g_capsules[IDB], &save_cb, sizeof(save_cb));
    simd_memcpy(&g_capsules[IDC], &save_cc, sizeof(save_cc));
    simd_memcpy(&g_stage2[IDA], &save_pa, sizeof(save_pa));
    simd_memcpy(&g_stage2[IDB], &save_pb, sizeof(save_pb));
    simd_memcpy(&g_stage2[IDC], &save_pc, sizeof(save_pc));
    simd_memcpy(g_stage2_root[IDA], save_l1a, sizeof(save_l1a));
    simd_memcpy(g_stage2_l2[IDA], save_l2a, sizeof(save_l2a));
    simd_memcpy(g_stage2_root[IDB], save_l1b, sizeof(save_l1b));
    simd_memcpy(g_stage2_l2[IDB], save_l2b, sizeof(save_l2b));
    g_stage2[IDC].configured = ca_was_configured;
    g_stage2[IDC].enabled = ca_was_enabled;

    return ok;
}

__asm__(".global __el2_integrity_end\n__el2_integrity_end:");
