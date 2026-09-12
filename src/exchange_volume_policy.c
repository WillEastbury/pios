/*
 * exchange_volume_policy.c - pure, offline dedicated-exchange selection.
 *
 * No block, partition-parser, filesystem, boot, WALFS, SD, or FAT32 code is
 * linked here.  The only state is caller-provided fixed storage, owned by
 * core 0 and protected against same-core IRQ interleaving.
 */
#include "types.h"
#include "exchange_volume_policy.h"

static const u8 exchange_label[] = {
    0x50U, 0x49U, 0x4FU, 0x53U, 0x58U, 0x46U, 0x45U, 0x52U
};

/* Microsoft basic data partition GUID, in GPT's on-disk byte order. */
static const u8 microsoft_basic_data_guid[] = {
    0xA2U, 0xA0U, 0xD0U, 0xEBU, 0xE5U, 0xB9U, 0x33U, 0x44U,
    0x87U, 0xC0U, 0x68U, 0xB6U, 0xB7U, 0x26U, 0x99U, 0xC7U
};

static inline u64 exchange_policy_irq_save(void)
{
#ifdef PIOS_HOST_TYPES_SHIM
    return 0U;
#else
    u64 daif;

    __asm__ volatile("mrs %0, daif" : "=r"(daif));
    __asm__ volatile("msr daifset, #2" ::: "memory");
    return daif;
#endif
}

static inline void exchange_policy_irq_restore(u64 daif)
{
#ifdef PIOS_HOST_TYPES_SHIM
    (void)daif;
#else
    __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
#endif
}

static void zero_bytes(void *dst, usize bytes)
{
    u8 *out = (u8 *)dst;
    usize i;

    for (i = 0U; i < bytes; i++)
        out[i] = 0U;
}

static bool bytes_zero(const void *src, usize bytes)
{
    const u8 *in = (const u8 *)src;
    usize i;

    for (i = 0U; i < bytes; i++) {
        if (in[i] != 0U)
            return false;
    }
    return true;
}

static bool bytes_equal(const u8 a[], const u8 b[], u32 bytes)
{
    u32 i;

    for (i = 0U; i < bytes; i++) {
        if (a[i] != b[i])
            return false;
    }
    return true;
}

static bool range_valid(u64 first, u64 count, u64 total, u64 *end_out)
{
    u64 end;

    if (first == 0U || count == 0U || first >= total ||
        count > total - first)
        return false;
    end = first + count;
    if (end_out)
        *end_out = end;
    return true;
}

static bool mbr_type_is_fat32(u8 type)
{
    return type == 0x0BU || type == 0x0CU;
}

static bool label_canonical(const struct exchange_volume_partition_fact *fact)
{
    if (fact->filesystem_label_bytes > EXCHANGE_VOLUME_POLICY_LABEL_BYTES)
        return false;
    return bytes_zero(fact->filesystem_label + fact->filesystem_label_bytes,
                      EXCHANGE_VOLUME_POLICY_LABEL_BYTES -
                      fact->filesystem_label_bytes);
}

static bool fact_valid(const struct exchange_volume_partition_fact *fact,
                       const struct exchange_volume_policy_input *input)
{
    if (!fact || !input || fact->identity == 0U ||
        fact->table_scheme != input->table_scheme ||
        fact->filesystem > EXCHANGE_VOLUME_FILESYSTEM_FAT32 ||
        (fact->flags & ~(EXCHANGE_VOLUME_FACT_BOOT |
                         EXCHANGE_VOLUME_FACT_PIOS_SYSTEM)) != 0U ||
        !bytes_zero(fact->_reserved0, sizeof(fact->_reserved0)) ||
        !bytes_zero(fact->_pad, sizeof(fact->_pad)) ||
        !label_canonical(fact) ||
        !range_valid(fact->first_lba, fact->block_count, input->total_blocks,
                     NULL))
        return false;
    if (fact->table_scheme == EXCHANGE_VOLUME_TABLE_MBR)
        return fact->table_index < 4U && fact->mbr_type != 0U &&
               bytes_zero(fact->gpt_type_guid, sizeof(fact->gpt_type_guid));
    if (fact->table_scheme == EXCHANGE_VOLUME_TABLE_GPT)
        return fact->table_index != 0U && fact->mbr_type == 0U &&
               !bytes_zero(fact->gpt_type_guid, sizeof(fact->gpt_type_guid));
    return false;
}

static bool fact_candidate(const struct exchange_volume_partition_fact *fact)
{
    if (fact->filesystem != EXCHANGE_VOLUME_FILESYSTEM_FAT32 ||
        fact->flags != 0U ||
        fact->filesystem_label_bytes != sizeof(exchange_label) ||
        !bytes_equal(fact->filesystem_label, exchange_label,
                     sizeof(exchange_label)))
        return false;
    if (fact->table_scheme == EXCHANGE_VOLUME_TABLE_MBR)
        return mbr_type_is_fat32(fact->mbr_type);
    return fact->table_scheme == EXCHANGE_VOLUME_TABLE_GPT &&
           bytes_equal(fact->gpt_type_guid, microsoft_basic_data_guid,
                       sizeof(microsoft_basic_data_guid));
}

static bool input_valid(const struct exchange_volume_policy_input *input)
{
    u32 i;
    u32 j;

    if (!input || !input->facts || input->total_blocks == 0U ||
        input->enumeration_generation == 0U || input->fact_count == 0U ||
        input->fact_count > EXCHANGE_VOLUME_POLICY_FACT_CAPACITY ||
        (input->table_scheme != EXCHANGE_VOLUME_TABLE_MBR &&
         input->table_scheme != EXCHANGE_VOLUME_TABLE_GPT))
        return false;
    for (i = 0U; i < input->fact_count; i++) {
        u64 end_i;

        if (!fact_valid(&input->facts[i], input) ||
            !range_valid(input->facts[i].first_lba,
                         input->facts[i].block_count, input->total_blocks,
                         &end_i))
            return false;
        for (j = 0U; j < i; j++) {
            u64 end_j;

            if (input->facts[i].identity == input->facts[j].identity ||
                input->facts[i].table_index == input->facts[j].table_index ||
                !range_valid(input->facts[j].first_lba,
                             input->facts[j].block_count, input->total_blocks,
                             &end_j) ||
                (input->facts[i].first_lba < end_j &&
                 input->facts[j].first_lba < end_i))
                return false;
        }
    }
    return true;
}

static u64 fingerprint_mix(u64 value, u8 byte)
{
    return (value ^ byte) * 0x100000001B3ULL;
}

static u64 attachment_fingerprint(
    const struct exchange_volume_partition_fact *fact, u64 total_blocks,
    u64 enumeration_generation)
{
    const u8 *bytes = (const u8 *)fact;
    u64 value = 0xCBF29CE484222325ULL ^ total_blocks ^
                (enumeration_generation << 1);
    usize i;

    for (i = 0U; i < sizeof(*fact); i++)
        value = fingerprint_mix(value, bytes[i]);
    return value == 0U ? 1U : value;
}

static bool owner_core(u32 caller_core)
{
    return caller_core == EXCHANGE_VOLUME_POLICY_OWNER_CORE &&
           core_id() == EXCHANGE_VOLUME_POLICY_OWNER_CORE;
}

static bool policy_fresh(const struct exchange_volume_policy *policy)
{
    return policy && bytes_zero(policy, sizeof(*policy));
}

static bool policy_initialized(const struct exchange_volume_policy *policy,
                               u32 caller_core)
{
    return policy && owner_core(caller_core) &&
           policy->control.magic == EXCHANGE_VOLUME_POLICY_MAGIC &&
           policy->control.instance_epoch != 0U &&
           policy->control.owner_core == EXCHANGE_VOLUME_POLICY_OWNER_CORE;
}

static void handle_clear(struct exchange_volume_policy_handle *handle)
{
    if (handle)
        zero_bytes(handle, sizeof(*handle));
}

static void handle_set(const struct exchange_volume_policy *policy,
                       struct exchange_volume_policy_handle *handle)
{
    *handle = (struct exchange_volume_policy_handle) {
        .magic = EXCHANGE_VOLUME_POLICY_HANDLE_MAGIC,
        .instance_epoch = policy->control.instance_epoch,
        .enumeration_generation = policy->control.enumeration_generation,
        .attachment_generation = policy->control.attachment_generation,
        .identity = policy->attachment.identity,
        .first_lba = policy->attachment.first_lba,
        .block_count = policy->attachment.block_count,
        .fingerprint = policy->control.attachment_fingerprint,
    };
}

static bool handle_valid(const struct exchange_volume_policy *policy,
                         const struct exchange_volume_policy_handle *handle,
                         u32 caller_core)
{
    u64 fingerprint;

    if (!policy_initialized(policy, caller_core) || !handle ||
        policy->control.state != EXCHANGE_VOLUME_POLICY_ATTACHED ||
        policy->control.mutation_guard != 0U ||
        handle->magic != EXCHANGE_VOLUME_POLICY_HANDLE_MAGIC ||
        handle->instance_epoch != policy->control.instance_epoch ||
        handle->enumeration_generation !=
        policy->control.enumeration_generation ||
        handle->attachment_generation !=
        policy->control.attachment_generation ||
        handle->identity != policy->attachment.identity ||
        handle->first_lba != policy->attachment.first_lba ||
        handle->block_count != policy->attachment.block_count)
        return false;
    fingerprint = attachment_fingerprint(
        &policy->attachment, policy->control.total_blocks,
        policy->control.enumeration_generation);
    return fingerprint == policy->control.attachment_fingerprint &&
           handle->fingerprint == fingerprint;
}

static enum exchange_volume_policy_result policy_init_locked(
    struct exchange_volume_policy *policy, u64 instance_epoch,
    u32 caller_core)
{
    if (!policy || instance_epoch == 0U)
        return EXCHANGE_VOLUME_POLICY_INVALID;
    if (!owner_core(caller_core))
        return EXCHANGE_VOLUME_POLICY_OWNER;
    if (!policy_fresh(policy))
        return EXCHANGE_VOLUME_POLICY_BUSY;
    policy->control.magic = EXCHANGE_VOLUME_POLICY_MAGIC;
    policy->control.instance_epoch = instance_epoch;
    policy->control.owner_core = EXCHANGE_VOLUME_POLICY_OWNER_CORE;
    dmb_ishst();
    policy->control.state = EXCHANGE_VOLUME_POLICY_READY;
    dmb_ishst();
    return EXCHANGE_VOLUME_POLICY_OK;
}

static enum exchange_volume_policy_result policy_attach_locked(
    struct exchange_volume_policy *policy,
    const struct exchange_volume_policy_input *input,
    const struct exchange_volume_policy_request *request,
    u32 caller_core, struct exchange_volume_policy_handle *handle_out)
{
    const struct exchange_volume_partition_fact *candidate = NULL;
    u32 candidates = 0U;
    u32 i;

    handle_clear(handle_out);
    if (!policy_initialized(policy, caller_core))
        return owner_core(caller_core) ? EXCHANGE_VOLUME_POLICY_UNINITIALIZED :
                                         EXCHANGE_VOLUME_POLICY_OWNER;
    if (!handle_out || !input_valid(input) || !request ||
        request->requested_identity == 0U)
        return EXCHANGE_VOLUME_POLICY_INVALID;
    if (policy->control.state != EXCHANGE_VOLUME_POLICY_READY ||
        policy->control.mutation_guard != 0U)
        return EXCHANGE_VOLUME_POLICY_BUSY;
    policy->control.mutation_guard = 1U;
    for (i = 0U; i < input->fact_count; i++) {
        if (!fact_candidate(&input->facts[i]))
            continue;
        candidates++;
        candidate = &input->facts[i];
    }
    if (candidates != 1U) {
        policy->control.mutation_guard = 0U;
        return candidates == 0U ? EXCHANGE_VOLUME_POLICY_REJECTED :
                                  EXCHANGE_VOLUME_POLICY_DUPLICATE;
    }
    if (candidate->identity != request->requested_identity) {
        policy->control.mutation_guard = 0U;
        return EXCHANGE_VOLUME_POLICY_REJECTED;
    }
    policy->attachment = *candidate;
    policy->control.enumeration_generation = input->enumeration_generation;
    policy->control.attachment_generation = 1U;
    policy->control.total_blocks = input->total_blocks;
    policy->control.attachment_fingerprint = attachment_fingerprint(
        candidate, input->total_blocks, input->enumeration_generation);
    dmb_ishst();
    policy->control.state = EXCHANGE_VOLUME_POLICY_ATTACHED;
    policy->control.mutation_guard = 0U;
    dmb_ishst();
    handle_set(policy, handle_out);
    return EXCHANGE_VOLUME_POLICY_OK;
}

static enum exchange_volume_policy_result policy_attachment_get_locked(
    const struct exchange_volume_policy *policy,
    const struct exchange_volume_policy_handle *handle, u32 caller_core,
    struct exchange_volume_partition_fact *fact_out)
{
    if (fact_out)
        zero_bytes(fact_out, sizeof(*fact_out));
    if (!fact_out)
        return EXCHANGE_VOLUME_POLICY_INVALID;
    if (!policy_initialized(policy, caller_core))
        return owner_core(caller_core) ? EXCHANGE_VOLUME_POLICY_UNINITIALIZED :
                                         EXCHANGE_VOLUME_POLICY_OWNER;
    if (!handle_valid(policy, handle, caller_core))
        return EXCHANGE_VOLUME_POLICY_STALE;
    *fact_out = policy->attachment;
    return EXCHANGE_VOLUME_POLICY_OK;
}

enum exchange_volume_policy_result exchange_volume_policy_init(
    struct exchange_volume_policy *policy, u64 instance_epoch,
    u32 caller_core)
{
    u64 irq = exchange_policy_irq_save();
    enum exchange_volume_policy_result result = policy_init_locked(
        policy, instance_epoch, caller_core);

    exchange_policy_irq_restore(irq);
    return result;
}

enum exchange_volume_policy_result exchange_volume_policy_attach(
    struct exchange_volume_policy *policy,
    const struct exchange_volume_policy_input *input,
    const struct exchange_volume_policy_request *request,
    u32 caller_core, struct exchange_volume_policy_handle *handle_out)
{
    u64 irq = exchange_policy_irq_save();
    enum exchange_volume_policy_result result = policy_attach_locked(
        policy, input, request, caller_core, handle_out);

    exchange_policy_irq_restore(irq);
    return result;
}

enum exchange_volume_policy_result exchange_volume_policy_attachment_get(
    const struct exchange_volume_policy *policy,
    const struct exchange_volume_policy_handle *handle, u32 caller_core,
    struct exchange_volume_partition_fact *fact_out)
{
    u64 irq = exchange_policy_irq_save();
    enum exchange_volume_policy_result result = policy_attachment_get_locked(
        policy, handle, caller_core, fact_out);

    exchange_policy_irq_restore(irq);
    return result;
}
