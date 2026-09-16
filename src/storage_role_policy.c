/*
 * storage_role_policy.c - offline role-marker validation.
 */
#include "types.h"
#include "storage_role_policy.h"

static bool zero_bytes(const void *ptr, usize bytes)
{
    const u8 *p = (const u8 *)ptr;
    usize i;
    for (i = 0U; i < bytes; i++)
        if (p[i] != 0U)
            return false;
    return true;
}

static u64 mix(u64 value, u8 byte)
{
    return (value ^ byte) * 0x100000001B3ULL;
}

static u64 fingerprint(const struct storage_role_policy_input *input)
{
    const u8 *bytes = (const u8 *)input->facts;
    u64 value = 0xCBF29CE484222325ULL ^ input->total_blocks ^
                (input->enumeration_generation << 1);
    usize i;
    for (i = 0U; i < (usize)input->fact_count *
                         sizeof(struct storage_role_fact); i++)
        value = mix(value, bytes[i]);
    return value == 0U ? 1U : value;
}

static bool owner(u32 caller_core)
{
    return caller_core == STORAGE_ROLE_POLICY_OWNER_CORE &&
           core_id() == STORAGE_ROLE_POLICY_OWNER_CORE;
}

static bool range_valid(const struct storage_role_fact *fact, u64 total)
{
    return fact->first_lba != 0U && fact->block_count != 0U &&
           fact->first_lba < total &&
           fact->block_count <= total - fact->first_lba;
}

static bool fact_valid(const struct storage_role_fact *fact, u64 total)
{
    if (!fact || fact->identity == 0U || !range_valid(fact, total) ||
        fact->table_index >= STORAGE_ROLE_POLICY_MAX_FACTS ||
        fact->role < STORAGE_ROLE_BOOT || fact->role > STORAGE_ROLE_EXCHANGE ||
        fact->marker_version == 0U ||
        !zero_bytes(fact->_reserved, sizeof(fact->_reserved)))
        return false;
    if (fact->role == STORAGE_ROLE_WALFS)
        return fact->marker == STORAGE_ROLE_MARKER_WALFS_SUPERBLOCK ||
               fact->marker == STORAGE_ROLE_MARKER_PIOS_RESERVED;
    if (fact->role == STORAGE_ROLE_BOOT ||
        fact->role == STORAGE_ROLE_EXCHANGE)
        return fact->marker == STORAGE_ROLE_MARKER_FAT32_BPB;
    return false;
}

static bool overlap(const struct storage_role_fact *a,
                    const struct storage_role_fact *b)
{
    return a->first_lba < b->first_lba + b->block_count &&
           b->first_lba < a->first_lba + a->block_count;
}

static bool handle_valid(const struct storage_role_policy *policy,
                         const struct storage_role_policy_handle *handle,
                         u32 caller_core)
{
    return owner(caller_core) && policy &&
           policy->control.magic == STORAGE_ROLE_POLICY_MAGIC &&
           policy->control.fingerprint != 0U && handle &&
           handle->magic == STORAGE_ROLE_POLICY_HANDLE_MAGIC &&
           handle->instance_epoch == policy->control.instance_epoch &&
           handle->enumeration_generation ==
               policy->control.enumeration_generation &&
           handle->attachment_generation ==
               policy->control.attachment_generation &&
           handle->fingerprint == policy->control.fingerprint;
}

enum storage_role_policy_result storage_role_policy_init(
    struct storage_role_policy *policy, u64 instance_epoch, u32 caller_core)
{
    if (!policy || instance_epoch == 0U)
        return STORAGE_ROLE_POLICY_INVALID;
    if (!owner(caller_core))
        return STORAGE_ROLE_POLICY_OWNER;
    if (!zero_bytes(policy, sizeof(*policy)))
        return STORAGE_ROLE_POLICY_BUSY;
    policy->control.magic = STORAGE_ROLE_POLICY_MAGIC;
    policy->control.instance_epoch = instance_epoch;
    policy->control.owner_core = STORAGE_ROLE_POLICY_OWNER_CORE;
    dmb_ishst();
    return STORAGE_ROLE_POLICY_OK;
}

enum storage_role_policy_result storage_role_policy_validate(
    struct storage_role_policy *policy,
    const struct storage_role_policy_input *input, u32 caller_core,
    struct storage_role_policy_handle *handle_out)
{
    u32 i;
    u32 j;

    if (handle_out)
        *handle_out = (struct storage_role_policy_handle){0};
    if (!owner(caller_core))
        return STORAGE_ROLE_POLICY_OWNER;
    if (!policy || policy->control.magic != STORAGE_ROLE_POLICY_MAGIC ||
        !input || !input->facts || !handle_out ||
        input->total_blocks == 0U || input->enumeration_generation == 0U ||
        input->fact_count != STORAGE_ROLE_POLICY_MAX_FACTS)
        return STORAGE_ROLE_POLICY_INVALID;
    if (policy->control.fingerprint != 0U)
        return STORAGE_ROLE_POLICY_BUSY;
    for (i = 0U; i < input->fact_count; i++) {
        if (!fact_valid(&input->facts[i], input->total_blocks))
            return STORAGE_ROLE_POLICY_REJECTED;
        for (j = 0U; j < i; j++)
            if (input->facts[i].identity == input->facts[j].identity ||
                input->facts[i].table_index == input->facts[j].table_index ||
                overlap(&input->facts[i], &input->facts[j]))
                return STORAGE_ROLE_POLICY_DUPLICATE;
    }
    if (input->facts[0].table_index != 0U ||
        input->facts[0].role != STORAGE_ROLE_BOOT ||
        input->facts[1].table_index != 1U ||
        input->facts[1].role != STORAGE_ROLE_WALFS ||
        input->facts[2].table_index != 2U ||
        input->facts[2].role != STORAGE_ROLE_EXCHANGE)
        return STORAGE_ROLE_POLICY_REJECTED;
    policy->facts[0] = input->facts[0];
    policy->facts[1] = input->facts[1];
    policy->facts[2] = input->facts[2];
    policy->control.enumeration_generation = input->enumeration_generation;
    policy->control.attachment_generation = 1U;
    policy->control.fingerprint = fingerprint(input);
    dmb_ishst();
    handle_out->magic = STORAGE_ROLE_POLICY_HANDLE_MAGIC;
    handle_out->instance_epoch = policy->control.instance_epoch;
    handle_out->enumeration_generation = input->enumeration_generation;
    handle_out->attachment_generation = 1U;
    handle_out->identity = policy->facts[1].identity;
    handle_out->fingerprint = policy->control.fingerprint;
    return STORAGE_ROLE_POLICY_OK;
}

enum storage_role_policy_result storage_role_policy_get(
    const struct storage_role_policy *policy,
    const struct storage_role_policy_handle *handle, u32 caller_core,
    struct storage_role_fact *facts_out, u32 facts_capacity,
    u32 *facts_used)
{
    if (facts_used)
        *facts_used = 0U;
    if (!facts_out || facts_capacity < STORAGE_ROLE_POLICY_MAX_FACTS)
        return STORAGE_ROLE_POLICY_INVALID;
    if (!handle_valid(policy, handle, caller_core))
        return STORAGE_ROLE_POLICY_STALE;
    facts_out[0] = policy->facts[0];
    facts_out[1] = policy->facts[1];
    facts_out[2] = policy->facts[2];
    if (facts_used)
        *facts_used = STORAGE_ROLE_POLICY_MAX_FACTS;
    return STORAGE_ROLE_POLICY_OK;
}
