/*
 * storage_role_policy.h - offline partition-role authority contract.
 *
 * This records explicit role markers supplied by a future storage adapter.
 * It does not read, format, mount, or write a partition.
 */
#pragma once

#include "types.h"

#define STORAGE_ROLE_POLICY_OWNER_CORE 0U
#define STORAGE_ROLE_POLICY_MAX_FACTS 3U
#define STORAGE_ROLE_POLICY_MAGIC 0x53524F4C45504F4FULL
#define STORAGE_ROLE_POLICY_HANDLE_MAGIC 0x53524F4C4548414EULL

enum storage_role {
    STORAGE_ROLE_UNKNOWN = 0U,
    STORAGE_ROLE_BOOT = 1U,
    STORAGE_ROLE_WALFS = 2U,
    STORAGE_ROLE_EXCHANGE = 3U,
};

enum storage_role_marker {
    STORAGE_ROLE_MARKER_NONE = 0U,
    STORAGE_ROLE_MARKER_PIOS_RESERVED = 1U,
    STORAGE_ROLE_MARKER_WALFS_SUPERBLOCK = 2U,
    STORAGE_ROLE_MARKER_FAT32_BPB = 3U,
};

enum storage_role_policy_result {
    STORAGE_ROLE_POLICY_OK = 0U,
    STORAGE_ROLE_POLICY_INVALID,
    STORAGE_ROLE_POLICY_OWNER,
    STORAGE_ROLE_POLICY_BUSY,
    STORAGE_ROLE_POLICY_REJECTED,
    STORAGE_ROLE_POLICY_DUPLICATE,
    STORAGE_ROLE_POLICY_STALE,
};

/*
 * A marker is evidence read from the partition, not an MBR type guess.
 * A WALFS role requires both the PIOS reserved-area marker and WALFS marker.
 */
struct storage_role_fact {
    u64 identity;
    u64 first_lba;
    u64 block_count;
    u32 table_index;
    u32 role;
    u32 marker;
    u32 marker_version;
    u32 filesystem;
    u8 _reserved[20U];
} ALIGNED(64);

struct storage_role_policy_input {
    const struct storage_role_fact *facts;
    u64 total_blocks;
    u64 enumeration_generation;
    u32 fact_count;
    u32 _reserved;
};

struct storage_role_policy_handle {
    u64 magic;
    u64 instance_epoch;
    u64 enumeration_generation;
    u64 attachment_generation;
    u64 identity;
    u64 fingerprint;
} ALIGNED(64);

struct storage_role_policy_control {
    u64 magic;
    u64 instance_epoch;
    u64 enumeration_generation;
    u64 attachment_generation;
    u64 fingerprint;
    u32 owner_core;
    u32 state;
    u32 mutation_guard;
    u32 _reserved;
} ALIGNED(64);

struct storage_role_policy {
    struct storage_role_policy_control control;
    struct storage_role_fact facts[STORAGE_ROLE_POLICY_MAX_FACTS];
} ALIGNED(64);

_Static_assert(sizeof(struct storage_role_fact) == 64U,
               "storage role facts need cache-line stride");
_Static_assert(sizeof(struct storage_role_policy_handle) == 64U,
               "storage role handles need cache-line stride");
_Static_assert(sizeof(struct storage_role_policy_control) == 64U,
               "storage role control needs cache-line stride");
_Static_assert(__builtin_offsetof(struct storage_role_policy, facts) == 64U,
               "storage role facts must not share control");

enum storage_role_policy_result storage_role_policy_init(
    struct storage_role_policy *policy, u64 instance_epoch, u32 caller_core);

enum storage_role_policy_result storage_role_policy_validate(
    struct storage_role_policy *policy,
    const struct storage_role_policy_input *input, u32 caller_core,
    struct storage_role_policy_handle *handle_out);

enum storage_role_policy_result storage_role_policy_get(
    const struct storage_role_policy *policy,
    const struct storage_role_policy_handle *handle, u32 caller_core,
    struct storage_role_fact *facts_out, u32 facts_capacity,
    u32 *facts_used);
