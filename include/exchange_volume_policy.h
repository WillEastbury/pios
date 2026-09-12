/*
 * exchange_volume_policy.h - offline #193 FAT32 exchange-partition policy.
 *
 * This consumes caller-enumerated partition facts only.  It neither reads a
 * table nor mounts, formats, reads, or writes a filesystem.  In particular it
 * deliberately does not depend on partition_table: an adapter may translate a
 * validated snapshot later, but this policy has a complete independent input.
 */
#pragma once

#include "types.h"

#define EXCHANGE_VOLUME_POLICY_OWNER_CORE       0U
#define EXCHANGE_VOLUME_POLICY_FACT_CAPACITY    16U
#define EXCHANGE_VOLUME_POLICY_LABEL_BYTES      16U
#define EXCHANGE_VOLUME_POLICY_MAGIC            0x50494F5358464552ULL
#define EXCHANGE_VOLUME_POLICY_HANDLE_MAGIC     0x5846455248414E44ULL

enum exchange_volume_table_scheme {
    EXCHANGE_VOLUME_TABLE_INVALID = 0U,
    EXCHANGE_VOLUME_TABLE_MBR,
    EXCHANGE_VOLUME_TABLE_GPT,
};

enum exchange_volume_filesystem {
    EXCHANGE_VOLUME_FILESYSTEM_UNKNOWN = 0U,
    EXCHANGE_VOLUME_FILESYSTEM_FAT32,
};

enum exchange_volume_fact_flags {
    EXCHANGE_VOLUME_FACT_BOOT = 1U << 0,
    EXCHANGE_VOLUME_FACT_PIOS_SYSTEM = 1U << 1,
};

enum exchange_volume_policy_state {
    EXCHANGE_VOLUME_POLICY_EMPTY = 0U,
    EXCHANGE_VOLUME_POLICY_READY,
    EXCHANGE_VOLUME_POLICY_ATTACHED,
};

enum exchange_volume_policy_result {
    EXCHANGE_VOLUME_POLICY_OK = 0U,
    EXCHANGE_VOLUME_POLICY_INVALID,
    EXCHANGE_VOLUME_POLICY_OWNER,
    EXCHANGE_VOLUME_POLICY_UNINITIALIZED,
    EXCHANGE_VOLUME_POLICY_BUSY,
    EXCHANGE_VOLUME_POLICY_REJECTED,
    EXCHANGE_VOLUME_POLICY_DUPLICATE,
    EXCHANGE_VOLUME_POLICY_STALE,
};

/*
 * One fact is a canonical, immutable statement about one enumerated
 * partition.  `identity` is an externally allocated, nonzero partition
 * identity; it is not an LBA-derived substitute.  GPT GUID bytes are in UEFI
 * on-disk order.  Label bytes are explicit-length and zero-padded.
 */
struct exchange_volume_partition_fact {
    u64 identity;
    u64 first_lba;
    u64 block_count;
    u32 table_index;
    u32 table_scheme;
    u32 filesystem;
    u32 flags;
    u8 mbr_type;
    u8 filesystem_label_bytes;
    u8 _reserved0[6U];
    u8 gpt_type_guid[16U];
    u8 filesystem_label[EXCHANGE_VOLUME_POLICY_LABEL_BYTES];
    u8 _pad[48U];
} ALIGNED(64);

/*
 * `enumeration_generation` identifies the immutable enumeration used to make
 * this decision.  The facts pointer is borrowed only for the call and is
 * never retained.  One input must describe exactly one table scheme.
 */
struct exchange_volume_policy_input {
    const struct exchange_volume_partition_fact *facts;
    u64 total_blocks;
    u64 enumeration_generation;
    u32 fact_count;
    u32 table_scheme;
};

/* An operator must name this identity; there is intentionally no default. */
struct exchange_volume_policy_request {
    u64 requested_identity;
};

/* Opaque numeric attachment evidence; it conveys no filesystem authority. */
struct exchange_volume_policy_handle {
    u64 magic;
    u64 instance_epoch;
    u64 enumeration_generation;
    u64 attachment_generation;
    u64 identity;
    u64 first_lba;
    u64 block_count;
    u64 fingerprint;
} ALIGNED(64);

/* Mutable state and copied immutable evidence occupy separate cache lines. */
struct exchange_volume_policy_control {
    u64 magic;
    u64 instance_epoch;
    u64 enumeration_generation;
    u64 attachment_generation;
    u64 attachment_fingerprint;
    u64 total_blocks;
    u32 owner_core;
    u32 state;
    u32 mutation_guard;
    u32 _reserved;
} ALIGNED(64);

struct exchange_volume_policy {
    struct exchange_volume_policy_control control;
    struct exchange_volume_partition_fact attachment;
} ALIGNED(64);

_Static_assert(sizeof(struct exchange_volume_partition_fact) == 128U,
               "exchange facts must retain two cache-line stride");
_Static_assert(sizeof(struct exchange_volume_policy_handle) == 64U,
               "exchange handle must own one cache line");
_Static_assert(sizeof(struct exchange_volume_policy_control) == 64U,
               "exchange mutable control must own one cache line");
_Static_assert(__builtin_offsetof(struct exchange_volume_policy, attachment) ==
               64U, "exchange evidence must not share mutable control");

/*
 * Initialization is one-shot over all-zero fixed storage.  `instance_epoch`
 * must be nonzero and never reused for replacement policy storage.
 */
enum exchange_volume_policy_result exchange_volume_policy_init(
    struct exchange_volume_policy *policy, u64 instance_epoch,
    u32 caller_core);

/*
 * The sole selection operation.  It accepts exactly one non-boot,
 * non-PIOS-system FAT32 fact, with exact `PIOSXFER` label and the table-type
 * constraints documented in ADR-067.  Any multiple eligible facts fail
 * closed, even if the operator names one of them.
 */
enum exchange_volume_policy_result exchange_volume_policy_attach(
    struct exchange_volume_policy *policy,
    const struct exchange_volume_policy_input *input,
    const struct exchange_volume_policy_request *request,
    u32 caller_core, struct exchange_volume_policy_handle *handle_out);

/* Returns only the immutable fact bound to a current policy handle. */
enum exchange_volume_policy_result exchange_volume_policy_attachment_get(
    const struct exchange_volume_policy *policy,
    const struct exchange_volume_policy_handle *handle, u32 caller_core,
    struct exchange_volume_partition_fact *fact_out);
