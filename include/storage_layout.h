/*
 * storage_layout.h - fixed PIOS MBR storage-layout validation.
 *
 * This is an observation-only parser for the pre-created primary MBR layout.
 * It neither reads a device nor creates, formats, or writes a partition.
 */
#pragma once

#include "types.h"

#define STORAGE_LAYOUT_MBR_BYTES 512U
#define STORAGE_LAYOUT_FACT_COUNT 3U

enum storage_layout_role {
    STORAGE_LAYOUT_ROLE_BOOT = 1U,
    STORAGE_LAYOUT_ROLE_SYSTEM = 2U,
    STORAGE_LAYOUT_ROLE_EXCHANGE = 3U,
};

enum storage_layout_kind {
    STORAGE_LAYOUT_NONE = 0U,
    STORAGE_LAYOUT_LEGACY_TWO_PARTITION,
    STORAGE_LAYOUT_THREE_PARTITION,
};

enum storage_layout_result {
    STORAGE_LAYOUT_OK = 0U,
    STORAGE_LAYOUT_INVALID_ARGUMENT,
    STORAGE_LAYOUT_MBR_SIGNATURE,
    STORAGE_LAYOUT_MALFORMED,
    STORAGE_LAYOUT_ROLE_MISMATCH,
};

/* Observed numeric facts. MBR status is retained as evidence, never authority. */
struct storage_layout_fact {
    u64 first_lba;
    u64 block_count;
    u32 disk_id;
    u8 role;
    u8 primary_index;
    u8 mbr_type;
    u8 mbr_status;
    u8 _pad[40U];
} ALIGNED(64);

struct storage_layout {
    u64 total_blocks;
    u32 disk_id;
    u32 kind;
    u32 result;
    u32 _reserved;
    u8 _pad[40U];
    struct storage_layout_fact facts[STORAGE_LAYOUT_FACT_COUNT];
} ALIGNED(64);

_Static_assert(sizeof(struct storage_layout_fact) == 64U,
               "storage facts require cache-line stride");
_Static_assert(sizeof(struct storage_layout) ==
               64U + STORAGE_LAYOUT_FACT_COUNT * 64U,
               "storage layout must isolate immutable facts");

/*
 * Validates an MBR against the fixed PIOS roles:
 *   p1 (index 0): FAT32 0x0b/0x0c boot
 *   p2 (index 1): raw PIOS/WALFS 0xda system
 *   p3 (index 2): FAT32 0x0b/0x0c PIOSXFER
 *   p4 (index 3): empty
 *
 * A historical two-partition p1/p2 layout remains observable as
 * STORAGE_LAYOUT_LEGACY_TWO_PARTITION. It accepts the old p2 type so old
 * cards can mount read-only/explicitly formatted WALFS without migration.
 * `total_blocks` is the authoritative count of 512-byte sectors.
 */
enum storage_layout_result storage_layout_validate(
    const u8 mbr[STORAGE_LAYOUT_MBR_BYTES], u64 total_blocks,
    struct storage_layout *out);

const char *storage_layout_kind_name(enum storage_layout_kind kind);
