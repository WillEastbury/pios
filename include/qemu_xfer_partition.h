/*
 * qemu_xfer_partition.h - QEMU selection of shared-layout PIOSXFER MBR p3.
 *
 * This recognizes one deliberately fixed test layout.  It grants no block
 * authority; qemu_xfer.c binds the resulting facts to its QEMU-only callback.
 */
#pragma once

#include "types.h"

#define QEMU_XFER_MBR_BYTES       512U
#define QEMU_XFER_PARTITION_INDEX 3U
#define QEMU_XFER_LABEL_BYTES     11U

enum qemu_xfer_partition_result {
    QEMU_XFER_PARTITION_OK = 0,
    QEMU_XFER_PARTITION_INVALID,
    QEMU_XFER_PARTITION_MBR,
    QEMU_XFER_PARTITION_LAYOUT,
};

struct qemu_xfer_partition {
    u64 identity;
    u32 disk_id;
    u32 first_lba;
    u32 block_count;
    u8 expected_label[QEMU_XFER_LABEL_BYTES];
    u8 _pad[33U];
} ALIGNED(64);

_Static_assert(sizeof(struct qemu_xfer_partition) == 64U,
               "QEMU xfer partition facts require one cache line");

/*
 * Selects exactly MBR primary partition 3 only after the shared validator
 * accepts the fixed production p1/p2/p3 roles. `total_blocks` is the
 * authoritative capacity in 512-byte blocks.
 */
enum qemu_xfer_partition_result qemu_xfer_partition_select(
    const u8 mbr[QEMU_XFER_MBR_BYTES], u64 total_blocks,
    struct qemu_xfer_partition *out);
