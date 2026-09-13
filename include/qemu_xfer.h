/*
 * qemu_xfer.h - narrow, QEMU-only FAT32 exchange acceptance adapter.
 *
 * This is unavailable outside QEMU.  It can only authorize the validated
 * PIOSXFER MBR partition 3 and never discovers or accesses p1/p2.
 */
#pragma once

#include "types.h"

#define QEMU_XFER_COMMAND_BYTES 1024U

enum qemu_xfer_result {
    QEMU_XFER_OK = 0,
    QEMU_XFER_UNAVAILABLE,
    QEMU_XFER_PARTITION_REJECTED,
    QEMU_XFER_CORE_FAILED,
    QEMU_XFER_ARGUMENT,
};

struct qemu_xfer_status {
    u32 available;
    u32 mounted;
    u32 first_lba;
    u32 block_count;
    u32 last_result;
    u32 core_result;
    u32 disk_id;
    u32 _reserved;
} ALIGNED(64);

_Static_assert(sizeof(struct qemu_xfer_status) == 64U,
               "QEMU xfer status requires one cache line");

void qemu_xfer_init(void);
void qemu_xfer_status(struct qemu_xfer_status *out);
const char *qemu_xfer_result_name(enum qemu_xfer_result result);

enum qemu_xfer_result qemu_xfer_write(const char *name, const u8 *data,
                                      u32 data_len);
enum qemu_xfer_result qemu_xfer_append(const char *name, const u8 *data,
                                       u32 data_len);
enum qemu_xfer_result qemu_xfer_read(const char *name, u8 *out, u32 capacity,
                                     u32 *out_read);
enum qemu_xfer_result qemu_xfer_rename(const char *old_name,
                                       const char *new_name);
enum qemu_xfer_result qemu_xfer_delete(const char *name);
enum qemu_xfer_result qemu_xfer_verify_remount(void);
