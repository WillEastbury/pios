/*
 * fat32_exchange_core.h - offline, callback-backed FAT32 exchange volume.
 *
 * This is deliberately not the live fat32/sd path.  A caller supplies one
 * already-authorized partition identity and exact 512-byte block callbacks.
 */
#pragma once

#include "types.h"

#define FAT32_EXCHANGE_SECTOR_BYTES       512U
#define FAT32_EXCHANGE_NAME_BYTES         11U
#define FAT32_EXCHANGE_MAX_CLUSTERS       1048576U
#define FAT32_EXCHANGE_MAX_FAT_SECTORS    8192U
#define FAT32_EXCHANGE_MAX_FILE_BYTES     (2U * 1024U * 1024U)

enum fat32_exchange_result {
    FAT32_EXCHANGE_OK = 0,
    FAT32_EXCHANGE_INVALID,
    FAT32_EXCHANGE_IO,
    FAT32_EXCHANGE_BPB,
    FAT32_EXCHANGE_LABEL,
    FAT32_EXCHANGE_FAT_MISMATCH,
    FAT32_EXCHANGE_CORRUPT,
    FAT32_EXCHANGE_NOT_FOUND,
    FAT32_EXCHANGE_EXISTS,
    FAT32_EXCHANGE_NO_SPACE,
    FAT32_EXCHANGE_STALE,
    FAT32_EXCHANGE_OWNER,
    FAT32_EXCHANGE_FAULTED,
    FAT32_EXCHANGE_RANGE,
    FAT32_EXCHANGE_LFN,
};

/*
 * These facts are supplied by the exchange-partition policy layer.  This
 * module does not discover partitions or authorize a disk.  The BPB label is
 * an exact eleven-byte FAT field, not a C string.
 */
struct fat32_exchange_attachment {
    u64 identity;
    u32 epoch;
    u32 first_lba;
    u32 block_count;
    u8 expected_label[FAT32_EXCHANGE_NAME_BYTES];
} ALIGNED(64);

_Static_assert(sizeof(struct fat32_exchange_attachment) == 64U,
               "attachment facts must occupy one cache line");

/*
 * Every callback transfers exactly one 512-byte sector on true.  A short
 * transfer must return false.  Callbacks are retained only while mounted.
 */
typedef bool (*fat32_exchange_read_sector_fn)(void *context, u32 lba,
                                              u8 out[FAT32_EXCHANGE_SECTOR_BYTES]);
typedef bool (*fat32_exchange_write_sector_fn)(void *context, u32 lba,
                                               const u8 in[FAT32_EXCHANGE_SECTOR_BYTES]);

struct fat32_exchange_io {
    fat32_exchange_read_sector_fn read;
    fat32_exchange_write_sector_fn write;
    void *context;
};

/* Explicit-length, uppercase ASCII 8.3 spelling such as {'A','.','T','X','T'}. */
struct fat32_exchange_name {
    const u8 *bytes;
    u32 len;
};

/*
 * A file capability is valid only on the mounted core-0 instance that issued
 * it.  `generation` changes after every metadata mutation; callers receive a
 * refreshed capability through the out parameter of a successful mutation.
 */
struct fat32_exchange_file {
    u64 identity;
    u32 epoch;
    u32 mount_generation;
    u32 generation;
    u32 dir_lba;
    u16 dir_offset;
    u16 _reserved;
    u8 short_name[FAT32_EXCHANGE_NAME_BYTES];
    u8 _pad[17U];
} ALIGNED(64);

_Static_assert(sizeof(struct fat32_exchange_file) == 64U,
               "file capabilities must have cache-line stride");

/*
 * Caller-owned state. Initialize it with fat32_exchange_volume_init() before
 * the first mount. It is a single-core (core 0), non-IRQ object and is not
 * safe for concurrent use. No state is global and no allocation occurs.
 */
struct fat32_exchange_volume {
    struct fat32_exchange_io io;
    u64 identity;
    u32 epoch;
    u32 first_lba;
    u32 block_count;
    u32 fat_lba;
    u32 fat_sectors;
    u32 data_lba;
    u32 sectors_per_cluster;
    u32 root_cluster;
    u32 cluster_count;
    u32 alloc_hint;
    u32 mount_generation;
    u32 generation;
    u32 mounted;
    u32 faulted;
    u32 fsinfo_lba;
    u32 fsinfo_needs_unknown;
    u8 sector_a[FAT32_EXCHANGE_SECTOR_BYTES];
    u8 sector_b[FAT32_EXCHANGE_SECTOR_BYTES];
} ALIGNED(64);

/* Call before first mount; it also invalidates every prior file capability. */
void fat32_exchange_volume_init(struct fat32_exchange_volume *volume);

enum fat32_exchange_result fat32_exchange_mount(
    struct fat32_exchange_volume *volume,
    const struct fat32_exchange_attachment *attachment,
    const struct fat32_exchange_io *io, u32 caller_core, bool in_irq);

enum fat32_exchange_result fat32_exchange_open(
    struct fat32_exchange_volume *volume,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_name *name,
    struct fat32_exchange_file *out_file);

enum fat32_exchange_result fat32_exchange_create(
    struct fat32_exchange_volume *volume,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_name *name,
    struct fat32_exchange_file *out_file);

enum fat32_exchange_result fat32_exchange_read(
    struct fat32_exchange_volume *volume,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_file *file,
    u32 offset, u8 *out, u32 capacity, u32 requested, u32 *out_read);

enum fat32_exchange_result fat32_exchange_rewrite(
    struct fat32_exchange_volume *volume,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_file *file,
    const u8 *data, u32 data_len, struct fat32_exchange_file *out_file);

enum fat32_exchange_result fat32_exchange_append(
    struct fat32_exchange_volume *volume,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_file *file,
    const u8 *data, u32 data_len, struct fat32_exchange_file *out_file);

enum fat32_exchange_result fat32_exchange_delete(
    struct fat32_exchange_volume *volume,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_file *file);

enum fat32_exchange_result fat32_exchange_rename(
    struct fat32_exchange_volume *volume,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq, const struct fat32_exchange_file *file,
    const struct fat32_exchange_name *new_name,
    struct fat32_exchange_file *out_file);

/* Verifies every mirrored FAT sector after prior synchronous writes. */
enum fat32_exchange_result fat32_exchange_flush(
    struct fat32_exchange_volume *volume,
    const struct fat32_exchange_attachment *attachment,
    u32 caller_core, bool in_irq);
