/*
 * exchange_service.h - bounded, callback-backed PIOSXFER boot attachment.
 *
 * The caller provides the already-read MBR and a block callback.  The service
 * validates only p3, then permanently fences every FAT32 callback to that
 * validated span. It mounts a valid FAT32 p3 regardless of label. An
 * explicitly raw 0xDA p3 with no FAT signature may be formatted in place;
 * it never discovers or repartitions a partition.
 */
#pragma once

#include "types.h"
#include "exchange_volume_policy.h"
#include "fat32_exchange_core.h"
#include "storage_layout.h"

enum exchange_service_result {
    EXCHANGE_SERVICE_OK = 0,
    EXCHANGE_SERVICE_UNAVAILABLE,
    EXCHANGE_SERVICE_PARTITION_REJECTED,
    EXCHANGE_SERVICE_POLICY_REJECTED,
    EXCHANGE_SERVICE_CORE_FAILED,
    EXCHANGE_SERVICE_INVALID,
    EXCHANGE_SERVICE_READ_ONLY,
};

struct exchange_service_backend {
    fat32_exchange_read_sector_fn read;
    fat32_exchange_write_sector_fn write;
    void *context;
    bool writable;
    bool format_writable;
};

enum exchange_service_format_reason {
    EXCHANGE_SERVICE_FORMAT_NONE = 0U,
    EXCHANGE_SERVICE_FORMAT_EXISTING,
    EXCHANGE_SERVICE_FORMAT_RAW_READ_ONLY,
    EXCHANGE_SERVICE_FORMAT_RAW_NOT_ALLOWED,
    EXCHANGE_SERVICE_FORMAT_RAW_FORMATTED,
    EXCHANGE_SERVICE_FORMAT_CORRUPT,
    EXCHANGE_SERVICE_FORMAT_IO,
};

struct exchange_service_status {
    u32 available;
    u32 mounted;
    u32 read_only;
    u32 first_lba;
    u32 block_count;
    u32 disk_id;
    u32 last_result;
    u32 layout_result;
    u32 policy_result;
    u32 core_result;
    u32 formatted_this_boot;
    u32 format_reason;
    u8 _pad[16U];
} ALIGNED(64);

struct exchange_service {
    struct exchange_volume_policy policy;
    struct exchange_volume_policy_handle policy_handle ALIGNED(64);
    struct fat32_exchange_attachment attachment ALIGNED(64);
    struct fat32_exchange_volume volume ALIGNED(64);
    struct exchange_service_backend backend;
    struct exchange_service_status status ALIGNED(64);
} ALIGNED(64);

_Static_assert(sizeof(struct exchange_service_status) == 64U,
               "exchange status requires one cache line");

/*
 * Reads and, when format_writable, writes only the supplied p3 range through
 * backend callbacks. `mbr` is an
 * immutable snapshot acquired by the platform storage adapter, not through
 * this service's exchange callback. A successful mount is read-only unless
 * backend->writable is explicitly true.
 */
enum exchange_service_result exchange_service_init(
    struct exchange_service *service,
    const u8 mbr[STORAGE_LAYOUT_MBR_BYTES], u64 total_blocks,
    const struct exchange_service_backend *backend);

void exchange_service_status(const struct exchange_service *service,
                             struct exchange_service_status *out);
const char *exchange_service_result_name(enum exchange_service_result result);

enum fat32_exchange_result exchange_service_open(
    struct exchange_service *service, const struct fat32_exchange_name *name,
    struct fat32_exchange_file *out_file);
enum fat32_exchange_result exchange_service_read(
    struct exchange_service *service, const struct fat32_exchange_file *file,
    u32 offset, u8 *out, u32 capacity, u32 requested, u32 *out_read);
enum fat32_exchange_result exchange_service_create(
    struct exchange_service *service, const struct fat32_exchange_name *name,
    struct fat32_exchange_file *out_file);
enum fat32_exchange_result exchange_service_rewrite(
    struct exchange_service *service, const struct fat32_exchange_file *file,
    const u8 *data, u32 data_len, struct fat32_exchange_file *out_file);
enum fat32_exchange_result exchange_service_append(
    struct exchange_service *service, const struct fat32_exchange_file *file,
    const u8 *data, u32 data_len, struct fat32_exchange_file *out_file);
enum fat32_exchange_result exchange_service_rename(
    struct exchange_service *service, const struct fat32_exchange_file *file,
    const struct fat32_exchange_name *new_name,
    struct fat32_exchange_file *out_file);
enum fat32_exchange_result exchange_service_delete(
    struct exchange_service *service, const struct fat32_exchange_file *file);
enum fat32_exchange_result exchange_service_flush(struct exchange_service *service);
