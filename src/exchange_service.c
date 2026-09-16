/*
 * Generic PIOSXFER attachment. It has no SD/WALFS/platform dependency:
 * callers must supply an MBR snapshot and exact-sector backend.
 */
#include "types.h"
#include "exchange_service.h"

static bool authorized_lba(const struct exchange_service *service, u32 lba)
{
    return service->attachment.block_count != 0U &&
           lba >= service->attachment.first_lba &&
           lba - service->attachment.first_lba < service->attachment.block_count;
}

static bool service_read(void *context, u32 lba, u8 out[512])
{
    struct exchange_service *service = context;

    return service && authorized_lba(service, lba) &&
           service->backend.read(service->backend.context, lba, out);
}

static bool service_write(void *context, u32 lba, const u8 in[512])
{
    struct exchange_service *service = context;

    return service && service->backend.writable && service->backend.write &&
           authorized_lba(service, lba) &&
           service->backend.write(service->backend.context, lba, in);
}

static void policy_fact_make(const struct storage_layout_fact *p3,
                             struct exchange_volume_partition_fact *fact)
{
    *fact = (struct exchange_volume_partition_fact){0};
    fact->identity = ((u64)p3->disk_id << 32) | p3->first_lba;
    fact->first_lba = p3->first_lba;
    fact->block_count = p3->block_count;
    fact->table_index = p3->primary_index;
    fact->table_scheme = EXCHANGE_VOLUME_TABLE_MBR;
    fact->filesystem = EXCHANGE_VOLUME_FILESYSTEM_FAT32;
    fact->mbr_type = p3->mbr_type;
}

static enum exchange_service_result core_result(
    struct exchange_service *service, enum fat32_exchange_result result)
{
    service->status.core_result = (u32)result;
    service->status.last_result = result == FAT32_EXCHANGE_OK ?
                                  EXCHANGE_SERVICE_OK :
                                  EXCHANGE_SERVICE_CORE_FAILED;
    service->status.available = result == FAT32_EXCHANGE_OK;
    service->status.mounted = service->volume.mounted;
    return (enum exchange_service_result)service->status.last_result;
}

enum exchange_service_result exchange_service_init(
    struct exchange_service *service,
    const u8 mbr[STORAGE_LAYOUT_MBR_BYTES], u64 total_blocks,
    const struct exchange_service_backend *backend)
{
    struct storage_layout layout;
    struct exchange_volume_partition_fact fact;
    struct exchange_volume_partition_fact attached;
    struct exchange_volume_policy_input input;
    struct exchange_volume_policy_request request;
    struct fat32_exchange_io io;
    const struct storage_layout_fact *p3;
    enum storage_layout_result layout_result;
    enum exchange_volume_policy_result policy_result;
    enum fat32_exchange_result mount_result;

    if (!service)
        return EXCHANGE_SERVICE_INVALID;
    *service = (struct exchange_service){0};
    if (!mbr || !backend || !backend->read || total_blocks == 0U) {
        service->status.last_result = EXCHANGE_SERVICE_INVALID;
        return EXCHANGE_SERVICE_INVALID;
    }
    service->status.read_only = !backend->writable;
    service->backend = *backend;

    layout_result = storage_layout_validate_exchange(mbr, total_blocks, &layout);
    service->status.layout_result = (u32)layout_result;
    if (layout_result == STORAGE_LAYOUT_P3_ABSENT) {
        service->status.last_result = EXCHANGE_SERVICE_UNAVAILABLE;
        return EXCHANGE_SERVICE_UNAVAILABLE;
    }
    if (layout_result != STORAGE_LAYOUT_OK) {
        service->status.last_result = EXCHANGE_SERVICE_PARTITION_REJECTED;
        return EXCHANGE_SERVICE_PARTITION_REJECTED;
    }
    p3 = &layout.facts[STORAGE_LAYOUT_ROLE_EXCHANGE - 1U];
    if (p3->disk_id == 0U || p3->first_lba > 0xFFFFFFFFULL ||
        p3->block_count > 0xFFFFFFFFULL) {
        service->status.last_result = EXCHANGE_SERVICE_PARTITION_REJECTED;
        return EXCHANGE_SERVICE_PARTITION_REJECTED;
    }
    service->status.first_lba = (u32)p3->first_lba;
    service->status.block_count = (u32)p3->block_count;
    service->status.disk_id = p3->disk_id;

    policy_fact_make(p3, &fact);
    input = (struct exchange_volume_policy_input) {
        .facts = &fact,
        .total_blocks = total_blocks,
        .enumeration_generation = fact.identity,
        .fact_count = 1U,
        .table_scheme = EXCHANGE_VOLUME_TABLE_MBR,
    };
    request = (struct exchange_volume_policy_request) {
        .requested_identity = fact.identity,
    };
    policy_result = exchange_volume_policy_init(&service->policy, fact.identity,
                                                EXCHANGE_VOLUME_POLICY_OWNER_CORE);
    if (policy_result == EXCHANGE_VOLUME_POLICY_OK)
        policy_result = exchange_volume_policy_attach(&service->policy, &input,
                                                       &request,
                                                       EXCHANGE_VOLUME_POLICY_OWNER_CORE,
                                                       &service->policy_handle);
    if (policy_result == EXCHANGE_VOLUME_POLICY_OK)
        policy_result = exchange_volume_policy_attachment_get(
            &service->policy, &service->policy_handle,
            EXCHANGE_VOLUME_POLICY_OWNER_CORE, &attached);
    service->status.policy_result = (u32)policy_result;
    if (policy_result != EXCHANGE_VOLUME_POLICY_OK) {
        service->status.last_result = EXCHANGE_SERVICE_POLICY_REJECTED;
        return EXCHANGE_SERVICE_POLICY_REJECTED;
    }

    service->attachment.identity = attached.identity;
    service->attachment.epoch = p3->disk_id;
    service->attachment.first_lba = (u32)attached.first_lba;
    service->attachment.block_count = (u32)attached.block_count;
    io = (struct fat32_exchange_io) {
        .read = service_read,
        .write = service_write,
        .context = service,
    };
    fat32_exchange_volume_init(&service->volume);
    mount_result = fat32_exchange_mount(&service->volume, &service->attachment,
                                        &io, 0U, false);
    return core_result(service, mount_result);
}

void exchange_service_status(const struct exchange_service *service,
                             struct exchange_service_status *out)
{
    if (out)
        *out = service ? service->status : (struct exchange_service_status){0};
}

const char *exchange_service_result_name(enum exchange_service_result result)
{
    switch (result) {
    case EXCHANGE_SERVICE_OK: return "ok";
    case EXCHANGE_SERVICE_UNAVAILABLE: return "unavailable";
    case EXCHANGE_SERVICE_PARTITION_REJECTED: return "partition-rejected";
    case EXCHANGE_SERVICE_POLICY_REJECTED: return "policy-rejected";
    case EXCHANGE_SERVICE_CORE_FAILED: return "fat32-failed";
    case EXCHANGE_SERVICE_READ_ONLY: return "read-only";
    case EXCHANGE_SERVICE_INVALID: return "invalid";
    default: return "unknown";
    }
}

enum fat32_exchange_result exchange_service_open(
    struct exchange_service *service, const struct fat32_exchange_name *name,
    struct fat32_exchange_file *out_file)
{
    return service ? fat32_exchange_open(&service->volume, &service->attachment,
                                         0U, false, name, out_file) :
                     FAT32_EXCHANGE_INVALID;
}

enum fat32_exchange_result exchange_service_read(
    struct exchange_service *service, const struct fat32_exchange_file *file,
    u32 offset, u8 *out, u32 capacity, u32 requested, u32 *out_read)
{
    return service ? fat32_exchange_read(&service->volume, &service->attachment,
                                         0U, false, file, offset, out, capacity,
                                         requested, out_read) : FAT32_EXCHANGE_INVALID;
}

static enum fat32_exchange_result writable(struct exchange_service *service)
{
    return service && service->backend.writable ? FAT32_EXCHANGE_OK :
                                                   FAT32_EXCHANGE_OWNER;
}

enum fat32_exchange_result exchange_service_create(
    struct exchange_service *service, const struct fat32_exchange_name *name,
    struct fat32_exchange_file *out_file)
{
    return writable(service) == FAT32_EXCHANGE_OK ?
           fat32_exchange_create(&service->volume, &service->attachment, 0U,
                                 false, name, out_file) : FAT32_EXCHANGE_OWNER;
}

enum fat32_exchange_result exchange_service_rewrite(
    struct exchange_service *service, const struct fat32_exchange_file *file,
    const u8 *data, u32 data_len, struct fat32_exchange_file *out_file)
{
    return writable(service) == FAT32_EXCHANGE_OK ?
           fat32_exchange_rewrite(&service->volume, &service->attachment, 0U,
                                  false, file, data, data_len, out_file) :
           FAT32_EXCHANGE_OWNER;
}

enum fat32_exchange_result exchange_service_append(
    struct exchange_service *service, const struct fat32_exchange_file *file,
    const u8 *data, u32 data_len, struct fat32_exchange_file *out_file)
{
    return writable(service) == FAT32_EXCHANGE_OK ?
           fat32_exchange_append(&service->volume, &service->attachment, 0U,
                                 false, file, data, data_len, out_file) :
           FAT32_EXCHANGE_OWNER;
}

enum fat32_exchange_result exchange_service_rename(
    struct exchange_service *service, const struct fat32_exchange_file *file,
    const struct fat32_exchange_name *new_name,
    struct fat32_exchange_file *out_file)
{
    return writable(service) == FAT32_EXCHANGE_OK ?
           fat32_exchange_rename(&service->volume, &service->attachment, 0U,
                                 false, file, new_name, out_file) :
           FAT32_EXCHANGE_OWNER;
}

enum fat32_exchange_result exchange_service_delete(
    struct exchange_service *service, const struct fat32_exchange_file *file)
{
    return writable(service) == FAT32_EXCHANGE_OK ?
           fat32_exchange_delete(&service->volume, &service->attachment, 0U,
                                 false, file) : FAT32_EXCHANGE_OWNER;
}

enum fat32_exchange_result exchange_service_flush(struct exchange_service *service)
{
    return service ? fat32_exchange_flush(&service->volume, &service->attachment,
                                          0U, false) : FAT32_EXCHANGE_INVALID;
}
