/*
 * QEMU command compatibility over the generic production exchange service.
 * Partition selection, BPB verification, and p3 range fencing are owned by
 * exchange_service; this file only retains the existing acceptance commands.
 */
#include "types.h"
#include "platform.h"
#include "qemu_xfer.h"
#include "exchange.h"

static enum qemu_xfer_result core_result(enum fat32_exchange_result result)
{
    return result == FAT32_EXCHANGE_OK ? QEMU_XFER_OK : QEMU_XFER_CORE_FAILED;
}

static bool name_make(const char *text, struct fat32_exchange_name *out)
{
    u32 len = 0U;

    if (!text || !out)
        return false;
    while (len <= 12U && text[len] != 0)
        len++;
    if (len == 0U || len > 12U)
        return false;
    out->bytes = (const u8 *)text;
    out->len = len;
    return true;
}

void qemu_xfer_status(struct qemu_xfer_status *out)
{
    struct exchange_service_status status;

    if (!out)
        return;
    exchange_status(&status);
    *out = (struct qemu_xfer_status) {
        .available = status.available,
        .mounted = status.mounted,
        .first_lba = status.first_lba,
        .block_count = status.block_count,
        .last_result =
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
            status.last_result == EXCHANGE_SERVICE_OK ? QEMU_XFER_OK :
            status.last_result == EXCHANGE_SERVICE_UNAVAILABLE ?
            QEMU_XFER_UNAVAILABLE : QEMU_XFER_PARTITION_REJECTED,
#else
            status.last_result == EXCHANGE_SERVICE_OK ? QEMU_XFER_READ_ONLY :
            status.last_result == EXCHANGE_SERVICE_UNAVAILABLE ?
            QEMU_XFER_UNAVAILABLE : QEMU_XFER_PARTITION_REJECTED,
#endif
        .core_result = status.core_result,
        .disk_id = status.disk_id,
        .writable = !status.read_only,
    };
}

const char *qemu_xfer_result_name(enum qemu_xfer_result result)
{
    switch (result) {
    case QEMU_XFER_OK: return "ok";
    case QEMU_XFER_UNAVAILABLE: return "unavailable";
    case QEMU_XFER_PARTITION_REJECTED: return "partition-rejected";
    case QEMU_XFER_CORE_FAILED: return "fat32-failed";
    case QEMU_XFER_ARGUMENT: return "argument";
    case QEMU_XFER_READ_ONLY: return "read-only";
    default: return "unknown";
    }
}

static enum qemu_xfer_result require_name(const char *text,
                                          struct fat32_exchange_name *name)
{
#if PIOS_PLATFORM != PIOS_PLATFORM_QEMU_VIRT
    struct qemu_xfer_status status;

    (void)text;
    (void)name;
    qemu_xfer_status(&status);
    return (enum qemu_xfer_result)status.last_result;
#else
    struct qemu_xfer_status status;

    qemu_xfer_status(&status);
    if (!status.available)
        return (enum qemu_xfer_result)status.last_result;
    return name_make(text, name) ? QEMU_XFER_OK : QEMU_XFER_ARGUMENT;
#endif
}

enum qemu_xfer_result qemu_xfer_write(const char *name, const u8 *data,
                                      u32 data_len)
{
    struct fat32_exchange_name file_name;
    struct fat32_exchange_file file;
    struct fat32_exchange_file updated;
    enum fat32_exchange_result result;
    enum qemu_xfer_result ready = require_name(name, &file_name);

    if (ready != QEMU_XFER_OK || (data_len != 0U && !data) ||
        data_len > QEMU_XFER_COMMAND_BYTES)
        return ready != QEMU_XFER_OK ? ready : QEMU_XFER_ARGUMENT;
    result = exchange_service_open(exchange_runtime_service(), &file_name, &file);
    if (result == FAT32_EXCHANGE_NOT_FOUND)
        result = exchange_service_create(exchange_runtime_service(), &file_name,
                                         &file);
    if (result != FAT32_EXCHANGE_OK)
        return core_result(result);
    return core_result(exchange_service_rewrite(exchange_runtime_service(), &file,
                                                data, data_len, &updated));
}

enum qemu_xfer_result qemu_xfer_append(const char *name, const u8 *data,
                                       u32 data_len)
{
    struct fat32_exchange_name file_name;
    struct fat32_exchange_file file;
    struct fat32_exchange_file updated;
    enum fat32_exchange_result result;
    enum qemu_xfer_result ready = require_name(name, &file_name);

    if (ready != QEMU_XFER_OK || (data_len != 0U && !data) ||
        data_len > QEMU_XFER_COMMAND_BYTES)
        return ready != QEMU_XFER_OK ? ready : QEMU_XFER_ARGUMENT;
    result = exchange_service_open(exchange_runtime_service(), &file_name, &file);
    if (result == FAT32_EXCHANGE_NOT_FOUND)
        result = exchange_service_create(exchange_runtime_service(), &file_name,
                                         &file);
    if (result != FAT32_EXCHANGE_OK)
        return core_result(result);
    return core_result(exchange_service_append(exchange_runtime_service(), &file,
                                               data, data_len, &updated));
}

enum qemu_xfer_result qemu_xfer_read(const char *name, u8 *out, u32 capacity,
                                     u32 *out_read)
{
    struct fat32_exchange_name file_name;
    struct fat32_exchange_file file;
    enum qemu_xfer_result ready = require_name(name, &file_name);
    enum fat32_exchange_result result;

    if (out_read)
        *out_read = 0U;
    if (ready != QEMU_XFER_OK || !out || !out_read ||
        capacity > QEMU_XFER_COMMAND_BYTES)
        return ready != QEMU_XFER_OK ? ready : QEMU_XFER_ARGUMENT;
    result = exchange_service_open(exchange_runtime_service(), &file_name, &file);
    if (result != FAT32_EXCHANGE_OK)
        return core_result(result);
    return core_result(exchange_service_read(exchange_runtime_service(), &file, 0U,
                                             out, capacity, capacity, out_read));
}

enum qemu_xfer_result qemu_xfer_rename(const char *old_name,
                                       const char *new_name)
{
    struct fat32_exchange_name old_file_name;
    struct fat32_exchange_name new_file_name;
    struct fat32_exchange_file file;
    struct fat32_exchange_file renamed;
    enum qemu_xfer_result ready = require_name(old_name, &old_file_name);
    enum fat32_exchange_result result;

    if (ready != QEMU_XFER_OK)
        return ready;
    if (!name_make(new_name, &new_file_name))
        return QEMU_XFER_ARGUMENT;
    result = exchange_service_open(exchange_runtime_service(), &old_file_name,
                                   &file);
    if (result != FAT32_EXCHANGE_OK)
        return core_result(result);
    return core_result(exchange_service_rename(exchange_runtime_service(), &file,
                                               &new_file_name, &renamed));
}

enum qemu_xfer_result qemu_xfer_delete(const char *name)
{
    struct fat32_exchange_name file_name;
    struct fat32_exchange_file file;
    enum qemu_xfer_result ready = require_name(name, &file_name);
    enum fat32_exchange_result result;

    if (ready != QEMU_XFER_OK)
        return ready;
    result = exchange_service_open(exchange_runtime_service(), &file_name, &file);
    if (result != FAT32_EXCHANGE_OK)
        return core_result(result);
    return core_result(exchange_service_delete(exchange_runtime_service(), &file));
}

enum qemu_xfer_result qemu_xfer_verify_remount(void)
{
#if PIOS_PLATFORM != PIOS_PLATFORM_QEMU_VIRT
    return QEMU_XFER_UNAVAILABLE;
#else
    struct qemu_xfer_status status;

    exchange_init();
    qemu_xfer_status(&status);
    if (!status.available)
        return QEMU_XFER_UNAVAILABLE;
    return core_result(exchange_service_flush(exchange_runtime_service()));
#endif
}
