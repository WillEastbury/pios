/*
 * qemu_xfer.c - QEMU-only acceptance adapter for the isolated PIOSXFER p3.
 *
 * The core never receives an unrestricted SD callback: each transfer is
 * range-checked against the attachment copied from validated p3 facts.
 */
#include "types.h"
#include "platform.h"
#include "qemu_xfer.h"

#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
#include "sd.h"
#include "fat32_exchange_core.h"
#include "qemu_xfer_partition.h"

struct qemu_xfer_control {
    struct qemu_xfer_status status;
} ALIGNED(64);

static struct fat32_exchange_volume g_volume ALIGNED(64);
static struct fat32_exchange_attachment g_attachment ALIGNED(64);
static struct qemu_xfer_partition g_partition ALIGNED(64);
static struct qemu_xfer_control g_control ALIGNED(64);

static void bytes_zero(void *dst, u32 bytes)
{
    u8 *p = dst;

    for (u32 i = 0U; i < bytes; i++)
        p[i] = 0U;
}

static bool authorized_lba(u32 lba)
{
    return g_attachment.block_count != 0U &&
           lba >= g_attachment.first_lba &&
           lba - g_attachment.first_lba < g_attachment.block_count;
}

static bool xfer_read_sector(void *context, u32 lba, u8 out[512])
{
    (void)context;
    return authorized_lba(lba) && sd_read_block(lba, out);
}

static bool xfer_write_sector(void *context, u32 lba, const u8 in[512])
{
    (void)context;
    return authorized_lba(lba) && sd_write_block(lba, in);
}

static enum qemu_xfer_result core_result(enum fat32_exchange_result result)
{
    g_control.status.core_result = (u32)result;
    g_control.status.last_result = result == FAT32_EXCHANGE_OK ?
                                   QEMU_XFER_OK : QEMU_XFER_CORE_FAILED;
    if (result != FAT32_EXCHANGE_OK && g_volume.faulted)
        g_control.status.available = 0U;
    return (enum qemu_xfer_result)g_control.status.last_result;
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

static enum qemu_xfer_result mount_exchange(void)
{
    u8 mbr[512] ALIGNED(64);
    const sd_card_t *card;
    struct fat32_exchange_io io;
    enum qemu_xfer_partition_result selected;
    enum fat32_exchange_result mounted;

    bytes_zero(&g_control, sizeof(g_control));
    bytes_zero(&g_attachment, sizeof(g_attachment));
    bytes_zero(&g_partition, sizeof(g_partition));
    fat32_exchange_volume_init(&g_volume);
    if (!sd_qemu_virtio_blk_ready()) {
        g_control.status.last_result = QEMU_XFER_UNAVAILABLE;
        return QEMU_XFER_UNAVAILABLE;
    }
    card = sd_get_card_info();
    if (!card || card->capacity / SD_BLOCK_SIZE > 0x100000000ULL ||
        !sd_read_block(0U, mbr)) {
        g_control.status.last_result = QEMU_XFER_PARTITION_REJECTED;
        return QEMU_XFER_PARTITION_REJECTED;
    }
    selected = qemu_xfer_partition_select(mbr, card->capacity / SD_BLOCK_SIZE,
                                          &g_partition);
    if (selected != QEMU_XFER_PARTITION_OK) {
        g_control.status.last_result = QEMU_XFER_PARTITION_REJECTED;
        g_control.status.core_result = (u32)selected;
        return QEMU_XFER_PARTITION_REJECTED;
    }

    g_attachment.identity = g_partition.identity;
    g_attachment.epoch = g_partition.disk_id;
    g_attachment.first_lba = g_partition.first_lba;
    g_attachment.block_count = g_partition.block_count;
    for (u32 i = 0U; i < FAT32_EXCHANGE_NAME_BYTES; i++)
        g_attachment.expected_label[i] = g_partition.expected_label[i];
    io.read = xfer_read_sector;
    io.write = xfer_write_sector;
    io.context = NULL;
    mounted = fat32_exchange_mount(&g_volume, &g_attachment, &io, 0U, false);
    g_control.status.available = mounted == FAT32_EXCHANGE_OK;
    g_control.status.mounted = g_volume.mounted;
    g_control.status.first_lba = g_partition.first_lba;
    g_control.status.block_count = g_partition.block_count;
    g_control.status.disk_id = g_partition.disk_id;
    return core_result(mounted);
}

void qemu_xfer_init(void)
{
    (void)mount_exchange();
}

void qemu_xfer_status(struct qemu_xfer_status *out)
{
    if (!out)
        return;
    *out = g_control.status;
}

const char *qemu_xfer_result_name(enum qemu_xfer_result result)
{
    switch (result) {
    case QEMU_XFER_OK: return "ok";
    case QEMU_XFER_UNAVAILABLE: return "unavailable";
    case QEMU_XFER_PARTITION_REJECTED: return "partition-rejected";
    case QEMU_XFER_CORE_FAILED: return "fat32-failed";
    case QEMU_XFER_ARGUMENT: return "argument";
    default: return "unknown";
    }
}

static enum qemu_xfer_result require_name(const char *text,
                                          struct fat32_exchange_name *name)
{
    if (!g_control.status.available)
        return (enum qemu_xfer_result)g_control.status.last_result;
    if (!name_make(text, name)) {
        g_control.status.last_result = QEMU_XFER_ARGUMENT;
        return QEMU_XFER_ARGUMENT;
    }
    return QEMU_XFER_OK;
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
        data_len > QEMU_XFER_COMMAND_BYTES) {
        if (ready == QEMU_XFER_OK)
            g_control.status.last_result = QEMU_XFER_ARGUMENT;
        return ready != QEMU_XFER_OK ? ready : QEMU_XFER_ARGUMENT;
    }
    result = fat32_exchange_open(&g_volume, &g_attachment, 0U, false,
                                 &file_name, &file);
    if (result == FAT32_EXCHANGE_NOT_FOUND)
        result = fat32_exchange_create(&g_volume, &g_attachment, 0U, false,
                                       &file_name, &file);
    if (result != FAT32_EXCHANGE_OK)
        return core_result(result);
    return core_result(fat32_exchange_rewrite(&g_volume, &g_attachment, 0U,
                                               false, &file, data, data_len,
                                               &updated));
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
        data_len > QEMU_XFER_COMMAND_BYTES) {
        if (ready == QEMU_XFER_OK)
            g_control.status.last_result = QEMU_XFER_ARGUMENT;
        return ready != QEMU_XFER_OK ? ready : QEMU_XFER_ARGUMENT;
    }
    result = fat32_exchange_open(&g_volume, &g_attachment, 0U, false,
                                 &file_name, &file);
    if (result == FAT32_EXCHANGE_NOT_FOUND)
        result = fat32_exchange_create(&g_volume, &g_attachment, 0U, false,
                                       &file_name, &file);
    if (result != FAT32_EXCHANGE_OK)
        return core_result(result);
    return core_result(fat32_exchange_append(&g_volume, &g_attachment, 0U,
                                              false, &file, data, data_len,
                                              &updated));
}

enum qemu_xfer_result qemu_xfer_read(const char *name, u8 *out, u32 capacity,
                                     u32 *out_read)
{
    struct fat32_exchange_name file_name;
    struct fat32_exchange_file file;
    enum qemu_xfer_result ready = require_name(name, &file_name);

    if (out_read)
        *out_read = 0U;
    if (ready != QEMU_XFER_OK || !out || !out_read ||
        capacity > QEMU_XFER_COMMAND_BYTES) {
        if (ready == QEMU_XFER_OK)
            g_control.status.last_result = QEMU_XFER_ARGUMENT;
        return ready != QEMU_XFER_OK ? ready : QEMU_XFER_ARGUMENT;
    }
    {
        enum fat32_exchange_result result =
            fat32_exchange_open(&g_volume, &g_attachment, 0U, false,
                                &file_name, &file);
        if (result != FAT32_EXCHANGE_OK)
            return core_result(result);
        return core_result(fat32_exchange_read(&g_volume, &g_attachment, 0U,
                                                false, &file, 0U, out, capacity,
                                                capacity, out_read));
    }
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
    if (!name_make(new_name, &new_file_name)) {
        g_control.status.last_result = QEMU_XFER_ARGUMENT;
        return QEMU_XFER_ARGUMENT;
    }
    result = fat32_exchange_open(&g_volume, &g_attachment, 0U, false,
                                 &old_file_name, &file);
    if (result != FAT32_EXCHANGE_OK)
        return core_result(result);
    return core_result(fat32_exchange_rename(&g_volume, &g_attachment, 0U,
                                              false, &file, &new_file_name,
                                              &renamed));
}

enum qemu_xfer_result qemu_xfer_delete(const char *name)
{
    struct fat32_exchange_name file_name;
    struct fat32_exchange_file file;
    enum qemu_xfer_result ready = require_name(name, &file_name);
    enum fat32_exchange_result result;

    if (ready != QEMU_XFER_OK)
        return ready;
    result = fat32_exchange_open(&g_volume, &g_attachment, 0U, false,
                                 &file_name, &file);
    if (result != FAT32_EXCHANGE_OK)
        return core_result(result);
    return core_result(fat32_exchange_delete(&g_volume, &g_attachment, 0U,
                                              false, &file));
}

enum qemu_xfer_result qemu_xfer_verify_remount(void)
{
    enum qemu_xfer_result mounted = mount_exchange();

    if (mounted != QEMU_XFER_OK)
        return mounted;
    return core_result(fat32_exchange_flush(&g_volume, &g_attachment, 0U,
                                            false));
}

#else

void qemu_xfer_init(void) {}

void qemu_xfer_status(struct qemu_xfer_status *out)
{
    if (out)
        *out = (struct qemu_xfer_status){.last_result = QEMU_XFER_UNAVAILABLE};
}

const char *qemu_xfer_result_name(enum qemu_xfer_result result)
{
    return result == QEMU_XFER_UNAVAILABLE ? "unavailable" : "qemu-only";
}

enum qemu_xfer_result qemu_xfer_write(const char *name, const u8 *data,
                                      u32 data_len)
{
    (void)name; (void)data; (void)data_len;
    return QEMU_XFER_UNAVAILABLE;
}
enum qemu_xfer_result qemu_xfer_append(const char *name, const u8 *data,
                                       u32 data_len)
{
    (void)name; (void)data; (void)data_len;
    return QEMU_XFER_UNAVAILABLE;
}
enum qemu_xfer_result qemu_xfer_read(const char *name, u8 *out, u32 capacity,
                                     u32 *out_read)
{
    (void)name; (void)out; (void)capacity;
    if (out_read) *out_read = 0U;
    return QEMU_XFER_UNAVAILABLE;
}
enum qemu_xfer_result qemu_xfer_rename(const char *old_name,
                                       const char *new_name)
{
    (void)old_name; (void)new_name;
    return QEMU_XFER_UNAVAILABLE;
}
enum qemu_xfer_result qemu_xfer_delete(const char *name)
{
    (void)name;
    return QEMU_XFER_UNAVAILABLE;
}
enum qemu_xfer_result qemu_xfer_verify_remount(void)
{
    return QEMU_XFER_UNAVAILABLE;
}
#endif
