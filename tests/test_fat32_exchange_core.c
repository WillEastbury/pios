#include <stdio.h>
#include <string.h>

#include "fat32_exchange_core.h"

#define RESERVED 32U
#define FAT_SECTORS 513U
#define CLUSTERS 65525U
#define IMAGE_SECTORS (RESERVED + FAT_SECTORS * 2U + CLUSTERS)
#define FAT1_LBA RESERVED
#define FAT2_LBA (RESERVED + FAT_SECTORS)
#define DATA_LBA (RESERVED + FAT_SECTORS * 2U)

struct fake_disk {
    u8 image[IMAGE_SECTORS][FAT32_EXCHANGE_SECTOR_BYTES];
    u32 lba_base;
    u32 reads;
    u32 writes;
    u32 fail_read_lba;
    u32 fail_write_lba;
    bool fail_read_once;
    bool fail_write_once;
};

static struct fake_disk disk;
static const u8 label[11] = {'P','I','O','S','X','F','E','R',' ',' ',' '};
static const u8 data_name[] = {'D','A','T','A','.','B','I','N'};
static const u8 new_name[] = {'N','E','W','.','B','I','N'};
static const u8 other_name[] = {'O','T','H','E','R','.','T','X','T'};
static const u8 full_name[] = {'F','U','L','L','.','B','I','N'};

#define CHECK(x) do { if (!(x)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); return 1; \
} } while (0)

static void le16(u8 *p, u16 value)
{
    p[0] = (u8)value;
    p[1] = (u8)(value >> 8);
}

static void le32(u8 *p, u32 value)
{
    p[0] = (u8)value;
    p[1] = (u8)(value >> 8);
    p[2] = (u8)(value >> 16);
    p[3] = (u8)(value >> 24);
}

static u32 get32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) |
           ((u32)p[3] << 24);
}

static bool fake_read(void *opaque, u32 lba, u8 out[512])
{
    struct fake_disk *d = opaque;
    u32 index;

    d->reads++;
    if (lba < d->lba_base || (index = lba - d->lba_base) >= IMAGE_SECTORS ||
        (d->fail_read_once && lba == d->fail_read_lba)) {
        d->fail_read_once = false;
        return false;
    }
    memcpy(out, d->image[index], 512U);
    return true;
}

static bool fake_write(void *opaque, u32 lba, const u8 in[512])
{
    struct fake_disk *d = opaque;
    u32 index;

    d->writes++;
    if (lba < d->lba_base || (index = lba - d->lba_base) >= IMAGE_SECTORS ||
        (d->fail_write_once && lba == d->fail_write_lba)) {
        d->fail_write_once = false;
        return false;
    }
    memcpy(d->image[index], in, 512U);
    return true;
}

static void fat_set_raw(u32 cluster, u32 value)
{
    u32 byte = cluster * 4U;
    u32 sector = byte / 512U;
    u32 offset = byte % 512U;

    le32(disk.image[FAT1_LBA + sector] + offset, value);
    le32(disk.image[FAT2_LBA + sector] + offset, value);
}

static u32 fat_get_raw(u32 cluster)
{
    u32 byte = cluster * 4U;

    return get32(disk.image[FAT1_LBA + byte / 512U] + byte % 512U) &
           0x0FFFFFFFU;
}

static struct fat32_exchange_attachment attachment(void)
{
    struct fat32_exchange_attachment value = {0};

    value.identity = 0x584645522D313933ULL;
    value.epoch = 7U;
    value.first_lba = disk.lba_base;
    value.block_count = IMAGE_SECTORS;
    memcpy(value.expected_label, label, sizeof(label));
    return value;
}

static struct fat32_exchange_io io(void)
{
    struct fat32_exchange_io value = {
        .read = fake_read,
        .write = fake_write,
        .context = &disk,
    };
    return value;
}

static void setup_image(void)
{
    u8 *boot;

    memset(&disk, 0, sizeof(disk));
    boot = disk.image[0U];
    boot[0] = 0xEBU;
    boot[2] = 0x90U;
    le16(boot + 11U, 512U);
    boot[13U] = 1U;
    le16(boot + 14U, RESERVED);
    boot[16U] = 2U;
    le16(boot + 17U, 0U);
    le16(boot + 19U, 0U);
    le16(boot + 22U, 0U);
    boot[21U] = 0xF8U;
    le32(boot + 32U, IMAGE_SECTORS);
    le32(boot + 36U, FAT_SECTORS);
    le32(boot + 44U, 2U);
    le16(boot + 48U, 1U);
    boot[66U] = 0x29U;
    memcpy(boot + 71U, label, sizeof(label));
    memcpy(boot + 82U, "FAT32   ", 8U);
    boot[510U] = 0x55U;
    boot[511U] = 0xAAU;
    le32(disk.image[1U], 0x41615252U);
    le32(disk.image[1U] + 484U, 0x61417272U);
    le32(disk.image[1U] + 488U, 0xFFFFFFFFU);
    le32(disk.image[1U] + 492U, 0xFFFFFFFFU);
    le32(disk.image[1U] + 508U, 0xAA550000U);
    fat_set_raw(0U, 0x0FFFFFF8U);
    fat_set_raw(1U, 0x0FFFFFFFU);
    fat_set_raw(2U, 0x0FFFFFFFU); /* root directory */
    fat_set_raw(3U, 0x0FFFFFFFU); /* force fragmented first allocation at 4 */
}

static enum fat32_exchange_result mount(struct fat32_exchange_volume *v,
                                        struct fat32_exchange_attachment *a)
{
    struct fat32_exchange_io backend = io();

    return fat32_exchange_mount(v, a, &backend, 0U, false);
}

static struct fat32_exchange_name name_of(const u8 *bytes, u32 len)
{
    struct fat32_exchange_name name = {.bytes = bytes, .len = len};
    return name;
}

static int test_mutations(void)
{
    struct fat32_exchange_volume v;
    struct fat32_exchange_attachment a;
    struct fat32_exchange_file file;
    struct fat32_exchange_file fresh;
    struct fat32_exchange_file renamed;
    struct fat32_exchange_file other;
    struct fat32_exchange_name data = name_of(data_name, sizeof(data_name));
    struct fat32_exchange_name newer = name_of(new_name, sizeof(new_name));
    struct fat32_exchange_name other_n = name_of(other_name, sizeof(other_name));
    u8 first[700U];
    u8 tail[400U];
    u8 readback[1100U];
    u32 got;
    u32 i;

    fat32_exchange_volume_init(&v);
    setup_image();
    a = attachment();
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_create(&v, &a, 0U, false, &data, &file) ==
          FAT32_EXCHANGE_OK);
    for (i = 0U; i < sizeof(first); i++)
        first[i] = (u8)(i * 3U);
    for (i = 0U; i < sizeof(tail); i++)
        tail[i] = (u8)(0xA0U + i);
    CHECK(fat32_exchange_rewrite(&v, &a, 0U, false, &file, first,
                                 sizeof(first), &fresh) == FAT32_EXCHANGE_OK);
    CHECK(fresh.generation != file.generation);
    CHECK(fat32_exchange_read(&v, &a, 0U, false, &file, 0U, readback,
                              sizeof(readback), sizeof(first), &got) ==
          FAT32_EXCHANGE_STALE);
    CHECK(fat32_exchange_read(&v, &a, 0U, false, &fresh, 0U, readback,
                              sizeof(readback), sizeof(first), &got) ==
          FAT32_EXCHANGE_OK);
    CHECK(got == sizeof(first) && memcmp(readback, first, sizeof(first)) == 0);
    CHECK(fat32_exchange_append(&v, &a, 0U, false, &fresh, tail, sizeof(tail),
                                &file) == FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_read(&v, &a, 0U, false, &file, 0U, readback,
                              sizeof(readback), sizeof(readback), &got) ==
          FAT32_EXCHANGE_OK);
    CHECK(got == sizeof(readback));
    CHECK(memcmp(readback, first, sizeof(first)) == 0);
    CHECK(memcmp(readback + sizeof(first), tail, sizeof(tail)) == 0);
    CHECK(fat32_exchange_rewrite(&v, &a, 0U, false, &file, first, 100U,
                                 &fresh) == FAT32_EXCHANGE_OK);
    CHECK(fat_get_raw(4U) == 0U && fat_get_raw(5U) == 0U);
    CHECK(fat32_exchange_rename(&v, &a, 0U, false, &fresh, &newer, &renamed) ==
          FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_create(&v, &a, 0U, false, &other_n, &other) ==
          FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_open(&v, &a, 0U, false, &newer, &fresh) ==
          FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_rename(&v, &a, 0U, false, &fresh, &other_n, &file) ==
          FAT32_EXCHANGE_EXISTS);
    CHECK(fat32_exchange_delete(&v, &a, 0U, false, &fresh) ==
          FAT32_EXCHANGE_OK);
    CHECK(fat_get_raw(6U) == 0U);
    CHECK(fat32_exchange_open(&v, &a, 0U, false, &newer, &file) ==
          FAT32_EXCHANGE_NOT_FOUND);
    CHECK(fat32_exchange_flush(&v, &a, 0U, false) == FAT32_EXCHANGE_OK);
    return 0;
}

static int test_rejections(void)
{
    struct fat32_exchange_volume v;
    struct fat32_exchange_attachment a;
    struct fat32_exchange_file file;
    struct fat32_exchange_file fresh;
    struct fat32_exchange_name data = name_of(data_name, sizeof(data_name));
    u8 payload[700U] = {0};
    u8 output[1U];
    u32 got;

    fat32_exchange_volume_init(&v);
    setup_image();
    a = attachment();
    a.expected_label[0] = 'X';
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_LABEL);
    setup_image();
    a = attachment();
    le32(disk.image[0U] + 32U, IMAGE_SECTORS + 1U);
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_BPB);
    setup_image();
    a = attachment();
    disk.image[FAT2_LBA][8U] ^= 1U;
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_FAT_MISMATCH);
    setup_image();
    a = attachment();
    disk.image[1U][0] = 0U;
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_BPB);
    setup_image();
    a = attachment();
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_create(&v, &a, 1U, false, &data, &file) ==
          FAT32_EXCHANGE_OWNER);
    CHECK(fat32_exchange_create(&v, &a, 0U, true, &data, &file) ==
          FAT32_EXCHANGE_OWNER);
    CHECK(fat32_exchange_create(&v, &a, 0U, false, &data, &file) ==
          FAT32_EXCHANGE_OK);
    a.epoch++;
    CHECK(fat32_exchange_read(&v, &a, 0U, false, &file, 0U, output,
                              sizeof(output), 0U, &got) == FAT32_EXCHANGE_STALE);
    a.epoch--;
    CHECK(fat32_exchange_rewrite(&v, &a, 0U, false, &file, payload,
                                 sizeof(payload), &fresh) == FAT32_EXCHANGE_OK);
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_read(&v, &a, 0U, false, &fresh, 0U, payload,
                              sizeof(payload), 0U, &got) == FAT32_EXCHANGE_STALE);
    CHECK(fat32_exchange_open(&v, &a, 0U, false, &data, &fresh) ==
          FAT32_EXCHANGE_OK);
    /* A cyclic chain cannot be accepted merely because requested bytes fit. */
    fat_set_raw(4U, 4U);
    CHECK(fat32_exchange_read(&v, &a, 0U, false, &fresh, 0U, payload,
                              sizeof(payload), sizeof(payload), &got) ==
          FAT32_EXCHANGE_CORRUPT);
    return 0;
}

static int test_failure_and_full(void)
{
    struct fat32_exchange_volume v;
    struct fat32_exchange_attachment a;
    struct fat32_exchange_file file;
    struct fat32_exchange_file fresh;
    struct fat32_exchange_name data = name_of(data_name, sizeof(data_name));
    struct fat32_exchange_name full = name_of(full_name, sizeof(full_name));
    u8 byte = 1U;
    u32 cluster;

    fat32_exchange_volume_init(&v);
    setup_image();
    a = attachment();
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_create(&v, &a, 0U, false, &data, &file) ==
          FAT32_EXCHANGE_OK);
    disk.fail_write_lba = FAT2_LBA; /* first allocation's mirrored FAT write */
    disk.fail_write_once = true;
    CHECK(fat32_exchange_rewrite(&v, &a, 0U, false, &file, &byte, 1U, &fresh) ==
          FAT32_EXCHANGE_IO);
    CHECK(fat32_exchange_flush(&v, &a, 0U, false) == FAT32_EXCHANGE_FAULTED);
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_FAT_MISMATCH);

    setup_image();
    a = attachment();
    disk.fail_read_lba = 0U;
    disk.fail_read_once = true;
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_IO);

    setup_image();
    for (cluster = 3U; cluster <= CLUSTERS + 1U; cluster++)
        fat_set_raw(cluster, 0x0FFFFFFFU);
    a = attachment();
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_create(&v, &a, 0U, false, &full, &file) ==
          FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_rewrite(&v, &a, 0U, false, &file, &byte, 1U, &fresh) ==
          FAT32_EXCHANGE_NO_SPACE);
    return 0;
}

static int test_root_extension(void)
{
    struct fat32_exchange_volume v;
    struct fat32_exchange_attachment a;
    struct fat32_exchange_file file;
    struct fat32_exchange_name name;
    u8 bytes[] = {'F','0','0','.','T','X','T'};
    u32 i;

    fat32_exchange_volume_init(&v);
    setup_image();
    a = attachment();
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    for (i = 0U; i < 17U; i++) {
        bytes[1U] = (u8)('0' + i / 10U);
        bytes[2U] = (u8)('0' + i % 10U);
        name = name_of(bytes, sizeof(bytes));
        CHECK(fat32_exchange_create(&v, &a, 0U, false, &name, &file) ==
              FAT32_EXCHANGE_OK);
    }
    CHECK(fat_get_raw(2U) == 4U);
    name = name_of(bytes, sizeof(bytes));
    CHECK(fat32_exchange_open(&v, &a, 0U, false, &name, &file) ==
          FAT32_EXCHANGE_OK);
    return 0;
}

static int test_high_lba_and_cluster_bounds(void)
{
    struct fat32_exchange_volume v;
    struct fat32_exchange_attachment a;
    struct fat32_exchange_file file;
    struct fat32_exchange_name data = name_of(data_name, sizeof(data_name));
    u8 byte = 0xA5U;

    fat32_exchange_volume_init(&v);
    setup_image();
    disk.lba_base = ~0U - (IMAGE_SECTORS - 1U);
    a = attachment();
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_create(&v, &a, 0U, false, &data, &file) ==
          FAT32_EXCHANGE_OK);
    v.alloc_hint = v.cluster_count + 1U;
    CHECK(fat32_exchange_rewrite(&v, &a, 0U, false, &file, &byte, 1U,
                                 &file) == FAT32_EXCHANGE_OK);
    CHECK(fat_get_raw(CLUSTERS + 1U) == 0x0FFFFFF8U);
    CHECK(disk.image[IMAGE_SECTORS - 1U][0] == byte);

    fat32_exchange_volume_init(&v);
    setup_image();
    a = attachment();
    a.first_lba = ~0U - 10U;
    a.block_count = 20U;
    disk.reads = 0U;
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_INVALID);
    CHECK(disk.reads == 0U);
    return 0;
}

static int test_lfn_mutation_rejected(void)
{
    struct fat32_exchange_volume v;
    struct fat32_exchange_attachment a;
    struct fat32_exchange_file file;
    struct fat32_exchange_file out;
    struct fat32_exchange_name data = name_of(data_name, sizeof(data_name));
    struct fat32_exchange_name newer = name_of(new_name, sizeof(new_name));
    u8 before_lfn[32U];
    u8 before_short[32U];
    u8 *root;
    u8 *short_entry;
    u32 i;

    fat32_exchange_volume_init(&v);
    setup_image();
    root = disk.image[DATA_LBA];
    for (i = 0U; i < 15U; i++)
        root[i * 32U] = 0xE5U;
    root += 15U * 32U;
    root[0] = 0x41U;
    root[11U] = 0x0FU;
    root[13U] = 0xAAU;
    fat_set_raw(2U, 4U);
    fat_set_raw(4U, 0x0FFFFFFFU);
    short_entry = disk.image[DATA_LBA + 2U];
    memcpy(short_entry, "DATA    BIN", 11U);
    short_entry[11U] = 0x20U;
    memcpy(before_lfn, root, sizeof(before_lfn));
    memcpy(before_short, short_entry, sizeof(before_short));
    a = attachment();
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_open(&v, &a, 0U, false, &data, &file) ==
          FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_rename(&v, &a, 0U, false, &file, &newer, &out) ==
          FAT32_EXCHANGE_LFN);
    CHECK(fat32_exchange_delete(&v, &a, 0U, false, &file) ==
          FAT32_EXCHANGE_LFN);
    CHECK(memcmp(root, before_lfn, sizeof(before_lfn)) == 0);
    CHECK(memcmp(short_entry, before_short, sizeof(before_short)) == 0);
    CHECK(fat_get_raw(2U) == 4U && fat_get_raw(4U) == 0x0FFFFFFFU);
    return 0;
}

static void setup_lfn_preceded_root_slot(void)
{
    u8 *root = disk.image[DATA_LBA];
    u32 i;

    for (i = 0U; i < 15U; i++) {
        root[i * 32U] = (u8)('A' + i);
        root[i * 32U + 11U] = 0x08U;
    }
    root += 15U * 32U;
    root[0] = 0x41U;
    root[11U] = 0x0FU;
    fat_set_raw(2U, 4U);
    fat_set_raw(4U, 0x0FFFFFFFU);
}

static int test_lfn_deleted_create_rejected(void)
{
    struct fat32_exchange_volume v;
    struct fat32_exchange_attachment a;
    struct fat32_exchange_file file;
    struct fat32_exchange_name data = name_of(data_name, sizeof(data_name));
    u8 *slot;

    fat32_exchange_volume_init(&v);
    setup_image();
    setup_lfn_preceded_root_slot();
    le32(disk.image[1U] + 488U, 100U);
    le32(disk.image[1U] + 492U, 4U);
    slot = disk.image[DATA_LBA + 2U];
    slot[0] = 0xE5U;
    a = attachment();
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    disk.writes = 0U;
    CHECK(fat32_exchange_create(&v, &a, 0U, false, &data, &file) ==
          FAT32_EXCHANGE_LFN);
    CHECK(slot[0] == 0xE5U);
    CHECK(disk.writes == 0U);
    return 0;
}

static int test_lfn_end_marker_create_rejected(void)
{
    struct fat32_exchange_volume v;
    struct fat32_exchange_attachment a;
    struct fat32_exchange_file file;
    struct fat32_exchange_name data = name_of(data_name, sizeof(data_name));
    u8 *slot;

    fat32_exchange_volume_init(&v);
    setup_image();
    setup_lfn_preceded_root_slot();
    le32(disk.image[1U] + 488U, 100U);
    le32(disk.image[1U] + 492U, 4U);
    slot = disk.image[DATA_LBA + 2U];
    a = attachment();
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    disk.writes = 0U;
    CHECK(fat32_exchange_create(&v, &a, 0U, false, &data, &file) ==
          FAT32_EXCHANGE_LFN);
    CHECK(slot[0] == 0U);
    CHECK(disk.writes == 0U);
    return 0;
}

static int test_fsinfo_and_file_aliases(void)
{
    struct fat32_exchange_volume v;
    struct fat32_exchange_attachment a;
    struct fat32_exchange_file file;
    struct fat32_exchange_name data = name_of(data_name, sizeof(data_name));
    u8 first[1U] = {0x3CU};
    u8 tail[1U] = {0xC3U};
    u8 readback[2U];
    u32 got;

    fat32_exchange_volume_init(&v);
    setup_image();
    le32(disk.image[1U] + 488U, 100U);
    le32(disk.image[1U] + 492U, 4U);
    a = attachment();
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_create(&v, &a, 0U, false, &data, &file) ==
          FAT32_EXCHANGE_OK);
    CHECK(get32(disk.image[1U] + 488U) == ~0U);
    CHECK(get32(disk.image[1U] + 492U) == ~0U);
    CHECK(fat32_exchange_rewrite(&v, &a, 0U, false, &file, first,
                                 sizeof(first), &file) == FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_append(&v, &a, 0U, false, &file, tail, sizeof(tail),
                                &file) == FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_read(&v, &a, 0U, false, &file, 0U, readback,
                              sizeof(readback), sizeof(readback), &got) ==
          FAT32_EXCHANGE_OK);
    CHECK(got == sizeof(readback) && readback[0] == first[0] &&
          readback[1] == tail[0]);
    CHECK(fat32_exchange_delete(&v, &a, 0U, false, &file) ==
          FAT32_EXCHANGE_OK);
    CHECK(get32(disk.image[1U] + 488U) == ~0U);
    CHECK(get32(disk.image[1U] + 492U) == ~0U);

    fat32_exchange_volume_init(&v);
    setup_image();
    le32(disk.image[1U] + 488U, 100U);
    le32(disk.image[1U] + 492U, 4U);
    a = attachment();
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    disk.fail_write_lba = 1U;
    disk.fail_write_once = true;
    CHECK(fat32_exchange_create(&v, &a, 0U, false, &data, &file) ==
          FAT32_EXCHANGE_IO);
    CHECK(fat32_exchange_flush(&v, &a, 0U, false) == FAT32_EXCHANGE_FAULTED);
    CHECK(disk.image[DATA_LBA][0] == 0U);
    return 0;
}

static int test_count_free_sector_bounded(void)
{
    struct fat32_exchange_volume v;
    struct fat32_exchange_attachment a;
    struct fat32_exchange_file file;
    struct fat32_exchange_file fresh;
    struct fat32_exchange_name data = name_of(data_name, sizeof(data_name));
    u8 byte = 0x5AU;

    fat32_exchange_volume_init(&v);
    setup_image();
    a = attachment();
    CHECK(mount(&v, &a) == FAT32_EXCHANGE_OK);
    CHECK(fat32_exchange_create(&v, &a, 0U, false, &data, &file) ==
          FAT32_EXCHANGE_OK);
    disk.reads = 0U;
    CHECK(fat32_exchange_rewrite(&v, &a, 0U, false, &file, &byte, 1U,
                                 &fresh) == FAT32_EXCHANGE_OK);
    CHECK(disk.reads <= FAT_SECTORS * 2U + 40U);
    return 0;
}

int main(void)
{
    if (test_mutations() || test_rejections() || test_failure_and_full() ||
        test_root_extension() || test_high_lba_and_cluster_bounds() ||
        test_lfn_mutation_rejected() || test_lfn_deleted_create_rejected() ||
        test_lfn_end_marker_create_rejected() || test_fsinfo_and_file_aliases() ||
        test_count_free_sector_bounded())
        return 1;
    printf("fat32 exchange core: bounded root 8.3 mutations and fail-closed metadata passed\n");
    return 0;
}
