/* Boot attachment tests: no exchange callback can reach p1/p2 or format. */
#include <stdio.h>
#include <string.h>

#include "types.h"
#include "exchange_service.h"

#define P1_FIRST 2048U
#define P1_BLOCKS 1000U
#define P2_FIRST (P1_FIRST + P1_BLOCKS)
#define P2_BLOCKS 2000U
#define P3_FIRST (P2_FIRST + P2_BLOCKS)
#define P3_BLOCKS 67000U
#define TOTAL_BLOCKS (P3_FIRST + P3_BLOCKS)
#define RESERVED 32U
#define FAT_SECTORS 516U

struct fake_disk {
    u8 sectors[1100U][512];
    u32 reads;
    u32 writes;
    bool touched_non_p3;
    bool write_enabled;
};

static struct fake_disk disk;

#define CHECK(x) do { if (!(x)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); return 1; \
} } while (0)

static void put16(u8 *p, u16 value)
{
    p[0] = (u8)value;
    p[1] = (u8)(value >> 8);
}

static void put32(u8 *p, u32 value)
{
    p[0] = (u8)value;
    p[1] = (u8)(value >> 8);
    p[2] = (u8)(value >> 16);
    p[3] = (u8)(value >> 24);
}

static void entry(u8 mbr[512], u32 index, u8 type, u32 first, u32 count)
{
    u8 *p = mbr + 446U + index * 16U;

    p[4] = type;
    put32(p + 8U, first);
    put32(p + 12U, count);
}

static void valid_mbr(u8 mbr[512])
{
    memset(mbr, 0, 512U);
    put32(mbr + 440U, 0x58464552U);
    mbr[510U] = 0x55U;
    mbr[511U] = 0xAAU;
    entry(mbr, 0U, 0x0CU, P1_FIRST, P1_BLOCKS);
    entry(mbr, 1U, 0xDAU, P2_FIRST, P2_BLOCKS);
    entry(mbr, 2U, 0x0CU, P3_FIRST, P3_BLOCKS);
}

static void setup_fat32(const u8 label[11])
{
    memset(&disk, 0, sizeof(disk));
    disk.sectors[0U][0] = 0xEBU;
    disk.sectors[0U][2] = 0x90U;
    put16(disk.sectors[0U] + 11U, 512U);
    disk.sectors[0U][13U] = 1U;
    put16(disk.sectors[0U] + 14U, RESERVED);
    disk.sectors[0U][16U] = 2U;
    disk.sectors[0U][21U] = 0xF8U;
    put32(disk.sectors[0U] + 32U, P3_BLOCKS);
    put32(disk.sectors[0U] + 36U, FAT_SECTORS);
    put32(disk.sectors[0U] + 44U, 2U);
    put16(disk.sectors[0U] + 48U, 1U);
    disk.sectors[0U][66U] = 0x29U;
    memcpy(disk.sectors[0U] + 71U, label, 11U);
    memcpy(disk.sectors[0U] + 82U, "FAT32   ", 8U);
    disk.sectors[0U][510U] = 0x55U;
    disk.sectors[0U][511U] = 0xAAU;
    put32(disk.sectors[1U], 0x41615252U);
    put32(disk.sectors[1U] + 484U, 0x61417272U);
    put32(disk.sectors[1U] + 488U, 0xFFFFFFFFU);
    put32(disk.sectors[1U] + 492U, 0xFFFFFFFFU);
    put32(disk.sectors[1U] + 508U, 0xAA550000U);
}

static bool fake_read(void *context, u32 lba, u8 out[512])
{
    struct fake_disk *d = context;

    d->reads++;
    if (lba < P3_FIRST || lba - P3_FIRST >= P3_BLOCKS) {
        d->touched_non_p3 = true;
        return false;
    }
    memset(out, 0, 512U);
    if (lba - P3_FIRST < 1100U)
        memcpy(out, d->sectors[lba - P3_FIRST], 512U);
    return true;
}

static bool fake_write(void *context, u32 lba, const u8 in[512])
{
    struct fake_disk *d = context;

    d->writes++;
    if (lba < P3_FIRST || lba - P3_FIRST >= P3_BLOCKS ||
        lba - P3_FIRST >= 1100U) {
        d->touched_non_p3 = true;
        return false;
    }
    if (!d->write_enabled)
        return false;
    memcpy(d->sectors[lba - P3_FIRST], in, 512U);
    return true;
}

static struct exchange_service_backend backend(void)
{
    return (struct exchange_service_backend) {
        .read = fake_read,
        .write = fake_write,
        .context = &disk,
        .writable = false,
        .format_writable = false,
    };
}

static int test_legacy_is_nonfatal(void)
{
    struct exchange_service service;
    struct exchange_service_status status;
    struct exchange_service_backend io = backend();
    u8 mbr[512];

    valid_mbr(mbr);
    memset(mbr + 446U + 2U * 16U, 0, 16U);
    setup_fat32((const u8 *)"PIOSXFER   ");
    CHECK(exchange_service_init(&service, mbr, TOTAL_BLOCKS, &io) ==
          EXCHANGE_SERVICE_UNAVAILABLE);
    exchange_service_status(&service, &status);
    CHECK(!status.available && !status.mounted &&
          status.last_result == EXCHANGE_SERVICE_UNAVAILABLE);
    CHECK(disk.reads == 0U && disk.writes == 0U && !disk.touched_non_p3);
    return 0;
}

static int test_unlabeled_p3_mounts(void)
{
    struct exchange_service service;
    struct exchange_service_status status;
    struct exchange_service_backend io = backend();
    u8 mbr[512];

    valid_mbr(mbr);
    setup_fat32((const u8 *)"NOTXFER    ");
    CHECK(exchange_service_init(&service, mbr, TOTAL_BLOCKS, &io) ==
          EXCHANGE_SERVICE_OK);
    exchange_service_status(&service, &status);
    CHECK(status.available && status.mounted &&
          status.format_reason == EXCHANGE_SERVICE_FORMAT_EXISTING);
    CHECK(disk.reads > FAT_SECTORS * 2U && disk.writes == 0U &&
          !disk.touched_non_p3);

    valid_mbr(mbr);
    entry(mbr, 2U, 0x83U, P3_FIRST, P3_BLOCKS);
    setup_fat32((const u8 *)"PIOSXFER   ");
    CHECK(exchange_service_init(&service, mbr, TOTAL_BLOCKS, &io) ==
          EXCHANGE_SERVICE_PARTITION_REJECTED);
    CHECK(disk.reads == 0U && disk.writes == 0U && !disk.touched_non_p3);
    return 0;
}

static int test_raw_p3_formats_only_p3(void)
{
    struct exchange_service service;
    struct exchange_service_status status;
    struct exchange_service_backend io = backend();
    u8 mbr[512];

    valid_mbr(mbr);
    entry(mbr, 2U, 0xDAU, P3_FIRST, P3_BLOCKS);
    memset(&disk, 0, sizeof(disk));
    disk.write_enabled = true;
    io.format_writable = true;
    CHECK(exchange_service_init(&service, mbr, TOTAL_BLOCKS, &io) ==
          EXCHANGE_SERVICE_OK);
    exchange_service_status(&service, &status);
    CHECK(status.available && status.mounted && status.read_only &&
          status.formatted_this_boot &&
          status.format_reason == EXCHANGE_SERVICE_FORMAT_RAW_FORMATTED);
    CHECK(disk.writes > FAT_SECTORS * 2U && !disk.touched_non_p3);
    CHECK(disk.sectors[0U][510U] == 0x55U &&
          memcmp(disk.sectors[0U] + 71U, "PIOSXFER   ", 11U) == 0);
    return 0;
}

static int test_corrupt_raw_p3_never_overwrites(void)
{
    struct exchange_service service;
    struct exchange_service_status status;
    struct exchange_service_backend io = backend();
    u8 mbr[512];

    valid_mbr(mbr);
    entry(mbr, 2U, 0xDAU, P3_FIRST, P3_BLOCKS);
    memset(&disk, 0, sizeof(disk));
    disk.write_enabled = true;
    io.format_writable = true;
    disk.sectors[0U][510U] = 0x55U;
    disk.sectors[0U][511U] = 0xAAU;
    memcpy(disk.sectors[0U] + 82U, "FAT32   ", 8U);
    CHECK(exchange_service_init(&service, mbr, TOTAL_BLOCKS, &io) ==
          EXCHANGE_SERVICE_CORE_FAILED);
    exchange_service_status(&service, &status);
    CHECK(!status.available && !status.mounted &&
          status.format_reason == EXCHANGE_SERVICE_FORMAT_CORRUPT &&
          disk.writes == 0U && !disk.touched_non_p3);
    return 0;
}

static int test_raw_p3_write_failure_is_bounded(void)
{
    struct exchange_service service;
    struct exchange_service_status status;
    struct exchange_service_backend io = backend();
    u8 mbr[512];

    valid_mbr(mbr);
    entry(mbr, 2U, 0xDAU, P3_FIRST, P3_BLOCKS);
    memset(&disk, 0, sizeof(disk));
    io.format_writable = true;
    CHECK(exchange_service_init(&service, mbr, TOTAL_BLOCKS, &io) ==
          EXCHANGE_SERVICE_CORE_FAILED);
    exchange_service_status(&service, &status);
    CHECK(!status.available && !status.mounted &&
          status.format_reason == EXCHANGE_SERVICE_FORMAT_IO &&
          disk.writes == 1U && !disk.touched_non_p3);
    return 0;
}

static int test_failed_reinit_clears_prior_mount(void)
{
    struct exchange_service service;
    struct exchange_service_status status;
    struct exchange_service_backend io = backend();
    u8 mbr[512];

    valid_mbr(mbr);
    setup_fat32((const u8 *)"PIOSXFER   ");
    CHECK(exchange_service_init(&service, mbr, TOTAL_BLOCKS, &io) ==
          EXCHANGE_SERVICE_OK);
    CHECK(service.volume.mounted);
    disk.reads = 0U;
    disk.writes = 0U;
    disk.touched_non_p3 = false;

    entry(mbr, 2U, 0x83U, P3_FIRST, P3_BLOCKS);
    CHECK(exchange_service_init(&service, mbr, TOTAL_BLOCKS, &io) ==
          EXCHANGE_SERVICE_PARTITION_REJECTED);
    exchange_service_status(&service, &status);
    CHECK(!status.available && !status.mounted &&
          status.last_result == EXCHANGE_SERVICE_PARTITION_REJECTED &&
          status.layout_result == STORAGE_LAYOUT_ROLE_MISMATCH &&
          status.first_lba == 0U && status.block_count == 0U &&
          status.disk_id == 0U && !service.volume.mounted);
    CHECK(disk.reads == 0U && disk.writes == 0U && !disk.touched_non_p3);
    return 0;
}

static int test_valid_p3_mounts_read_only_once(void)
{
    struct exchange_service service;
    struct exchange_service_status status;
    struct exchange_service_backend io = backend();
    u8 mbr[512];

    valid_mbr(mbr);
    setup_fat32((const u8 *)"PIOSXFER   ");
    CHECK(exchange_service_init(&service, mbr, TOTAL_BLOCKS, &io) ==
          EXCHANGE_SERVICE_OK);
    exchange_service_status(&service, &status);
    CHECK(status.available && status.mounted && status.read_only &&
          status.first_lba == P3_FIRST && status.block_count == P3_BLOCKS);
    CHECK(service.volume.mount_generation == 1U);
    CHECK(disk.reads > FAT_SECTORS * 2U && disk.writes == 0U &&
          !disk.touched_non_p3);
    return 0;
}

int main(void)
{
    if (test_legacy_is_nonfatal() || test_unlabeled_p3_mounts() ||
        test_raw_p3_formats_only_p3() || test_corrupt_raw_p3_never_overwrites() ||
        test_raw_p3_write_failure_is_bounded() ||
        test_failed_reinit_clears_prior_mount() ||
        test_valid_p3_mounts_read_only_once())
        return 1;
    puts("exchange service: p3-only attachment and raw autoformat PASS");
    return 0;
}
