/*
 * Host tests for read-only, bounded MBR/GPT enumeration.
 * The mock is a byte-reader only: no filesystem, SD, or writable target exists.
 */
#include "types.h"
#include <stdio.h>
#include <string.h>

#include "partition_table.h"

#define MOCK_BLOCKS 1100U

static u8 disk[MOCK_BLOCKS][PARTITION_TABLE_BLOCK_BYTES];
static u64 short_lba;
static int failures;

#define CHECK(expr) do { \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static void store_le32(u8 dst[], u32 value)
{
    dst[0] = (u8)value;
    dst[1] = (u8)(value >> 8);
    dst[2] = (u8)(value >> 16);
    dst[3] = (u8)(value >> 24);
}

static void store_le64(u8 dst[], u64 value)
{
    store_le32(dst, (u32)value);
    store_le32(dst + 4U, (u32)(value >> 32));
}

static bool mock_read(void *context, u64 lba,
                      u8 dst[PARTITION_TABLE_BLOCK_BYTES])
{
    (void)context;
    if (lba >= MOCK_BLOCKS || lba == short_lba)
        return false;
    memcpy(dst, disk[lba], PARTITION_TABLE_BLOCK_BYTES);
    return true;
}

static struct partition_table_input test_input(void)
{
    struct partition_table_input input = {0};

    input.read = mock_read;
    input.total_blocks = MOCK_BLOCKS;
    input.max_gpt_entries = PARTITION_TABLE_GPT_ENTRY_LIMIT;
    return input;
}

static void reset_disk(void)
{
    memset(disk, 0, sizeof(disk));
    short_lba = ~0ULL;
}

static void mbr_signature(void)
{
    disk[0][510U] = 0x55U;
    disk[0][511U] = 0xAAU;
}

static void mbr_entry(u32 index, u8 status, u8 type, u32 first, u32 count)
{
    u8 *entry = disk[0] + 446U + index * 16U;

    entry[0U] = status;
    entry[4U] = type;
    store_le32(entry + 8U, first);
    store_le32(entry + 12U, count);
}

static bool records_cleared(const struct partition_table_snapshot *snapshot)
{
    u32 i;
    u32 j;
    const u8 *bytes;

    for (i = 0U; i < PARTITION_TABLE_RECORD_CAPACITY; i++) {
        bytes = (const u8 *)&snapshot->records[i];
        for (j = 0U; j < sizeof(snapshot->records[i]); j++) {
            if (bytes[j] != 0U)
                return false;
        }
    }
    return snapshot->partition_count == 0U &&
           snapshot->discovered_count == 0U;
}

static void gpt_finalize_header(void)
{
    u8 *header = disk[1U];

    store_le32(header + 16U, 0U);
    store_le32(header + 16U, partition_table_crc32(header, 92U));
}

static void gpt_reseal_table(void)
{
    store_le32(disk[1U] + 88U,
               partition_table_crc32(disk[2U], PARTITION_TABLE_BLOCK_BYTES));
    gpt_finalize_header();
}

static void make_valid_gpt(void)
{
    u8 *header;
    u8 *entry;

    reset_disk();
    mbr_signature();
    mbr_entry(0U, 0U, 0xEEU, 1U, MOCK_BLOCKS - 1U);
    header = disk[1U];
    header[0U] = 0x45U; header[1U] = 0x46U; header[2U] = 0x49U;
    header[3U] = 0x20U; header[4U] = 0x50U; header[5U] = 0x41U;
    header[6U] = 0x52U; header[7U] = 0x54U;
    store_le32(header + 8U, 0x00010000U);
    store_le32(header + 12U, 92U);
    store_le64(header + 24U, 1U);
    store_le64(header + 32U, MOCK_BLOCKS - 1U);
    store_le64(header + 40U, 34U);
    store_le64(header + 48U, 1000U);
    header[56U] = 0x11U;
    header[71U] = 0x22U;
    store_le64(header + 72U, 2U);
    store_le32(header + 80U, 4U);
    store_le32(header + 84U, 128U);

    entry = disk[2U];
    entry[0U] = 0xA1U;
    entry[15U] = 0x5AU;
    entry[16U] = 0x31U;
    entry[31U] = 0x32U;
    store_le64(entry + 32U, 40U);
    store_le64(entry + 40U, 49U);
    entry += 128U;
    entry[0U] = 0xB2U;
    entry[15U] = 0x6BU;
    entry[16U] = 0x41U;
    entry[31U] = 0x42U;
    store_le64(entry + 32U, 100U);
    store_le64(entry + 40U, 199U);
    gpt_reseal_table();
}

static void test_crc_and_mbr(void)
{
    struct partition_table_input input = test_input();
    struct partition_table_snapshot snapshot;
    static const u8 crc_vector[] = {
        0x31U, 0x32U, 0x33U, 0x34U, 0x35U,
        0x36U, 0x37U, 0x38U, 0x39U
    };

    CHECK(partition_table_crc32(crc_vector, sizeof(crc_vector)) == 0xCBF43926U);
    reset_disk();
    mbr_signature();
    mbr_entry(0U, 0x80U, 0x0CU, 40U, 100U);
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_OK);
    CHECK(snapshot.kind == PARTITION_TABLE_MBR);
    CHECK(snapshot.partition_count == 1U && snapshot.discovered_count == 1U);
    CHECK(snapshot.records[0].first_lba == 40U &&
          snapshot.records[0].block_count == 100U);
    CHECK(snapshot.records[0].status == 0x80U &&
          snapshot.records[0].mbr_type == 0x0CU &&
          snapshot.records[0].table_index == 0U);

    reset_disk();
    mbr_signature();
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_OK);
    CHECK(snapshot.kind == PARTITION_TABLE_NONE &&
          snapshot.partition_count == 0U);

    reset_disk();
    disk[0][0U] = 0x7FU;
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_MALFORMED);
    CHECK(snapshot.kind == PARTITION_TABLE_MALFORMED &&
          records_cleared(&snapshot));

    reset_disk();
    mbr_signature();
    disk[0][0U] = 0xEBU;
    disk[0][2U] = 0x90U;
    store_le32(disk[0] + 11U, 0x00000200U);
    disk[0][13U] = 8U;
    store_le32(disk[0] + 14U, 1U);
    disk[0][16U] = 2U;
    disk[0][54U] = 0x46U; disk[0][55U] = 0x41U; disk[0][56U] = 0x54U;
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_OK);
    CHECK(snapshot.kind == PARTITION_TABLE_SUPERFLOPPY);
}

static void test_mbr_rejections(void)
{
    struct partition_table_input input = test_input();
    struct partition_table_snapshot snapshot;

    reset_disk();
    mbr_signature();
    mbr_entry(0U, 0U, 0x83U, MOCK_BLOCKS - 1U, 2U);
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_MALFORMED);
    CHECK(records_cleared(&snapshot));

    reset_disk();
    mbr_signature();
    mbr_entry(0U, 0U, 0x83U, 50U, 100U);
    mbr_entry(1U, 0U, 0x07U, 100U, 100U);
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_MALFORMED);
    CHECK(records_cleared(&snapshot));

    reset_disk();
    mbr_signature();
    mbr_entry(0U, 0U, 0x0FU, 40U, 100U);
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_UNSUPPORTED);
    CHECK(snapshot.kind == PARTITION_TABLE_UNSUPPORTED_EXTENDED &&
          records_cleared(&snapshot));

    reset_disk();
    mbr_signature();
    mbr_entry(0U, 0U, 0x83U, 40U, 100U);
    short_lba = 0U;
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_READ_FAILED);
    CHECK(records_cleared(&snapshot));
}

static void test_gpt_valid_and_crc_rejections(void)
{
    struct partition_table_input input = test_input();
    struct partition_table_snapshot snapshot;

    make_valid_gpt();
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_OK);
    CHECK(snapshot.kind == PARTITION_TABLE_GPT &&
          snapshot.partition_count == 2U && snapshot.discovered_count == 2U);
    CHECK(snapshot.first_usable_lba == 34U && snapshot.last_usable_lba == 1000U);
    CHECK(snapshot.records[0].source == PARTITION_TABLE_SOURCE_GPT &&
          snapshot.records[0].table_index == 1U &&
          snapshot.records[0].first_lba == 40U &&
          snapshot.records[1].table_index == 2U &&
          snapshot.records[1].block_count == 100U);

    make_valid_gpt();
    disk[1U][16U] ^= 1U;
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT);
    CHECK(snapshot.kind == PARTITION_TABLE_PROTECTIVE_MBR_NO_GPT &&
          records_cleared(&snapshot));

    make_valid_gpt();
    memset(disk[1U] + 56U, 0, 16U);
    gpt_finalize_header();
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT);
    CHECK(records_cleared(&snapshot));

    make_valid_gpt();
    memset(disk[2U] + 16U, 0, 16U);
    gpt_reseal_table();
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT);
    CHECK(records_cleared(&snapshot));

    make_valid_gpt();
    disk[2U][0U] ^= 1U;
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT);
    CHECK(records_cleared(&snapshot));

    make_valid_gpt();
    short_lba = 2U;
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_READ_FAILED);
    CHECK(snapshot.kind == PARTITION_TABLE_PROTECTIVE_MBR_NO_GPT &&
          records_cleared(&snapshot));
}

static void test_gpt_geometry_bounds_and_overlap(void)
{
    struct partition_table_input input = test_input();
    struct partition_table_snapshot snapshot;

    make_valid_gpt();
    store_le64(disk[2U] + 32U, 20U);
    store_le64(disk[2U] + 40U, 49U);
    gpt_reseal_table();
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT);
    CHECK(records_cleared(&snapshot));

    make_valid_gpt();
    store_le64(disk[2U] + 128U + 32U, 45U);
    store_le64(disk[2U] + 128U + 40U, 55U);
    gpt_reseal_table();
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT);
    CHECK(records_cleared(&snapshot));

    make_valid_gpt();
    store_le64(disk[2U] + 32U, 100U);
    store_le64(disk[2U] + 40U, ~0ULL);
    gpt_reseal_table();
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT);
    CHECK(records_cleared(&snapshot));

    make_valid_gpt();
    store_le64(disk[1U] + 48U, MOCK_BLOCKS - 2U);
    store_le64(disk[2U] + 32U, MOCK_BLOCKS - 2U);
    store_le64(disk[2U] + 40U, MOCK_BLOCKS - 2U);
    gpt_reseal_table();
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT);
    CHECK(records_cleared(&snapshot));

    make_valid_gpt();
    store_le64(disk[1U] + 72U, ~0ULL);
    gpt_finalize_header();
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT);
    CHECK(records_cleared(&snapshot));

    make_valid_gpt();
    store_le32(disk[1U] + 84U, 129U);
    gpt_finalize_header();
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT);
    CHECK(records_cleared(&snapshot));

    reset_disk();
    mbr_signature();
    mbr_entry(0U, 0U, 0xEEU, 1U, MOCK_BLOCKS - 1U);
    CHECK(partition_table_enumerate(&input, &snapshot) ==
          PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT);
    CHECK(snapshot.kind == PARTITION_TABLE_PROTECTIVE_MBR_NO_GPT &&
          records_cleared(&snapshot));
}

int main(void)
{
    test_crc_and_mbr();
    test_mbr_rejections();
    test_gpt_valid_and_crc_rejections();
    test_gpt_geometry_bounds_and_overlap();
    if (failures != 0) {
        printf("%d partition table test(s) failed\n", failures);
        return 1;
    }
    printf("partition table tests passed\n");
    return 0;
}
