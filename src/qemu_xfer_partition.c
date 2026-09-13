/* Pure validation for the deliberately fixed QEMU storage acceptance layout. */
#include "types.h"
#include "qemu_xfer_partition.h"

#define MBR_PARTITION_OFFSET 0x1BEU
#define MBR_PARTITION_BYTES  16U

static u32 get_le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) |
           ((u32)p[3] << 24);
}

static void clear_partition(struct qemu_xfer_partition *out)
{
    u8 *p = (u8 *)out;

    for (u32 i = 0U; i < sizeof(*out); i++)
        p[i] = 0U;
}

static bool valid_span(u32 first, u32 count, u64 total_blocks)
{
    return first != 0U && count != 0U && (u64)first < total_blocks &&
           (u64)count <= total_blocks - (u64)first;
}

static bool spans_overlap(u32 a_first, u32 a_count, u32 b_first, u32 b_count)
{
    u64 a_end = (u64)a_first + a_count;
    u64 b_end = (u64)b_first + b_count;

    return (u64)a_first < b_end && (u64)b_first < a_end;
}

enum qemu_xfer_partition_result qemu_xfer_partition_select(
    const u8 mbr[QEMU_XFER_MBR_BYTES], u64 total_blocks,
    struct qemu_xfer_partition *out)
{
    const u8 *p1;
    const u8 *p2;
    const u8 *p3;
    u32 p1_first;
    u32 p1_count;
    u32 p2_first;
    u32 p2_count;
    u32 p3_first;
    u32 p3_count;
    u32 disk_id;
    static const u8 label[QEMU_XFER_LABEL_BYTES] = {
        'P', 'I', 'O', 'S', 'X', 'F', 'E', 'R', ' ', ' ', ' '
    };

    if (!out)
        return QEMU_XFER_PARTITION_INVALID;
    clear_partition(out);
    if (!mbr || total_blocks > 0x100000000ULL)
        return QEMU_XFER_PARTITION_INVALID;
    if (mbr[510U] != 0x55U || mbr[511U] != 0xAAU)
        return QEMU_XFER_PARTITION_MBR;

    p1 = mbr + MBR_PARTITION_OFFSET;
    p2 = p1 + MBR_PARTITION_BYTES;
    p3 = mbr + MBR_PARTITION_OFFSET +
         (QEMU_XFER_PARTITION_INDEX - 1U) * MBR_PARTITION_BYTES;
    p1_first = get_le32(p1 + 8U);
    p1_count = get_le32(p1 + 12U);
    p2_first = get_le32(p2 + 8U);
    p2_count = get_le32(p2 + 12U);
    p3_first = get_le32(p3 + 8U);
    p3_count = get_le32(p3 + 12U);
    disk_id = get_le32(mbr + 440U);

    if (disk_id == 0U || p3[0] != 0U ||
        (p3[4] != 0x0BU && p3[4] != 0x0CU) ||
        !valid_span(p1_first, p1_count, total_blocks) ||
        !valid_span(p2_first, p2_count, total_blocks) ||
        !valid_span(p3_first, p3_count, total_blocks) ||
        spans_overlap(p1_first, p1_count, p2_first, p2_count) ||
        spans_overlap(p1_first, p1_count, p3_first, p3_count) ||
        spans_overlap(p2_first, p2_count, p3_first, p3_count))
        return QEMU_XFER_PARTITION_LAYOUT;

    out->disk_id = disk_id;
    out->first_lba = p3_first;
    out->block_count = p3_count;
    out->identity = ((u64)disk_id << 32) | p3_first;
    for (u32 i = 0U; i < QEMU_XFER_LABEL_BYTES; i++)
        out->expected_label[i] = label[i];
    return QEMU_XFER_PARTITION_OK;
}
