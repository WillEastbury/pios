/*
 * storage_layout.c - pure validation of the pre-created PIOS MBR layout.
 *
 * No block, filesystem, formatting, or write dependency is allowed here.
 */
#include "types.h"
#include "storage_layout.h"

#define MBR_PARTITION_OFFSET 446U
#define MBR_PARTITION_BYTES  16U

static u32 load_le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) |
           ((u32)p[3] << 24);
}

static bool is_fat_type(u8 type)
{
    return type == 0x0BU || type == 0x0CU;
}

static bool span_valid(u32 first, u32 count, u64 total)
{
    return first != 0U && count != 0U && (u64)first < total &&
           (u64)count <= total - (u64)first;
}

static bool overlap(const struct storage_layout_fact *a,
                    const struct storage_layout_fact *b)
{
    return a->first_lba < b->first_lba + b->block_count &&
           b->first_lba < a->first_lba + a->block_count;
}

static bool entry_empty(const u8 *entry)
{
    u32 i;

    for (i = 0U; i < MBR_PARTITION_BYTES; i++) {
        if (entry[i] != 0U)
            return false;
    }
    return true;
}

static bool status_valid(u8 status)
{
    return status == 0U || status == 0x80U;
}

static void layout_clear(struct storage_layout *out)
{
    *out = (struct storage_layout){0};
}

static bool parse_fact(const u8 *entry, u32 disk_id, u64 total,
                       u8 role, u8 index, struct storage_layout_fact *out)
{
    u32 first = load_le32(entry + 8U);
    u32 count = load_le32(entry + 12U);

    if (!status_valid(entry[0U]) || entry[4U] == 0U ||
        !span_valid(first, count, total))
        return false;
    out->first_lba = first;
    out->block_count = count;
    out->disk_id = disk_id;
    out->role = role;
    out->primary_index = index;
    out->mbr_type = entry[4U];
    out->mbr_status = entry[0U];
    return true;
}

enum storage_layout_result storage_layout_validate(
    const u8 mbr[STORAGE_LAYOUT_MBR_BYTES], u64 total_blocks,
    struct storage_layout *out)
{
    const u8 *p1;
    const u8 *p2;
    const u8 *p3;
    const u8 *p4;
    bool three;
    u32 disk_id;
    u32 i;

    if (!out)
        return STORAGE_LAYOUT_INVALID_ARGUMENT;
    layout_clear(out);
    if (!mbr || total_blocks == 0U || total_blocks > 0x100000000ULL) {
        out->result = STORAGE_LAYOUT_INVALID_ARGUMENT;
        return STORAGE_LAYOUT_INVALID_ARGUMENT;
    }
    if (mbr[510U] != 0x55U || mbr[511U] != 0xAAU) {
        out->result = STORAGE_LAYOUT_MBR_SIGNATURE;
        return STORAGE_LAYOUT_MBR_SIGNATURE;
    }

    p1 = mbr + MBR_PARTITION_OFFSET;
    p2 = p1 + MBR_PARTITION_BYTES;
    p3 = p2 + MBR_PARTITION_BYTES;
    p4 = p3 + MBR_PARTITION_BYTES;
    disk_id = load_le32(mbr + 440U);
    if (!parse_fact(p1, disk_id, total_blocks, STORAGE_LAYOUT_ROLE_BOOT, 0U,
                    &out->facts[0U]) ||
        !parse_fact(p2, disk_id, total_blocks, STORAGE_LAYOUT_ROLE_SYSTEM, 1U,
                    &out->facts[1U]) ||
        !entry_empty(p4)) {
        out->result = STORAGE_LAYOUT_MALFORMED;
        return STORAGE_LAYOUT_MALFORMED;
    }
    if (!is_fat_type(out->facts[0U].mbr_type)) {
        out->result = STORAGE_LAYOUT_ROLE_MISMATCH;
        return STORAGE_LAYOUT_ROLE_MISMATCH;
    }

    three = !entry_empty(p3);
    if (three && !parse_fact(p3, disk_id, total_blocks,
                             STORAGE_LAYOUT_ROLE_EXCHANGE, 2U,
                             &out->facts[2U])) {
        layout_clear(out);
        out->result = STORAGE_LAYOUT_MALFORMED;
        return STORAGE_LAYOUT_MALFORMED;
    }
    for (i = 0U; i < (three ? 3U : 2U); i++) {
        u32 j;

        for (j = 0U; j < i; j++) {
            if (overlap(&out->facts[i], &out->facts[j])) {
                layout_clear(out);
                out->result = STORAGE_LAYOUT_MALFORMED;
                return STORAGE_LAYOUT_MALFORMED;
            }
        }
    }
    if (three) {
        if (out->facts[1U].mbr_type != 0xDAU ||
            !is_fat_type(out->facts[2U].mbr_type)) {
            layout_clear(out);
            out->result = STORAGE_LAYOUT_ROLE_MISMATCH;
            return STORAGE_LAYOUT_ROLE_MISMATCH;
        }
        out->kind = STORAGE_LAYOUT_THREE_PARTITION;
    } else {
        out->kind = STORAGE_LAYOUT_LEGACY_TWO_PARTITION;
    }
    out->total_blocks = total_blocks;
    out->disk_id = disk_id;
    out->result = STORAGE_LAYOUT_OK;
    return STORAGE_LAYOUT_OK;
}

const char *storage_layout_kind_name(enum storage_layout_kind kind)
{
    switch (kind) {
    case STORAGE_LAYOUT_LEGACY_TWO_PARTITION: return "legacy-2";
    case STORAGE_LAYOUT_THREE_PARTITION: return "three";
    default: return "invalid";
    }
}
