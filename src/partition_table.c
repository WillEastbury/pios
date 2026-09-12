/*
 * partition_table.c - pure bounded MBR/GPT partition observation.
 *
 * This intentionally has no storage, filesystem, allocation, MMIO, or block
 * driver dependency. All disk bytes enter only through the supplied 512-byte
 * reader span. It neither selects nor authorizes a writable partition.
 */
#include "types.h"
#include "partition_table.h"

struct partition_table_interval {
    u64 first;
    u64 end;
};

enum gpt_parse_result {
    GPT_PARSE_VALID = 0U,
    GPT_PARSE_INVALID,
    GPT_PARSE_READ_FAILED,
};

static void snapshot_clear(struct partition_table_snapshot *snapshot)
{
    *snapshot = (struct partition_table_snapshot){0};
}

static u16 load_le16(const u8 span[])
{
    return (u16)span[0] | ((u16)span[1] << 8);
}

static u32 load_le32(const u8 span[])
{
    return (u32)span[0] | ((u32)span[1] << 8) |
           ((u32)span[2] << 16) | ((u32)span[3] << 24);
}

static u64 load_le64(const u8 span[])
{
    return (u64)load_le32(span) | ((u64)load_le32(span + 4) << 32);
}

static u32 crc32_update(u32 crc, const u8 span[], u32 bytes)
{
    u32 i;
    u32 bit;

    for (i = 0U; i < bytes; i++) {
        crc ^= span[i];
        for (bit = 0U; bit < 8U; bit++)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
    }
    return crc;
}

u32 partition_table_crc32(const u8 *span, u32 bytes)
{
    if (!span && bytes != 0U)
        return 0U;
    return crc32_update(0xFFFFFFFFU, span, bytes) ^ 0xFFFFFFFFU;
}

static bool span_zero(const u8 span[], u32 bytes)
{
    u32 i;

    for (i = 0U; i < bytes; i++) {
        if (span[i] != 0U)
            return false;
    }
    return true;
}

static bool guid_zero(const u8 span[])
{
    return span_zero(span, 16U);
}

static bool range_valid(u64 first, u64 count, u64 total, u64 *end_out)
{
    u64 end;

    if (first == 0U || count == 0U || first >= total ||
        count > total - first)
        return false;
    end = first + count;
    if (end_out)
        *end_out = end;
    return true;
}

static bool intervals_overlap(const struct partition_table_interval *seen,
                              u32 count, u64 first, u64 end)
{
    u32 i;

    for (i = 0U; i < count; i++) {
        if (first < seen[i].end && seen[i].first < end)
            return true;
    }
    return false;
}

static bool is_extended_type(u8 type)
{
    return type == 0x05U || type == 0x0FU || type == 0x85U;
}

static bool is_superfloppy(const u8 block[])
{
    u8 sectors_per_cluster;
    u8 fats;
    bool fat_label;

    sectors_per_cluster = block[13U];
    fats = block[16U];
    fat_label = (block[54U] == 0x46U && block[55U] == 0x41U &&
                 block[56U] == 0x54U) ||
                (block[82U] == 0x46U && block[83U] == 0x41U &&
                 block[84U] == 0x54U);
    return (block[0U] == 0xEBU && block[2U] == 0x90U) ||
           (block[0U] == 0xE9U) ?
           load_le16(block + 11U) == PARTITION_TABLE_BLOCK_BYTES &&
           sectors_per_cluster != 0U &&
           sectors_per_cluster <= 128U &&
           (sectors_per_cluster & (sectors_per_cluster - 1U)) == 0U &&
           load_le16(block + 14U) != 0U && fats >= 1U && fats <= 2U &&
           fat_label : false;
}

static void snapshot_finish(struct partition_table_snapshot *snapshot,
                            enum partition_table_kind kind,
                            enum partition_table_result result)
{
    snapshot->kind = (u32)kind;
    snapshot->result = (u32)result;
}

static enum gpt_parse_result parse_gpt(
    const struct partition_table_input *input,
    struct partition_table_snapshot *snapshot)
{
    u8 block[PARTITION_TABLE_BLOCK_BYTES];
    struct partition_table_interval seen[PARTITION_TABLE_GPT_ENTRY_LIMIT];
    u64 backup_lba;
    u64 first_usable;
    u64 last_usable;
    u64 entries_lba;
    u64 table_bytes;
    u64 table_blocks;
    u64 table_end;
    u32 header_bytes;
    u32 header_crc;
    u32 entry_count;
    u32 entry_bytes;
    u32 entries_per_block;
    u32 table_crc;
    u32 crc;
    u32 i;
    u32 j;
    u32 retained = 0U;
    u32 discovered = 0U;

    if (!input->read(input->context, 1U, block))
        return GPT_PARSE_READ_FAILED;
    if (block[0U] != 0x45U || block[1U] != 0x46U ||
        block[2U] != 0x49U || block[3U] != 0x20U ||
        block[4U] != 0x50U || block[5U] != 0x41U ||
        block[6U] != 0x52U || block[7U] != 0x54U ||
        load_le32(block + 8U) != 0x00010000U)
        return GPT_PARSE_INVALID;
    header_bytes = load_le32(block + 12U);
    header_crc = load_le32(block + 16U);
    if (header_bytes < 92U || header_bytes > PARTITION_TABLE_BLOCK_BYTES ||
        load_le32(block + 20U) != 0U)
        return GPT_PARSE_INVALID;
    crc = 0xFFFFFFFFU;
    crc = crc32_update(crc, block, 16U);
    for (i = 0U; i < 4U; i++)
        crc = crc32_update(crc, (const u8[]){0U}, 1U);
    crc = crc32_update(crc, block + 20U, header_bytes - 20U) ^ 0xFFFFFFFFU;
    if (crc != header_crc || load_le64(block + 24U) != 1U)
        return GPT_PARSE_INVALID;
    if (guid_zero(block + 56U))
        return GPT_PARSE_INVALID;

    backup_lba = load_le64(block + 32U);
    first_usable = load_le64(block + 40U);
    last_usable = load_le64(block + 48U);
    entries_lba = load_le64(block + 72U);
    entry_count = load_le32(block + 80U);
    entry_bytes = load_le32(block + 84U);
    table_crc = load_le32(block + 88U);
    if (backup_lba != input->total_blocks - 1U ||
        first_usable == 0U || first_usable > last_usable ||
        last_usable >= input->total_blocks || entries_lba == 0U ||
        entry_count == 0U || entry_count > input->max_gpt_entries ||
        entry_bytes < 128U || entry_bytes > PARTITION_TABLE_BLOCK_BYTES ||
        (entry_bytes % 128U) != 0U ||
        (PARTITION_TABLE_BLOCK_BYTES % entry_bytes) != 0U)
        return GPT_PARSE_INVALID;
    table_bytes = (u64)entry_count * entry_bytes;
    table_blocks = (table_bytes + PARTITION_TABLE_BLOCK_BYTES - 1U) /
                   PARTITION_TABLE_BLOCK_BYTES;
    if (table_bytes / entry_bytes != entry_count ||
        entries_lba >= input->total_blocks ||
        table_blocks > input->total_blocks - entries_lba)
        return GPT_PARSE_INVALID;
    table_end = entries_lba + table_blocks;
    if ((entries_lba <= 1U && 1U < table_end) ||
        first_usable < table_end || backup_lba < table_end ||
        table_blocks >= backup_lba ||
        last_usable >= backup_lba - table_blocks)
        return GPT_PARSE_INVALID;

    entries_per_block = PARTITION_TABLE_BLOCK_BYTES / entry_bytes;
    crc = 0xFFFFFFFFU;
    for (i = 0U; i < (u32)table_blocks; i++) {
        u32 remain = (u32)(table_bytes -
            (u64)i * PARTITION_TABLE_BLOCK_BYTES);
        u32 crc_bytes = remain < PARTITION_TABLE_BLOCK_BYTES ? remain :
                        PARTITION_TABLE_BLOCK_BYTES;
        u32 in_block = entry_count - i * entries_per_block;

        if (!input->read(input->context, entries_lba + i, block))
            return GPT_PARSE_READ_FAILED;
        crc = crc32_update(crc, block, crc_bytes);
        if (in_block > entries_per_block)
            in_block = entries_per_block;
        for (j = 0U; j < in_block; j++) {
            const u8 *entry = block + j * entry_bytes;
            u64 entry_first;
            u64 entry_last;
            u64 entry_end;
            u32 type_byte;

            if (guid_zero(entry))
                continue;
            if (guid_zero(entry + 16U))
                return GPT_PARSE_INVALID;
            entry_first = load_le64(entry + 32U);
            entry_last = load_le64(entry + 40U);
            if (entry_first > entry_last || entry_first < first_usable ||
                entry_last > last_usable || entry_last == ~0ULL)
                return GPT_PARSE_INVALID;
            entry_end = entry_last + 1U;
            if (intervals_overlap(seen, discovered, entry_first, entry_end))
                return GPT_PARSE_INVALID;
            seen[discovered].first = entry_first;
            seen[discovered].end = entry_end;
            if (retained < PARTITION_TABLE_RECORD_CAPACITY) {
                struct partition_table_record *record =
                    &snapshot->records[retained];

                record->first_lba = entry_first;
                record->block_count = entry_end - entry_first;
                for (type_byte = 0U; type_byte < 16U; type_byte++)
                    record->type_guid[type_byte] = entry[type_byte];
                record->table_index = i * entries_per_block + j + 1U;
                record->source = PARTITION_TABLE_SOURCE_GPT;
                retained++;
            }
            discovered++;
        }
    }
    if ((crc ^ 0xFFFFFFFFU) != table_crc)
        return GPT_PARSE_INVALID;
    snapshot->total_blocks = input->total_blocks;
    snapshot->first_usable_lba = first_usable;
    snapshot->last_usable_lba = last_usable;
    snapshot->partition_count = retained;
    snapshot->discovered_count = discovered;
    return GPT_PARSE_VALID;
}

enum partition_table_result partition_table_enumerate(
    const struct partition_table_input *input,
    struct partition_table_snapshot *snapshot)
{
    u8 block[PARTITION_TABLE_BLOCK_BYTES];
    struct partition_table_interval seen[4U];
    bool protective = false;
    bool extended = false;
    u32 i;
    u32 retained = 0U;

    if (!snapshot)
        return PARTITION_TABLE_RESULT_INVALID_ARGUMENT;
    snapshot_clear(snapshot);
    if (!input || !input->read || input->total_blocks == 0U ||
        input->max_gpt_entries == 0U ||
        input->max_gpt_entries > PARTITION_TABLE_GPT_ENTRY_LIMIT) {
        snapshot_finish(snapshot, PARTITION_TABLE_MALFORMED,
                        PARTITION_TABLE_RESULT_INVALID_ARGUMENT);
        return PARTITION_TABLE_RESULT_INVALID_ARGUMENT;
    }
    if (!input->read(input->context, 0U, block)) {
        snapshot_finish(snapshot, PARTITION_TABLE_MALFORMED,
                        PARTITION_TABLE_RESULT_READ_FAILED);
        return PARTITION_TABLE_RESULT_READ_FAILED;
    }
    if (block[510U] != 0x55U || block[511U] != 0xAAU) {
        if (span_zero(block, PARTITION_TABLE_BLOCK_BYTES)) {
            snapshot_finish(snapshot, PARTITION_TABLE_NONE,
                            PARTITION_TABLE_RESULT_OK);
            return PARTITION_TABLE_RESULT_OK;
        }
        snapshot_finish(snapshot, PARTITION_TABLE_MALFORMED,
                        PARTITION_TABLE_RESULT_MALFORMED);
        return PARTITION_TABLE_RESULT_MALFORMED;
    }

    for (i = 0U; i < 4U; i++) {
        const u8 *entry = block + 446U + i * 16U;
        u8 status = entry[0U];
        u8 type = entry[4U];
        u64 first = load_le32(entry + 8U);
        u64 count = load_le32(entry + 12U);
        u64 end;

        if (type == 0U && first == 0U && count == 0U) {
            if (status != 0U) {
                snapshot_clear(snapshot);
                snapshot_finish(snapshot, PARTITION_TABLE_MALFORMED,
                                PARTITION_TABLE_RESULT_MALFORMED);
                return PARTITION_TABLE_RESULT_MALFORMED;
            }
            continue;
        }
        if ((status != 0U && status != 0x80U) ||
            type == 0U || !range_valid(first, count, input->total_blocks, &end) ||
            intervals_overlap(seen, retained, first, end)) {
            snapshot_clear(snapshot);
            snapshot_finish(snapshot, PARTITION_TABLE_MALFORMED,
                            PARTITION_TABLE_RESULT_MALFORMED);
            return PARTITION_TABLE_RESULT_MALFORMED;
        }
        seen[retained].first = first;
        seen[retained].end = end;
        if (type == 0xEEU)
            protective = true;
        if (is_extended_type(type))
            extended = true;
        snapshot->records[retained].first_lba = first;
        snapshot->records[retained].block_count = count;
        snapshot->records[retained].table_index = i;
        snapshot->records[retained].source = PARTITION_TABLE_SOURCE_MBR_PRIMARY;
        snapshot->records[retained].status = status;
        snapshot->records[retained].mbr_type = type;
        retained++;
    }
    if (extended) {
        snapshot_clear(snapshot);
        snapshot_finish(snapshot, PARTITION_TABLE_UNSUPPORTED_EXTENDED,
                        PARTITION_TABLE_RESULT_UNSUPPORTED);
        return PARTITION_TABLE_RESULT_UNSUPPORTED;
    }
    if (protective) {
        enum gpt_parse_result gpt;

        /* MBR metadata cannot remain in a GPT result or error snapshot. */
        snapshot_clear(snapshot);
        gpt = parse_gpt(input, snapshot);

        if (gpt == GPT_PARSE_VALID) {
            snapshot_finish(snapshot, PARTITION_TABLE_GPT,
                            PARTITION_TABLE_RESULT_OK);
            return PARTITION_TABLE_RESULT_OK;
        }
        snapshot_clear(snapshot);
        if (gpt == GPT_PARSE_READ_FAILED) {
            snapshot_finish(snapshot, PARTITION_TABLE_PROTECTIVE_MBR_NO_GPT,
                            PARTITION_TABLE_RESULT_READ_FAILED);
            return PARTITION_TABLE_RESULT_READ_FAILED;
        }
        snapshot_finish(snapshot, PARTITION_TABLE_PROTECTIVE_MBR_NO_GPT,
                        PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT);
        return PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT;
    }
    if (retained == 0U) {
        snapshot_finish(snapshot,
                        is_superfloppy(block) ? PARTITION_TABLE_SUPERFLOPPY :
                                                PARTITION_TABLE_NONE,
                        PARTITION_TABLE_RESULT_OK);
        return PARTITION_TABLE_RESULT_OK;
    }
    snapshot->total_blocks = input->total_blocks;
    snapshot->partition_count = retained;
    snapshot->discovered_count = retained;
    snapshot_finish(snapshot, PARTITION_TABLE_MBR, PARTITION_TABLE_RESULT_OK);
    return PARTITION_TABLE_RESULT_OK;
}
