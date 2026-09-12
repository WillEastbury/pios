/*
 * partition_table.h - bounded, read-only MBR/GPT partition observation.
 *
 * All multi-byte fields on disk are little-endian. GPT type GUID bytes are
 * retained in their on-disk order; this module neither formats GUIDs nor
 * assigns any storage authority. CRC32 is the reflected IEEE 802.3 form:
 * initial 0xffffffff, polynomial 0xedb88320, final xor 0xffffffff.
 */
#pragma once

#include "types.h"

#define PARTITION_TABLE_BLOCK_BYTES          512U
#define PARTITION_TABLE_RECORD_CAPACITY      16U
#define PARTITION_TABLE_GPT_ENTRY_LIMIT      128U

enum partition_table_kind {
    PARTITION_TABLE_NONE = 0U,
    PARTITION_TABLE_SUPERFLOPPY,
    PARTITION_TABLE_MBR,
    PARTITION_TABLE_GPT,
    PARTITION_TABLE_PROTECTIVE_MBR_NO_GPT,
    PARTITION_TABLE_MALFORMED,
    PARTITION_TABLE_UNSUPPORTED_EXTENDED,
};

enum partition_table_result {
    PARTITION_TABLE_RESULT_OK = 0U,
    PARTITION_TABLE_RESULT_PROTECTIVE_NO_GPT,
    PARTITION_TABLE_RESULT_UNSUPPORTED,
    PARTITION_TABLE_RESULT_MALFORMED,
    PARTITION_TABLE_RESULT_READ_FAILED,
    PARTITION_TABLE_RESULT_INVALID_ARGUMENT,
};

enum partition_table_source {
    PARTITION_TABLE_SOURCE_MBR_PRIMARY = 1U,
    PARTITION_TABLE_SOURCE_GPT = 2U,
};

/*
 * The callback must either fill exactly one 512-byte input span or return
 * false. `context` is opaque callback state; it is not retained.
 */
typedef bool (*partition_table_read_block_fn)(
    void *context, u64 lba, u8 dst[PARTITION_TABLE_BLOCK_BYTES]);

/*
 * `total_blocks` is the authoritative device capacity. `max_gpt_entries`
 * explicitly bounds parser work and must be in 1..PARTITION_TABLE_GPT_ENTRY_LIMIT.
 */
struct partition_table_input {
    partition_table_read_block_fn read;
    void *context;
    u64 total_blocks;
    u32 max_gpt_entries;
    u32 _reserved;
};

/*
 * Records and the snapshot header have 64-byte strides. The caller owns this
 * object and may publish it immutable after partition_table_enumerate()
 * returns; the parser keeps no global state or retained callback pointer.
 */
struct partition_table_record {
    u64 first_lba;
    u64 block_count;
    u8 type_guid[16U]; /* GPT wire order; zero for MBR. */
    u32 table_index;   /* MBR 0..3, GPT 1-based entry index. */
    u8 source;
    u8 status;         /* MBR boot indicator, zero for GPT. */
    u8 mbr_type;       /* MBR type, zero for GPT. */
    u8 _reserved0;
    u8 _pad[24U];
} ALIGNED(64);

struct partition_table_snapshot {
    u64 total_blocks;
    u64 first_usable_lba; /* GPT only; zero for MBR/superfloppy/none. */
    u64 last_usable_lba;  /* GPT only; zero for MBR/superfloppy/none. */
    u32 kind;
    u32 result;
    u32 partition_count;  /* Retained records, never above capacity. */
    u32 discovered_count; /* All validated usable entries. */
    u8 _pad[24U];
    struct partition_table_record records[PARTITION_TABLE_RECORD_CAPACITY];
} ALIGNED(64);

_Static_assert(sizeof(struct partition_table_record) == 64U,
               "partition records must have cache-line stride");
_Static_assert(sizeof(struct partition_table_snapshot) ==
               64U + PARTITION_TABLE_RECORD_CAPACITY * 64U,
               "partition snapshot must isolate header and records");
_Static_assert(__builtin_offsetof(struct partition_table_snapshot, records) ==
               64U, "partition snapshot records must not share header line");

/* Standard IEEE CRC32 used by GPT; exposed for independent host vectors. */
u32 partition_table_crc32(const u8 *span, u32 bytes);

/*
 * Clears `snapshot` before every attempt. On every non-OK result, no
 * partition record is retained; kind/result remain as classification evidence.
 * Protective-MBR-without-GPT and extended-MBR are explicit non-enumerable
 * observations, never fallback candidates.
 */
enum partition_table_result partition_table_enumerate(
    const struct partition_table_input *input,
    struct partition_table_snapshot *snapshot);
