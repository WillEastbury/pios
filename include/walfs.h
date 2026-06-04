/*
 * walfs.h - WAL-based append-only filesystem
 *
 * Crash-safe, deterministic, multi-core-friendly filesystem.
 * Append-only log of records on top of raw SD block I/O.
 * Tree-structured directories with Latin-1 filenames.
 *
 * On-disk: Superblock (block 0) + sequential WAL records.
 * Latest record for a given inode wins (append-only semantic).
 */

#pragma once
#include "types.h"

/* Superblock magic */
#define WALFS_MAGIC         0x57414C46  /* 'WALF' */
#define WALFS_VERSION       1
#define WALFS_BLOCK_SIZE    512
#define WALFS_NAME_MAX      127
#define WALFS_DATA_MAX      4096

/*
 * Partition-2 reserved layout. WALFS starts after this 10 MiB region; all
 * WALFS block addresses are relative to that post-reserved base.
 *
 * Offsets are relative to the start of partition 2:
 *   0x000000..0x0001FF  reserved-area header / PIOS signature
 *   0x000200..0x1FFFFF  second-stage kernel image payload
 *   0x200000..0x2C7FFF  TCP/IP stack area (800 KiB):
 *                       MAC, ARP, DHCP, DNS, ICMP, UDP, TCP, streams clients
 *   0x2C8000..0x2FFFFF  firewall rules + static IP config area (200 KiB)
 *   0x300000..0x3FFFFF  Admin Console HTTP service area
 *   0x400000..0x4FFFFF  kernel debugger / dump / inspector area
 *   0x500000..0x5FFFFF  user records
 *   0x600000..0x7FFFFF  circular hot log buffer
 *   0x800000..0x8FFFFF  kernel state crashdump zone
 *   0x900000..0x9FFFFF  future reserved system service area
 *   0xA00000..end       WALFS packs/cards/storage
 */
#define PIOS_RESERVED_BYTES             (10U * 1024U * 1024U)
#define PIOS_RESERVED_LBAS              (PIOS_RESERVED_BYTES / 512U)
#define PIOS_RESERVED_HEADER_OFFSET     0x000000U
#define PIOS_RESERVED_HEADER_BYTES      0x000200U
#define PIOS_RESERVED_HEADER_MAGIC      0x50494F53U  /* 'PIOS' */
#define PIOS_RESERVED_LAYOUT_VERSION    1U
#define PIOS_HDR_MAGIC_OFF              0U
#define PIOS_HDR_STAGE2_LEN_OFF         4U
#define PIOS_HDR_LAYOUT_VERSION_OFF     8U
#define PIOS_HDR_RESERVED_BYTES_OFF     12U
#define PIOS_HDR_STAGE2_OFFSET_OFF      16U
#define PIOS_HDR_STAGE2_BYTES_OFF       20U
#define PIOS_HDR_TCPIP_OFFSET_OFF       24U
#define PIOS_HDR_TCPIP_BYTES_OFF        28U
#define PIOS_HDR_FIREWALL_OFFSET_OFF    32U
#define PIOS_HDR_FIREWALL_BYTES_OFF     36U
#define PIOS_HDR_ADMIN_OFFSET_OFF       40U
#define PIOS_HDR_DEBUG_OFFSET_OFF       44U
#define PIOS_HDR_USER_RECORDS_OFF       48U
#define PIOS_HDR_HOT_LOGS_OFF           52U
#define PIOS_HDR_HOT_LOGS_BYTES_OFF     56U
#define PIOS_HDR_CRASHDUMP_OFF          60U
#define PIOS_HDR_FUTURE_OFF             64U
#define PIOS_HDR_WALFS_OFF              68U
#define PIOS_STAGE2_OFFSET              0x000200U
#define PIOS_STAGE2_END_OFFSET          0x1FFFFFU
#define PIOS_STAGE2_ZONE_BYTES          (PIOS_STAGE2_END_OFFSET - PIOS_STAGE2_OFFSET + 1U)
#define PIOS_TCPIP_STACK_OFFSET         0x200000U
#define PIOS_TCPIP_STACK_BYTES          0x0C8000U
#define PIOS_FIREWALL_CFG_OFFSET        0x2C8000U
#define PIOS_FIREWALL_CFG_BYTES         0x038000U
#define PIOS_TCPIP_FW_OFFSET            PIOS_TCPIP_STACK_OFFSET
#define PIOS_ADMIN_HTTP_OFFSET          0x300000U
#define PIOS_KERNEL_DEBUG_OFFSET        0x400000U
#define PIOS_USER_RECORDS_OFFSET        0x500000U
#define PIOS_HOT_LOGS_OFFSET            0x600000U
#define PIOS_HOT_LOGS_BYTES             0x200000U
#define PIOS_CRASHDUMP_OFFSET           0x800000U
#define PIOS_FUTURE_RESERVED_OFFSET     0x900000U
#define PIOS_WALFS_OFFSET               0xA00000U

#define WALFS_BOOT_SLOT_BYTES  PIOS_RESERVED_BYTES
#define WALFS_BOOT_SLOT_LBAS   PIOS_RESERVED_LBAS
#define WALFS_BASE_LBA         (2048U + WALFS_BOOT_SLOT_LBAS)

/* Runtime partition 2 base, set by walfs_discover_partition() */
u32 walfs_partition_lba(void);
u32 walfs_part2_lba(void);

/* Record magic and types */
#define WALFS_REC_MAGIC     0x5245434F  /* 'RECO' */
#define RECORD_INODE        1
#define RECORD_DATA         2
#define RECORD_DIRENT       3
#define RECORD_DELETE       4
#define RECORD_TX_BEGIN     5
#define RECORD_TX_COMMIT    6

/* Inode flags */
#define WALFS_DIR           0x01
#define WALFS_FILE          0x02
#define WALFS_DELETED       0x80

/* Root inode ID */
#define WALFS_ROOT_INODE    1

/* FIFO message types */
#define MSG_FS_CREATE       30
#define MSG_FS_WRITE        31
#define MSG_FS_DELETE       32
#define MSG_FS_MKDIR        33
#define MSG_FS_DONE         34
#define MSG_FS_ERROR        35
#define MSG_FS_READDIR      40
#define MSG_FS_READ         41
#define MSG_FS_STAT         42
#define MSG_FS_FIND         43
#define MSG_FS_SCAN_RESULT  44
#define MSG_FS_SYNC         45

/* ---- On-disk Structures ---- */

struct walfs_super {
    u32 magic;
    u32 version;
    u32 block_size;
    u32 total_blocks;
    u64 wal_head;       /* next free byte offset in WAL region */
    u64 tree_root;      /* byte offset of root inode record */
    u32 record_count;
    u32 crc32;
    u8  label[32];
    u8  _reserved[448];
} PACKED;

struct wal_record {
    u32 magic;
    u32 type;
    u32 length;         /* total record including header */
    u32 crc32;
    u64 seq;
    u64 timestamp;
    /* payload follows */
} PACKED;

struct walfs_inode {
    u64 inode_id;
    u64 parent_id;
    u32 mode;           /* rwxrwxrwx (9 bits) */
    u32 flags;
    u64 size;
    u64 created;
    u64 modified;
    u8  name[128];
} PACKED;

struct walfs_data {
    u64 inode_id;
    u64 offset;
    u32 length;
    /* data[] follows */
} PACKED;

struct walfs_dirent {
    u64 parent_id;
    u64 child_id;
    u8  name[128];
} PACKED;

struct walfs_delete {
    u64 inode_id;
} PACKED;

struct walfs_health {
    bool super_ok;
    bool wal_head_ok;
    bool open_tx;
    u32 valid_records;
    u32 crc_errors;
    u32 header_errors;
    u64 scan_end;
} PACKED;

struct walfs_status_snapshot {
    bool mounted;
    bool legacy_present;
    bool root_ok;
    bool super_ok;
    u32 partition_lba;
    u32 base_lba;
    u32 partition_blocks;
    u32 region_blocks;
    u32 super_magic;
    u32 super_version;
    u32 super_records;
    u64 super_head;
    u64 super_tree_root;
} PACKED;

/* ---- API ---- */

/* Init/format filesystem on SD card. If not formatted, creates one. */
bool walfs_init(void);

/* Create a file. Returns inode_id or 0 on error. */
u64 walfs_create(u64 parent_id, const char *name, u32 flags, u32 mode);

/* Write data to a file (appends at offset). */
bool walfs_write(u64 inode_id, u64 offset, const void *data, u32 len);

/* Read file content into buffer. Returns bytes read. */
u32 walfs_read(u64 inode_id, u64 offset, void *buf, u32 len);

/* Delete a file/directory. */
bool walfs_delete(u64 inode_id);

/* Stat: get latest inode metadata. Returns true if found. */
bool walfs_stat(u64 inode_id, struct walfs_inode *out);

/* Find inode by path (e.g. "/dir/file.txt"). Returns inode_id or 0. */
u64 walfs_find(const char *path);

/* List directory: calls callback for each child. */
typedef void (*walfs_readdir_cb)(const struct walfs_dirent *entry);
void walfs_readdir(u64 parent_id, walfs_readdir_cb cb);

/* MMAP-style read: copies file data into user buffer. */
bool walfs_mmap(u64 inode_id, u64 offset, u32 length, void *dest);

/* Process filesystem FIFO requests on Core 1 (called from disk loop). */
void walfs_handle_fifo(u32 from_core);

/* Compact WAL: rewrite with only live records, removing deleted inodes. */
bool walfs_compact(void);

/* Verify WALFS metadata and record chain integrity. */
bool walfs_verify(struct walfs_health *out);

/* Flush superblock to disk. */
void walfs_sync(void);
void walfs_status(struct walfs_status_snapshot *out);
bool walfs_format_reserved(void);
