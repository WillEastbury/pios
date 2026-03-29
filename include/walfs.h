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

/* Flush superblock to disk. */
void walfs_sync(void);

/* Compact WAL: rewrite with only live records, reclaim space. */
bool walfs_compact(void);
