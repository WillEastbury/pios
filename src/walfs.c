/*
 * walfs.c - WAL-based append-only filesystem
 *
 * On-disk layout:
 *   Block 0: Superblock (512 bytes)
 *   Block 1+: Sequential WAL records (append-only)
 *
 * Records are variable-length, CRC32C-checksummed, and never overwritten.
 * The latest record for a given inode wins. Deletions are soft (RECORD_DELETE).
 */

#include "walfs.h"
#include "sd.h"
#include "simd.h"
#include "uart.h"
#include "timer.h"
#include "fifo.h"
#include "core.h"

static inline u64 read_cntvct(void) {
    u64 v;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(v));
    return v;
}

/* ---- State ---- */

static struct walfs_super super;
static u64 next_inode;
static u64 next_seq;
static bool mounted;

/* I/O buffers */
static u8 blk_buf[SD_BLOCK_SIZE] ALIGNED(64);
static u8 rec_buf[4096 + 256] ALIGNED(64);

/* ---- Low-level block I/O for WAL ---- */

/* Read len bytes from WAL at byte offset into dst */
static bool wal_read(u64 byte_off, void *dst, u32 len) {
    u8 *d = (u8 *)dst;
    while (len > 0) {
        u32 lba = (u32)(byte_off / SD_BLOCK_SIZE);
        u32 off_in_blk = (u32)(byte_off % SD_BLOCK_SIZE);
        u32 chunk = SD_BLOCK_SIZE - off_in_blk;
        if (chunk > len) chunk = len;

        if (!sd_read_block(lba, blk_buf))
            return false;
        simd_memcpy(d, blk_buf + off_in_blk, chunk);

        d += chunk;
        byte_off += chunk;
        len -= chunk;
    }
    return true;
}

/* Write len bytes to WAL at byte offset from src */
static bool wal_write(u64 byte_off, const void *src, u32 len) {
    const u8 *s = (const u8 *)src;
    while (len > 0) {
        u32 lba = (u32)(byte_off / SD_BLOCK_SIZE);
        u32 off_in_blk = (u32)(byte_off % SD_BLOCK_SIZE);
        u32 chunk = SD_BLOCK_SIZE - off_in_blk;
        if (chunk > len) chunk = len;

        if (off_in_blk != 0 || chunk < SD_BLOCK_SIZE) {
            if (!sd_read_block(lba, blk_buf))
                return false;
        }
        simd_memcpy(blk_buf + off_in_blk, s, chunk);
        if (!sd_write_block(lba, blk_buf))
            return false;

        s += chunk;
        byte_off += chunk;
        len -= chunk;
    }
    return true;
}

/* ---- CRC helper ---- */

static u32 record_crc(const void *rec, u32 len) {
    /* Zero the CRC field (offset 12 in wal_record), compute, restore */
    u8 *p = (u8 *)rec;
    u32 saved = *(u32 *)(p + 12);
    *(u32 *)(p + 12) = 0;
    u32 crc = hw_crc32c(p, len);
    *(u32 *)(p + 12) = saved;
    return crc;
}

/* ---- WAL append ---- */

static bool wal_append(u32 type, const void *payload, u32 payload_len) {
    u32 total = sizeof(struct wal_record) + payload_len;
    struct wal_record *hdr = (struct wal_record *)rec_buf;

    hdr->magic = WALFS_REC_MAGIC;
    hdr->type = type;
    hdr->length = total;
    hdr->crc32 = 0;
    hdr->seq = next_seq++;
    hdr->timestamp = read_cntvct();

    if (payload_len > 0)
        simd_memcpy(rec_buf + sizeof(struct wal_record), payload, payload_len);

    hdr->crc32 = hw_crc32c(rec_buf, total);

    if (!wal_write(super.wal_head, rec_buf, total))
        return false;

    super.wal_head += total;
    /* Align to 4 bytes */
    super.wal_head = (super.wal_head + 3) & ~3ULL;
    super.record_count++;

    return true;
}

/* ---- Superblock I/O ---- */

void walfs_sync(void) {
    super.crc32 = 0;
    super.crc32 = hw_crc32c(&super, sizeof(super));
    sd_write_block(0, (u8 *)&super);
}

static bool load_super(void) {
    if (!sd_read_block(0, (u8 *)&super))
        return false;
    if (super.magic != WALFS_MAGIC)
        return false;
    u32 stored_crc = super.crc32;
    super.crc32 = 0;
    u32 calc = hw_crc32c(&super, sizeof(super));
    super.crc32 = stored_crc;
    return (calc == stored_crc);
}

static void format_disk(void) {
    uart_puts("[walfs] Formatting...\n");

    simd_zero(&super, sizeof(super));
    super.magic = WALFS_MAGIC;
    super.version = WALFS_VERSION;
    super.block_size = WALFS_BLOCK_SIZE;
    super.total_blocks = 0; /* unknown until sd reports */
    super.wal_head = SD_BLOCK_SIZE; /* WAL starts at block 1 */
    super.tree_root = 0;
    super.record_count = 0;

    const char *label = "PIOS";
    for (u32 i = 0; label[i] && i < 31; i++)
        super.label[i] = (u8)label[i];

    next_seq = 1;
    next_inode = 2; /* 1 = root */

    walfs_sync();

    /* Create root directory inode */
    struct walfs_inode root;
    simd_zero(&root, sizeof(root));
    root.inode_id = WALFS_ROOT_INODE;
    root.parent_id = 0;
    root.flags = WALFS_DIR;
    root.mode = 0755;
    root.created = read_cntvct();
    root.modified = root.created;
    root.name[0] = '/';

    wal_append(RECORD_INODE, &root, sizeof(root));
    super.tree_root = SD_BLOCK_SIZE;
    walfs_sync();
}

/* ---- Filename validation ---- */

static bool valid_name(const char *name) {
    if (!name || name[0] == 0) return false;
    for (u32 i = 0; name[i] && i < WALFS_NAME_MAX; i++) {
        u8 c = (u8)name[i];
        if (c < 0x20 || c == 0x7F || (c >= 0x80 && c <= 0x9F) || c == '/')
            return false;
    }
    return true;
}

static bool str_eq(const char *a, const char *b) {
    while (*a && *b) {
        if (*a != *b) return false;
        a++; b++;
    }
    return *a == *b;
}

/* ---- WAL scanning ---- */

/* Scan callback: returns true to stop scanning */
typedef bool (*scan_cb)(const struct wal_record *hdr, const u8 *payload, u64 offset);

static void wal_scan(scan_cb cb) {
    u64 off = SD_BLOCK_SIZE; /* skip superblock */
    while (off < super.wal_head) {
        struct wal_record hdr;
        if (!wal_read(off, &hdr, sizeof(hdr)))
            break;
        if (hdr.magic != WALFS_REC_MAGIC)
            break;
        if (hdr.length < sizeof(struct wal_record) || hdr.length > sizeof(rec_buf))
            break;

        /* Read full record for CRC check */
        if (!wal_read(off, rec_buf, hdr.length))
            break;
        u32 stored_crc = ((struct wal_record *)rec_buf)->crc32;
        u32 calc = record_crc(rec_buf, hdr.length);
        if (calc != stored_crc) {
            /* Corrupt record, skip */
            off += hdr.length;
            off = (off + 3) & ~3ULL;
            continue;
        }

        u8 *payload = rec_buf + sizeof(struct wal_record);
        if (cb(&hdr, payload, off))
            return;

        off += hdr.length;
        off = (off + 3) & ~3ULL;
    }
}

/* ---- Scan helpers ---- */

/* Find latest inode record */
static struct walfs_inode found_inode;
static bool found_inode_ok;
static u64 search_inode_id;
static bool deleted_flag;

static bool scan_find_inode(const struct wal_record *hdr, const u8 *payload, u64 offset) {
    (void)offset;
    if (hdr->type == RECORD_INODE) {
        const struct walfs_inode *in = (const struct walfs_inode *)payload;
        if (in->inode_id == search_inode_id) {
            simd_memcpy(&found_inode, in, sizeof(found_inode));
            found_inode_ok = true;
            deleted_flag = false;
        }
    } else if (hdr->type == RECORD_DELETE) {
        const struct walfs_delete *del = (const struct walfs_delete *)payload;
        if (del->inode_id == search_inode_id)
            deleted_flag = true;
    }
    return false; /* scan all records to find latest */
}

/* Find child inode_id by name in a directory */
static u64 find_child_id;
static const char *find_child_name;

static bool scan_find_child(const struct wal_record *hdr, const u8 *payload, u64 offset) {
    (void)offset;
    if (hdr->type == RECORD_DIRENT) {
        const struct walfs_dirent *de = (const struct walfs_dirent *)payload;
        if (de->parent_id == search_inode_id && str_eq((const char *)de->name, find_child_name))
            find_child_id = de->child_id;
    } else if (hdr->type == RECORD_DELETE) {
        const struct walfs_delete *del = (const struct walfs_delete *)payload;
        if (del->inode_id == find_child_id)
            find_child_id = 0;
    }
    return false;
}

/* ---- Public API ---- */

bool walfs_init(void) {
    mounted = false;
    next_seq = 1;
    next_inode = 2;

    if (load_super()) {
        uart_puts("[walfs] Mounted: ");
        uart_puts((const char *)super.label);
        uart_puts(" records=");
        uart_hex(super.record_count);
        uart_puts("\n");
        next_seq = super.record_count + 1;
        /* Scan to find max inode_id */
        next_inode = super.record_count + 2;
        mounted = true;
        return true;
    }

    format_disk();
    mounted = true;
    uart_puts("[walfs] Formatted and mounted\n");
    return true;
}

u64 walfs_create(u64 parent_id, const char *name, u32 flags, u32 mode) {
    if (!mounted || !valid_name(name)) return 0;

    u64 id = next_inode++;

    struct walfs_inode in;
    simd_zero(&in, sizeof(in));
    in.inode_id = id;
    in.parent_id = parent_id;
    in.flags = flags;
    in.mode = mode;
    in.created = read_cntvct();
    in.modified = in.created;
    for (u32 i = 0; name[i] && i < WALFS_NAME_MAX; i++)
        in.name[i] = (u8)name[i];

    if (!wal_append(RECORD_INODE, &in, sizeof(in)))
        return 0;

    struct walfs_dirent de;
    simd_zero(&de, sizeof(de));
    de.parent_id = parent_id;
    de.child_id = id;
    for (u32 i = 0; name[i] && i < WALFS_NAME_MAX; i++)
        de.name[i] = (u8)name[i];

    wal_append(RECORD_DIRENT, &de, sizeof(de));
    walfs_sync();
    return id;
}

bool walfs_write(u64 inode_id, u64 offset, const void *data, u32 len) {
    if (!mounted) return false;
    const u8 *src = (const u8 *)data;

    while (len > 0) {
        u32 chunk = (len > WALFS_DATA_MAX) ? WALFS_DATA_MAX : len;

        /* Build data record in rec_buf payload area */
        struct walfs_data dhdr;
        dhdr.inode_id = inode_id;
        dhdr.offset = offset;
        dhdr.length = chunk;

        /* We need to append header + data together */
        u32 pay_len = sizeof(struct walfs_data) + chunk;
        u8 pay_buf[sizeof(struct walfs_data) + WALFS_DATA_MAX];
        simd_memcpy(pay_buf, &dhdr, sizeof(dhdr));
        simd_memcpy(pay_buf + sizeof(dhdr), src, chunk);

        if (!wal_append(RECORD_DATA, pay_buf, pay_len))
            return false;

        src += chunk;
        offset += chunk;
        len -= chunk;
    }

    /* Update inode size */
    search_inode_id = inode_id;
    found_inode_ok = false;
    wal_scan(scan_find_inode);
    if (found_inode_ok) {
        if (offset > found_inode.size)
            found_inode.size = offset;
        found_inode.modified = read_cntvct();
        wal_append(RECORD_INODE, &found_inode, sizeof(found_inode));
    }

    walfs_sync();
    return true;
}

u32 walfs_read(u64 inode_id, u64 offset, void *buf, u32 len) {
    if (!mounted) return 0;
    u8 *dst = (u8 *)buf;
    u32 total = 0;

    /* Scan WAL for matching data records */
    u64 off = SD_BLOCK_SIZE;
    while (off < super.wal_head) {
        struct wal_record hdr;
        if (!wal_read(off, &hdr, sizeof(hdr))) break;
        if (hdr.magic != WALFS_REC_MAGIC) break;
        if (hdr.length < sizeof(struct wal_record)) break;

        if (hdr.type == RECORD_DATA) {
            struct walfs_data dhdr;
            if (wal_read(off + sizeof(struct wal_record), &dhdr, sizeof(dhdr))) {
                if (dhdr.inode_id == inode_id) {
                    /* Check overlap with requested range */
                    u64 rec_start = dhdr.offset;
                    u64 rec_end = rec_start + dhdr.length;
                    u64 req_end = offset + len;

                    if (rec_end > offset && rec_start < req_end) {
                        u64 copy_start = (rec_start > offset) ? rec_start : offset;
                        u64 copy_end = (rec_end < req_end) ? rec_end : req_end;
                        u32 copy_len = (u32)(copy_end - copy_start);
                        u32 src_off = (u32)(copy_start - rec_start);
                        u32 dst_off = (u32)(copy_start - offset);

                        u64 data_off = off + sizeof(struct wal_record) +
                                       sizeof(struct walfs_data) + src_off;
                        wal_read(data_off, dst + dst_off, copy_len);
                        total += copy_len;
                    }
                }
            }
        }

        off += hdr.length;
        off = (off + 3) & ~3ULL;
    }
    return total;
}

bool walfs_delete(u64 inode_id) {
    if (!mounted) return false;
    struct walfs_delete del;
    del.inode_id = inode_id;
    if (!wal_append(RECORD_DELETE, &del, sizeof(del)))
        return false;
    walfs_sync();
    return true;
}

bool walfs_stat(u64 inode_id, struct walfs_inode *out) {
    if (!mounted) return false;
    search_inode_id = inode_id;
    found_inode_ok = false;
    deleted_flag = false;
    wal_scan(scan_find_inode);
    if (!found_inode_ok || deleted_flag) return false;
    simd_memcpy(out, &found_inode, sizeof(found_inode));
    return true;
}

u64 walfs_find(const char *path) {
    if (!mounted || !path) return 0;
    if (path[0] == '/') path++;
    if (path[0] == 0) return WALFS_ROOT_INODE;

    u64 current = WALFS_ROOT_INODE;
    char component[128];

    while (*path) {
        u32 i = 0;
        while (*path && *path != '/' && i < 127)
            component[i++] = *path++;
        component[i] = 0;
        if (*path == '/') path++;

        search_inode_id = current;
        find_child_name = component;
        find_child_id = 0;
        wal_scan(scan_find_child);
        if (find_child_id == 0) return 0;
        current = find_child_id;
    }
    return current;
}

void walfs_readdir(u64 parent_id, walfs_readdir_cb cb) {
    if (!mounted || !cb) return;

    /* Collect dirents, filtering deleted */
    u64 off = SD_BLOCK_SIZE;
    while (off < super.wal_head) {
        struct wal_record hdr;
        if (!wal_read(off, &hdr, sizeof(hdr))) break;
        if (hdr.magic != WALFS_REC_MAGIC) break;
        if (hdr.length < sizeof(struct wal_record)) break;

        if (hdr.type == RECORD_DIRENT) {
            struct walfs_dirent de;
            if (wal_read(off + sizeof(struct wal_record), &de, sizeof(de))) {
                if (de.parent_id == parent_id) {
                    /* Check if child is deleted */
                    search_inode_id = de.child_id;
                    found_inode_ok = false;
                    deleted_flag = false;
                    wal_scan(scan_find_inode);
                    if (!deleted_flag)
                        cb(&de);
                }
            }
        }

        off += hdr.length;
        off = (off + 3) & ~3ULL;
    }
}

bool walfs_mmap(u64 inode_id, u64 offset, u32 length, void *dest) {
    return walfs_read(inode_id, offset, dest, length) > 0;
}

/* ---- FIFO handler for Core 1 ---- */

void walfs_handle_fifo(u32 from_core) {
    struct fifo_msg msg;
    if (!fifo_pop(CORE_DISK, from_core, &msg))
        return;

    struct fifo_msg reply = {0};
    reply.tag = msg.tag;

    switch (msg.type) {
    case MSG_FS_CREATE:
    case MSG_FS_MKDIR: {
        u32 flags = (msg.type == MSG_FS_MKDIR) ? WALFS_DIR : WALFS_FILE;
        u64 id = walfs_create(msg.param, (const char *)(usize)msg.buffer,
                              flags, 0755);
        reply.type = id ? MSG_FS_DONE : MSG_FS_ERROR;
        reply.param = (u32)id;
        reply.status = id ? 0 : 1;
        break;
    }

    case MSG_FS_WRITE: {
        bool ok = walfs_write(msg.param, msg.tag, (const void *)(usize)msg.buffer,
                              msg.length);
        reply.type = ok ? MSG_FS_DONE : MSG_FS_ERROR;
        reply.status = ok ? 0 : 1;
        break;
    }

    case MSG_FS_DELETE: {
        bool ok = walfs_delete(msg.param);
        reply.type = ok ? MSG_FS_DONE : MSG_FS_ERROR;
        reply.status = ok ? 0 : 1;
        break;
    }

    case MSG_FS_READ: {
        u32 got = walfs_read(msg.param, 0, (void *)(usize)msg.buffer, msg.length);
        reply.type = MSG_FS_SCAN_RESULT;
        reply.length = got;
        reply.status = 0;
        break;
    }

    case MSG_FS_STAT: {
        struct walfs_inode in;
        if (walfs_stat(msg.param, &in)) {
            simd_memcpy((void *)(usize)msg.buffer, &in, sizeof(in));
            reply.type = MSG_FS_SCAN_RESULT;
            reply.length = sizeof(in);
            reply.status = 0;
        } else {
            reply.type = MSG_FS_ERROR;
            reply.status = 1;
        }
        break;
    }

    case MSG_FS_FIND: {
        u64 id = walfs_find((const char *)(usize)msg.buffer);
        reply.type = id ? MSG_FS_SCAN_RESULT : MSG_FS_ERROR;
        reply.param = (u32)id;
        reply.status = id ? 0 : 1;
        break;
    }

    default:
        reply.type = MSG_FS_ERROR;
        reply.status = 1;
        break;
    }

    fifo_push(CORE_DISK, from_core, &reply);
}
