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
#include "bcache.h"
#include "simd.h"
#include "uart.h"
#include "timer.h"
#include "fifo.h"
#include "core.h"

#define WAL_START  SD_BLOCK_SIZE
#define WAL_REC_MIN  sizeof(struct wal_record)
#define WAL_REC_MAX  (WALFS_DATA_MAX + 256)

/* Validate record header length — prevents infinite loops on corrupt data */
static inline bool wal_rec_valid(const struct wal_record *h) {
    return h->magic == WALFS_REC_MAGIC &&
           h->length >= WAL_REC_MIN &&
           h->length <= WAL_REC_MAX;
}

static struct walfs_super super;
static u64 next_inode;
static u64 next_seq;
static bool mounted;

static u8 ALIGNED(64) iobuf[SD_BLOCK_SIZE];
static u8 ALIGNED(64) rec_buf[sizeof(struct wal_record) +
                               sizeof(struct walfs_data) + WALFS_DATA_MAX];
static u32 cached_lba = 0xFFFFFFFF;

static inline u64 read_cntvct(void)
{
    u64 v;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(v));
    return v;
}

/* ---- name helpers ---- */

static bool valid_name(const char *name)
{
    if (!name || !name[0]) return false;
    for (u32 i = 0; name[i]; i++) {
        if (i >= WALFS_NAME_MAX) return false;
        u8 c = (u8)name[i];
        if (c < 0x20 || c == 0x7F || (c >= 0x80 && c <= 0x9F) || c == '/')
            return false;
    }
    return true;
}

static bool name_eq(const u8 *a, const char *b)
{
    for (u32 i = 0; i < 128; i++) {
        if (a[i] != (u8)b[i]) return false;
        if (b[i] == '\0') return true;
    }
    return true;
}

static void name_copy(u8 *dst, const char *src)
{
    u32 i = 0;
    while (i < WALFS_NAME_MAX && src[i]) { dst[i] = (u8)src[i]; i++; }
    while (i < 128) dst[i++] = 0;
}

/* ---- low-level disk I/O with single-block read cache ---- */

static bool wal_read(u64 off, void *buf, u32 len)
{
    u8 *d = (u8 *)buf;
    while (len > 0) {
        u32 lba = (u32)(off / SD_BLOCK_SIZE);
        u32 blk_off = (u32)(off % SD_BLOCK_SIZE);
        u32 n = SD_BLOCK_SIZE - blk_off;
        if (n > len) n = len;
        if (lba != cached_lba) {
            if (!bcache_read(lba, iobuf)) return false;
            cached_lba = lba;
        }
        memcpy(d, iobuf + blk_off, n);
        d += n;
        off += n;
        len -= n;
    }
    return true;
}

static bool wal_write(u64 off, const void *buf, u32 len)
{
    const u8 *s = (const u8 *)buf;
    while (len > 0) {
        u32 lba = (u32)(off / SD_BLOCK_SIZE);
        u32 blk_off = (u32)(off % SD_BLOCK_SIZE);
        u32 n = SD_BLOCK_SIZE - blk_off;
        if (n > len) n = len;
        if (blk_off != 0 || n != SD_BLOCK_SIZE) {
            if (lba != cached_lba) {
                if (!bcache_read(lba, iobuf)) return false;
            }
        }
        memcpy(iobuf + blk_off, s, n);
        if (!bcache_write(lba, iobuf)) return false;
        cached_lba = lba;
        s += n;
        off += n;
        len -= n;
    }
    return true;
}

/* ---- superblock I/O ---- */

static void write_super(void)
{
    super.crc32 = 0;
    super.crc32 = hw_crc32c(&super, SD_BLOCK_SIZE);
    cached_lba = 0xFFFFFFFF;
    bcache_write(0, (const u8 *)&super);
    bcache_flush();
}

/* ---- WAL append (two-part payload to avoid large stack buffers) ---- */

static u64 wal_append(u32 type, const void *meta, u32 meta_len,
                      const void *data, u32 data_len)
{
    /* Bounds check: prevent rec_buf overflow */
    if (meta_len + data_len > sizeof(rec_buf) - sizeof(struct wal_record))
        return 0;

    u32 total = (u32)sizeof(struct wal_record) + meta_len + data_len;
    struct wal_record *r = (struct wal_record *)rec_buf;

    r->magic     = WALFS_REC_MAGIC;
    r->type      = type;
    r->length    = total;
    r->crc32     = 0;
    r->seq       = next_seq;
    r->timestamp = read_cntvct();

    if (meta_len)
        memcpy(rec_buf + sizeof(struct wal_record), meta, meta_len);
    if (data_len)
        memcpy(rec_buf + sizeof(struct wal_record) + meta_len, data, data_len);

    r->crc32 = hw_crc32c(rec_buf, total);

    u64 pos = super.wal_head;
    if (unlikely(!wal_write(pos, rec_buf, total))) return 0;

    super.wal_head += total;
    super.record_count++;
    next_seq++;
    return pos;
}

/* ---- scan helpers ---- */

static bool is_deleted(u64 inode_id)
{
    bool del = false;
    u64 pos = WAL_START;
    struct wal_record hdr;

    while (pos < super.wal_head) {
        if (!wal_read(pos, &hdr, sizeof(hdr))) break;
        if (!wal_rec_valid(&hdr)) break;
        if (hdr.type == RECORD_DELETE) {
            struct walfs_delete d;
            if (wal_read(pos + sizeof(hdr), &d, sizeof(d)))
                if (d.inode_id == inode_id) del = true;
        } else if (hdr.type == RECORD_INODE) {
            struct walfs_inode ino;
            if (wal_read(pos + sizeof(hdr), &ino, sizeof(ino)))
                if (ino.inode_id == inode_id)
                    del = (ino.flags & WALFS_DELETED) != 0;
        }
        pos += hdr.length;
    }
    return del;
}

static void scan_recovery(void)
{
    next_inode = WALFS_ROOT_INODE + 1;
    next_seq = 0;
    u64 pos = WAL_START;
    struct wal_record hdr;

    while (pos < super.wal_head) {
        if (!wal_read(pos, &hdr, sizeof(hdr))) break;
        if (!wal_rec_valid(&hdr)) break;
        if (hdr.seq >= next_seq) next_seq = hdr.seq + 1;
        if (hdr.type == RECORD_INODE) {
            struct walfs_inode ino;
            if (wal_read(pos + sizeof(hdr), &ino, sizeof(ino)))
                if (ino.inode_id >= next_inode)
                    next_inode = ino.inode_id + 1;
        }
        pos += hdr.length;
    }
}

/* ---- init / format ---- */

static bool format_disk(void)
{
    memset(&super, 0, sizeof(super));
    super.magic        = WALFS_MAGIC;
    super.version      = WALFS_VERSION;
    super.block_size   = WALFS_BLOCK_SIZE;
    super.wal_head     = WAL_START;
    super.record_count = 0;

    const sd_card_t *card = sd_get_card_info();
    if (card) super.total_blocks = (u32)(card->capacity / SD_BLOCK_SIZE);
    memcpy(super.label, "PIOS", 4);

    next_seq   = 0;
    next_inode = WALFS_ROOT_INODE + 1;

    struct walfs_inode root;
    memset(&root, 0, sizeof(root));
    root.inode_id  = WALFS_ROOT_INODE;
    root.parent_id = 0;
    root.flags     = WALFS_DIR;
    root.mode      = 0755;
    root.created   = read_cntvct();
    root.modified  = root.created;
    root.name[0]   = '/';

    u64 rp = wal_append(RECORD_INODE, &root, sizeof(root), NULL, 0);
    if (!rp) return false;
    super.tree_root = rp;

    write_super();
    uart_puts("[walfs] formatted\n");
    return true;
}

bool walfs_init(void)
{
    mounted = false;
    cached_lba = 0xFFFFFFFF;

    if (!bcache_read(0, (u8 *)&super)) return false;

    if (super.magic == WALFS_MAGIC && super.version == WALFS_VERSION) {
        u32 saved = super.crc32;
        super.crc32 = 0;
        u32 crc = hw_crc32c(&super, SD_BLOCK_SIZE);
        super.crc32 = saved;
        if (crc != saved) {
            uart_puts("[walfs] bad superblock crc\n");
            return false;
        }
        scan_recovery();
        mounted = true;
        uart_puts("[walfs] mounted, records=");
        uart_hex(super.record_count);
        uart_puts("\n");
        return true;
    }

    if (!format_disk()) return false;
    mounted = true;
    return true;
}

/* ---- public API ---- */

u64 walfs_create(u64 parent_id, const char *name, u32 flags, u32 mode)
{
    if (!mounted || !valid_name(name)) return 0;

    struct walfs_inode parent;
    if (!walfs_stat(parent_id, &parent)) return 0;
    if (!(parent.flags & WALFS_DIR)) return 0;

    u64 id = next_inode++;
    u64 ts = read_cntvct();

    struct walfs_inode ino;
    memset(&ino, 0, sizeof(ino));
    ino.inode_id  = id;
    ino.parent_id = parent_id;
    ino.flags     = flags;
    ino.mode      = mode;
    ino.created   = ts;
    ino.modified  = ts;
    name_copy(ino.name, name);

    if (!wal_append(RECORD_INODE, &ino, sizeof(ino), NULL, 0)) {
        next_inode--;
        return 0;
    }

    struct walfs_dirent de;
    memset(&de, 0, sizeof(de));
    de.parent_id = parent_id;
    de.child_id  = id;
    name_copy(de.name, name);

    wal_append(RECORD_DIRENT, &de, sizeof(de), NULL, 0);
    write_super();
    return id;
}

bool walfs_write(u64 inode_id, u64 offset, const void *data, u32 len)
{
    if (!mounted) return false;
    const u8 *src = (const u8 *)data;
    u64 cur = offset;
    u32 rem = len;

    while (rem > 0) {
        u32 chunk = rem > WALFS_DATA_MAX ? WALFS_DATA_MAX : rem;
        struct walfs_data dh;
        dh.inode_id = inode_id;
        dh.offset   = cur;
        dh.length   = chunk;

        if (!wal_append(RECORD_DATA, &dh, sizeof(dh), src, chunk))
            return false;
        src += chunk;
        cur += chunk;
        rem -= chunk;
    }

    struct walfs_inode ino;
    if (!walfs_stat(inode_id, &ino)) return false;
    if (cur > ino.size) ino.size = cur;
    ino.modified = read_cntvct();
    if (!wal_append(RECORD_INODE, &ino, sizeof(ino), NULL, 0))
        return false;

    write_super();
    return true;
}

u32 walfs_read(u64 inode_id, u64 offset, void *buf, u32 len)
{
    if (!mounted) return 0;
    u8 *dst = (u8 *)buf;
    simd_zero(dst, len);
    u32 high = 0;
    u64 pos = WAL_START;
    struct wal_record hdr;

    while (pos < super.wal_head) {
        if (!wal_read(pos, &hdr, sizeof(hdr))) break;
        if (!wal_rec_valid(&hdr)) break;
        if (hdr.length < sizeof(struct wal_record)) break;

        if (hdr.type == RECORD_DATA) {
            struct walfs_data dh;
            if (wal_read(pos + sizeof(hdr), &dh, sizeof(dh)) &&
                dh.inode_id == inode_id) {
                u64 ds = dh.offset, de = ds + dh.length;
                u64 rs = offset,    re = offset + len;
                if (ds < re && de > rs) {
                    u64 cs = ds > rs ? ds : rs;
                    u64 ce = de < re ? de : re;
                    u32 cl = (u32)(ce - cs);
                    u32 soff = (u32)(cs - ds);
                    u32 doff = (u32)(cs - rs);
                    wal_read(pos + (u32)sizeof(hdr) + (u32)sizeof(dh) + soff,
                             dst + doff, cl);
                    if (doff + cl > high) high = doff + cl;
                }
            }
        }
        pos += hdr.length;
    }
    return high;
}

bool walfs_delete(u64 inode_id)
{
    if (!mounted || inode_id == WALFS_ROOT_INODE) return false;

    struct walfs_inode ino;
    if (!walfs_stat(inode_id, &ino)) return false;

    struct walfs_delete del;
    del.inode_id = inode_id;
    if (!wal_append(RECORD_DELETE, &del, sizeof(del), NULL, 0))
        return false;

    write_super();
    return true;
}

bool walfs_stat(u64 inode_id, struct walfs_inode *out)
{
    if (!mounted) return false;
    bool found = false, deleted = false;
    u64 pos = WAL_START;
    struct wal_record hdr;

    while (pos < super.wal_head) {
        if (!wal_read(pos, &hdr, sizeof(hdr))) break;
        if (!wal_rec_valid(&hdr)) break;

        if (hdr.type == RECORD_INODE) {
            struct walfs_inode ino;
            if (wal_read(pos + sizeof(hdr), &ino, sizeof(ino)) &&
                ino.inode_id == inode_id) {
                memcpy(out, &ino, sizeof(ino));
                found   = true;
                deleted = (ino.flags & WALFS_DELETED) != 0;
            }
        } else if (hdr.type == RECORD_DELETE) {
            struct walfs_delete d;
            if (wal_read(pos + sizeof(hdr), &d, sizeof(d)) &&
                d.inode_id == inode_id)
                deleted = true;
        }
        pos += hdr.length;
    }
    return found && !deleted;
}

u64 walfs_find(const char *path)
{
    if (!mounted || !path || path[0] != '/') return 0;
    if (path[1] == '\0') return WALFS_ROOT_INODE;

    u64 cur = WALFS_ROOT_INODE;
    const char *p = path + 1;

    while (*p) {
        char comp[128];
        u32 ci = 0;
        while (*p && *p != '/' && ci < WALFS_NAME_MAX)
            comp[ci++] = *p++;
        comp[ci] = '\0';
        if (*p == '/') p++;
        if (ci == 0) continue;

        u64 found = 0;
        u64 pos = WAL_START;
        struct wal_record hdr;
        while (pos < super.wal_head) {
            if (!wal_read(pos, &hdr, sizeof(hdr))) break;
            if (!wal_rec_valid(&hdr)) break;
            if (hdr.type == RECORD_DIRENT) {
                struct walfs_dirent de;
                if (wal_read(pos + sizeof(hdr), &de, sizeof(de)) &&
                    de.parent_id == cur && name_eq(de.name, comp))
                    found = de.child_id;
            }
            pos += hdr.length;
        }
        if (!found || is_deleted(found)) return 0;
        cur = found;
    }
    return cur;
}

void walfs_readdir(u64 parent_id, walfs_readdir_cb cb)
{
    if (!mounted || !cb) return;
    u64 pos = WAL_START;
    struct wal_record hdr;
    u64 seen[128];
    u32 nseen = 0;

    while (pos < super.wal_head) {
        if (!wal_read(pos, &hdr, sizeof(hdr))) break;
        if (!wal_rec_valid(&hdr)) break;

        if (hdr.type == RECORD_DIRENT) {
            struct walfs_dirent de;
            if (wal_read(pos + sizeof(hdr), &de, sizeof(de)) &&
                de.parent_id == parent_id && !is_deleted(de.child_id)) {
                bool dup = false;
                for (u32 i = 0; i < nseen; i++)
                    if (seen[i] == de.child_id) { dup = true; break; }
                if (!dup) {
                    if (nseen < 128) seen[nseen++] = de.child_id;
                    cb(&de);
                }
            }
        }
        pos += hdr.length;
    }
}

bool walfs_mmap(u64 inode_id, u64 offset, u32 length, void *dest)
{
    return walfs_read(inode_id, offset, dest, length) > 0;
}

/* ---- FIFO handler for Core 1 ---- */

void walfs_handle_fifo(u32 from_core)
{
    struct fifo_msg msg;
    if (!fifo_pop(CORE_DISK, from_core, &msg)) return;

    struct fifo_msg reply;
    memset(&reply, 0, sizeof(reply));
    reply.tag = msg.tag;

    switch (msg.type) {
    case MSG_FS_CREATE: {
        u64 id = walfs_create(msg.tag, (const char *)(usize)msg.buffer,
                              WALFS_FILE, msg.param);
        reply.type = id ? MSG_FS_DONE : MSG_FS_ERROR;
        reply.tag  = id;
        break;
    }
    case MSG_FS_MKDIR: {
        u64 id = walfs_create(msg.tag, (const char *)(usize)msg.buffer,
                              WALFS_DIR, msg.param);
        reply.type = id ? MSG_FS_DONE : MSG_FS_ERROR;
        reply.tag  = id;
        break;
    }
    case MSG_FS_WRITE: {
        bool ok = walfs_write(msg.tag, (u64)msg.param,
                              (const void *)(usize)msg.buffer, msg.length);
        reply.type = ok ? MSG_FS_DONE : MSG_FS_ERROR;
        break;
    }
    case MSG_FS_READ: {
        u32 n = walfs_read(msg.tag, (u64)msg.param,
                           (void *)(usize)msg.buffer, msg.length);
        reply.type   = MSG_FS_DONE;
        reply.length = n;
        break;
    }
    case MSG_FS_DELETE: {
        bool ok = walfs_delete(msg.tag);
        reply.type = ok ? MSG_FS_DONE : MSG_FS_ERROR;
        break;
    }
    case MSG_FS_STAT: {
        bool ok = walfs_stat(msg.tag,
                             (struct walfs_inode *)(usize)msg.buffer);
        reply.type = ok ? MSG_FS_DONE : MSG_FS_ERROR;
        break;
    }
    case MSG_FS_FIND: {
        u64 id = walfs_find((const char *)(usize)msg.buffer);
        reply.type = id ? MSG_FS_DONE : MSG_FS_ERROR;
        reply.tag  = id;
        break;
    }
    case MSG_FS_READDIR:
        walfs_readdir(msg.tag, (walfs_readdir_cb)(usize)msg.buffer);
        reply.type = MSG_FS_DONE;
        break;
    default:
        reply.type = MSG_FS_ERROR;
        break;
    }

    fifo_push(CORE_DISK, from_core, &reply);
}

void walfs_sync(void)
{
    write_super();
}
