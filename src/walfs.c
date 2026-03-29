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
#include "lru.h"
#include "principal.h"

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

/* Transaction recovery: seq range of an uncommitted TX_BEGIN (0 = none) */
static u64 uncommitted_tx_start;
static u64 uncommitted_tx_end;

static inline bool is_uncommitted(u64 seq) {
    return uncommitted_tx_start != 0 &&
           seq >= uncommitted_tx_start && seq <= uncommitted_tx_end;
}

static u8 ALIGNED(64) iobuf[SD_BLOCK_SIZE];
static u8 ALIGNED(64) rec_buf[sizeof(struct wal_record) +
                               sizeof(struct walfs_data) + WALFS_DATA_MAX];
static u32 cached_lba = 0xFFFFFFFF;

/*
 * Inode cache: inode_id → latest WAL byte offset of its RECORD_INODE.
 * Avoids full WAL scan on walfs_stat() and is_deleted().
 * Populated during scan_recovery() at mount and on every write.
 */
static struct lru_cache inode_cache;

/*
 * Path cache: hash(path) → inode_id.
 * Avoids repeated WAL scans for walfs_find().
 */
static struct lru_cache path_cache;

/*
 * Append-ordered RECORD_DATA index for faster reads.
 * Falls back to linear scan if index capacity is exceeded.
 */
#define WAL_DATA_INDEX_MAX 2048
struct wal_data_index_entry {
    u64 inode_id;
    u64 data_off;
    u32 data_len;
    u64 payload_pos;
    u64 seq;
};
static struct wal_data_index_entry data_index[WAL_DATA_INDEX_MAX];
static u32 data_index_count;
static bool data_index_overflow;

static void dindex_reset(void)
{
    data_index_count = 0;
    data_index_overflow = false;
}

static void dindex_add(u64 inode_id, u64 data_off, u32 data_len, u64 payload_pos, u64 seq)
{
    if (data_index_overflow) return;
    if (data_index_count >= WAL_DATA_INDEX_MAX) {
        data_index_overflow = true;
        return;
    }
    struct wal_data_index_entry *e = &data_index[data_index_count++];
    e->inode_id = inode_id;
    e->data_off = data_off;
    e->data_len = data_len;
    e->payload_pos = payload_pos;
    e->seq = seq;
}

/* Cache helpers */
static void icache_put(u64 inode_id, u64 wal_offset) {
    lru_put(&inode_cache, (const u8 *)&inode_id, sizeof(inode_id),
            &wal_offset, sizeof(wal_offset));
}

static bool icache_get(u64 inode_id, u64 *wal_offset) {
    struct lru_entry *e = lru_get(&inode_cache, (const u8 *)&inode_id, sizeof(inode_id));
    if (!e) return false;
    *wal_offset = *(u64 *)e->value;
    return true;
}

static u32 path_hash(const char *path) {
    u32 h = 5381;
    while (*path) h = ((h << 5) + h) + (u8)*path++;
    return h;
}

static void pcache_put(const char *path, u64 inode_id) {
    u32 h = path_hash(path);
    lru_put(&path_cache, (const u8 *)&h, sizeof(h), &inode_id, sizeof(inode_id));
}

static bool pcache_get(const char *path, u64 *inode_id) {
    u32 h = path_hash(path);
    struct lru_entry *e = lru_get(&path_cache, (const u8 *)&h, sizeof(h));
    if (!e) return false;
    *inode_id = *(u64 *)e->value;
    return true;
}

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
    /* Block path traversal: reject '.' and '..' as names */
    if (name[0] == '.' && (name[1] == '\0' || (name[1] == '.' && name[2] == '\0')))
        return false;
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
        simd_memcpy(d, iobuf + blk_off, n);
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
        simd_memcpy(iobuf + blk_off, s, n);
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
    /* CRITICAL: flush WAL data blocks to SD BEFORE updating superblock.
     * This ensures wal_head never points past actually-written data.
     * Without this ordering, power loss after superblock write but before
     * WAL data write causes unrecoverable data loss. */
    bcache_flush();

    super.crc32 = 0;
    super.crc32 = hw_crc32c(&super, SD_BLOCK_SIZE);
    cached_lba = 0xFFFFFFFF;

    /* Write superblock directly to SD, bypassing cache, to ensure
     * it hits disk AFTER all WAL data blocks are persisted. */
    sd_write_block(0, (const u8 *)&super);
    bcache_invalidate(0); /* keep cache consistent */
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
        simd_memcpy(rec_buf + sizeof(struct wal_record), meta, meta_len);
    if (data_len)
        simd_memcpy(rec_buf + sizeof(struct wal_record) + meta_len, data, data_len);

    r->crc32 = hw_crc32c(rec_buf, total);

    u64 pos = super.wal_head;
    if (unlikely(!wal_write(pos, rec_buf, total))) return 0;

    super.wal_head += total;
    super.record_count++;
    next_seq++;

    if (type == RECORD_DATA && meta_len >= sizeof(struct walfs_data)) {
        const struct walfs_data *dh = (const struct walfs_data *)meta;
        if (dh->length == data_len) {
            u64 payload_pos = pos + sizeof(struct wal_record) + sizeof(struct walfs_data);
            dindex_add(dh->inode_id, dh->offset, dh->length, payload_pos, r->seq);
        }
    }
    return pos;
}

/* ---- scan helpers ---- */

static bool is_deleted(u64 inode_id)
{
    /* Fast path: check inode_cache for last known record position */
    u64 cached_pos;
    if (icache_get(inode_id, &cached_pos)) {
        struct wal_record hdr;
        if (wal_read(cached_pos, &hdr, sizeof(hdr)) && wal_rec_valid(&hdr) &&
            !is_uncommitted(hdr.seq)) {
            if (hdr.type == RECORD_DELETE) return true;
            if (hdr.type == RECORD_INODE) {
                struct walfs_inode ino;
                if (wal_read(cached_pos + sizeof(hdr), &ino, sizeof(ino)))
                    if (ino.inode_id == inode_id)
                        return (ino.flags & WALFS_DELETED) != 0;
            }
        }
    }

    /* Slow path: full scan, updating cache as we go */
    bool del = false;
    u64 pos = WAL_START;
    u64 last_pos = 0;
    struct wal_record hdr;

    while (pos < super.wal_head) {
        if (!wal_read(pos, &hdr, sizeof(hdr))) break;
        if (!wal_rec_valid(&hdr)) break;
        if (is_uncommitted(hdr.seq)) { pos += hdr.length; continue; }
        if (hdr.type == RECORD_DELETE) {
            struct walfs_delete d;
            if (wal_read(pos + sizeof(hdr), &d, sizeof(d)))
                if (d.inode_id == inode_id) { del = true; last_pos = pos; }
        } else if (hdr.type == RECORD_INODE) {
            struct walfs_inode ino;
            if (wal_read(pos + sizeof(hdr), &ino, sizeof(ino)))
                if (ino.inode_id == inode_id) {
                    del = (ino.flags & WALFS_DELETED) != 0;
                    last_pos = pos;
                }
        }
        pos += hdr.length;
    }
    if (last_pos) icache_put(inode_id, last_pos);
    return del;
}

static void scan_recovery(void)
{
    dindex_reset();
    next_inode = WALFS_ROOT_INODE + 1;
    next_seq = 0;
    u64 pos = WAL_START;
    u64 last_valid_pos = WAL_START;
    struct wal_record hdr;
    u64 open_tx_id = 0, open_tx_seq = 0;

    /* Scan from start, ignoring superblock's wal_head — trust the data,
     * not the metadata. Stop at first record that fails CRC. */
    while (pos + sizeof(hdr) <= super.wal_head + WAL_REC_MAX) {
        if (!wal_read(pos, &hdr, sizeof(hdr))) break;
        if (!wal_rec_valid(&hdr)) break;

        /* CRC validation: read full record and verify */
        if (hdr.length > sizeof(rec_buf)) break;
        if (!wal_read(pos, rec_buf, hdr.length)) break;
        u32 stored_crc = ((struct wal_record *)rec_buf)->crc32;
        ((struct wal_record *)rec_buf)->crc32 = 0;
        u32 calc_crc = hw_crc32c(rec_buf, hdr.length);
        ((struct wal_record *)rec_buf)->crc32 = stored_crc;
        if (calc_crc != stored_crc) break; /* corrupt record — stop */

        if (hdr.seq >= next_seq) next_seq = hdr.seq + 1;
        if (hdr.type == RECORD_INODE) {
            struct walfs_inode *ino = (struct walfs_inode *)(rec_buf + sizeof(hdr));
            if (ino->inode_id >= next_inode)
                next_inode = ino->inode_id + 1;
            icache_put(ino->inode_id, pos);
        } else if (hdr.type == RECORD_DELETE) {
            struct walfs_delete *d = (struct walfs_delete *)(rec_buf + sizeof(hdr));
            icache_put(d->inode_id, pos);
        } else if (hdr.type == RECORD_DATA) {
            struct walfs_data *dh = (struct walfs_data *)(rec_buf + sizeof(hdr));
            if (dh->length <= WALFS_DATA_MAX &&
                (u32)sizeof(hdr) + (u32)sizeof(*dh) + dh->length <= hdr.length) {
                u64 payload_pos = pos + sizeof(hdr) + sizeof(*dh);
                dindex_add(dh->inode_id, dh->offset, dh->length, payload_pos, hdr.seq);
            }
        } else if (hdr.type == RECORD_TX_BEGIN) {
            u64 tx_id;
            memcpy(&tx_id, rec_buf + sizeof(hdr), sizeof(tx_id));
            open_tx_id = tx_id;
            open_tx_seq = hdr.seq;
        } else if (hdr.type == RECORD_TX_COMMIT) {
            u64 tx_id;
            memcpy(&tx_id, rec_buf + sizeof(hdr), sizeof(tx_id));
            if (tx_id == open_tx_id) {
                open_tx_id = 0;
                open_tx_seq = 0;
            }
        }
        last_valid_pos = pos + hdr.length;
        pos += hdr.length;
    }

    /* Track uncommitted transaction from recovery */
    if (open_tx_id != 0) {
        uncommitted_tx_start = open_tx_seq;
        uncommitted_tx_end = next_seq > 0 ? next_seq - 1 : 0;
    } else {
        uncommitted_tx_start = 0;
        uncommitted_tx_end = 0;
    }

    /* Repair wal_head if it's inconsistent with actual data */
    if (last_valid_pos != super.wal_head) {
        uart_puts("[walfs] Repaired wal_head: ");
        uart_hex((u32)(super.wal_head >> 32));
        uart_hex((u32)super.wal_head);
        uart_puts(" -> ");
        uart_hex((u32)(last_valid_pos >> 32));
        uart_hex((u32)last_valid_pos);
        uart_puts("\n");
        super.wal_head = last_valid_pos;
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
    dindex_reset();
    lru_init(&inode_cache, NULL, 0);  /* no TTL — invalidated on write */
    lru_init(&path_cache, NULL, 30000); /* 30s TTL for path lookups */

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
        /* Validate wal_head is within sane bounds */
        if (super.wal_head < WAL_START ||
            (super.total_blocks > 0 &&
             super.wal_head > (u64)super.total_blocks * SD_BLOCK_SIZE)) {
            uart_puts("[walfs] wal_head out of bounds\n");
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
    u64 tx_id = next_seq;

    /* Transaction: BEGIN → INODE → DIRENT → COMMIT */
    if (!wal_append(RECORD_TX_BEGIN, &tx_id, sizeof(tx_id), NULL, 0)) {
        next_inode--;
        return 0;
    }

    struct walfs_inode ino;
    memset(&ino, 0, sizeof(ino));
    ino.inode_id  = id;
    ino.parent_id = parent_id;
    ino.flags     = flags;
    ino.mode      = mode;
    ino.created   = ts;
    ino.modified  = ts;
    name_copy(ino.name, name);

    u64 ino_pos = wal_append(RECORD_INODE, &ino, sizeof(ino), NULL, 0);
    if (!ino_pos) {
        next_inode--;
        return 0;
    }
    icache_put(id, ino_pos);

    struct walfs_dirent de;
    memset(&de, 0, sizeof(de));
    de.parent_id = parent_id;
    de.child_id  = id;
    name_copy(de.name, name);

    wal_append(RECORD_DIRENT, &de, sizeof(de), NULL, 0);
    wal_append(RECORD_TX_COMMIT, &tx_id, sizeof(tx_id), NULL, 0);
    lru_flush(&path_cache);
    write_super();
    return id;
}

bool walfs_write(u64 inode_id, u64 offset, const void *data, u32 len)
{
    if (!mounted) return false;
    const u8 *src = (const u8 *)data;
    u64 cur = offset;
    u32 rem = len;

    /* Begin transaction */
    u64 tx_id = next_seq;
    if (!wal_append(RECORD_TX_BEGIN, &tx_id, sizeof(tx_id), NULL, 0))
        return false;

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
    u64 ino_pos = wal_append(RECORD_INODE, &ino, sizeof(ino), NULL, 0);
    if (!ino_pos) return false;
    icache_put(inode_id, ino_pos);

    /* Commit transaction */
    wal_append(RECORD_TX_COMMIT, &tx_id, sizeof(tx_id), NULL, 0);
    write_super();
    return true;
}

static u32 walfs_read_linear(u64 inode_id, u64 offset, void *buf, u32 len)
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
        if (is_uncommitted(hdr.seq)) { pos += hdr.length; continue; }

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

u32 walfs_read(u64 inode_id, u64 offset, void *buf, u32 len)
{
    if (!mounted) return 0;
    if (len == 0) return 0;
    if (data_index_overflow || data_index_count == 0)
        return walfs_read_linear(inode_id, offset, buf, len);

    u8 *dst = (u8 *)buf;
    simd_zero(dst, len);
    u32 high = 0;
    u64 rs = offset, re = offset + len;

    for (u32 i = 0; i < data_index_count; i++) {
        const struct wal_data_index_entry *e = &data_index[i];
        if (e->inode_id != inode_id) continue;
        if (is_uncommitted(e->seq)) continue;
        u64 ds = e->data_off;
        u64 de = ds + e->data_len;
        if (ds < re && de > rs) {
            u64 cs = ds > rs ? ds : rs;
            u64 ce = de < re ? de : re;
            u32 cl = (u32)(ce - cs);
            u32 soff = (u32)(cs - ds);
            u32 doff = (u32)(cs - rs);
            wal_read(e->payload_pos + soff, dst + doff, cl);
            if (doff + cl > high) high = doff + cl;
        }
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
    u64 del_pos = wal_append(RECORD_DELETE, &del, sizeof(del), NULL, 0);
    if (!del_pos) return false;

    /* Update cache to point to delete record */
    icache_put(inode_id, del_pos);
    lru_flush(&path_cache);

    write_super();
    return true;
}

bool walfs_stat(u64 inode_id, struct walfs_inode *out)
{
    if (!mounted) return false;

    /* Fast path: check inode_cache for cached WAL position */
    u64 cached_pos;
    if (icache_get(inode_id, &cached_pos)) {
        struct wal_record hdr;
        if (wal_read(cached_pos, &hdr, sizeof(hdr)) && wal_rec_valid(&hdr) &&
            !is_uncommitted(hdr.seq)) {
            if (hdr.type == RECORD_DELETE) return false;
            if (hdr.type == RECORD_INODE) {
                struct walfs_inode ino;
                if (wal_read(cached_pos + sizeof(hdr), &ino, sizeof(ino)) &&
                    ino.inode_id == inode_id && !(ino.flags & WALFS_DELETED)) {
                    simd_memcpy(out, &ino, sizeof(ino));
                    return true;
                }
            }
        }
    }

    /* Slow path: full scan, updating cache */
    bool found = false, deleted = false;
    u64 pos = WAL_START;
    u64 last_pos = 0;
    struct wal_record hdr;

    while (pos < super.wal_head) {
        if (!wal_read(pos, &hdr, sizeof(hdr))) break;
        if (!wal_rec_valid(&hdr)) break;
        if (is_uncommitted(hdr.seq)) { pos += hdr.length; continue; }

        if (hdr.type == RECORD_INODE) {
            struct walfs_inode ino;
            if (wal_read(pos + sizeof(hdr), &ino, sizeof(ino)) &&
                ino.inode_id == inode_id) {
                simd_memcpy(out, &ino, sizeof(ino));
                found   = true;
                deleted = (ino.flags & WALFS_DELETED) != 0;
                last_pos = pos;
            }
        } else if (hdr.type == RECORD_DELETE) {
            struct walfs_delete d;
            if (wal_read(pos + sizeof(hdr), &d, sizeof(d)) &&
                d.inode_id == inode_id) {
                deleted = true;
                last_pos = pos;
            }
        }
        pos += hdr.length;
    }
    if (last_pos) icache_put(inode_id, last_pos);
    return found && !deleted;
}

u64 walfs_find(const char *path)
{
    if (!mounted || !path || path[0] != '/') return 0;
    if (path[1] == '\0') return WALFS_ROOT_INODE;

    /* Fast path: check path cache */
    u64 cached_id;
    if (pcache_get(path, &cached_id) && cached_id != 0)
        return cached_id;

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
        /* Block path traversal */
        if (comp[0] == '.' && (ci == 1 || (ci == 2 && comp[1] == '.')))
            return 0;

        u64 found = 0;
        u64 pos = WAL_START;
        struct wal_record hdr;
        while (pos < super.wal_head) {
            if (!wal_read(pos, &hdr, sizeof(hdr))) break;
            if (!wal_rec_valid(&hdr)) break;
            if (is_uncommitted(hdr.seq)) { pos += hdr.length; continue; }
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

    /* Cache the result */
    pcache_put(path, cur);
    return cur;
}

void walfs_readdir(u64 parent_id, walfs_readdir_cb cb)
{
    if (!mounted || !cb) return;

    /* Single-pass O(N): collect dirents + deleted set in one WAL scan */
    #define RDIR_MAX 64
    static struct walfs_dirent dir_buf[RDIR_MAX];
    static u64 del_set[RDIR_MAX];
    u32 dir_count = 0, del_count = 0;

    u64 pos = WAL_START;
    struct wal_record hdr;
    while (pos < super.wal_head) {
        if (!wal_read(pos, &hdr, sizeof(hdr))) break;
        if (!wal_rec_valid(&hdr)) break;
        if (is_uncommitted(hdr.seq)) { pos += hdr.length; continue; }

        if (hdr.type == RECORD_DIRENT && dir_count < RDIR_MAX) {
            struct walfs_dirent de;
            if (wal_read(pos + sizeof(hdr), &de, sizeof(de)) &&
                de.parent_id == parent_id) {
                /* Deduplicate: replace existing entry for same child_id */
                bool dup = false;
                for (u32 i = 0; i < dir_count; i++) {
                    if (dir_buf[i].child_id == de.child_id) {
                        dir_buf[i] = de;
                        dup = true;
                        break;
                    }
                }
                if (!dup)
                    dir_buf[dir_count++] = de;
            }
        } else if (hdr.type == RECORD_DELETE) {
            struct walfs_delete d;
            if (wal_read(pos + sizeof(hdr), &d, sizeof(d)) &&
                del_count < RDIR_MAX)
                del_set[del_count++] = d.inode_id;
        } else if (hdr.type == RECORD_INODE) {
            struct walfs_inode ino;
            if (wal_read(pos + sizeof(hdr), &ino, sizeof(ino)) &&
                (ino.flags & WALFS_DELETED) && del_count < RDIR_MAX)
                del_set[del_count++] = ino.inode_id;
        }
        pos += hdr.length;
    }

    /* Emit non-deleted entries */
    for (u32 i = 0; i < dir_count; i++) {
        bool deleted = false;
        for (u32 j = 0; j < del_count; j++) {
            if (del_set[j] == dir_buf[i].child_id) { deleted = true; break; }
        }
        if (!deleted)
            cb(&dir_buf[i]);
    }
    #undef RDIR_MAX
}

bool walfs_mmap(u64 inode_id, u64 offset, u32 length, void *dest)
{
    return walfs_read(inode_id, offset, dest, length) > 0;
}

bool walfs_compact(void)
{
    if (!mounted) return false;
    uart_puts("[walfs] Compacting WAL...\n");

    /* Phase 1: Scan WAL, determine live inodes + latest RECORD_INODE pos */
    #define COMPACT_MAX 256
    static u64 cmp_ids[COMPACT_MAX];
    static u64 cmp_ipos[COMPACT_MAX]; /* position of LATEST RECORD_INODE */
    static u8  cmp_dead[COMPACT_MAX];
    u32 cmp_n = 0;

    u64 pos = WAL_START;
    struct wal_record hdr;
    while (pos < super.wal_head) {
        if (!wal_read(pos, &hdr, sizeof(hdr))) break;
        if (!wal_rec_valid(&hdr)) break;

        if (hdr.type == RECORD_INODE) {
            struct walfs_inode ino;
            if (wal_read(pos + sizeof(hdr), &ino, sizeof(ino))) {
                u32 idx = cmp_n;
                for (u32 i = 0; i < cmp_n; i++)
                    if (cmp_ids[i] == ino.inode_id) { idx = i; break; }
                if (idx == cmp_n && cmp_n < COMPACT_MAX) {
                    cmp_ids[cmp_n] = ino.inode_id;
                    cmp_dead[cmp_n] = 0;
                    cmp_ipos[cmp_n] = pos;
                    cmp_n++;
                }
                if (idx < cmp_n) {
                    cmp_ipos[idx] = pos; /* always update to latest */
                    cmp_dead[idx] = (ino.flags & WALFS_DELETED) ? 1 : 0;
                }
            }
        } else if (hdr.type == RECORD_DELETE) {
            struct walfs_delete d;
            if (wal_read(pos + sizeof(hdr), &d, sizeof(d))) {
                for (u32 i = 0; i < cmp_n; i++)
                    if (cmp_ids[i] == d.inode_id) { cmp_dead[i] = 1; break; }
            }
        }
        pos += hdr.length;
    }

    /* Build live set: remove dead entries */
    u32 live_count = 0;
    for (u32 i = 0; i < cmp_n; i++)
        if (!cmp_dead[i]) { cmp_ids[live_count] = cmp_ids[i];
                            cmp_ipos[live_count] = cmp_ipos[i];
                            live_count++; }

    /* Phase 2: Reset WAL and re-append only live records */
    u64 old_head = super.wal_head;
    super.wal_head = WAL_START;
    super.record_count = 0;
    super.tree_root = 0;
    next_seq = 0;
    cached_lba = 0xFFFFFFFF;
    dindex_reset();

    pos = WAL_START;
    while (pos < old_head) {
        cached_lba = 0xFFFFFFFF; /* force disk reads (avoid stale cache) */
        if (!wal_read(pos, &hdr, sizeof(hdr))) break;
        if (!wal_rec_valid(&hdr)) break;
        if (hdr.length > sizeof(rec_buf)) { pos += hdr.length; continue; }
        if (!wal_read(pos, rec_buf, hdr.length)) break;

        bool keep = false;
        if (hdr.type == RECORD_INODE) {
            struct walfs_inode *ino = (struct walfs_inode *)(rec_buf + sizeof(hdr));
            for (u32 i = 0; i < live_count; i++) {
                if (cmp_ids[i] == ino->inode_id && cmp_ipos[i] == pos) {
                    keep = true;
                    break;
                }
            }
        } else if (hdr.type == RECORD_DIRENT) {
            struct walfs_dirent *de = (struct walfs_dirent *)(rec_buf + sizeof(hdr));
            bool parent_live = false, child_live = false;
            for (u32 i = 0; i < live_count; i++) {
                if (cmp_ids[i] == de->parent_id) parent_live = true;
                if (cmp_ids[i] == de->child_id)  child_live = true;
            }
            keep = parent_live && child_live;
        } else if (hdr.type == RECORD_DATA) {
            struct walfs_data *d = (struct walfs_data *)(rec_buf + sizeof(hdr));
            for (u32 i = 0; i < live_count; i++)
                if (cmp_ids[i] == d->inode_id) { keep = true; break; }
        }
        /* TX markers and DELETE records are not re-written */

        if (keep) {
            /* Assign fresh seq and recompute CRC */
            struct wal_record *rh = (struct wal_record *)rec_buf;
            rh->seq  = next_seq;
            rh->crc32 = 0;
            rh->crc32 = hw_crc32c(rec_buf, hdr.length);

            wal_write(super.wal_head, rec_buf, hdr.length);

            if (hdr.type == RECORD_INODE) {
                struct walfs_inode *ino = (struct walfs_inode *)(rec_buf + sizeof(hdr));
                if (ino->inode_id == WALFS_ROOT_INODE)
                    super.tree_root = super.wal_head;
                icache_put(ino->inode_id, super.wal_head);
            } else if (hdr.type == RECORD_DATA) {
                struct walfs_data *dh = (struct walfs_data *)(rec_buf + sizeof(hdr));
                if (dh->length <= WALFS_DATA_MAX &&
                    (u32)sizeof(hdr) + (u32)sizeof(*dh) + dh->length <= hdr.length) {
                    u64 payload_pos = super.wal_head + sizeof(hdr) + sizeof(*dh);
                    dindex_add(dh->inode_id, dh->offset, dh->length, payload_pos, next_seq);
                }
            }
            super.wal_head += hdr.length;
            super.record_count++;
            next_seq++;
        }

        pos += hdr.length;
    }

    /* Reset transaction state and flush caches */
    uncommitted_tx_start = 0;
    uncommitted_tx_end = 0;
    cached_lba = 0xFFFFFFFF;
    lru_flush(&path_cache);
    write_super();

    uart_puts("[walfs] Compacted: ");
    uart_hex(super.record_count);
    uart_puts(" records, head=");
    uart_hex((u32)super.wal_head);
    uart_puts("\n");
    return true;
    #undef COMPACT_MAX
}

/* ---- FIFO handler for Core 1 ---- */

void walfs_handle_fifo(u32 from_core)
{
    struct fifo_msg msg;
    if (!fifo_pop(CORE_DISK, from_core, &msg)) return;

    struct fifo_msg reply;
    memset(&reply, 0, sizeof(reply));
    reply.tag = msg.tag;

    /* Permission checks: derive principal from requesting core.
     * Core 2 = CORE_USER0 (principal slot 0), Core 3 = CORE_USER1 (slot 1). */
    u32 pid = principal_current_for(from_core);

    switch (msg.type) {
    case MSG_FS_CREATE:
    case MSG_FS_MKDIR: {
        if (!principal_has_cap(pid, PRINCIPAL_DISK)) {
            reply.type = MSG_FS_ERROR;
            reply.status = 1;
            break;
        }
        if (!msg.buffer) {
            reply.type = MSG_FS_ERROR;
            reply.status = 1;
            break;
        }
        u32 flags = (msg.type == MSG_FS_MKDIR) ? WALFS_DIR : WALFS_FILE;
        u32 mode = (u32)(msg.tag & 0xFFFFFFFFU);
        u64 id = walfs_create((u64)msg.param, (const char *)(usize)msg.buffer, flags, mode);
        reply.type = id ? MSG_FS_DONE : MSG_FS_ERROR;
        reply.status = id ? 0 : 1;
        reply.param = (u32)id;
        reply.tag = id;
        break;
    }
    case MSG_FS_WRITE: {
        if (!principal_has_cap(pid, PRINCIPAL_DISK)) {
            reply.type = MSG_FS_ERROR;
            reply.status = 1;
            break;
        }
        if (!msg.buffer || msg.param == 0) {
            reply.type = MSG_FS_ERROR;
            reply.status = 1;
            break;
        }
        bool ok = walfs_write((u64)msg.param, msg.tag,
                              (const void *)(usize)msg.buffer, msg.length);
        reply.type = ok ? MSG_FS_DONE : MSG_FS_ERROR;
        reply.status = ok ? 0 : 1;
        reply.length = ok ? msg.length : 0;
        break;
    }
    case MSG_FS_READ: {
        if (!msg.buffer || msg.param == 0) {
            reply.type = MSG_FS_ERROR;
            reply.status = 1;
            break;
        }
        u32 n = walfs_read((u64)msg.param, msg.tag,
                           (void *)(usize)msg.buffer, msg.length);
        reply.type   = MSG_FS_DONE;
        reply.status = 0;
        reply.length = n;
        break;
    }
    case MSG_FS_DELETE: {
        if (!principal_has_cap(pid, PRINCIPAL_DISK)) {
            reply.type = MSG_FS_ERROR;
            reply.status = 1;
            break;
        }
        u64 inode_id = msg.tag;
        if (msg.buffer)
            inode_id = walfs_find((const char *)(usize)msg.buffer);
        bool ok = inode_id && walfs_delete(inode_id);
        reply.type = ok ? MSG_FS_DONE : MSG_FS_ERROR;
        reply.status = ok ? 0 : 1;
        break;
    }
    case MSG_FS_STAT: {
        bool ok = false;
        if (msg.buffer && msg.length == sizeof(struct walfs_inode)) {
            ok = walfs_stat(msg.tag, (struct walfs_inode *)(usize)msg.buffer);
        } else if (msg.buffer && msg.tag) {
            u64 inode_id = walfs_find((const char *)(usize)msg.buffer);
            if (inode_id)
                ok = walfs_stat(inode_id, (struct walfs_inode *)(usize)msg.tag);
        }
        reply.type = ok ? MSG_FS_DONE : MSG_FS_ERROR;
        reply.status = ok ? 0 : 1;
        break;
    }
    case MSG_FS_FIND: {
        if (!msg.buffer) {
            reply.type = MSG_FS_ERROR;
            reply.status = 1;
            break;
        }
        u64 id = walfs_find((const char *)(usize)msg.buffer);
        reply.type = id ? MSG_FS_DONE : MSG_FS_ERROR;
        reply.status = id ? 0 : 1;
        reply.param = (u32)id;
        reply.tag = id;
        break;
    }
    case MSG_FS_SYNC: {
        if (!principal_has_cap(pid, PRINCIPAL_DISK)) {
            reply.type = MSG_FS_ERROR;
            reply.status = 1;
            break;
        }
        walfs_sync();
        reply.type = MSG_FS_DONE;
        reply.status = 0;
        break;
    }
    case MSG_FS_READDIR:
        /* msg.buffer is a user-space data buffer, NOT a function pointer.
         * We cannot call it as a callback. Instead, do nothing — readdir
         * from userland should use the proc.c kernel API path which handles
         * serialization safely via FIFO reply messages. */
        reply.type = MSG_FS_ERROR;
        reply.status = 1;
        break;
    default:
        reply.type = MSG_FS_ERROR;
        reply.status = 1;
        break;
    }

    fifo_push(CORE_DISK, from_core, &reply);
}

void walfs_sync(void)
{
    write_super();
}
