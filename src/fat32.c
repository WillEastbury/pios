/*
 * fat32.c - Read-only FAT32 driver with LFN support
 *
 * Reads partition 1 (FAT32 boot partition) on the SD card.
 * Uses sd_read_block() for raw 512-byte sector access.
 *
 * FAT32 layout:
 *   [MBR] [Reserved sectors (incl BPB)] [FAT1] [FAT2] [Data clusters]
 *
 * Cluster numbering starts at 2. Cluster N is at:
 *   data_start_lba + (N - 2) * sectors_per_cluster
 *
 * Reference: Microsoft FAT32 File System Specification (fatgen103.doc)
 */

#include "fat32.h"
#include "sd.h"
#include "uart.h"

/* ── BPB / volume state ── */

static bool fat32_ready;
static u32 part1_lba;           /* MBR partition 1 start LBA */
static u32 sectors_per_cluster;
static u32 reserved_sectors;
static u32 fat_start_lba;       /* first FAT sector (absolute) */
static u32 fat_size_sectors;    /* sectors per FAT */
static u32 data_start_lba;      /* first data sector (absolute) */
static u32 root_cluster;
static u32 total_clusters;

/* Bounce buffer for sector reads */
static u8 ALIGNED(64) fat32_buf[FAT32_SECTOR_SIZE];

/* FAT cache — one sector */
static u8 ALIGNED(64) fat_cache[FAT32_SECTOR_SIZE];
static u32 fat_cache_sector = 0xFFFFFFFF;

/* ── Helpers ── */

static u32 read_le16(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8);
}

static u32 read_le32(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

static u32 cluster_to_lba(u32 cluster)
{
    return data_start_lba + (cluster - 2) * sectors_per_cluster;
}

/* Read one sector into caller buffer */
static bool read_sector(u32 lba, u8 *buf)
{
    return sd_read_block(lba, buf);
}

/* Get next cluster from FAT, returns 0 on error/EOC */
static u32 fat_next_cluster(u32 cluster)
{
    u32 fat_offset = cluster * 4;
    u32 fat_sector = fat_start_lba + (fat_offset / FAT32_SECTOR_SIZE);
    u32 entry_off  = fat_offset % FAT32_SECTOR_SIZE;

    if (fat_sector != fat_cache_sector) {
        if (!read_sector(fat_sector, fat_cache))
            return 0;
        fat_cache_sector = fat_sector;
    }

    u32 entry = read_le32(&fat_cache[entry_off]) & 0x0FFFFFFF;

    /* EOC range or bad cluster */
    if (entry >= 0x0FFFFFF8 || entry == 0x0FFFFFF7 || entry < 2)
        return 0;
    /* Bounds check */
    if (entry > total_clusters + 1)
        return 0;

    return entry;
}

/* Case-insensitive char compare */
static char to_upper(char c)
{
    return (c >= 'a' && c <= 'z') ? c - 32 : c;
}

static bool name_eq_ci(const char *a, const char *b)
{
    while (*a && *b) {
        if (to_upper(*a) != to_upper(*b))
            return false;
        a++; b++;
    }
    return *a == *b;
}

/* ── LFN reconstruction ── */

/*
 * LFN entries precede the 8.3 entry in reverse order.
 * Each LFN entry holds up to 13 UCS-2 characters spread across
 * offsets 1-10, 14-25, 28-31 within the 32-byte entry.
 */

#define LFN_CHARS_PER_ENTRY 13

static void lfn_extract_chars(const u8 *entry, u16 *chars)
{
    /* Part 1: bytes 1-10 (5 UCS-2 chars) */
    chars[0]  = read_le16(&entry[1]);
    chars[1]  = read_le16(&entry[3]);
    chars[2]  = read_le16(&entry[5]);
    chars[3]  = read_le16(&entry[7]);
    chars[4]  = read_le16(&entry[9]);
    /* Part 2: bytes 14-25 (6 UCS-2 chars) */
    chars[5]  = read_le16(&entry[14]);
    chars[6]  = read_le16(&entry[16]);
    chars[7]  = read_le16(&entry[18]);
    chars[8]  = read_le16(&entry[20]);
    chars[9]  = read_le16(&entry[22]);
    chars[10] = read_le16(&entry[24]);
    /* Part 3: bytes 28-31 (2 UCS-2 chars) */
    chars[11] = read_le16(&entry[28]);
    chars[12] = read_le16(&entry[30]);
}

/* Build 8.3 name from directory entry, trimming trailing spaces */
static void build_short_name(const u8 *entry, char *out)
{
    u32 i, len = 0;

    /* Base name (8 chars) */
    for (i = 0; i < 8; i++) {
        if (entry[i] != ' ')
            out[len++] = entry[i];
    }

    /* Extension (3 chars) */
    if (entry[8] != ' ') {
        out[len++] = '.';
        for (i = 8; i < 11; i++) {
            if (entry[i] != ' ')
                out[len++] = entry[i];
        }
    }

    out[len] = '\0';
}

/* ── Init ── */

bool fat32_init(void)
{
    fat32_ready = false;
    fat_cache_sector = 0xFFFFFFFF;

    /* Read MBR */
    if (!read_sector(0, fat32_buf)) {
        uart_puts("[fat] MBR read fail\n");
        return false;
    }

    if (fat32_buf[510] != 0x55 || fat32_buf[511] != 0xAA) {
        uart_puts("[fat] no MBR sig\n");
        return false;
    }

    /* Partition 1 entry at offset 0x1BE */
    u8 p1_type = fat32_buf[0x1BE + 4];
    part1_lba  = read_le32(&fat32_buf[0x1BE + 8]);
    u32 p1_size = read_le32(&fat32_buf[0x1BE + 12]);

    if (part1_lba == 0 || p1_size == 0) {
        uart_puts("[fat] p1 not found\n");
        return false;
    }

    /* Accept FAT32 types: 0x0B (FAT32), 0x0C (FAT32 LBA) */
    if (p1_type != 0x0B && p1_type != 0x0C) {
        uart_puts("[fat] p1 type=");
        uart_hex(p1_type);
        uart_puts(" !FAT32\n");
        return false;
    }

    /* Read BPB (first sector of partition) */
    if (!read_sector(part1_lba, fat32_buf)) {
        uart_puts("[fat] BPB read fail\n");
        return false;
    }

    /* Validate BPB */
    u32 bps = read_le16(&fat32_buf[11]);
    if (bps != 512) {
        uart_puts("[fat] bad sectsz=");
        uart_hex(bps);
        uart_puts("\n");
        return false;
    }

    sectors_per_cluster = fat32_buf[13];
    if (sectors_per_cluster == 0 ||
        (sectors_per_cluster & (sectors_per_cluster - 1)) != 0) {
        uart_puts("[fat] bad spc=");
        uart_hex(sectors_per_cluster);
        uart_puts("\n");
        return false;
    }

    reserved_sectors = read_le16(&fat32_buf[14]);
    u32 num_fats     = fat32_buf[16];
    fat_size_sectors = read_le32(&fat32_buf[36]);
    root_cluster     = read_le32(&fat32_buf[44]);

    if (reserved_sectors == 0 || num_fats == 0 || fat_size_sectors == 0) {
        uart_puts("[fat] bad BPB\n");
        return false;
    }

    /* Root entry count must be 0 for FAT32 */
    u32 root_entry_count = read_le16(&fat32_buf[17]);
    if (root_entry_count != 0) {
        uart_puts("[fat] !FAT32 rootent!=0\n");
        return false;
    }

    /* Compute layout */
    fat_start_lba  = part1_lba + reserved_sectors;
    data_start_lba = fat_start_lba + (num_fats * fat_size_sectors);

    u32 total_sectors = read_le32(&fat32_buf[32]);
    if (total_sectors == 0)
        total_sectors = read_le16(&fat32_buf[19]);
    u32 data_sectors = total_sectors - reserved_sectors - (num_fats * fat_size_sectors);
    total_clusters = data_sectors / sectors_per_cluster;

    /* Sanity: FAT32 requires >= 65525 clusters */
    if (total_clusters < 65525) {
        uart_puts("[fat] too few cl=");
        uart_hex(total_clusters);
        uart_puts("\n");
        return false;
    }

    fat32_ready = true;

    uart_puts("[fat] p1 LBA=");
    uart_hex(part1_lba);
    uart_puts(" cl=");
    uart_hex(total_clusters);
    uart_puts(" spc=");
    uart_hex(sectors_per_cluster);
    uart_puts(" rt=");
    uart_hex(root_cluster);
    uart_puts("\n");

    return true;
}

/* ── Directory traversal ── */

/*
 * Read directory entries from a cluster chain, reconstructing LFN names.
 * Returns one entry at a time via fat32_readdir().
 */

bool fat32_opendir(const char *path, fat32_dir_t *d)
{
    if (!fat32_ready) return false;

    if (path[0] == '/' || path[0] == '\\') path++;

    /* Start at root */
    u32 cluster = root_cluster;

    /* Walk path components */
    while (*path) {
        /* Extract component */
        char comp[FAT32_MAX_NAME];
        u32 clen = 0;
        while (*path && *path != '/' && *path != '\\' && clen < FAT32_MAX_NAME - 1)
            comp[clen++] = *path++;
        comp[clen] = '\0';
        if (*path == '/' || *path == '\\') path++;
        if (clen == 0) continue;

        /* Search directory for this component */
        fat32_dir_t tmp;
        tmp.start_cluster = cluster;
        tmp.current_cluster = cluster;
        tmp.entry_offset = 0;
        tmp.done = false;

        bool found = false;
        fat32_dirent_t ent;
        while (fat32_readdir(&tmp, &ent)) {
            if (name_eq_ci(ent.name, comp) && ent.is_dir) {
                cluster = ent.cluster;
                found = true;
                break;
            }
        }
        if (!found) return false;
    }

    d->start_cluster = cluster;
    d->current_cluster = cluster;
    d->entry_offset = 0;
    d->done = false;
    return true;
}

bool fat32_readdir(fat32_dir_t *d, fat32_dirent_t *entry)
{
    if (!fat32_ready || d->done) return false;

    /* LFN accumulation buffer */
    u16 lfn_buf[LFN_CHARS_PER_ENTRY * 20]; /* max 20 LFN entries = 260 chars */
    u32 lfn_len = 0;
    bool have_lfn = false;

    u32 cluster = d->current_cluster;
    u32 offset  = d->entry_offset;
    u32 bytes_per_cluster = sectors_per_cluster * FAT32_SECTOR_SIZE;
    /* Bound traversal to prevent infinite loops on corrupt FAT */
    u32 max_entries = total_clusters * (bytes_per_cluster / 32);
    u32 entries_seen = 0;

    while (cluster >= 2 && entries_seen < max_entries) {
        /* Which sector within this cluster? */
        u32 sector_in_cluster = offset / FAT32_SECTOR_SIZE;
        u32 byte_in_sector = offset % FAT32_SECTOR_SIZE;

        if (sector_in_cluster >= sectors_per_cluster) {
            /* Move to next cluster */
            cluster = fat_next_cluster(cluster);
            if (cluster == 0) { d->done = true; return false; }
            offset = 0;
            continue;
        }

        u32 lba = cluster_to_lba(cluster) + sector_in_cluster;
        if (!read_sector(lba, fat32_buf)) {
            d->done = true;
            return false;
        }

        /* Process entries in this sector starting at byte_in_sector */
        while (byte_in_sector < FAT32_SECTOR_SIZE) {
            const u8 *e = &fat32_buf[byte_in_sector];
            entries_seen++;

            if (e[0] == 0x00) {
                /* End of directory */
                d->done = true;
                return false;
            }

            if (e[0] == 0xE5) {
                /* Deleted entry — reset LFN */
                have_lfn = false;
                lfn_len = 0;
                byte_in_sector += 32;
                offset += 32;
                continue;
            }

            u8 attr = e[11];

            if (attr == FAT32_ATTR_LFN) {
                /* LFN entry */
                u8 seq = e[0] & 0x3F;
                if (e[0] & 0x40) {
                    /* First (last logical) LFN entry — reset */
                    lfn_len = seq * LFN_CHARS_PER_ENTRY;
                    if (lfn_len > FAT32_MAX_NAME - 1)
                        lfn_len = FAT32_MAX_NAME - 1;
                    have_lfn = true;
                    /* Clear buffer */
                    for (u32 i = 0; i < lfn_len; i++)
                        lfn_buf[i] = 0xFFFF;
                }

                if (have_lfn && seq >= 1 && seq <= 20) {
                    u16 chars[LFN_CHARS_PER_ENTRY];
                    lfn_extract_chars(e, chars);
                    u32 base = (seq - 1) * LFN_CHARS_PER_ENTRY;
                    for (u32 i = 0; i < LFN_CHARS_PER_ENTRY && base + i < FAT32_MAX_NAME; i++)
                        lfn_buf[base + i] = chars[i];
                }

                byte_in_sector += 32;
                offset += 32;
                continue;
            }

            /* Skip volume label */
            if (attr & FAT32_ATTR_VOLUME_ID) {
                have_lfn = false;
                lfn_len = 0;
                byte_in_sector += 32;
                offset += 32;
                continue;
            }

            /* Regular 8.3 entry — this is the real entry */
            u32 cl_hi = read_le16(&e[20]);
            u32 cl_lo = read_le16(&e[26]);
            u32 cl    = (cl_hi << 16) | cl_lo;
            u32 size  = read_le32(&e[28]);

            entry->cluster = cl;
            entry->size = size;
            entry->attr = attr;
            entry->is_dir = (attr & FAT32_ATTR_DIRECTORY) != 0;

            if (have_lfn && lfn_len > 0) {
                /* Convert UCS-2 LFN to ASCII */
                u32 nlen = 0;
                for (u32 i = 0; i < lfn_len && nlen < FAT32_MAX_NAME - 1; i++) {
                    if (lfn_buf[i] == 0x0000 || lfn_buf[i] == 0xFFFF)
                        break;
                    entry->name[nlen++] = (char)(lfn_buf[i] & 0x7F);
                }
                entry->name[nlen] = '\0';
            } else {
                build_short_name(e, entry->name);
            }

            have_lfn = false;
            lfn_len = 0;

            /* Advance past this entry */
            byte_in_sector += 32;
            offset += 32;

            /* Save state for next call */
            d->current_cluster = cluster;
            d->entry_offset = offset;

            return true;
        }

        /* Sector exhausted, move to next sector in cluster */
        /* offset already advanced by the inner loop */
    }

    d->done = true;
    return false;
}

void fat32_closedir(fat32_dir_t *d)
{
    d->done = true;
}

/* ── File lookup — search by path ── */

static bool find_file(const char *path, u32 *out_cluster, u32 *out_size)
{
    if (!fat32_ready) return false;

    if (path[0] == '/' || path[0] == '\\') path++;

    u32 cluster = root_cluster;
    bool is_last = false;

    while (*path) {
        char comp[FAT32_MAX_NAME];
        u32 clen = 0;
        while (*path && *path != '/' && *path != '\\' && clen < FAT32_MAX_NAME - 1)
            comp[clen++] = *path++;
        comp[clen] = '\0';
        if (*path == '/' || *path == '\\') path++;
        if (clen == 0) continue;

        is_last = (*path == '\0');

        fat32_dir_t dir;
        dir.start_cluster = cluster;
        dir.current_cluster = cluster;
        dir.entry_offset = 0;
        dir.done = false;

        bool found = false;
        fat32_dirent_t ent;
        while (fat32_readdir(&dir, &ent)) {
            if (name_eq_ci(ent.name, comp)) {
                if (is_last) {
                    *out_cluster = ent.cluster;
                    *out_size = ent.size;
                    return true;
                }
                if (ent.is_dir) {
                    cluster = ent.cluster;
                    found = true;
                    break;
                }
                return false; /* not a dir but more path remains */
            }
        }
        if (!found && !is_last) return false;
    }

    return false;
}

/* ── Public file API ── */

bool fat32_exists(const char *path)
{
    u32 cl, sz;
    return find_file(path, &cl, &sz);
}

u32 fat32_file_size(const char *path)
{
    u32 cl, sz;
    if (!find_file(path, &cl, &sz)) return 0;
    return sz;
}

bool fat32_open(const char *path, fat32_file_t *f)
{
    u32 cl, sz;
    if (!find_file(path, &cl, &sz)) return false;

    f->start_cluster = cl;
    f->current_cluster = cl;
    f->file_size = sz;
    f->position = 0;
    f->cluster_offset = 0;
    return true;
}

u32 fat32_read(fat32_file_t *f, u8 *buf, u32 len)
{
    if (!fat32_ready) return 0;

    u32 remaining = f->file_size - f->position;
    if (len > remaining) len = remaining;
    if (len == 0) return 0;

    u32 bytes_per_cluster = sectors_per_cluster * FAT32_SECTOR_SIZE;
    u32 total_read = 0;

    while (total_read < len && f->current_cluster >= 2) {
        u32 sector_in_cluster = f->cluster_offset / FAT32_SECTOR_SIZE;
        u32 byte_in_sector = f->cluster_offset % FAT32_SECTOR_SIZE;

        if (f->cluster_offset >= bytes_per_cluster) {
            /* Move to next cluster */
            f->current_cluster = fat_next_cluster(f->current_cluster);
            if (f->current_cluster == 0) break;
            f->cluster_offset = 0;
            continue;
        }

        u32 lba = cluster_to_lba(f->current_cluster) + sector_in_cluster;

        /* Full sector direct read when aligned */
        if (byte_in_sector == 0 && (len - total_read) >= FAT32_SECTOR_SIZE) {
            u32 sectors_left_in_cluster = sectors_per_cluster - sector_in_cluster;
            u32 sectors_to_read = (len - total_read) / FAT32_SECTOR_SIZE;
            if (sectors_to_read > sectors_left_in_cluster)
                sectors_to_read = sectors_left_in_cluster;

            /* Also bound by file size */
            u32 bytes_left_in_file = f->file_size - f->position - total_read;
            u32 max_sectors_for_file = (bytes_left_in_file + FAT32_SECTOR_SIZE - 1) / FAT32_SECTOR_SIZE;
            if (sectors_to_read > max_sectors_for_file)
                sectors_to_read = max_sectors_for_file;

            if (sectors_to_read > 1) {
                if (!sd_read_blocks(lba, sectors_to_read, buf + total_read))
                    break;
            } else {
                if (!read_sector(lba, buf + total_read))
                    break;
            }

            u32 bytes = sectors_to_read * FAT32_SECTOR_SIZE;
            if (total_read + bytes > len) bytes = len - total_read;
            total_read += bytes;
            f->cluster_offset += bytes;
            f->position += bytes;
            continue;
        }

        /* Partial sector — use bounce buffer */
        if (!read_sector(lba, fat32_buf))
            break;

        u32 avail = FAT32_SECTOR_SIZE - byte_in_sector;
        u32 want = len - total_read;
        if (want > avail) want = avail;

        /* Don't read past end of file */
        u32 file_left = f->file_size - f->position;
        if (want > file_left) want = file_left;

        for (u32 i = 0; i < want; i++)
            buf[total_read + i] = fat32_buf[byte_in_sector + i];

        total_read += want;
        f->cluster_offset += want;
        f->position += want;
    }

    return total_read;
}

void fat32_close(fat32_file_t *f)
{
    f->current_cluster = 0;
    f->position = 0;
}
