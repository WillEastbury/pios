/*
 * fat32.h - Read-only FAT32 driver with LFN support
 *
 * Reads from partition 1 (the FAT32 boot partition) on the SD card.
 * Standalone — does not integrate with walfs VFS.
 * Uses sd_read_block() for raw sector access.
 *
 * Supports:
 *   - FAT32 with standard BPB
 *   - Long File Names (VFAT LFN entries)
 *   - 8.3 short filenames (case-insensitive)
 *   - Subdirectory traversal
 *   - Sequential file reading
 *
 * Does NOT support:
 *   - Writing
 *   - FAT12/FAT16
 *   - Sector sizes != 512
 */

#pragma once
#include "types.h"

#define FAT32_MAX_PATH      256
#define FAT32_MAX_NAME      256
#define FAT32_SECTOR_SIZE   512

/* File handle */
typedef struct {
    u32 start_cluster;
    u32 current_cluster;
    u32 file_size;
    u32 position;
    u32 cluster_offset;     /* byte offset within current cluster */
} fat32_file_t;

/* Directory entry (returned by readdir) */
typedef struct {
    char name[FAT32_MAX_NAME];
    u32  size;
    u32  cluster;
    u8   attr;
    bool is_dir;
} fat32_dirent_t;

/* Directory handle */
typedef struct {
    u32 start_cluster;
    u32 current_cluster;
    u32 entry_offset;       /* byte offset within current cluster */
    bool done;
} fat32_dir_t;

/* File attributes */
#define FAT32_ATTR_READONLY     0x01
#define FAT32_ATTR_HIDDEN       0x02
#define FAT32_ATTR_SYSTEM       0x04
#define FAT32_ATTR_VOLUME_ID    0x08
#define FAT32_ATTR_DIRECTORY    0x10
#define FAT32_ATTR_ARCHIVE      0x20
#define FAT32_ATTR_LFN          0x0F

/* Init — reads MBR + BPB, validates FAT32 */
bool fat32_init(void);

/* File operations */
bool fat32_open(const char *path, fat32_file_t *f);
u32  fat32_read(fat32_file_t *f, u8 *buf, u32 len);
void fat32_close(fat32_file_t *f);

/* Query */
bool fat32_exists(const char *path);
u32  fat32_file_size(const char *path);

/* Directory listing */
bool fat32_opendir(const char *path, fat32_dir_t *d);
bool fat32_readdir(fat32_dir_t *d, fat32_dirent_t *entry);
void fat32_closedir(fat32_dir_t *d);
