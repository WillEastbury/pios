/*
 * bcache.h — Block cache layer between WALFS and raw SD driver
 *
 * 256-entry write-back cache with LRU-2 eviction and sequential prefetch.
 * Single-core use only (Core 1 disk I/O path). No locking.
 */

#pragma once
#include "types.h"

void  bcache_init(void);
bool  bcache_read(u32 lba, u8 *buf);
bool  bcache_write(u32 lba, const u8 *buf);
void  bcache_flush(void);
void  bcache_invalidate(u32 lba);
void  bcache_invalidate_all(void);
void  bcache_pin(u32 lba);

typedef struct {
    u32 hits;
    u32 misses;
    u32 evictions;
    u32 writebacks;
    u32 prefetches;
    u32 dirty_count;
    u32 valid_count;
} bcache_stats_t;

const bcache_stats_t *bcache_get_stats(void);
