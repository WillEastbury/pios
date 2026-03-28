/*
 * bcache.c — Write-back block cache with LRU-2 eviction and sequential prefetch
 *
 * 256 cache entries, each holding one 512-byte SD block.
 * Designed for single-core use on Core 1 (disk I/O path).
 */

#include "bcache.h"
#include "sd.h"
#include "simd.h"
#include "timer.h"
#include "uart.h"

#define BCACHE_ENTRIES  256
#define PREFETCH_WINDOW 4

/* Entry flags */
#define FLAG_VALID   0x01
#define FLAG_DIRTY   0x02
#define FLAG_PREFETCH 0x04
#define FLAG_PINNED  0x08

typedef struct {
    u32 lba;
    u32 flags;
    u64 last_access;
    u32 access_count;
    u8  data[SD_BLOCK_SIZE] ALIGNED(64);
} bcache_entry_t;

static bcache_entry_t cache[BCACHE_ENTRIES] ALIGNED(64);
static bcache_stats_t stats;

/* Sequential access detector: ring buffer of last 4 LBAs */
static u32 history[PREFETCH_WINDOW];
static u32 history_idx;

/* ---- internal helpers ---- */

static bcache_entry_t *find_entry(u32 lba)
{
    for (u32 i = 0; i < BCACHE_ENTRIES; i++) {
        if ((cache[i].flags & FLAG_VALID) && cache[i].lba == lba)
            return &cache[i];
    }
    return NULL;
}

static bcache_entry_t *find_free(void)
{
    for (u32 i = 0; i < BCACHE_ENTRIES; i++) {
        if (!(cache[i].flags & FLAG_VALID))
            return &cache[i];
    }
    return NULL;
}

/*
 * LRU-2 eviction: prefer entries with access_count == 1 (only touched once).
 * Among those, pick the oldest last_access. If none exist, fall back to the
 * globally oldest entry. Never evict PINNED entries.
 */
static bcache_entry_t *find_victim(void)
{
    bcache_entry_t *single_best = NULL;
    bcache_entry_t *global_best = NULL;
    u64 single_oldest = (u64)-1;
    u64 global_oldest = (u64)-1;

    for (u32 i = 0; i < BCACHE_ENTRIES; i++) {
        bcache_entry_t *e = &cache[i];
        if (!(e->flags & FLAG_VALID) || (e->flags & FLAG_PINNED))
            continue;

        if (e->access_count <= 1 && e->last_access < single_oldest) {
            single_oldest = e->last_access;
            single_best = e;
        }
        if (e->last_access < global_oldest) {
            global_oldest = e->last_access;
            global_best = e;
        }
    }

    return single_best ? single_best : global_best;
}

static bool writeback(bcache_entry_t *e)
{
    if (!(e->flags & FLAG_DIRTY))
        return true;

    if (unlikely(!sd_write_block(e->lba, e->data))) {
        uart_puts("bcache: writeback failed lba=0x");
        uart_hex(e->lba);
        uart_puts("\n");
        return false;
    }

    e->flags &= ~FLAG_DIRTY;
    stats.writebacks++;
    return true;
}

static bcache_entry_t *evict(void)
{
    bcache_entry_t *free = find_free();
    if (free)
        return free;

    bcache_entry_t *victim = find_victim();
    if (unlikely(!victim))
        return NULL;

    if (victim->flags & FLAG_DIRTY) {
        writeback(victim);
        stats.dirty_count--;
    }

    victim->flags = 0;
    stats.evictions++;
    stats.valid_count--;
    return victim;
}

static bool populate(bcache_entry_t *e, u32 lba)
{
    if (unlikely(!sd_read_block(lba, e->data))) {
        uart_puts("bcache: read failed lba=0x");
        uart_hex(lba);
        uart_puts("\n");
        return false;
    }

    e->lba = lba;
    e->flags = FLAG_VALID;
    e->last_access = timer_ticks();
    e->access_count = 1;
    stats.valid_count++;
    return true;
}

/* Record LBA in history ring and check for sequential pattern */
static bool is_sequential(void)
{
    /* Need a full window to detect a pattern */
    for (u32 i = 1; i < PREFETCH_WINDOW; i++) {
        u32 prev_idx = (history_idx - i) & (PREFETCH_WINDOW - 1);
        u32 curr_idx = (history_idx - i + 1) & (PREFETCH_WINDOW - 1);
        if (history[curr_idx] != history[prev_idx] + 1)
            return false;
    }
    return true;
}

static void record_access(u32 lba)
{
    history[history_idx] = lba;
    history_idx = (history_idx + 1) & (PREFETCH_WINDOW - 1);
}

static void try_prefetch(u32 lba)
{
    if (!is_sequential())
        return;

    for (u32 i = 1; i <= PREFETCH_WINDOW; i++) {
        u32 pf_lba = lba + i;

        if (find_entry(pf_lba))
            continue;

        /* Only use free slots or replace existing prefetch entries */
        bcache_entry_t *slot = find_free();
        if (!slot) {
            for (u32 j = 0; j < BCACHE_ENTRIES; j++) {
                bcache_entry_t *e = &cache[j];
                if ((e->flags & FLAG_PREFETCH) && !(e->flags & FLAG_DIRTY) &&
                    !(e->flags & FLAG_PINNED)) {
                    slot = e;
                    if (slot->flags & FLAG_VALID)
                        stats.valid_count--;
                    break;
                }
            }
        }
        if (!slot)
            break;

        if (sd_read_block(pf_lba, slot->data)) {
            slot->lba = pf_lba;
            slot->flags = FLAG_VALID | FLAG_PREFETCH;
            slot->last_access = timer_ticks();
            slot->access_count = 0;
            stats.prefetches++;
            stats.valid_count++;
        }
    }
}

/* Recount dirty/valid for stats accuracy */
static void recount_stats(void)
{
    u32 dirty = 0, valid = 0;
    for (u32 i = 0; i < BCACHE_ENTRIES; i++) {
        if (cache[i].flags & FLAG_VALID) valid++;
        if (cache[i].flags & FLAG_DIRTY) dirty++;
    }
    stats.dirty_count = dirty;
    stats.valid_count = valid;
}

/* ---- public API ---- */

void bcache_init(void)
{
    simd_zero(cache, sizeof(cache));
    simd_zero(&stats, sizeof(stats));
    for (u32 i = 0; i < PREFETCH_WINDOW; i++)
        history[i] = 0xFFFFFFFF;
    history_idx = 0;
}

bool bcache_read(u32 lba, u8 *buf)
{
    record_access(lba);

    bcache_entry_t *e = find_entry(lba);
    if (likely(e != NULL)) {
        /* Cache hit */
        e->last_access = timer_ticks();
        e->access_count++;
        e->flags &= ~FLAG_PREFETCH;
        simd_memcpy(buf, e->data, SD_BLOCK_SIZE);
        stats.hits++;
        return true;
    }

    /* Cache miss */
    stats.misses++;

    e = evict();
    if (unlikely(!e))
        return sd_read_block(lba, buf);

    if (!populate(e, lba))
        return false;

    simd_memcpy(buf, e->data, SD_BLOCK_SIZE);
    try_prefetch(lba);
    return true;
}

bool bcache_write(u32 lba, const u8 *buf)
{
    record_access(lba);

    bcache_entry_t *e = find_entry(lba);
    if (e) {
        /* Update existing entry in-place */
        simd_memcpy(e->data, buf, SD_BLOCK_SIZE);
        if (!(e->flags & FLAG_DIRTY)) {
            e->flags |= FLAG_DIRTY;
            stats.dirty_count++;
        }
        e->flags &= ~FLAG_PREFETCH;
        e->last_access = timer_ticks();
        e->access_count++;
        stats.hits++;
        return true;
    }

    /* Miss: allocate entry */
    stats.misses++;

    e = evict();
    if (unlikely(!e))
        return sd_write_block(lba, buf);

    /* Write-allocate: copy caller data directly (no SD read needed) */
    e->lba = lba;
    e->flags = FLAG_VALID | FLAG_DIRTY;
    e->last_access = timer_ticks();
    e->access_count = 1;
    simd_memcpy(e->data, buf, SD_BLOCK_SIZE);
    stats.valid_count++;
    stats.dirty_count++;
    return true;
}

void bcache_flush(void)
{
    for (u32 i = 0; i < BCACHE_ENTRIES; i++) {
        if (cache[i].flags & FLAG_DIRTY)
            writeback(&cache[i]);
    }
    recount_stats();
}

void bcache_invalidate(u32 lba)
{
    bcache_entry_t *e = find_entry(lba);
    if (!e)
        return;

    if (e->flags & FLAG_PINNED)
        return;

    if (e->flags & FLAG_DIRTY)
        writeback(e);

    e->flags = 0;
    recount_stats();
}

void bcache_invalidate_all(void)
{
    for (u32 i = 0; i < BCACHE_ENTRIES; i++) {
        bcache_entry_t *e = &cache[i];
        if (!(e->flags & FLAG_VALID))
            continue;

        if (e->flags & FLAG_DIRTY)
            writeback(e);

        if (!(e->flags & FLAG_PINNED))
            e->flags = 0;
    }
    recount_stats();
}

void bcache_pin(u32 lba)
{
    bcache_entry_t *e = find_entry(lba);
    if (e) {
        e->flags |= FLAG_PINNED;
        return;
    }

    /* Not cached yet — load and pin */
    e = evict();
    if (unlikely(!e))
        return;

    if (populate(e, lba))
        e->flags |= FLAG_PINNED;
}

const bcache_stats_t *bcache_get_stats(void)
{
    return &stats;
}
