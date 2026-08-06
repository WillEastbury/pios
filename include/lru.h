/*
 * lru.h - Generic LRU cache with TTL expiry and eviction callbacks
 *
 * Fixed-size hash-table-backed cache with O(1) lookup, insert, and evict.
 * Doubly-linked list maintains LRU order. Entries expire after a configurable
 * TTL. Eviction callback notifies the owner when an entry is removed.
 *
 * Usage:
 *   struct lru_cache cache;
 *   lru_init(&cache, my_evict_cb, 60000);  // 60s TTL
 *   lru_put(&cache, key, value, value_len);
 *   struct lru_entry *e = lru_get(&cache, key);
 *   lru_tick(&cache);  // call periodically to expire stale entries
 */

#pragma once
#include "types.h"

#define LRU_CAPACITY    64
#define LRU_KEY_MAX     32
#define LRU_VAL_MAX     128
#define LRU_HASH_BUCKETS 32

struct lru_entry {
    u8   key[LRU_KEY_MAX];
    u8   value[LRU_VAL_MAX];
    u32  key_len;
    u32  val_len;
    u64  last_access;       /* timer_ticks() at last get/put */
    u64  created;           /* timer_ticks() at insertion */
    u32  access_count;
    bool valid;

    /* Doubly-linked list (LRU order: head=MRU, tail=LRU) */
    i32  prev;              /* index or -1 */
    i32  next;              /* index or -1 */

    /* Hash chain */
    i32  hash_next;         /* index or -1 */
};

typedef void (*lru_evict_cb)(const u8 *key, u32 key_len,
                              const u8 *value, u32 val_len);

struct lru_cache {
    struct lru_entry entries[LRU_CAPACITY];
    i32    hash_heads[LRU_HASH_BUCKETS]; /* head of chain per bucket */
    i32    lru_head;        /* most recently used */
    i32    lru_tail;        /* least recently used (evict candidate) */
    u32    count;           /* number of valid entries */
    u64    ttl_ms;          /* time-to-live in milliseconds (0=infinite) */
    lru_evict_cb on_evict;  /* called before eviction (may be NULL) */

    /* Stats */
    u64    hits;
    u64    misses;
    u64    evictions;
    u64    expirations;
};

/* Init cache with optional eviction callback and TTL (0=no expiry) */
void lru_init(struct lru_cache *c, lru_evict_cb on_evict, u64 ttl_ms);

/* Lookup by key. Returns entry pointer or NULL. Moves to MRU on hit. */
struct lru_entry *lru_get(struct lru_cache *c, const u8 *key, u32 key_len);

/* Insert or update. Evicts LRU entry if full. */
bool lru_put(struct lru_cache *c, const u8 *key, u32 key_len,
             const void *value, u32 val_len);

/* Explicitly remove an entry by key. Returns true if found. */
bool lru_remove(struct lru_cache *c, const u8 *key, u32 key_len);

/* Expire entries older than TTL. Call periodically (e.g., every second). */
u32 lru_tick(struct lru_cache *c);

/* Flush all entries (calls evict callback for each). */
void lru_flush(struct lru_cache *c);

/* Get stats */
void lru_stats(const struct lru_cache *c, u64 *hits, u64 *misses,
               u64 *evictions, u64 *expirations);
