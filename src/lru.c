/*
 * lru.c - Generic LRU cache with TTL expiry
 *
 * Hash table (32 buckets, chained) for O(1) key lookup.
 * Doubly-linked list for O(1) LRU eviction and MRU promotion.
 * TTL-based expiry scanned from tail (oldest first) in lru_tick().
 */

#include "lru.h"
#include "simd.h"
#include "timer.h"

/* ---- Hash ---- */

static u32 hash_key(const u8 *key, u32 len) {
    u32 h = 5381;
    for (u32 i = 0; i < len; i++)
        h = ((h << 5) + h) + key[i];
    return h & (LRU_HASH_BUCKETS - 1);
}

static bool key_eq(const struct lru_entry *e, const u8 *key, u32 len) {
    if (e->key_len != len) return false;
    for (u32 i = 0; i < len; i++)
        if (e->key[i] != key[i]) return false;
    return true;
}

/* ---- Linked list ops ---- */

static void list_remove(struct lru_cache *c, i32 idx) {
    struct lru_entry *e = &c->entries[idx];
    if (e->prev >= 0)
        c->entries[e->prev].next = e->next;
    else
        c->lru_head = e->next;
    if (e->next >= 0)
        c->entries[e->next].prev = e->prev;
    else
        c->lru_tail = e->prev;
    e->prev = -1;
    e->next = -1;
}

static void list_push_front(struct lru_cache *c, i32 idx) {
    struct lru_entry *e = &c->entries[idx];
    e->prev = -1;
    e->next = c->lru_head;
    if (c->lru_head >= 0)
        c->entries[c->lru_head].prev = idx;
    c->lru_head = idx;
    if (c->lru_tail < 0)
        c->lru_tail = idx;
}

/* ---- Hash table ops ---- */

static void hash_insert(struct lru_cache *c, i32 idx) {
    u32 bucket = hash_key(c->entries[idx].key, c->entries[idx].key_len);
    c->entries[idx].hash_next = c->hash_heads[bucket];
    c->hash_heads[bucket] = idx;
}

static void hash_remove(struct lru_cache *c, i32 idx) {
    u32 bucket = hash_key(c->entries[idx].key, c->entries[idx].key_len);
    i32 *pp = &c->hash_heads[bucket];
    while (*pp >= 0) {
        if (*pp == idx) {
            *pp = c->entries[idx].hash_next;
            c->entries[idx].hash_next = -1;
            return;
        }
        pp = &c->entries[*pp].hash_next;
    }
}

static i32 hash_find(struct lru_cache *c, const u8 *key, u32 key_len) {
    u32 bucket = hash_key(key, key_len);
    i32 idx = c->hash_heads[bucket];
    while (idx >= 0) {
        if (c->entries[idx].valid && key_eq(&c->entries[idx], key, key_len))
            return idx;
        idx = c->entries[idx].hash_next;
    }
    return -1;
}

/* ---- Evict ---- */

static void evict_entry(struct lru_cache *c, i32 idx) {
    struct lru_entry *e = &c->entries[idx];
    if (c->on_evict)
        c->on_evict(e->key, e->key_len, e->value, e->val_len);
    hash_remove(c, idx);
    list_remove(c, idx);
    e->valid = false;
    c->count--;
}

static i32 alloc_entry(struct lru_cache *c) {
    /* Find free slot */
    for (u32 i = 0; i < LRU_CAPACITY; i++)
        if (!c->entries[i].valid)
            return (i32)i;

    /* Evict LRU (tail) */
    if (c->lru_tail >= 0) {
        i32 victim = c->lru_tail;
        c->evictions++;
        evict_entry(c, victim);
        return victim;
    }
    return -1;
}

/* ---- Public API ---- */

void lru_init(struct lru_cache *c, lru_evict_cb on_evict, u64 ttl_ms) {
    for (u32 i = 0; i < LRU_CAPACITY; i++) {
        c->entries[i].valid = false;
        c->entries[i].prev = -1;
        c->entries[i].next = -1;
        c->entries[i].hash_next = -1;
    }
    for (u32 i = 0; i < LRU_HASH_BUCKETS; i++)
        c->hash_heads[i] = -1;
    c->lru_head = -1;
    c->lru_tail = -1;
    c->count = 0;
    c->ttl_ms = ttl_ms;
    c->on_evict = on_evict;
    c->hits = 0;
    c->misses = 0;
    c->evictions = 0;
    c->expirations = 0;
}

struct lru_entry *lru_get(struct lru_cache *c, const u8 *key, u32 key_len) {
    if (key_len > LRU_KEY_MAX) { c->misses++; return NULL; }

    i32 idx = hash_find(c, key, key_len);
    if (idx < 0) { c->misses++; return NULL; }

    struct lru_entry *e = &c->entries[idx];

    /* Check TTL */
    if (c->ttl_ms > 0 && timer_ticks() - e->created > c->ttl_ms) {
        c->expirations++;
        evict_entry(c, idx);
        c->misses++;
        return NULL;
    }

    /* Move to MRU (front of list) */
    list_remove(c, idx);
    list_push_front(c, idx);
    e->last_access = timer_ticks();
    e->access_count++;
    c->hits++;
    return e;
}

bool lru_put(struct lru_cache *c, const u8 *key, u32 key_len,
             const void *value, u32 val_len) {
    if (key_len > LRU_KEY_MAX || val_len > LRU_VAL_MAX)
        return false;

    /* Check if key already exists → update in place */
    i32 idx = hash_find(c, key, key_len);
    if (idx >= 0) {
        struct lru_entry *e = &c->entries[idx];
        simd_memcpy(e->value, value, val_len);
        e->val_len = val_len;
        e->last_access = timer_ticks();
        e->access_count++;
        /* Move to MRU */
        list_remove(c, idx);
        list_push_front(c, idx);
        return true;
    }

    /* Allocate new entry (evicts LRU if full) */
    idx = alloc_entry(c);
    if (idx < 0) return false;

    struct lru_entry *e = &c->entries[idx];
    simd_memcpy(e->key, key, key_len);
    e->key_len = key_len;
    simd_memcpy(e->value, value, val_len);
    e->val_len = val_len;
    e->last_access = timer_ticks();
    e->created = timer_ticks();
    e->access_count = 1;
    e->valid = true;
    e->hash_next = -1;

    hash_insert(c, idx);
    list_push_front(c, idx);
    c->count++;
    return true;
}

bool lru_remove(struct lru_cache *c, const u8 *key, u32 key_len) {
    i32 idx = hash_find(c, key, key_len);
    if (idx < 0) return false;
    evict_entry(c, idx);
    return true;
}

u32 lru_tick(struct lru_cache *c) {
    if (c->ttl_ms == 0) return 0;
    u64 now = timer_ticks();
    u32 expired = 0;

    /* Scan from tail (oldest) towards head */
    i32 idx = c->lru_tail;
    while (idx >= 0) {
        i32 prev = c->entries[idx].prev;
        if (now - c->entries[idx].created > c->ttl_ms) {
            c->expirations++;
            evict_entry(c, idx);
            expired++;
        } else {
            break; /* entries towards head are newer — stop */
        }
        idx = prev;
    }
    return expired;
}

void lru_flush(struct lru_cache *c) {
    for (u32 i = 0; i < LRU_CAPACITY; i++) {
        if (c->entries[i].valid)
            evict_entry(c, (i32)i);
    }
}

void lru_stats(const struct lru_cache *c, u64 *hits, u64 *misses,
               u64 *evictions, u64 *expirations) {
    if (hits) *hits = c->hits;
    if (misses) *misses = c->misses;
    if (evictions) *evictions = c->evictions;
    if (expirations) *expirations = c->expirations;
}
