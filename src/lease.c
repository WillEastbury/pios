/*
 * lease.c - leased-span / descriptor-ownership foundation.
 *
 * See include/lease.h for the model. This is the additive foundation: an
 * arena registry, a reusable descriptor pool with a generation-poisoned
 * lifecycle, explicit cache-visibility at the lifecycle edges, and the
 * intra-capsule "move ownership" + cross-boundary "copy once" primitives.
 *
 * Existing PIOS span/FIFO primitives (proc_ipc_span_desc, fifo_span_msg)
 * remain unchanged; later phases route them through this layer.
 */

#include "types.h"
#include "lease.h"
#include "mmu.h"   /* dcache_clean_range / dcache_invalidate_range */

#define LEASE_ALIGN   64ULL   /* cache-line granular spans */

static struct lease_arena      g_arenas[LEASE_ARENA_MAX];
static struct lease_descriptor g_pool[LEASE_POOL_MAX];
static u64  g_next_lease_id;
static u32  g_next_arena_id;
static bool g_inited;

static struct lease_stats g_stats;
static const struct lease_mmu_ops *g_mmu_ops;

void lease_set_mmu_ops(const struct lease_mmu_ops *ops)
{
    g_mmu_ops = ops;
}

static void lease_zero(void *p, u32 n)
{
    u8 *b = (u8 *)p;
    for (u32 i = 0; i < n; i++) b[i] = 0;
}

static void lease_memcpy(void *dst, const void *src, u64 n)
{
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;
    for (u64 i = 0; i < n; i++) d[i] = s[i];
}

static u64 lease_align_up(u64 v, u64 a)
{
    return (v + (a - 1ULL)) & ~(a - 1ULL);
}

static struct lease_arena *arena_lookup(u32 arena_id)
{
    for (u32 i = 0; i < LEASE_ARENA_MAX; i++) {
        if (g_arenas[i].in_use && g_arenas[i].arena_id == arena_id)
            return &g_arenas[i];
    }
    return 0;
}

void lease_init(void)
{
    lease_zero(g_arenas, sizeof(g_arenas));
    lease_zero(g_pool, sizeof(g_pool));
    lease_zero(&g_stats, sizeof(g_stats));
    for (u32 i = 0; i < LEASE_POOL_MAX; i++) {
        g_pool[i].slot_id = i;
        g_pool[i].generation = 1;   /* start at 1 so a zeroed copy (gen 0) is stale */
        g_pool[i].state = LEASE_STATE_FREE;
        g_pool[i].arena_id = LEASE_ARENA_NONE;
    }
    g_next_lease_id = 1;
    g_next_arena_id = 1;
    g_stats.slots_total = LEASE_POOL_MAX;
    g_inited = true;
}

u32 lease_arena_register(u32 kind, u32 cache_policy, u32 owner_capsule,
                         u64 base, u64 size)
{
    if (!g_inited) lease_init();
    if (size == 0 || base == 0)
        return LEASE_ARENA_NONE;
    if (kind != LEASE_ARENA_KIND_KERNEL && kind != LEASE_ARENA_KIND_CAPSULE)
        return LEASE_ARENA_NONE;
    if (cache_policy != LEASE_CACHE_WBC_OFF && cache_policy != LEASE_CACHE_WBC_ON)
        return LEASE_ARENA_NONE;
    for (u32 i = 0; i < LEASE_ARENA_MAX; i++) {
        if (!g_arenas[i].in_use) {
            struct lease_arena *a = &g_arenas[i];
            a->arena_id = g_next_arena_id++;
            a->kind = kind;
            a->cache_policy = cache_policy;
            a->owner_capsule = owner_capsule;
            a->base = base;
            a->size = size;
            a->cursor = 0;
            a->generation = 1;
            a->in_use = 1;
            g_stats.arenas++;
            return a->arena_id;
        }
    }
    return LEASE_ARENA_NONE;
}

const struct lease_arena *lease_arena_get(u32 arena_id)
{
    return arena_lookup(arena_id);
}

void lease_arena_reset(u32 arena_id)
{
    struct lease_arena *a = arena_lookup(arena_id);
    if (!a) return;
    a->cursor = 0;
    a->generation++;
}

/* Validate a descriptor against the live pool slot it claims. */
static i32 lease_check(const struct lease_descriptor *d, struct lease_descriptor **slot_out)
{
    if (!d || d->slot_id >= LEASE_POOL_MAX)
        return LEASE_ERR_INVAL;
    struct lease_descriptor *s = &g_pool[d->slot_id];
    if (s->state == LEASE_STATE_FREE || s->generation != d->generation ||
        s->lease_id != d->lease_id) {
        g_stats.stale_rejects++;
        return LEASE_ERR_STALE;
    }
    if (slot_out) *slot_out = s;
    return LEASE_OK;
}

i32 lease_acquire(u32 arena_id, u64 length, u32 flags, u32 source_type,
                  u32 owner_capsule, struct lease_descriptor *out)
{
    if (!g_inited) lease_init();
    if (!out || length == 0)
        return LEASE_ERR_INVAL;
    struct lease_arena *a = arena_lookup(arena_id);
    if (!a)
        return LEASE_ERR_NOENT;

    u64 off = lease_align_up(a->cursor, LEASE_ALIGN);
    u64 take = lease_align_up(length, LEASE_ALIGN);
    if (off + take > a->size)
        return LEASE_ERR_NOSPC;

    for (u32 i = 0; i < LEASE_POOL_MAX; i++) {
        struct lease_descriptor *s = &g_pool[i];
        if (s->state != LEASE_STATE_FREE)
            continue;
        a->cursor = off + take;
        s->arena_id = arena_id;
        s->source_type = source_type;
        s->offset = off;
        s->length = length;
        s->owner_capsule = owner_capsule;
        s->flags = flags;
        s->state = LEASE_STATE_WRITING;
        s->lease_id = g_next_lease_id++;
        /* slot_id and generation persist across reuse; generation was bumped at
         * the previous release, so any stale copy is already poisoned. */
        g_stats.slots_live++;
        g_stats.acquires++;
        g_stats.next_lease_id = g_next_lease_id;
        *out = *s;
        return LEASE_OK;
    }
    return LEASE_ERR_NOSPC;
}

i32 lease_publish(const struct lease_descriptor *d)
{
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) return rc;
    if (s->state != LEASE_STATE_WRITING) {
        g_stats.state_rejects++;
        return LEASE_ERR_STATE;
    }
    struct lease_arena *a = arena_lookup(s->arena_id);
    if (a && a->cache_policy == LEASE_CACHE_WBC_ON)
        dcache_clean_range(a->base + s->offset, s->length);
    s->state = LEASE_STATE_READY;
    return LEASE_OK;
}

i32 lease_begin_read(const struct lease_descriptor *d)
{
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) return rc;
    if (s->state != LEASE_STATE_READY) {
        g_stats.state_rejects++;
        return LEASE_ERR_STATE;
    }
    struct lease_arena *a = arena_lookup(s->arena_id);
    if (a && a->cache_policy == LEASE_CACHE_WBC_ON)
        dcache_invalidate_range(a->base + s->offset, s->length);
    s->state = LEASE_STATE_READING;
    return LEASE_OK;
}

i32 lease_end_read(const struct lease_descriptor *d)
{
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) return rc;
    if (s->state != LEASE_STATE_READING) {
        g_stats.state_rejects++;
        return LEASE_ERR_STATE;
    }
    s->state = LEASE_STATE_EMPTY;
    return LEASE_OK;
}

i32 lease_release(const struct lease_descriptor *d)
{
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) return rc;
    /* RELEASE then poison: bump generation so any stale copy is rejected. */
    s->state = LEASE_STATE_RELEASE;
    s->generation++;
    if (s->generation == 0) s->generation = 1;
    s->lease_id = 0;
    s->arena_id = LEASE_ARENA_NONE;
    s->offset = 0;
    s->length = 0;
    s->owner_capsule = 0;
    s->flags = 0;
    s->source_type = 0;
    s->state = LEASE_STATE_FREE;
    if (g_stats.slots_live) g_stats.slots_live--;
    g_stats.releases++;
    return LEASE_OK;
}

i32 lease_transfer(const struct lease_descriptor *d, u32 new_owner_capsule)
{
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) return rc;
    s->owner_capsule = new_owner_capsule;
    g_stats.transfers++;
    return LEASE_OK;
}

i32 lease_grant(const struct lease_descriptor *d, u32 capsule, u32 access)
{
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) return rc;
    if (access != LEASE_F_READONLY && access != LEASE_F_READWRITE)
        return LEASE_ERR_INVAL;
    struct lease_arena *a = arena_lookup(s->arena_id);
    if (!a)
        return LEASE_ERR_NOENT;
    /* Hardware invariant: kernel arenas (DMA/FIFO/NIC-RX/device) are never
     * exposed to a capsule. Only the kernel (EL1) may touch them. */
    if (a->kind == LEASE_ARENA_KIND_KERNEL) {
        g_stats.mmu_rejects++;
        return LEASE_ERR_INVAL;
    }
    if (g_mmu_ops && g_mmu_ops->map) {
        rc = g_mmu_ops->map(capsule, a->base + s->offset, s->length, access);
        if (rc != LEASE_OK) {
            g_stats.mmu_rejects++;
            return rc;
        }
    }
    g_stats.grants++;
    return LEASE_OK;
}

i32 lease_revoke(const struct lease_descriptor *d, u32 capsule)
{
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) return rc;
    struct lease_arena *a = arena_lookup(s->arena_id);
    if (!a)
        return LEASE_ERR_NOENT;
    if (g_mmu_ops && g_mmu_ops->unmap)
        (void)g_mmu_ops->unmap(capsule, a->base + s->offset, s->length);
    g_stats.revokes++;
    return LEASE_OK;
}

void lease_handle_of(const struct lease_descriptor *d, struct lease_handle *out)
{
    if (!d || !out) return;
    out->lease_id = d->lease_id;
    out->slot_id = d->slot_id;
    out->generation = d->generation;
}

i32 lease_resolve(const struct lease_handle *h, struct lease_descriptor *out)
{
    if (!h || !out || h->slot_id >= LEASE_POOL_MAX)
        return LEASE_ERR_INVAL;
    struct lease_descriptor *s = &g_pool[h->slot_id];
    if (s->state == LEASE_STATE_FREE || s->generation != h->generation ||
        s->lease_id != h->lease_id) {
        g_stats.stale_rejects++;
        return LEASE_ERR_STALE;
    }
    *out = *s;
    return LEASE_OK;
}

i32 lease_copy_into(const struct lease_descriptor *src, u32 dst_arena,
                    u32 dst_capsule, u32 dst_flags, struct lease_descriptor *out_dst)
{
    struct lease_descriptor *s;
    i32 rc = lease_check(src, &s);
    if (rc != LEASE_OK) return rc;
    if (!out_dst)
        return LEASE_ERR_INVAL;
    struct lease_arena *sa = arena_lookup(s->arena_id);
    if (!sa)
        return LEASE_ERR_NOENT;

    rc = lease_acquire(dst_arena, s->length, dst_flags, LEASE_SRC_CAPSULE_MEM,
                       dst_capsule, out_dst);
    if (rc != LEASE_OK)
        return rc;
    struct lease_arena *da = arena_lookup(dst_arena);
    /* If the source lives in a cacheable arena, make sure we read fresh bytes. */
    if (sa->cache_policy == LEASE_CACHE_WBC_ON)
        dcache_invalidate_range(sa->base + s->offset, s->length);
    lease_memcpy((void *)(usize)(da->base + out_dst->offset),
                 (const void *)(usize)(sa->base + s->offset), s->length);
    g_stats.copies++;
    return lease_publish(out_dst);
}

i32 lease_validate(const struct lease_descriptor *d, u32 expect_owner)
{
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) return rc;
    struct lease_arena *a = arena_lookup(s->arena_id);
    if (!a)
        return LEASE_ERR_NOENT;
    if (s->offset + s->length > a->size)
        return LEASE_ERR_INVAL;
    if (expect_owner != LEASE_ARENA_NONE && s->owner_capsule != expect_owner)
        return LEASE_ERR_INVAL;
    return LEASE_OK;
}

void *lease_ptr(const struct lease_descriptor *d)
{
    struct lease_descriptor *s;
    if (lease_check(d, &s) != LEASE_OK)
        return 0;
    struct lease_arena *a = arena_lookup(s->arena_id);
    if (!a)
        return 0;
    return (void *)(usize)(a->base + s->offset);
}

void lease_get_stats(struct lease_stats *out)
{
    if (!out) return;
    *out = g_stats;
}

/* --- selftest: exercises the lifecycle, poison, transfer, copy-once,
 *     MMU grant/revoke (mock), handle round-trip, and TLS-direct-write. --- */
static u8 g_lease_test_kern[8192] ALIGNED(64);
static u8 g_lease_test_caps[8192] ALIGNED(64);

static u32 g_test_map_calls;
static u32 g_test_unmap_calls;
static u32 g_test_last_access;

static i32 test_mmu_map(u32 capsule, u64 pa, u64 length, u32 access)
{
    (void)capsule; (void)pa; (void)length;
    g_test_map_calls++;
    g_test_last_access = access;
    return LEASE_OK;
}
static i32 test_mmu_unmap(u32 capsule, u64 pa, u64 length)
{
    (void)capsule; (void)pa; (void)length;
    g_test_unmap_calls++;
    return LEASE_OK;
}
static const struct lease_mmu_ops g_test_mmu_ops = {
    .map = test_mmu_map,
    .unmap = test_mmu_unmap,
};

bool lease_selftest(void)
{
    const struct lease_mmu_ops *saved_ops = g_mmu_ops;
    bool ok = false;
    lease_init();
    g_test_map_calls = 0;
    g_test_unmap_calls = 0;
    lease_set_mmu_ops(&g_test_mmu_ops);

    u32 karena = lease_arena_register(LEASE_ARENA_KIND_KERNEL, LEASE_CACHE_WBC_OFF,
                                      LEASE_OWNER_KERNEL,
                                      (u64)(usize)g_lease_test_kern, sizeof(g_lease_test_kern));
    u32 carena = lease_arena_register(LEASE_ARENA_KIND_CAPSULE, LEASE_CACHE_WBC_ON,
                                      7 /* fake capsule */,
                                      (u64)(usize)g_lease_test_caps, sizeof(g_lease_test_caps));
    if (karena == LEASE_ARENA_NONE || carena == LEASE_ARENA_NONE)
        goto done;

    /* 1) Acquire -> write -> publish -> read -> release in the kernel arena. */
    struct lease_descriptor d;
    if (lease_acquire(karena, 100, LEASE_F_KERNEL | LEASE_F_READWRITE,
                      LEASE_SRC_KERNEL_MEM, LEASE_OWNER_KERNEL, &d) != LEASE_OK)
        goto done;
    u8 *p = (u8 *)lease_ptr(&d);
    if (!p) goto done;
    for (u32 i = 0; i < 100; i++) p[i] = (u8)(i + 1);
    if (lease_publish(&d) != LEASE_OK) goto done;
    if (lease_begin_read(&d) != LEASE_OK) goto done;
    if (lease_validate(&d, LEASE_OWNER_KERNEL) != LEASE_OK) goto done;
    u8 *rp = (u8 *)lease_ptr(&d);
    if (rp[0] != 1 || rp[99] != 100) goto done;
    if (lease_end_read(&d) != LEASE_OK) goto done;

    /* 2) Stash a stale copy, release, confirm the copy is poisoned. */
    struct lease_descriptor stale = d;
    if (lease_release(&d) != LEASE_OK) goto done;
    if (lease_check(&stale, 0) != LEASE_ERR_STALE) goto done;
    if (lease_ptr(&stale) != 0) goto done;

    /* 3) Cross-boundary copy-once: kernel span -> capsule arena. */
    struct lease_descriptor src;
    if (lease_acquire(karena, 16, LEASE_F_KERNEL | LEASE_F_READWRITE,
                      LEASE_SRC_KERNEL_MEM, LEASE_OWNER_KERNEL, &src) != LEASE_OK)
        goto done;
    u8 *sp = (u8 *)lease_ptr(&src);
    for (u32 i = 0; i < 16; i++) sp[i] = (u8)(0xA0 + i);
    if (lease_publish(&src) != LEASE_OK) goto done;

    struct lease_descriptor dst;
    if (lease_copy_into(&src, carena, 7, LEASE_F_CAPSULE | LEASE_F_READONLY, &dst) != LEASE_OK)
        goto done;
    if (dst.owner_capsule != 7 || dst.length != 16) goto done;
    if (lease_begin_read(&dst) != LEASE_OK) goto done;
    u8 *dp = (u8 *)lease_ptr(&dst);
    for (u32 i = 0; i < 16; i++) if (dp[i] != (u8)(0xA0 + i)) goto done;
    if (lease_end_read(&dst) != LEASE_OK) goto done;

    /* 4) Ownership transfer (intra-capsule move). */
    if (lease_transfer(&dst, 9) != LEASE_OK) goto done;
    if (lease_validate(&dst, 9) != LEASE_OK) goto done;

    /* 5) MMU enforcement: granting a CAPSULE arena maps; granting a KERNEL
     *    arena is refused (kernel buffers never reach a capsule). */
    if (lease_grant(&dst, 9, LEASE_F_READONLY) != LEASE_OK) goto done;
    if (g_test_map_calls != 1 || g_test_last_access != LEASE_F_READONLY) goto done;
    if (lease_grant(&src, 9, LEASE_F_READWRITE) != LEASE_ERR_INVAL) goto done; /* kernel arena */
    if (g_test_map_calls != 1) goto done; /* refused before calling map */
    if (lease_revoke(&dst, 9) != LEASE_OK || g_test_unmap_calls != 1) goto done;

    /* 6) Zero-copy handle round-trip (the FIFO message-passing unit). */
    struct lease_handle h;
    lease_handle_of(&dst, &h);
    struct lease_descriptor rd;
    if (lease_resolve(&h, &rd) != LEASE_OK) goto done;
    if (rd.lease_id != dst.lease_id || rd.slot_id != dst.slot_id) goto done;

    if (lease_release(&src) != LEASE_OK) goto done;
    if (lease_release(&dst) != LEASE_OK) goto done;
    /* resolving a handle for a released lease must now fail (poisoned). */
    if (lease_resolve(&h, &rd) != LEASE_ERR_STALE) goto done;

    /* 7) TLS-offload shape: kernel acquires a RW lease in the capsule arena,
     *    writes the "decrypted" bytes straight in, publishes, posts a handle.
     *    The capsule then resolves + reads it - no staging copy. */
    struct lease_descriptor plain;
    if (lease_acquire(carena, 32, LEASE_F_CAPSULE | LEASE_F_READWRITE,
                      LEASE_SRC_KERNEL_MEM, 7, &plain) != LEASE_OK)
        goto done;
    u8 *pp = (u8 *)lease_ptr(&plain);
    for (u32 i = 0; i < 32; i++) pp[i] = (u8)(0x50 ^ i);   /* "tls_decrypt(...)" */
    if (lease_publish(&plain) != LEASE_OK) goto done;
    struct lease_handle ch;
    lease_handle_of(&plain, &ch);                          /* post on capsule FIFO */
    struct lease_descriptor got;
    if (lease_resolve(&ch, &got) != LEASE_OK) goto done;   /* capsule side */
    if (lease_begin_read(&got) != LEASE_OK) goto done;
    u8 *gp = (u8 *)lease_ptr(&got);
    for (u32 i = 0; i < 32; i++) if (gp[i] != (u8)(0x50 ^ i)) goto done;
    if (lease_end_read(&got) != LEASE_OK) goto done;
    if (lease_release(&plain) != LEASE_OK) goto done;

    struct lease_stats st;
    lease_get_stats(&st);
    if (st.acquires < 4 || st.releases < 4 || st.copies < 1 ||
        st.transfers < 1 || st.stale_rejects < 1 ||
        st.grants < 1 || st.revokes < 1 || st.mmu_rejects < 1)
        goto done;

    ok = true;
done:
    lease_set_mmu_ops(saved_ops);
    lease_init();
    return ok;
}
