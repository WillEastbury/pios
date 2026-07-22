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
#include "kspin.h" /* shared WB kspin control-page registry lock */

#define LEASE_ALIGN   64ULL   /* cache-line granular spans */

/*
 * CROSS-CORE LOCKING (registry serialization)
 * -------------------------------------------
 * The lease registry (g_arenas / g_pool / g_stats / g_next_* / g_mmu_ops) is a
 * control-path structure shared by every core. All of these APIs are
 * syscall/control-path only; NONE of them run in scheduler or IRQ hot paths, so
 * a short bounded spinlock is the correct primitive. We take exactly one shared
 * kspinlock (slot 3 of the WB kspin control page; slots 0-2 belong to the IPC
 * queue/stream/pipe registries). The lock owns a full cache line, so its
 * contention byte never shares a line with any protected state.
 *
 * Locking discipline / linearization:
 *   - Every public entry point acquires the registry lock, calls an internal
 *     `_locked` helper that assumes the lock is held, then releases it. A lease
 *     operation is linearized at the instant its `_locked` body runs.
 *   - The lock is NOT recursive. Composite operations (e.g. lease_copy_into,
 *     which internally acquires a destination lease and publishes it) must call
 *     the `_locked` helpers, never the public wrappers, or they would
 *     self-deadlock.
 *   - MMU map/unmap callbacks may block or call back into the kernel, so they
 *     are NEVER invoked while the registry lock is held. lease_grant/lease_revoke
 *     snapshot the validated PA/length (and the mmu-ops pointer) under the lock,
 *     drop the lock, invoke the callback, then reacquire the lock only to
 *     revalidate the descriptor generation and update stats. A grant is
 *     linearized at the successful MMU map; if the descriptor was released/
 *     recycled while the lock was dropped, the just-installed mapping is rolled
 *     back (unmap) and the grant fails STALE.
 *
 * Initialization:
 *   lease_init() runs once before the secondary cores start, so by the time any
 *   cross-core call can occur the lock and registry are already initialized.
 *   The lazy `if (!g_inited)` fallback inside the locked helpers is retained for
 *   fail-closed standalone/early-boot use; it re-inits registry DATA only and
 *   never touches the lock, so it is safe to run while the lock is held. The
 *   lock's contention byte lives in zero-initialized control RAM, so even a
 *   never-explicitly-initialized lock starts unlocked.
 */
#define LEASE_LOCK_SLOT   3U

static inline struct kspinlock *lease_lock(void)
{
    return kspin_shared(LEASE_LOCK_SLOT);
}

static struct lease_arena      g_arenas[LEASE_ARENA_MAX];
static struct lease_descriptor g_pool[LEASE_POOL_MAX];
_Static_assert(sizeof(struct lease_arena) == 64,
               "lease arenas must have cache-line stride");
_Static_assert(sizeof(struct lease_descriptor) == 64,
               "lease descriptors must have cache-line stride");
static u64  g_next_lease_id;
static u32  g_next_arena_id;
static bool g_inited;
static bool g_lock_initialized;

static struct lease_stats g_stats;
static const struct lease_mmu_ops *g_mmu_ops;

void lease_set_mmu_ops(const struct lease_mmu_ops *ops)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    g_mmu_ops = ops;
    kspin_unlock(lk);
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

/* Registry DATA reset only. Assumes the caller holds the lock (or is running
 * single-threaded before secondaries). Does NOT touch the lock itself. */
static void lease_init_data(void)
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

void lease_init(void)
{
    struct kspinlock *lk = lease_lock();
#ifdef PIOS_HOST_TYPES_SHIM
    /* Host unit tests intentionally reset between independent cases. */
    kspin_init(lk);
    kspin_lock(lk);
    lease_init_data();
    kspin_unlock(lk);
#else
    /* Production initialization is one-shot and idempotent. Never clear a live
     * lock or registry from a diagnostic/selftest path after secondaries start. */
    if (!g_lock_initialized) {
        kspin_init(lk);
        g_lock_initialized = true;
    }
    kspin_lock(lk);
    if (!g_inited)
        lease_init_data();
    kspin_unlock(lk);
#endif
}

static u32 lease_arena_register_locked(u32 kind, u32 cache_policy, u32 owner_capsule,
                                       u64 base, u64 size)
{
    if (!g_inited) lease_init_data();
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

u32 lease_arena_register(u32 kind, u32 cache_policy, u32 owner_capsule,
                         u64 base, u64 size)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    u32 id = lease_arena_register_locked(kind, cache_policy, owner_capsule, base, size);
    kspin_unlock(lk);
    return id;
}

/*
 * lease_arena_get returns a pointer to the internal arena record. The identity
 * fields (arena_id/kind/cache_policy/owner_capsule/base/size) are immutable
 * after register and the record is never freed, so reading them through this
 * pointer is safe. The `cursor` and `generation` fields ARE mutated concurrently
 * (by lease_acquire / lease_arena_reset) and must not be trusted through this
 * pointer; use lease_arena_snapshot() for a consistent cache-line copy instead.
 */
const struct lease_arena *lease_arena_get(u32 arena_id)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    struct lease_arena *a = arena_lookup(arena_id);
    kspin_unlock(lk);
    return a;
}

/* Consistent cache-line snapshot of an arena record (all fields captured atomic
 * to the lock). Returns true if the arena exists. This is the race-free way to
 * read the mutable cursor/generation. */
bool lease_arena_snapshot(u32 arena_id, struct lease_arena *out)
{
    if (!out) return false;
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    struct lease_arena *a = arena_lookup(arena_id);
    bool found = false;
    if (a) { *out = *a; found = true; }
    kspin_unlock(lk);
    return found;
}

void lease_arena_reset(u32 arena_id)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    struct lease_arena *a = arena_lookup(arena_id);
    if (a) {
        a->cursor = 0;
        a->generation++;
    }
    kspin_unlock(lk);
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

static i32 lease_acquire_locked(u32 arena_id, u64 length, u32 flags, u32 source_type,
                                u32 owner_capsule, struct lease_descriptor *out)
{
    if (!g_inited) lease_init_data();
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

i32 lease_acquire(u32 arena_id, u64 length, u32 flags, u32 source_type,
                  u32 owner_capsule, struct lease_descriptor *out)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    i32 rc = lease_acquire_locked(arena_id, length, flags, source_type,
                                  owner_capsule, out);
    kspin_unlock(lk);
    return rc;
}

static i32 lease_publish_locked(const struct lease_descriptor *d)
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

i32 lease_publish(const struct lease_descriptor *d)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    i32 rc = lease_publish_locked(d);
    kspin_unlock(lk);
    return rc;
}

i32 lease_begin_read(const struct lease_descriptor *d)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) { kspin_unlock(lk); return rc; }
    if (s->state != LEASE_STATE_READY) {
        g_stats.state_rejects++;
        kspin_unlock(lk);
        return LEASE_ERR_STATE;
    }
    struct lease_arena *a = arena_lookup(s->arena_id);
    if (a && a->cache_policy == LEASE_CACHE_WBC_ON)
        dcache_invalidate_range(a->base + s->offset, s->length);
    s->state = LEASE_STATE_READING;
    kspin_unlock(lk);
    return LEASE_OK;
}

i32 lease_end_read(const struct lease_descriptor *d)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) { kspin_unlock(lk); return rc; }
    if (s->state != LEASE_STATE_READING) {
        g_stats.state_rejects++;
        kspin_unlock(lk);
        return LEASE_ERR_STATE;
    }
    s->state = LEASE_STATE_EMPTY;
    kspin_unlock(lk);
    return LEASE_OK;
}

i32 lease_release(const struct lease_descriptor *d)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) { kspin_unlock(lk); return rc; }
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
    kspin_unlock(lk);
    return LEASE_OK;
}

i32 lease_transfer(const struct lease_descriptor *d, u32 new_owner_capsule)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) { kspin_unlock(lk); return rc; }
    s->owner_capsule = new_owner_capsule;
    g_stats.transfers++;
    kspin_unlock(lk);
    return LEASE_OK;
}

i32 lease_grant(const struct lease_descriptor *d, u32 capsule, u32 access)
{
    if (access != LEASE_F_READONLY && access != LEASE_F_READWRITE)
        return LEASE_ERR_INVAL;

    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) { kspin_unlock(lk); return rc; }
    struct lease_arena *a = arena_lookup(s->arena_id);
    if (!a) { kspin_unlock(lk); return LEASE_ERR_NOENT; }
    /* Hardware invariant: kernel arenas (DMA/FIFO/NIC-RX/device) are never
     * exposed to a capsule. Only the kernel (EL1) may touch them. */
    if (a->kind == LEASE_ARENA_KIND_KERNEL) {
        g_stats.mmu_rejects++;
        kspin_unlock(lk);
        return LEASE_ERR_INVAL;
    }
    /* Snapshot the validated span + identity, then drop the lock before the MMU
     * callback (which may block or re-enter the kernel). */
    u64 pa = a->base + s->offset;
    u64 length = s->length;
    u32 slot = s->slot_id;
    u32 gen = s->generation;
    u64 lid = s->lease_id;
    const struct lease_mmu_ops *ops = g_mmu_ops;
    kspin_unlock(lk);

    if (ops && ops->map) {
        rc = ops->map(capsule, pa, length, access);
        if (rc != LEASE_OK) {
            kspin_lock(lk);
            g_stats.mmu_rejects++;
            kspin_unlock(lk);
            return rc;
        }
    }

    /* Reacquire only to revalidate the descriptor and update stats. Grant is
     * linearized at the successful map above. */
    kspin_lock(lk);
    struct lease_descriptor *s2 = &g_pool[slot];
    if (s2->state == LEASE_STATE_FREE || s2->generation != gen || s2->lease_id != lid) {
        /* Released/recycled while unlocked: roll back the mapping we installed. */
        g_stats.stale_rejects++;
        const struct lease_mmu_ops *ops2 = g_mmu_ops;
        kspin_unlock(lk);
        if (ops2 && ops2->unmap)
            (void)ops2->unmap(capsule, pa, length);
        return LEASE_ERR_STALE;
    }
    g_stats.grants++;
    kspin_unlock(lk);
    return LEASE_OK;
}

i32 lease_revoke(const struct lease_descriptor *d, u32 capsule)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) { kspin_unlock(lk); return rc; }
    struct lease_arena *a = arena_lookup(s->arena_id);
    if (!a) { kspin_unlock(lk); return LEASE_ERR_NOENT; }
    /* Snapshot span, drop lock before the (possibly blocking) unmap callback. */
    u64 pa = a->base + s->offset;
    u64 length = s->length;
    const struct lease_mmu_ops *ops = g_mmu_ops;
    kspin_unlock(lk);

    if (ops && ops->unmap)
        (void)ops->unmap(capsule, pa, length);

    kspin_lock(lk);
    g_stats.revokes++;
    kspin_unlock(lk);
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
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    struct lease_descriptor *s = &g_pool[h->slot_id];
    if (s->state == LEASE_STATE_FREE || s->generation != h->generation ||
        s->lease_id != h->lease_id) {
        g_stats.stale_rejects++;
        kspin_unlock(lk);
        return LEASE_ERR_STALE;
    }
    *out = *s;
    kspin_unlock(lk);
    return LEASE_OK;
}

/*
 * Cross-boundary copy-once. Runs entirely under the registry lock so the whole
 * "acquire dst -> read fresh src bytes -> copy -> publish dst" sequence is
 * linearized as a single step. It calls the `_locked` helpers (never the public
 * wrappers) because the lock is non-recursive. No MMU callback is involved here,
 * so holding the lock across the (control-path) copy is safe.
 */
static i32 lease_copy_into_locked(const struct lease_descriptor *src, u32 dst_arena,
                                  u32 dst_capsule, u32 dst_flags,
                                  struct lease_descriptor *out_dst)
{
    struct lease_descriptor *s;
    i32 rc = lease_check(src, &s);
    if (rc != LEASE_OK) return rc;
    if (!out_dst)
        return LEASE_ERR_INVAL;
    struct lease_arena *sa = arena_lookup(s->arena_id);
    if (!sa)
        return LEASE_ERR_NOENT;

    /* Capture the source span before acquiring the destination (acquire mutates
     * pool/arena state but not this source descriptor). */
    u64 src_pa = sa->base + s->offset;
    u64 src_len = s->length;
    u32 src_policy = sa->cache_policy;

    rc = lease_acquire_locked(dst_arena, src_len, dst_flags, LEASE_SRC_CAPSULE_MEM,
                              dst_capsule, out_dst);
    if (rc != LEASE_OK)
        return rc;
    struct lease_arena *da = arena_lookup(dst_arena);
    if (!da)
        return LEASE_ERR_NOENT;
    /* If the source lives in a cacheable arena, make sure we read fresh bytes. */
    if (src_policy == LEASE_CACHE_WBC_ON)
        dcache_invalidate_range(src_pa, src_len);
    lease_memcpy((void *)(usize)(da->base + out_dst->offset),
                 (const void *)(usize)src_pa, src_len);
    g_stats.copies++;
    return lease_publish_locked(out_dst);
}

i32 lease_copy_into(const struct lease_descriptor *src, u32 dst_arena,
                    u32 dst_capsule, u32 dst_flags, struct lease_descriptor *out_dst)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    i32 rc = lease_copy_into_locked(src, dst_arena, dst_capsule, dst_flags, out_dst);
    kspin_unlock(lk);
    return rc;
}

i32 lease_validate(const struct lease_descriptor *d, u32 expect_owner)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    struct lease_descriptor *s;
    i32 rc = lease_check(d, &s);
    if (rc != LEASE_OK) { kspin_unlock(lk); return rc; }
    struct lease_arena *a = arena_lookup(s->arena_id);
    if (!a) { kspin_unlock(lk); return LEASE_ERR_NOENT; }
    if (s->offset + s->length > a->size) { kspin_unlock(lk); return LEASE_ERR_INVAL; }
    if (expect_owner != LEASE_ARENA_NONE && s->owner_capsule != expect_owner) {
        kspin_unlock(lk);
        return LEASE_ERR_INVAL;
    }
    kspin_unlock(lk);
    return LEASE_OK;
}

/*
 * Resolve a live lease to a kernel-side pointer. The returned PA is stable while
 * the arena exists and the descriptor's offset is immutable while the lease is
 * live, so the pointer itself never dangles. The BYTES it addresses, however,
 * are governed by the lease lifecycle: the caller must hold a live lease in the
 * correct state (WRITING to produce, READING to consume) and MUST NOT touch the
 * pointer after releasing/transferring the lease -- once poisoned, the span may
 * be recycled by another core's acquire. Validation here is a point-in-time
 * check under the lock; it does not pin the descriptor after return.
 */
void *lease_ptr(const struct lease_descriptor *d)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    struct lease_descriptor *s;
    if (lease_check(d, &s) != LEASE_OK) { kspin_unlock(lk); return 0; }
    struct lease_arena *a = arena_lookup(s->arena_id);
    if (!a) { kspin_unlock(lk); return 0; }
    void *p = (void *)(usize)(a->base + s->offset);
    kspin_unlock(lk);
    return p;
}

void lease_get_stats(struct lease_stats *out)
{
    if (!out) return;
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    *out = g_stats;
    kspin_unlock(lk);
}

/* --- selftest: exercises the lifecycle, poison, transfer, copy-once,
 *     MMU grant/revoke (mock), handle round-trip, and TLS-direct-write. --- */
static u8 g_lease_test_kern[8192] ALIGNED(64);
static u8 g_lease_test_caps[8192] ALIGNED(64);

static void lease_selftest_cleanup(u32 karena, u32 carena)
{
    struct kspinlock *lk = lease_lock();
    kspin_lock(lk);
    for (u32 i = 0; i < LEASE_POOL_MAX; i++) {
        struct lease_descriptor *s = &g_pool[i];
        if (s->arena_id != karena && s->arena_id != carena)
            continue;
        if (s->state != LEASE_STATE_FREE && g_stats.slots_live)
            g_stats.slots_live--;
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
    }
    for (u32 i = 0; i < LEASE_ARENA_MAX; i++) {
        if (!g_arenas[i].in_use)
            continue;
        if (g_arenas[i].arena_id != karena && g_arenas[i].arena_id != carena)
            continue;
        lease_zero(&g_arenas[i], sizeof(g_arenas[i]));
        if (g_stats.arenas)
            g_stats.arenas--;
    }
    kspin_unlock(lk);
}

bool lease_selftest(void)
{
    bool ok = false;
    u32 karena = LEASE_ARENA_NONE;
    u32 carena = LEASE_ARENA_NONE;
    bool test_mmu_safe = false;
    lease_init();
    {
        struct kspinlock *lk = lease_lock();
        kspin_lock(lk);
        test_mmu_safe = (g_mmu_ops == 0);
        kspin_unlock(lk);
    }

    karena = lease_arena_register(LEASE_ARENA_KIND_KERNEL, LEASE_CACHE_WBC_OFF,
                                  LEASE_OWNER_KERNEL,
                                  (u64)(usize)g_lease_test_kern, sizeof(g_lease_test_kern));
    carena = lease_arena_register(LEASE_ARENA_KIND_CAPSULE, LEASE_CACHE_WBC_ON,
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
    /* lease_validate re-runs the slot/generation check under the lock; a
     * released (poisoned) descriptor must come back STALE. */
    if (lease_validate(&stale, LEASE_ARENA_NONE) != LEASE_ERR_STALE) goto done;
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
    if (test_mmu_safe && lease_grant(&dst, 9, LEASE_F_READONLY) != LEASE_OK) goto done;
    if (lease_grant(&src, 9, LEASE_F_READWRITE) != LEASE_ERR_INVAL) goto done; /* kernel arena */
    if (test_mmu_safe && lease_revoke(&dst, 9) != LEASE_OK) goto done;

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

    /* 8) Cross-core locking discipline: after EVERY error-returning API call the
     *    registry lock must be released (no leaked lock), and the non-recursive
     *    nested copy path must not have self-deadlocked (we only reach here if it
     *    didn't). We drive representative error paths through each public entry
     *    point and assert the shared lock byte is clear afterwards. */
    {
        struct kspinlock *lk = lease_lock();
        struct lease_descriptor bogus;
        lease_zero(&bogus, sizeof(bogus));   /* generation 0 -> always poisoned */

        /* Each of these must take an error branch and still unlock. */
        (void)lease_publish(&bogus);      if (lk->held) goto done;   /* STALE  */
        (void)lease_begin_read(&bogus);   if (lk->held) goto done;   /* STALE  */
        (void)lease_end_read(&bogus);     if (lk->held) goto done;   /* STALE  */
        (void)lease_release(&bogus);      if (lk->held) goto done;   /* STALE  */
        (void)lease_transfer(&bogus, 1);  if (lk->held) goto done;   /* STALE  */
        (void)lease_validate(&bogus, 0);  if (lk->held) goto done;   /* STALE  */
        (void)lease_grant(&bogus, 1, 0xBAD); if (lk->held) goto done;/* INVAL access */
        (void)lease_grant(&bogus, 1, LEASE_F_READONLY); if (lk->held) goto done; /* STALE */
        (void)lease_revoke(&bogus, 1);    if (lk->held) goto done;   /* STALE  */
        (void)lease_ptr(&bogus);          if (lk->held) goto done;   /* NULL   */
        (void)lease_acquire(0xDEAD, 16, 0, 0, 0, &bogus); if (lk->held) goto done; /* NOENT */
        (void)lease_copy_into(&bogus, carena, 7, 0, &bogus); if (lk->held) goto done; /* STALE src */
        (void)lease_arena_snapshot(0xDEAD, 0); if (lk->held) goto done; /* bad out ptr */

        /* Nested copy path once more with a genuinely live source, proving the
         * non-recursive lock composes (acquire+publish inside copy_into). */
        struct lease_descriptor csrc, cdst;
        if (lease_acquire(karena, 24, LEASE_F_KERNEL | LEASE_F_READWRITE,
                          LEASE_SRC_KERNEL_MEM, LEASE_OWNER_KERNEL, &csrc) != LEASE_OK)
            goto done;
        u8 *cp = (u8 *)lease_ptr(&csrc);
        for (u32 i = 0; i < 24; i++) cp[i] = (u8)(0x30 + i);
        if (lease_publish(&csrc) != LEASE_OK) goto done;
        if (lease_copy_into(&csrc, carena, 7, LEASE_F_CAPSULE | LEASE_F_READONLY,
                            &cdst) != LEASE_OK) goto done;
        if (lk->held) goto done;   /* copy_into must release the lock */
        if (cdst.length != 24) goto done;
        if (lease_release(&csrc) != LEASE_OK) goto done;
        if (lease_release(&cdst) != LEASE_OK) goto done;
        if (lk->held) goto done;
    }

    struct lease_stats st;
    lease_get_stats(&st);
    if (st.acquires < 4 || st.releases < 4 || st.copies < 1 ||
        st.transfers < 1 || st.stale_rejects < 1 || st.mmu_rejects < 1 ||
        (test_mmu_safe && (st.grants < 1 || st.revokes < 1)))
        goto done;

    ok = true;
done:
    lease_selftest_cleanup(karena, carena);
    return ok;
}
