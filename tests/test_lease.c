/*
 * test_lease.c - host unit test for the lease registry's cross-core locking.
 *
 * Built by tests/run_host_tests.py with tests/stubinc ahead of include/, so
 * "types.h", "mmu.h" and "kspin.h" resolve to the host shims. lease.c is the
 * module under test.
 *
 * The board is where true multi-core stress lives (the in-kernel lease_selftest
 * runs live under qemu_smoke). This host test deterministically pins the
 * serialization CONTRACT that the locking must uphold:
 *   - every public entry point releases the registry lock on every error path
 *     (no leaked lock),
 *   - the non-recursive nested copy path (lease_copy_into -> acquire+publish)
 *     composes without self-deadlock,
 *   - MMU map/unmap callbacks run with the lock DROPPED, so a callback may
 *     re-enter the lease API; and if the descriptor is released underneath a
 *     grant while the lock is dropped, the grant rolls the mapping back and
 *     fails STALE (the snapshot / relock / revalidate linearization).
 */

#include <stdio.h>
#include <string.h>
#include "types.h"   /* resolves to tests/stubinc/types.h (sets host shim guard) */
#include "lease.h"
#include "kspin.h"   /* host shim: lets us inspect the lock byte directly */

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("  FAIL: %s\n", msg); failures++; } } while (0)

/* Single definition of the host-shim shared lock array (declared extern in the
 * kspin.h shim) so lease.c and this TU operate on the very same lock bytes. */
struct kspinlock g_kspin_host_slots[KSPIN_SHARED_SLOTS];
int g_kspin_host_recursive_lock;

/* Lease registry lock lives in slot 3 (see src/lease.c). */
static volatile u8 *lease_lock_byte(void) { return &kspin_shared(3)->held; }
#define LOCK_FREE()  (*lease_lock_byte() == 0)

static u8 g_kern[8192] ALIGNED(64);
static u8 g_caps[8192] ALIGNED(64);

/* ---- mock MMU ops that re-enter the lease API from inside the callback ---- */
static u32 g_map_calls, g_unmap_calls;
static const struct lease_descriptor *g_release_on_map;  /* NULL = don't */

static i32 mock_map(u32 capsule, u64 pa, u64 length, u32 access)
{
    (void)capsule; (void)pa; (void)length; (void)access;
    g_map_calls++;
    /* Prove the lock is NOT held during the callback: re-entering a lock-taking
     * lease API here must not deadlock. Simulate the concurrent-recycle race by
     * releasing the descriptor out from under the in-flight grant. */
    if (g_release_on_map)
        (void)lease_release(g_release_on_map);
    return LEASE_OK;
}
static i32 mock_unmap(u32 capsule, u64 pa, u64 length)
{
    (void)capsule; (void)pa; (void)length;
    g_unmap_calls++;
    return LEASE_OK;
}
static const struct lease_mmu_ops g_mock_ops = { .map = mock_map, .unmap = mock_unmap };

static void test_lock_release_on_error(void)
{
    lease_init();
    CHECK(LOCK_FREE(), "lock free after init");

    struct lease_descriptor bogus;
    memset(&bogus, 0, sizeof(bogus));   /* generation 0 => always poisoned */

    (void)lease_publish(&bogus);            CHECK(LOCK_FREE(), "publish err unlocks");
    (void)lease_begin_read(&bogus);         CHECK(LOCK_FREE(), "begin_read err unlocks");
    (void)lease_end_read(&bogus);           CHECK(LOCK_FREE(), "end_read err unlocks");
    (void)lease_release(&bogus);            CHECK(LOCK_FREE(), "release err unlocks");
    (void)lease_transfer(&bogus, 1);        CHECK(LOCK_FREE(), "transfer err unlocks");
    (void)lease_validate(&bogus, 0);        CHECK(LOCK_FREE(), "validate err unlocks");
    (void)lease_grant(&bogus, 1, 0xBAD);    CHECK(LOCK_FREE(), "grant bad-access unlocks");
    (void)lease_grant(&bogus, 1, LEASE_F_READONLY); CHECK(LOCK_FREE(), "grant stale unlocks");
    (void)lease_revoke(&bogus, 1);          CHECK(LOCK_FREE(), "revoke err unlocks");
    (void)lease_ptr(&bogus);                CHECK(LOCK_FREE(), "ptr err unlocks");
    (void)lease_acquire(0xDEAD, 16, 0, 0, 0, &bogus); CHECK(LOCK_FREE(), "acquire noent unlocks");
    (void)lease_copy_into(&bogus, 1, 7, 0, &bogus);   CHECK(LOCK_FREE(), "copy_into stale unlocks");
    (void)lease_arena_snapshot(0xDEAD, 0);  CHECK(LOCK_FREE(), "snapshot bad-out unlocks");
}

static void test_nested_copy_no_deadlock(void)
{
    lease_init();
    u32 ka = lease_arena_register(LEASE_ARENA_KIND_KERNEL, LEASE_CACHE_WBC_OFF,
                                  LEASE_OWNER_KERNEL, (u64)(usize)g_kern, sizeof(g_kern));
    u32 ca = lease_arena_register(LEASE_ARENA_KIND_CAPSULE, LEASE_CACHE_WBC_ON,
                                  7, (u64)(usize)g_caps, sizeof(g_caps));
    CHECK(ka != LEASE_ARENA_NONE && ca != LEASE_ARENA_NONE, "arenas registered");

    struct lease_descriptor src, dst;
    CHECK(lease_acquire(ka, 40, LEASE_F_KERNEL | LEASE_F_READWRITE,
                        LEASE_SRC_KERNEL_MEM, LEASE_OWNER_KERNEL, &src) == LEASE_OK,
          "acquire src");
    u8 *sp = (u8 *)lease_ptr(&src);
    CHECK(sp != 0, "src ptr");
    for (u32 i = 0; i < 40; i++) sp[i] = (u8)(0x11 + i);
    CHECK(lease_publish(&src) == LEASE_OK, "publish src");

    /* If copy_into took the public (non-recursive) lock it would deadlock here;
     * reaching the assertion proves the _locked composition works. */
    CHECK(lease_copy_into(&src, ca, 7, LEASE_F_CAPSULE | LEASE_F_READONLY, &dst) == LEASE_OK,
          "nested copy_into composes");
    CHECK(LOCK_FREE(), "copy_into unlocks");
    CHECK(g_kspin_host_recursive_lock == 0, "copy_into did not recursively lock");
    CHECK(dst.length == 40 && dst.owner_capsule == 7, "copy dst shape");

    CHECK(lease_begin_read(&dst) == LEASE_OK, "begin_read dst");
    u8 *dp = (u8 *)lease_ptr(&dst);
    int match = dp != 0;
    for (u32 i = 0; i < 40; i++) if (dp[i] != (u8)(0x11 + i)) match = 0;
    CHECK(match, "copied bytes match");
    CHECK(lease_end_read(&dst) == LEASE_OK, "end_read dst");
    CHECK(lease_release(&src) == LEASE_OK, "release src");
    CHECK(lease_release(&dst) == LEASE_OK, "release dst");
}

static void test_arena_snapshot_immutability(void)
{
    lease_init();
    u32 ka = lease_arena_register(LEASE_ARENA_KIND_KERNEL, LEASE_CACHE_WBC_OFF,
                                  LEASE_OWNER_KERNEL, (u64)(usize)g_kern, sizeof(g_kern));
    struct lease_arena snap0;
    CHECK(lease_arena_snapshot(ka, &snap0), "snapshot ok");
    CHECK(snap0.cursor == 0, "cursor starts 0");

    struct lease_descriptor d;
    CHECK(lease_acquire(ka, 128, LEASE_F_KERNEL, LEASE_SRC_KERNEL_MEM,
                        LEASE_OWNER_KERNEL, &d) == LEASE_OK, "acquire bumps cursor");
    struct lease_arena snap1;
    CHECK(lease_arena_snapshot(ka, &snap1), "snapshot2 ok");
    CHECK(snap1.cursor >= 128, "cursor advanced in fresh snapshot");
    /* Identity fields immutable across snapshots. */
    CHECK(snap0.base == snap1.base && snap0.size == snap1.size &&
          snap0.arena_id == snap1.arena_id && snap0.kind == snap1.kind,
          "identity fields immutable");
    CHECK(lease_release(&d) == LEASE_OK, "release");
}

static void test_grant_callback_reentry_rollback(void)
{
    lease_init();
    lease_set_mmu_ops(&g_mock_ops);
    g_map_calls = g_unmap_calls = 0;

    u32 ca = lease_arena_register(LEASE_ARENA_KIND_CAPSULE, LEASE_CACHE_WBC_ON,
                                  7, (u64)(usize)g_caps, sizeof(g_caps));
    struct lease_descriptor d;
    CHECK(lease_acquire(ca, 64, LEASE_F_CAPSULE | LEASE_F_READWRITE,
                        LEASE_SRC_CAPSULE_MEM, 7, &d) == LEASE_OK, "acquire capsule lease");
    CHECK(lease_publish(&d) == LEASE_OK, "publish");
    CHECK(lease_begin_read(&d) == LEASE_OK, "begin_read");

    /* Case A: normal grant/revoke, callback re-enters no lease API. */
    g_release_on_map = 0;
    CHECK(lease_grant(&d, 7, LEASE_F_READONLY) == LEASE_OK, "grant ok");
    CHECK(g_map_calls == 1, "map called once");
    CHECK(LOCK_FREE(), "grant unlocks");
    CHECK(lease_revoke(&d, 7) == LEASE_OK, "revoke ok");
    CHECK(g_unmap_calls == 1, "unmap called once");
    CHECK(LOCK_FREE(), "revoke unlocks");

    /* Case B: the map callback re-enters the lease API and releases the
     * descriptor (simulating a concurrent recycle). The grant must detect the
     * generation change on relock, roll the mapping back (unmap), and fail
     * STALE -- and it must NOT deadlock re-entering lease_release from inside
     * the callback (proving the lock was dropped for the callback). */
    struct lease_descriptor d2;
    CHECK(lease_acquire(ca, 64, LEASE_F_CAPSULE | LEASE_F_READWRITE,
                        LEASE_SRC_CAPSULE_MEM, 7, &d2) == LEASE_OK, "acquire lease2");
    CHECK(lease_publish(&d2) == LEASE_OK, "publish2");
    g_map_calls = g_unmap_calls = 0;
    g_release_on_map = &d2;
    i32 rc = lease_grant(&d2, 7, LEASE_F_READONLY);
    g_release_on_map = 0;
    CHECK(rc == LEASE_ERR_STALE, "grant rolls back to STALE on concurrent release");
    CHECK(g_map_calls == 1, "map attempted once");
    CHECK(g_unmap_calls == 1, "mapping rolled back via unmap");
    CHECK(LOCK_FREE(), "grant rollback unlocks");

    lease_set_mmu_ops(0);
}

int main(void)
{
    printf("test_lease:\n");
    test_lock_release_on_error();
    test_nested_copy_no_deadlock();
    test_arena_snapshot_immutability();
    test_grant_callback_reentry_rollback();

    /* Finally, run the module's own end-to-end self-test (includes its
     * lock-release + nested-copy section 8). It intentionally drives stale,
     * invalid-state and MMU-denial paths, but must not leave those diagnostic
     * counters looking like real runtime failures. */
    struct lease_stats before;
    struct lease_stats after;
    lease_get_stats(&before);
    CHECK(lease_selftest(), "in-module lease_selftest passes");
    lease_get_stats(&after);
    CHECK(after.mmu_rejects == before.mmu_rejects,
          "selftest restores MMU reject counter");
    CHECK(after.stale_rejects == before.stale_rejects,
          "selftest restores stale reject counter");
    CHECK(after.state_rejects == before.state_rejects,
          "selftest restores state reject counter");
    CHECK(LOCK_FREE(), "lock free after selftest");

    if (failures == 0) { printf("  OK: all lease cross-core locking assertions passed\n"); return 0; }
    printf("  %d assertion(s) failed\n", failures);
    return 1;
}
