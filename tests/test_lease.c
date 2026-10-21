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
 *     grant while the lock is dropped, the reservation refuses release until
 *     revocation has succeeded.
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
static u8 g_export[16384] ALIGNED(4096);

/* ---- mock MMU ops that re-enter the lease API from inside the callback ---- */
static u32 g_map_calls, g_unmap_calls;
static const struct lease_descriptor *g_release_on_map;  /* NULL = don't */
static i32 g_callback_release_result;
static i32 g_map_result, g_unmap_result;

static i32 mock_map(u32 capsule, u64 pa, u64 length, u32 access)
{
    (void)capsule; (void)pa; (void)length; (void)access;
    g_map_calls++;
    /* Prove the lock is NOT held during the callback: re-entering a lock-taking
     * lease API here must not deadlock. Simulate the concurrent-recycle race by
     * releasing the descriptor out from under the in-flight grant. */
    if (g_release_on_map)
        g_callback_release_result = lease_release(g_release_on_map);
    return g_map_result;
}
static i32 mock_unmap(u32 capsule, u64 pa, u64 length)
{
    (void)capsule; (void)pa; (void)length;
    g_unmap_calls++;
    return g_unmap_result;
}
static const struct lease_mmu_ops g_mock_ops = { .map = mock_map, .unmap = mock_unmap };
static u32 g_replacement_unmaps;
static i32 replacement_unmap(u32 capsule, u64 pa, u64 length)
{
    (void)capsule; (void)pa; (void)length;
    g_replacement_unmaps++;
    return LEASE_OK;
}
static const struct lease_mmu_ops g_replacement_ops = {
    .map = mock_map, .unmap = replacement_unmap
};

static void test_failed_grants_retain_authority(void)
{
    lease_init();
    lease_set_mmu_ops(0);
    u32 arena = lease_arena_register(LEASE_ARENA_KIND_CAPSULE, LEASE_CACHE_WBC_OFF,
                                    7, (u64)(usize)g_caps, sizeof(g_caps));
    struct lease_descriptor d;
    CHECK(lease_acquire(arena, 64, LEASE_F_READONLY, LEASE_SRC_CAPSULE_MEM, 7, &d) == 0,
          "acquire grant test");
    CHECK(lease_publish(&d) == 0, "publish grant test");
    CHECK(lease_grant(&d, 7, LEASE_F_READONLY) == LEASE_ERR_MMU, "no backend is not success");
    static const struct lease_mmu_ops map_only = {.map = mock_map};
    lease_set_mmu_ops(&map_only);
    g_map_calls = 0;
    CHECK(lease_grant(&d, 7, LEASE_F_READONLY) == LEASE_ERR_MMU, "unmap is mandatory");
    CHECK(g_map_calls == 0, "incomplete backend never maps");

    lease_set_mmu_ops(&g_mock_ops);
    g_map_result = LEASE_ERR_MMU;
    g_unmap_result = LEASE_ERR_MMU;
    CHECK(lease_grant(&d, 7, LEASE_F_READONLY) == LEASE_ERR_REVOKE,
          "partial map plus failed cleanup reports retained authority");
    CHECK(lease_release(&d) == LEASE_ERR_BUSY, "failed cleanup pins descriptor");
    CHECK(lease_transfer(&d, 8) == LEASE_ERR_BUSY, "failed cleanup forbids transfer");
    CHECK(lease_arena_reset(arena) == LEASE_ERR_BUSY, "failed cleanup pins arena");
    CHECK(lease_revoke(&d, 8) == LEASE_ERR_AUTHORITY, "wrong recipient cannot revoke");
    CHECK(lease_revoke(&d, 7) == LEASE_ERR_MMU, "failed revoke remains explicit");
    CHECK(lease_release(&d) == LEASE_ERR_BUSY, "failed revoke retains pin");
    g_unmap_result = 0;
    CHECK(lease_revoke(&d, 7) == 0, "retry completes original revocation");

    g_map_result = 0;
    CHECK(lease_grant(&d, 7, LEASE_F_READONLY) == 0, "regrant after successful revoke");
    CHECK(lease_grant(&d, 8, LEASE_F_READONLY) == LEASE_ERR_BUSY, "exclusive recipient");
    CHECK(lease_begin_read(&d) == LEASE_ERR_BUSY, "mapped lifecycle stays pinned");
    g_unmap_calls = g_replacement_unmaps = 0;
    lease_set_mmu_ops(&g_replacement_ops);
    CHECK(lease_revoke(&d, 7) == 0, "backend replacement retains original unmap");
    CHECK(g_unmap_calls == 1 && !g_replacement_unmaps, "uses issuing backend, not replacement");
    CHECK(lease_release(&d) == 0, "release after revoke");
    CHECK(lease_arena_reset(arena) == 0, "reset after last allocation release");
    lease_set_mmu_ops(0);
}

static void test_export_bounds_and_reset(void)
{
    lease_init();
    u32 arena = lease_arena_register(LEASE_ARENA_KIND_CAPSULE, LEASE_CACHE_WBC_OFF,
                                    0, (u64)(usize)g_export, sizeof(g_export));
    CHECK(lease_arena_register(LEASE_ARENA_KIND_CAPSULE, LEASE_CACHE_WBC_OFF,
                               8, (u64)(usize)g_export + 64, 64) == LEASE_ARENA_NONE,
          "overlapping arena cannot create another authority domain");
    CHECK(lease_arena_register(LEASE_ARENA_KIND_CAPSULE, LEASE_CACHE_WBC_OFF,
                               8, ~(u64)0 - 3U, 16) == LEASE_ARENA_NONE,
          "arena range overflow rejected");
    struct lease_descriptor d, next;
    CHECK(lease_acquire(arena, ~(u64)0, LEASE_F_READWRITE, LEASE_SRC_KERNEL_MEM, 0, &d) ==
          LEASE_ERR_INVAL, "allocation rounding cannot wrap");
    memset(g_export, 0xA5, sizeof(g_export));
    CHECK(lease_acquire(arena, 64, LEASE_F_EXPORTABLE | LEASE_F_READWRITE,
                        LEASE_SRC_KERNEL_MEM, 0, &d) == 0, "exportable allocation");
    for (u32 i = 0; i < 4096U; i++)
        CHECK(g_export[i] == 0, "page padding initialized before export");
    CHECK(lease_acquire(arena, 32, LEASE_F_EXPORTABLE | LEASE_F_READWRITE,
                        LEASE_SRC_KERNEL_MEM, 0, &next) == 0, "second export allocation");
    CHECK(next.offset >= d.offset + 4096U, "exports never share a page");
    CHECK(lease_arena_reset(arena) == LEASE_ERR_BUSY, "live non-granted allocation pins arena");
    CHECK(lease_release(&d) == 0 && lease_release(&next) == 0, "release export allocations");
    CHECK(lease_arena_reset(arena) == 0, "all released permits reset");
}

static void test_process_tickets(void)
{
    lease_init();
    lease_set_mmu_ops(0);
    u32 arena = lease_arena_register(LEASE_ARENA_KIND_CAPSULE, LEASE_CACHE_WBC_OFF,
                                    0, (u64)(usize)g_export, sizeof(g_export));
    struct lease_descriptor d;
    CHECK(lease_acquire(arena, 64, LEASE_F_EXPORTABLE | LEASE_F_READWRITE,
                        LEASE_SRC_KERNEL_MEM, 0, &d) == 0, "acquire process handoff");
    memset(lease_ptr(&d), 0xA5, 64);
    CHECK(lease_publish(&d) == 0, "publish process handoff");
    struct lease_ticket ticket, out = {.grant_id = 0xAABB};
    CHECK(lease_ticket_prepare(&d, 41, 7, LEASE_F_READONLY, 16, &ticket) == 0,
          "prepare incarnation-bound ticket");
    CHECK(ticket.length == 16 && ticket.capacity == 64, "logical bounds are not page padding");
    for (u32 i = 16; i < 4096U; i++)
        CHECK(g_export[i] == 0, "unadvertised bytes cannot leak through mapped padding");
    CHECK(lease_ticket_redeem(&ticket, 41, 7, LEASE_F_READONLY, &out) == LEASE_ERR_STATE &&
          out.grant_id == 0xAABB, "prepared ticket grants no access or success output");
    CHECK(lease_ptr(&d) == 0, "producer cannot reacquire an offered pointer");
    CHECK(lease_release(&d) == LEASE_ERR_BUSY, "prepared ticket pins allocation");
    struct lease_descriptor copied;
    CHECK(lease_copy_into(&d, arena, 0, LEASE_F_READONLY, &copied) == LEASE_ERR_BUSY,
          "copy path cannot bypass a reserved process grant");
    for (u32 field = 0; field < 11U; field++) {
        struct lease_ticket forged = ticket;
        switch (field) {
        case 0: forged.recipient_pid++; break;
        case 1: forged.recipient_incarnation++; break;
        case 2: forged.access = LEASE_F_READWRITE; break;
        case 3: forged.offset = ~(u64)0; break;
        case 4: forged.length++; break;
        case 5: forged.capacity++; break;
        case 6: forged.reserved = 1; break;
        case 7: forged.lease.generation++; break;
        case 8: forged.lease.lease_id++; break;
        case 9: forged.grant_id++; break;
        case 10: forged.lease.slot_id = LEASE_POOL_MAX; break;
        }
        struct lease_mapping mapping = {.grant_id = 0xAABB};
        CHECK(lease_ticket_map_begin(&forged, &mapping) != 0 &&
              mapping.grant_id == 0xAABB, "forgery cannot begin mapping or change output");
    }
    struct lease_mapping mapping;
    CHECK(lease_ticket_map_begin(&ticket, &mapping) == 0, "begin owner-routed map");
    CHECK(mapping.pa == (u64)(usize)g_export && mapping.extent == 4096U &&
          mapping.cache_policy == LEASE_CACHE_WBC_OFF, "canonical MMU range and attributes");
    CHECK(lease_ticket_cancel(&ticket) == LEASE_ERR_BUSY, "in-flight map cannot be cancelled");
    CHECK(lease_ticket_map_complete(&ticket, 41, 8, 0) == LEASE_ERR_AUTHORITY,
          "wrong-incarnation ACK cannot publish authority");
    CHECK(lease_ticket_map_complete(&ticket, 41, 7, 0) == 0, "matching map ACK activates grant");
    CHECK(lease_ticket_redeem(&ticket, 42, 7, LEASE_F_READONLY, &out) == LEASE_ERR_AUTHORITY,
          "forwarded ticket rejected");
    CHECK(lease_ticket_redeem(&ticket, 41, 8, LEASE_F_READONLY, &out) == LEASE_ERR_AUTHORITY,
          "same PID different incarnation rejected");
    CHECK(lease_ticket_redeem(&ticket, 41, 7, LEASE_F_READWRITE, &out) == LEASE_ERR_AUTHORITY,
          "read-only ticket cannot escalate");
    CHECK(lease_ticket_redeem(&ticket, 41, 7, LEASE_F_READONLY, &out) == 0 &&
          out.grant_id == ticket.grant_id && out.length == 16, "matching recipient redeems");
    CHECK(lease_ticket_redeem(&ticket, 41, 7, LEASE_F_READONLY, &out) == LEASE_ERR_STALE,
          "same-incarnation redemption replay rejected");
    CHECK(lease_ticket_map_complete(&ticket, 41, 7, 0) == LEASE_ERR_STATE, "duplicate map ACK rejected");
    CHECK(lease_ticket_revoke_begin(&ticket, &mapping) == 0, "begin revoke");
    CHECK(lease_ticket_redeem(&ticket, 41, 7, LEASE_F_READONLY, &out) == LEASE_ERR_STATE,
          "revoking grant cannot be redeemed");
    CHECK(lease_ticket_revoke_complete(&ticket, 41, 7, LEASE_ERR_MMU) == LEASE_ERR_MMU,
          "failed revoke ACK explicit");
    CHECK(lease_release(&d) == LEASE_ERR_BUSY && lease_arena_reset(arena) == LEASE_ERR_BUSY,
          "failed revoke cannot release or rewind");
    CHECK(lease_ticket_revoke_begin(&ticket, &mapping) == 0 &&
          lease_ticket_revoke_complete(&ticket, 41, 7, 0) == 0, "retry proves revocation");
    CHECK(lease_ticket_redeem(&ticket, 41, 7, LEASE_F_READONLY, &out) == LEASE_ERR_STALE,
          "revoked ticket stale");
    struct lease_ticket again;
    CHECK(lease_ticket_prepare(&d, 41, 8, LEASE_F_READONLY, 16, &again) == 0 &&
          again.grant_id != ticket.grant_id, "new incarnation has a distinct grant");
    CHECK(lease_ticket_map_complete(&ticket, 41, 7, 0) == LEASE_ERR_STALE,
          "old ACK cannot activate a new grant");
    CHECK(lease_ticket_cancel(&again) == 0 && lease_release(&d) == 0,
          "unmapped preparation cancels safely");

    CHECK(lease_acquire(arena, 64, LEASE_F_EXPORTABLE | LEASE_F_READWRITE,
                        LEASE_SRC_KERNEL_MEM, 0, &d) == 0, "acquire mutable handoff");
    CHECK(lease_ticket_prepare(&d, 41, 9, LEASE_F_READWRITE, 0, &again) == 0,
          "empty mutable output lease");
    CHECK(lease_ticket_map_begin(&again, &mapping) == 0 &&
          lease_ticket_map_complete(&again, 41, 9, LEASE_ERR_MMU) == LEASE_ERR_MMU,
          "failed map ACK quarantines uncertain authority");
    CHECK(lease_release(&d) == LEASE_ERR_BUSY, "failed map is not a release");
    CHECK(lease_ticket_revoke_begin(&again, &mapping) == 0 &&
          lease_ticket_revoke_complete(&again, 41, 9, 0) == 0 &&
          lease_release(&d) == 0, "uncertain map requires successful revoke before reuse");

    CHECK(lease_acquire(arena, 32, LEASE_F_EXPORTABLE | LEASE_F_READWRITE,
                        LEASE_SRC_KERNEL_MEM, 0, &d) == 0, "acquire mutable return");
    CHECK(lease_ticket_prepare(&d, 41, 10, LEASE_F_READWRITE, 0, &again) == 0 &&
          lease_ticket_map_begin(&again, &mapping) == 0 &&
          lease_ticket_map_complete(&again, 41, 10, 0) == 0, "activate mutable return");
    CHECK(lease_ticket_redeem(&again, 41, 10, LEASE_F_READWRITE, &out) == 0,
          "intended recipient redeems mutable grant");
    g_export[d.offset] = 0x5A;
    CHECK(lease_publish(&d) == LEASE_ERR_BUSY, "producer cannot publish while recipient owns bytes");
    CHECK(lease_ticket_revoke_begin(&again, &mapping) == 0 &&
          lease_ticket_revoke_complete(&again, 41, 10, 0) == 0, "return mutable grant");
    CHECK(lease_ptr(&d) && *(u8 *)lease_ptr(&d) == 0x5A &&
          lease_publish(&d) == 0 && lease_release(&d) == 0,
          "only completed revocation returns producer access");
}

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

    /* Re-entry must not deadlock or recycle bytes before map completes. */
    struct lease_descriptor d2;
    CHECK(lease_acquire(ca, 64, LEASE_F_CAPSULE | LEASE_F_READWRITE,
                        LEASE_SRC_CAPSULE_MEM, 7, &d2) == LEASE_OK, "acquire lease2");
    CHECK(lease_publish(&d2) == LEASE_OK, "publish2");
    g_map_calls = g_unmap_calls = 0;
    g_release_on_map = &d2;
    i32 rc = lease_grant(&d2, 7, LEASE_F_READONLY);
    g_release_on_map = 0;
    CHECK(rc == LEASE_OK, "pinned grant completes despite attempted concurrent release");
    CHECK(g_callback_release_result == LEASE_ERR_BUSY, "in-flight map pins descriptor");
    CHECK(g_map_calls == 1, "map attempted once");
    CHECK(g_unmap_calls == 0, "successful map is not spuriously rolled back");
    CHECK(LOCK_FREE(), "grant rollback unlocks");
    CHECK(lease_release(&d2) == LEASE_ERR_BUSY, "active map retains descriptor");
    CHECK(lease_revoke(&d2, 7) == LEASE_OK, "revoke pinned grant");
    CHECK(lease_release(&d2) == LEASE_OK, "release after acknowledged revoke");

    lease_set_mmu_ops(0);
}

int main(void)
{
    printf("test_lease:\n");
    test_lock_release_on_error();
    test_nested_copy_no_deadlock();
    test_arena_snapshot_immutability();
    test_grant_callback_reentry_rollback();
    test_failed_grants_retain_authority();
    test_export_bounds_and_reset();
    test_process_tickets();

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
