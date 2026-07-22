/*
 * Host stub for kspin.h, used only by the lease host unit test build.
 *
 * The real kspin.h places locks at a fixed physical control-page address and
 * uses AArch64 load-acquire/store-exclusive; neither is valid on the host. This
 * shim provides an equivalent single-address-space spinlock backed by a static
 * array of cache-line-owning locks, so lease.c's registry serialization logic
 * (and the `->held` assertions in the host test) compile and behave sensibly on
 * the host. The host test is single-threaded, so a plain byte flag is exact.
 *
 * Put tests/stubinc on the include path BEFORE include/ so this shadows the
 * bare-metal header. Only TUs that include kspin.h pick this up.
 */
#pragma once
#include "types.h"

struct kspinlock {
    volatile u8 held;
    u8 _pad[63];
} ALIGNED(64);

#define KSPIN_SHARED_SLOTS 16U

/* Single shared definition across all TUs in the host-test link (defined in
 * tests/test_lease.c) so the test can observe the exact lock byte lease.c uses. */
extern struct kspinlock g_kspin_host_slots[KSPIN_SHARED_SLOTS];
extern int g_kspin_host_recursive_lock;

static inline struct kspinlock *kspin_shared(u32 slot)
{
    return &g_kspin_host_slots[slot % KSPIN_SHARED_SLOTS];
}

static inline void kspin_init(struct kspinlock *lock) { lock->held = 0; }

static inline void kspin_lock(struct kspinlock *lock)
{
    /* Fail observably if a `_locked` helper accidentally calls a public
     * wrapper and tries to recursively acquire the non-recursive target lock. */
    if (lock->held) {
        g_kspin_host_recursive_lock++;
        return;
    }
    lock->held = 1;
}

static inline void kspin_unlock(struct kspinlock *lock) { lock->held = 0; }
