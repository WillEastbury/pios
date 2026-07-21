#pragma once
#include "types.h"

/* Small syscall-path spinlock for short, bounded cross-core registry updates.
 * Scheduler code must not take these locks. Each lock owns a complete line so
 * contention metadata cannot share a cache line with protected state. */
struct kspinlock {
    volatile u8 held;
    u8 _pad[63];
} ALIGNED(64);

_Static_assert(sizeof(struct kspinlock) == 64,
               "kspinlock must own exactly one cache line");

static inline void kspin_lock(struct kspinlock *lock)
{
    u32 value;
    u32 failed;
    do {
        do {
            __asm__ volatile("ldaxrb %w0, [%1]"
                             : "=&r"(value)
                             : "r"(&lock->held)
                             : "memory");
            if (value)
                __asm__ volatile("yield");
        } while (value);
        __asm__ volatile("stxrb %w0, %w2, [%1]"
                         : "=&r"(failed)
                         : "r"(&lock->held), "r"(1U)
                         : "memory");
    } while (failed);
}

static inline void kspin_unlock(struct kspinlock *lock)
{
    __asm__ volatile("stlrb wzr, [%0]"
                     :: "r"(&lock->held) : "memory");
}
