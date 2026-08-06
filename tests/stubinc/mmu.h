/*
 * Host stub for mmu.h, used only by the lease host unit test build.
 *
 * lease.c pulls in mmu.h purely for the cache-maintenance range helpers. On the
 * host there are no caches to maintain, so these are no-ops. Put tests/stubinc
 * on the include path BEFORE include/ so this shim shadows the bare-metal header
 * (which is full of AArch64 asm that will not compile on the host). Only TUs
 * that actually include mmu.h pick this up.
 */
#pragma once
#include "types.h"

static inline void dcache_clean_range(u64 start, u64 size) { (void)start; (void)size; }
static inline void dcache_invalidate_range(u64 start, u64 size) { (void)start; (void)size; }
