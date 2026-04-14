/*
 * core_env.h - Per-core RAM isolation and environment
 *
 * Each core gets 16MB of private RAM (heap/buffers).
 * Shared regions for FIFO and DMA are at fixed addresses.
 * User cores additionally apply per-process MMU slot windows at runtime.
 *
 * Memory Map (Pi 5, assuming ≥1GB RAM):
 *   0x00080000 - 0x001FFFFF  Kernel image + BSS + stacks (~1.5MB)
 *   0x00200000 - 0x011FFFFF  Core 0 private (16MB) - Network
 *   0x01200000 - 0x021FFFFF  Core 1 private (16MB) - Disk I/O
 *   0x02200000 - 0x031FFFFF  Core 2 private (16MB) - User 0
 *   0x03200000 - 0x041FFFFF  Core 3 private (16MB) - User 1
 *   0x04200000 - 0x042FFFFF  Shared FIFO ring (1MB)
 *   0x04300000 - 0x044FFFFF  DMA NET buffers (2MB)
 *   0x04500000 - 0x046FFFFF  DMA DISK buffers (2MB)
 *   0x04700000 - 0x047FFFFF  Shared process IPC SHM pool (1MB)
 */

#pragma once
#include "types.h"

#define CORE_PRIV_SIZE      0x01000000UL  /* 16MB per core */

#define CORE0_RAM_BASE      0x00200000UL
#define CORE1_RAM_BASE      0x01200000UL
#define CORE2_RAM_BASE      0x02200000UL
#define CORE3_RAM_BASE      0x03200000UL

#define SHARED_FIFO_BASE    0x04200000UL
#define SHARED_FIFO_SIZE    0x00100000UL  /* 1MB */

#define DMA_NET_BASE        0x04300000UL
#define DMA_NET_SIZE        0x00200000UL  /* 2MB */

#define DMA_DISK_BASE       0x04500000UL
#define DMA_DISK_SIZE       0x00200000UL  /* 2MB */

#define IPC_SHM_BASE        0x04700000UL
#define IPC_SHM_SIZE        0x00100000UL  /* 1MB */

static const u64 core_ram_bases[4] = {
    CORE0_RAM_BASE, CORE1_RAM_BASE, CORE2_RAM_BASE, CORE3_RAM_BASE
};

struct core_env {
    u32  id;
    u8  *ram_base;
    u8  *ram_end;
    u8  *heap_ptr;      /* bump allocator watermark */

    /* Counters */
    u64  msg_sent;
    u64  msg_recv;
    u64  poll_count;
    u64  idle_count;
    u64  bytes_processed;
} ALIGNED(64);

/* Validate a pointer+length falls entirely within a core's private RAM */
static inline bool ptr_in_core_ram(u32 core, u64 ptr, u32 len) {
    u64 base = core_ram_bases[core & 3];
    u64 end  = base + CORE_PRIV_SIZE;
    return ptr >= base && ptr + len <= end && ptr + len >= ptr;
}

/* One env per core, stored at start of each core's private RAM */
static inline struct core_env *core_env_of(u32 id) {
    return (struct core_env *)(usize)core_ram_bases[id & 3];
}

/* Initialise a core's environment (call once per core at startup) */
static inline void core_env_init(u32 id) {
    struct core_env *e = core_env_of(id);
    e->id             = id;
    e->ram_base       = (u8 *)(usize)core_ram_bases[id & 3];
    e->ram_end        = e->ram_base + CORE_PRIV_SIZE;
    /* heap starts after the env struct, 64-byte aligned */
    e->heap_ptr       = (u8 *)(((usize)e + sizeof(*e) + 63) & ~63UL);
    e->msg_sent       = 0;
    e->msg_recv       = 0;
    e->poll_count     = 0;
    e->idle_count     = 0;
    e->bytes_processed = 0;
}

/* Bump allocator from core's private 16MB region.
 * Returns NULL if out of space. Never frees. */
static inline void *core_alloc(u32 id, usize size, usize align) {
    struct core_env *e = core_env_of(id);
    usize ptr = ((usize)e->heap_ptr + align - 1) & ~(align - 1);
    usize end = ptr + size;
    if (end > (usize)e->ram_end)
        return NULL;
    e->heap_ptr = (u8 *)end;
    return (void *)ptr;
}

/* Read the ARM generic timer (cycle-level timing) */
static inline u64 read_cntvct(void) {
    u64 v;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(v));
    return v;
}

static inline u64 read_cntfrq(void) {
    u64 v;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(v));
    return v;
}
