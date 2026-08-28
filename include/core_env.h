/*
 * core_env.h - Per-core RAM isolation and environment
 *
 * Each core gets 16MB of private RAM (heap/buffers).
 * Shared regions for FIFO and DMA are at fixed addresses.
 * User cores additionally apply per-process MMU slot windows at runtime.
 *
 * Memory Map (Pi 5, assuming ≥1GB RAM):
 *   0x00080000 - <__heap_start> Kernel image + BSS + stacks/static reservations
 *   0x00800000 - 0x017FFFFF  Core 0 private (16MB) - Network
 *   0x01800000 - 0x027FFFFF  Core 1 private (16MB) - Disk I/O
 *   0x02800000 - 0x037FFFFF  Core 2 private (16MB) - User 0
 *   0x03800000 - 0x047FFFFF  Core 3 private (16MB) - User 1
 *   0x04800000 - 0x048FFFFF  Shared FIFO ring (1MB)
 *   0x04900000 - 0x04AFFFFF  DMA NET buffers (2MB)
 *   0x04B00000 - 0x04CFFFFF  DMA DISK buffers (2MB)
 *   0x04D00000 - 0x04DFFFFF  Shared process IPC SHM pool (1MB)
 *   0x05000000 - 0x05FFFFFF  HDMI double-buffer back buffer (16MB)
 *
 * QEMU's equivalent map is offset upward by 2MB to leave room for the larger
 * kernel image while preserving the 0x48000000 stage0 staging window:
 *   0x42200000 - 0x431FFFFF  Core 0 private (16MB)
 *   0x43200000 - 0x441FFFFF  Core 1 private (16MB)
 *   0x44200000 - 0x451FFFFF  Core 2 private (16MB)
 *   0x45200000 - 0x461FFFFF  Core 3 private (16MB)
 *   0x46200000 - 0x462FFFFF  Shared FIFO ring (1MB)
 *   0x46300000 - 0x464FFFFF  DMA NET buffers (2MB)
 *   0x46500000 - 0x466FFFFF  DMA DISK buffers (2MB)
 *   0x46700000 - 0x467FFFFF  Shared process IPC SHM pool (1MB)
 *   0x46A00000 - 0x479FFFFF  HDMI double-buffer back buffer (16MB)
 */

#pragma once
#include "types.h"
#include "platform.h"

#define CORE_PRIV_SIZE      PIOS_CORE_PRIV_SIZE

#define CORE0_RAM_BASE      PIOS_CORE0_RAM_BASE
#define CORE1_RAM_BASE      PIOS_CORE1_RAM_BASE
#define CORE2_RAM_BASE      PIOS_CORE2_RAM_BASE
#define CORE3_RAM_BASE      PIOS_CORE3_RAM_BASE

#define SHARED_FIFO_BASE    PIOS_SHARED_FIFO_BASE
#define SHARED_FIFO_SIZE    PIOS_SHARED_FIFO_SIZE

#define DMA_NET_BASE        PIOS_DMA_NET_BASE
#define DMA_NET_SIZE        PIOS_DMA_NET_SIZE

#define DMA_DISK_BASE       PIOS_DMA_DISK_BASE
#define DMA_DISK_SIZE       PIOS_DMA_DISK_SIZE

#define IPC_SHM_BASE        PIOS_IPC_SHM_BASE
#define IPC_SHM_SIZE        PIOS_IPC_SHM_SIZE

/* HDMI double-buffer: rendering goes to this cached back buffer, then a DMA
 * blit pushes dirty rows to the VideoCore scanout buffer. Sized for 1080p32
 * (1920*1080*4 = 8.29MB) with headroom for pitch padding. */
#define FB_BACK_BASE        PIOS_FB_BACK_BASE
#define FB_BACK_SIZE        PIOS_FB_BACK_SIZE

/* Global process arena (ADR-024). Process slots are allocated from here rather
 * than from the owning core's private RAM, so process memory no longer belongs
 * to a core. Mapped Normal-WB at the first MMU enable. */
#define PROC_ARENA_BASE     PIOS_PROC_ARENA_BASE
#define PROC_ARENA_SIZE     PIOS_PROC_ARENA_SIZE

/* Core 0's first 4 KiB is reserved for the core_env owner record plus small
 * cross-core control objects that require WB Inner-Shareable memory (notably
 * exclusive-access spinlocks). The bump allocator starts after this page. */
#define CORE0_CONTROL_RESERVE  0x1000UL
#define CORE0_AIRQ_ATOMIC_BASE (CORE0_RAM_BASE + 0x500UL)
#define CORE0_AIRQ_ATOMIC_SIZE 0x100UL

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
    /* Core 0 reserves its first page for shared WB control records. Other
     * cores retain the compact env-followed-by-heap layout. */
    usize heap_start = ((usize)e + sizeof(*e) + 63) & ~63UL;
    if ((id & 3U) == 0U)
        heap_start = CORE0_RAM_BASE + CORE0_CONTROL_RESERVE;
    e->heap_ptr       = (u8 *)heap_start;
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
