/*
 * dwc2_dma_arena.h - BCM2837-only DWC2 Normal-NC DMA ownership contract.
 *
 * This is physical reservation metadata only. It contains no DWC2 MMIO,
 * controller initialization, DMA execution, CPU pointer, or cache-maintenance
 * operation. A BCM bus address is numeric DMA authority, never a CPU mapping.
 */
#pragma once
#include "types.h"
#include "platform.h"
#include "dwc2_contract.h"

#define DWC2_DMA_ARENA_SLOT_COUNT       8U
#define DWC2_DMA_ARENA_SLOT_SIZE        0x00040000U
#define DWC2_DMA_ARENA_OWNER_CORE       0U
#define DWC2_DMA_ARENA_TOKEN_MAGIC      0xD2A1ULL
#define DWC2_DMA_ARENA_TOKEN_SHIFT      48U
#define DWC2_DMA_ARENA_TOKEN_SLOT_MASK  (DWC2_DMA_ARENA_SLOT_COUNT - 1U)
#define DWC2_DMA_ARENA_TOKEN_MASK       \
    ((DWC2_DMA_ARENA_TOKEN_MAGIC << DWC2_DMA_ARENA_TOKEN_SHIFT) | \
     DWC2_DMA_ARENA_TOKEN_SLOT_MASK)

_Static_assert((DWC2_DMA_ARENA_SLOT_COUNT &
                (DWC2_DMA_ARENA_SLOT_COUNT - 1U)) == 0U,
               "DWC2 DMA arena token slots must be a power of two");
_Static_assert(DWC2_DMA_ARENA_SLOT_COUNT * DWC2_DMA_ARENA_SLOT_SIZE ==
               0x00200000U,
               "DWC2 DMA slots must consume the exact reserved arena");

enum dwc2_dma_direction {
    DWC2_DMA_TO_DEVICE = 0U,
    DWC2_DMA_FROM_DEVICE = 1U,
};

enum dwc2_dma_cache_policy {
    DWC2_DMA_CACHE_NORMAL_NC = 1U,
};

enum dwc2_dma_barrier_scope {
    DWC2_DMA_BARRIER_SYSTEM = 1U,
};

enum dwc2_dma_arena_state {
    DWC2_DMA_ARENA_FREE = 0U,
    DWC2_DMA_ARENA_CPU_OWNED,
    DWC2_DMA_ARENA_DEVICE_OWNED,
    DWC2_DMA_ARENA_CPU_COMPLETE,
    DWC2_DMA_ARENA_FAILED,
    DWC2_DMA_ARENA_RELEASED,
};

/* Full generation and controller identity are deliberately outside the token.
 * The opaque token contains only a magic marker plus the slot selector. */
struct dwc2_dma_arena_handle {
    u64 token;
    u64 generation;
    u32 controller_id;
    u32 _reserved;
};

/* Owner/control metadata has exclusive cache-line ownership. */
struct dwc2_dma_arena_owner {
    u32 controller_id;
    u32 owner_core;
    u32 initialized;
    u32 _reserved;
    u8 _pad[48U];
} ALIGNED(64);

struct dwc2_dma_arena_slot_control {
    u64 generation;
    u32 state;
    u32 actual;
    u32 exhausted;
    u8 _pad[44U];
} ALIGNED(64);

/*
 * This is an immutable numeric span copied at acquire time. No raw CPU
 * pointer crosses the ownership boundary or is retained by this contract.
 */
struct dwc2_dma_arena_span {
    u64 cpu_phys;
    u64 bcm_bus_addr;
    u64 generation;
    u32 used;
    u32 requested;
    u32 capacity;
    u32 slot;
    u32 direction;
    u32 cache_policy;
    u32 _reserved;
    u8 _pad[12U];
} ALIGNED(64);

struct dwc2_dma_arena {
    struct dwc2_dma_arena_owner owner;
    struct dwc2_dma_arena_slot_control controls[DWC2_DMA_ARENA_SLOT_COUNT];
    struct dwc2_dma_arena_span spans[DWC2_DMA_ARENA_SLOT_COUNT];
} ALIGNED(64);

_Static_assert(sizeof(struct dwc2_dma_arena_owner) == 64U,
               "DWC2 DMA owner must own exactly one cache line");
_Static_assert(sizeof(struct dwc2_dma_arena_slot_control) == 64U,
               "DWC2 DMA slot control must own exactly one cache line");
_Static_assert(sizeof(struct dwc2_dma_arena_span) == 64U,
               "DWC2 DMA numeric spans must retain a fixed stride");
_Static_assert(sizeof(struct dwc2_dma_arena_handle) == 24U,
               "DWC2 DMA handles must carry full identity");
_Static_assert(__builtin_offsetof(struct dwc2_dma_arena, controls) == 64U,
               "DWC2 DMA controls must not share the owner cache line");
_Static_assert(__builtin_offsetof(struct dwc2_dma_arena, spans) ==
               64U + DWC2_DMA_ARENA_SLOT_COUNT * 64U,
               "DWC2 DMA spans must be separate from mutable controls");
_Static_assert(sizeof(struct dwc2_dma_arena) == 64U +
               2U * DWC2_DMA_ARENA_SLOT_COUNT * 64U,
               "DWC2 DMA arena layout must have fixed cache-line strides");

#if PIOS_PLATFORM == PIOS_PLATFORM_PI3 || \
    PIOS_PLATFORM == PIOS_PLATFORM_PIZERO2W
#define DWC2_DMA_ARENA_END (PIOS_DWC2_DMA_BASE + PIOS_DWC2_DMA_SIZE)
#define DWC2_DMA_ARENA_STAGING_BASE 0x08000000UL
#define DWC2_DMA_ARENA_DISJOINT(base, size) \
    (DWC2_DMA_ARENA_END <= (base) || (base) + (size) <= PIOS_DWC2_DMA_BASE)

_Static_assert(PIOS_DWC2_DMA_BASE ==
               PIOS_SHARED_ASSET_BASE + PIOS_SHARED_ASSET_SIZE,
               "DWC2 DMA arena must begin after the shared asset window");
_Static_assert(DWC2_DMA_ARENA_END <= DWC2_DMA_ARENA_STAGING_BASE,
               "DWC2 DMA arena must remain below stage0 staging");
_Static_assert(DWC2_DMA_ARENA_END <= DWC2_DMA_PHYS_LIMIT,
               "DWC2 DMA arena must remain below BCM2837 DMA limit");
_Static_assert(DWC2_DMA_ARENA_DISJOINT(PIOS_CORE0_RAM_BASE, PIOS_CORE_PRIV_SIZE) &&
               DWC2_DMA_ARENA_DISJOINT(PIOS_CORE1_RAM_BASE, PIOS_CORE_PRIV_SIZE) &&
               DWC2_DMA_ARENA_DISJOINT(PIOS_CORE2_RAM_BASE, PIOS_CORE_PRIV_SIZE) &&
               DWC2_DMA_ARENA_DISJOINT(PIOS_CORE3_RAM_BASE, PIOS_CORE_PRIV_SIZE) &&
               DWC2_DMA_ARENA_DISJOINT(PIOS_SHARED_FIFO_BASE, PIOS_SHARED_FIFO_SIZE) &&
               DWC2_DMA_ARENA_DISJOINT(PIOS_DMA_NET_BASE, PIOS_DMA_NET_SIZE) &&
               DWC2_DMA_ARENA_DISJOINT(PIOS_DMA_DISK_BASE, PIOS_DMA_DISK_SIZE) &&
               DWC2_DMA_ARENA_DISJOINT(PIOS_IPC_SHM_BASE, PIOS_IPC_SHM_SIZE) &&
               DWC2_DMA_ARENA_DISJOINT(PIOS_DMA_PCIE1_BASE, PIOS_DMA_PCIE1_SIZE) &&
               DWC2_DMA_ARENA_DISJOINT(PIOS_FB_BACK_BASE, PIOS_FB_BACK_SIZE) &&
               DWC2_DMA_ARENA_DISJOINT(PIOS_SHARED_ASSET_BASE, PIOS_SHARED_ASSET_SIZE) &&
               DWC2_DMA_ARENA_DISJOINT(PIOS_PROC_ARENA_BASE, PIOS_PROC_ARENA_SIZE),
               "DWC2 DMA arena must not overlap another platform region");
#endif

/*
 * One-shot init only accepts freshly zeroed storage on Pi3/Zero2W and core 0.
 * `controller_id` is an externally assigned, nonzero, unique controller
 * identity; concurrent contracts must not reuse it.
 */
bool dwc2_dma_arena_init(struct dwc2_dma_arena *arena, u32 controller_id,
                         u32 caller_core);

bool dwc2_dma_arena_acquire(struct dwc2_dma_arena *arena, u32 controller_id,
                            u32 caller_core, u32 requested,
                            enum dwc2_dma_direction direction,
                            struct dwc2_dma_arena_handle *handle_out);
bool dwc2_dma_arena_publish_to_device(
    struct dwc2_dma_arena *arena, u32 caller_core,
    const struct dwc2_dma_arena_handle *handle);
bool dwc2_dma_arena_complete_from_device(
    struct dwc2_dma_arena *arena, u32 caller_core,
    const struct dwc2_dma_arena_handle *handle, u32 actual, bool success);
bool dwc2_dma_arena_cancel(struct dwc2_dma_arena *arena, u32 caller_core,
                           const struct dwc2_dma_arena_handle *handle);
bool dwc2_dma_arena_release(struct dwc2_dma_arena *arena, u32 caller_core,
                            const struct dwc2_dma_arena_handle *handle);

bool dwc2_dma_arena_state_get(const struct dwc2_dma_arena *arena,
                              u32 caller_core,
                              const struct dwc2_dma_arena_handle *handle,
                              enum dwc2_dma_arena_state *state_out);
bool dwc2_dma_arena_span_get(const struct dwc2_dma_arena *arena,
                             u32 caller_core,
                             const struct dwc2_dma_arena_handle *handle,
                             struct dwc2_dma_arena_span *span_out);
bool dwc2_dma_arena_completion_get(
    const struct dwc2_dma_arena *arena, u32 caller_core,
    const struct dwc2_dma_arena_handle *handle, u32 *actual_out,
    bool *success_out);
bool dwc2_dma_arena_cache_contract_get(
    const struct dwc2_dma_arena *arena, u32 caller_core,
    const struct dwc2_dma_arena_handle *handle,
    enum dwc2_dma_cache_policy *policy_out, bool *publish_barrier_only_out,
    bool *consume_barrier_only_out,
    enum dwc2_dma_barrier_scope *barrier_scope_out);
