/*
 * dwc2_contract.h - offline DWC2 DMA and transfer-ownership contract.
 *
 * This deliberately contains no MMIO, IRQ unmasking, platform setup, or
 * controller command logic. A future backend consumes only these facts and
 * generation-checked transfer handles.
 */
#pragma once
#include "types.h"

#define DWC2_BCM2837_BASE             0x3F980000ULL
#define DWC2_ARMCTRL_BANK1_IRQ_BIT    9U
#define DWC2_GPU_IRQ                  41U
#define DWC2_DMA_BUS_ALIAS            0xC0000000ULL
#define DWC2_DMA_PHYS_LIMIT           0x3F000000ULL
#define DWC2_DMA_ALIGNMENT            4U
#define DWC2_TRANSFER_CAPACITY        8U

_Static_assert(DWC2_GPU_IRQ == 32U + DWC2_ARMCTRL_BANK1_IRQ_BIT,
               "DWC2 GPU IRQ must match ARMCTRL bank 1 bit");
_Static_assert((DWC2_DMA_ALIGNMENT & (DWC2_DMA_ALIGNMENT - 1U)) == 0U,
               "DWC2 DMA alignment must be a power of two");

enum dwc2_transfer_state {
    DWC2_TRANSFER_FREE = 0,
    DWC2_TRANSFER_PREPARED,
    DWC2_TRANSFER_HARDWARE,
    DWC2_TRANSFER_COMPLETED,
    DWC2_TRANSFER_FAILED,
    /* A generation that reaches UINT64_MAX is permanently unavailable. */
    DWC2_TRANSFER_RETIRED,
};

/*
 * A handle is the only transfer identity that crosses ownership domains.
 * It is never a pointer; both its slot and generation are validated.
 */
struct dwc2_transfer_handle {
    u64 generation;
    u32 slot;
    u32 _reserved;
};

/*
 * Each mutable record owns a full cache line and has a 64-byte stride. The
 * numeric physical address is retained solely as DMA authority; no raw
 * buffer pointer is published to another ownership domain.
 */
struct dwc2_transfer_record {
    u64 generation;
    u64 cpu_phys;
    u64 dma_bus;
    u32 capacity;
    u32 requested;
    u32 actual;
    u32 state;
    u8 _pad[24U];
} ALIGNED(64);

_Static_assert(sizeof(struct dwc2_transfer_record) == 64U,
               "DWC2 transfer records must own one cache line");

struct dwc2_transfer_pool {
    struct dwc2_transfer_record records[DWC2_TRANSFER_CAPACITY];
} ALIGNED(64);

_Static_assert((sizeof(struct dwc2_transfer_pool) % 64U) == 0U,
               "DWC2 transfer pool records must retain cache-line stride");

/* Initialization is one-shot over fresh all-zero storage. */
bool dwc2_transfer_pool_init(struct dwc2_transfer_pool *pool);

/*
 * Validate a CPU-physical DMA span and return its BCM2837 bus address.
 * The physical range is [1, DWC2_DMA_PHYS_LIMIT), and address, length, and
 * capacity must all meet the four-byte DMA alignment contract.
 */
bool dwc2_dma_bus_addr(u64 cpu_phys, u32 length, u32 capacity,
                       u64 *bus_addr_out);

bool dwc2_transfer_prepare(struct dwc2_transfer_pool *pool, u64 cpu_phys,
                           u32 length, u32 capacity,
                           struct dwc2_transfer_handle *handle_out);
bool dwc2_transfer_submit(struct dwc2_transfer_pool *pool,
                          const struct dwc2_transfer_handle *handle);
bool dwc2_transfer_complete(struct dwc2_transfer_pool *pool,
                            const struct dwc2_transfer_handle *handle,
                            u32 actual);
bool dwc2_transfer_fail(struct dwc2_transfer_pool *pool,
                        const struct dwc2_transfer_handle *handle);
bool dwc2_transfer_release(struct dwc2_transfer_pool *pool,
                           const struct dwc2_transfer_handle *handle);

bool dwc2_transfer_get_state(const struct dwc2_transfer_pool *pool,
                             const struct dwc2_transfer_handle *handle,
                             enum dwc2_transfer_state *state_out);
bool dwc2_transfer_get_dma_bus(const struct dwc2_transfer_pool *pool,
                               const struct dwc2_transfer_handle *handle,
                               u64 *bus_addr_out);
