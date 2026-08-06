/*
 * rp1_dma.h - RP1 internal Synopsys DesignWare AXI DMAC
 */

#pragma once
#include "types.h"

enum rp1_dma_error {
    RP1_DMA_ERR_NONE = 0,
    RP1_DMA_ERR_UNAVAILABLE,
    RP1_DMA_ERR_CLOCK,
    RP1_DMA_ERR_PROBE,
    RP1_DMA_ERR_RESET_TIMEOUT,
    RP1_DMA_ERR_QUARANTINED,
    RP1_DMA_ERR_BUSY,
    RP1_DMA_ERR_TRANSFER_TIMEOUT,
    RP1_DMA_ERR_TRANSFER_HW,
    RP1_DMA_ERR_MISMATCH,
};

struct rp1_dma_diag {
    u32 probed;
    u32 ready;
    u32 quarantined;
    u32 last_error;
    u32 selftest_runs;
    u32 selftest_failures;
    u32 dmac_id;
    u32 comp_version;
    u32 cfg;
    u32 chen;
    u32 int_status;
    u32 chan_status;
    u32 chan_int_status;
    u32 last_mismatch;
    u32 last_got;
    u32 last_expected;
} ALIGNED(64);

_Static_assert(sizeof(struct rp1_dma_diag) == 64U,
               "RP1 DMA diagnostics must occupy one cache line");

bool rp1_dma_probe(void);
bool rp1_dma_selftest(void);
void rp1_dma_diag_snapshot(struct rp1_dma_diag *out);
