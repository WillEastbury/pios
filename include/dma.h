/*
 * dma.h - BCM2712 DMA Engine driver
 *
 * Hardware scatter-gather DMA with 40-bit addressing.
 * Frees CPU from bulk memory copies (SD I/O, net packets).
 * No PCIe/RP1 needed — this is on the SoC directly.
 */

#pragma once
#include "types.h"

/* BCM2712 dma32 node from DT: dma@10000 reg = <0x10 0x00010000 0 0x600>.
 * This is not the legacy 0x107c007000 DMA block. */
#define DMA_BASE            0x1000010000UL
#define DMA_CHAN_STRIDE      0x100

/* Legacy-register BCM2712 channels 0-5, not the DMA40 block at +0x600. */
#define DMA_NUM_CHANNELS     6
#define DMA_MAX_CB_BYTES     65532U /* LITE channels: 16-bit length, whole words */

/* Control Block (CB) — 32 bytes, must be 32-byte aligned */
struct dma_cb {
    u32 ti;             /* Transfer Information */
    u32 src_addr;       /* Source address (low 32 bits) */
    u32 dst_addr;       /* Destination address (low 32 bits) */
    u32 xfer_len;       /* Transfer length in bytes */
    u32 stride;         /* BCM2712 source [39:32] | destination [39:32] << 8 */
    u32 next_cb;        /* Next CB address (0 = end of chain) */
    u32 _pad[2];        /* Pad to 32 bytes */
} ALIGNED(32);

struct dma_channel_diag {
    u32 cs;
    u32 cbaddr;
    u32 ti;
    u32 src;
    u32 dst;
    u32 len;
    u32 debug;
} PACKED;

struct dma_diag_snapshot {
    bool hw_memcpy_enabled;
    bool direct_mode;
    bool cbaddr_shifted;
    u32 selftest_runs;
    u32 selftest_failures;
    u32 last_error;
    u32 last_channel;
    u32 last_len;
    u32 last_mismatch_off;
    u32 last_got;
    u32 last_expected;
    u32 channel_mask;
    u32 hw_copies;
    u32 hw_zeroes;
    u32 last_cs;
    u32 last_debug;
    u32 last_cbaddr;
    struct dma_channel_diag channel[DMA_NUM_CHANNELS];
} PACKED;

/* Transfer Information (TI) register bits */
#define DMA_TI_INTEN        (1 << 0)     /* Interrupt enable */
#define DMA_TI_TDMODE       (1 << 1)     /* 2D mode */
#define DMA_TI_WAIT_RESP    (1 << 3)     /* Wait for write response */
#define DMA_TI_DEST_INC     (1 << 4)     /* Destination address increment */
#define DMA_TI_DEST_WIDTH   (1 << 5)     /* 128-bit destination writes */
#define DMA_TI_DEST_DREQ    (1 << 6)     /* Destination DREQ pacing */
#define DMA_TI_SRC_INC      (1 << 8)     /* Source address increment */
#define DMA_TI_SRC_WIDTH    (1 << 9)     /* 128-bit source reads */
#define DMA_TI_SRC_DREQ     (1 << 10)    /* Source DREQ pacing */
#define DMA_TI_BURST(x)     ((x) << 12)  /* Burst length */
#define DMA_TI_NO_WIDE      (1 << 26)    /* Disable wide bursts */

/* Channel CS (Control & Status) register bits */
#define DMA_CS_ACTIVE        (1 << 0)
#define DMA_CS_END           (1 << 1)
#define DMA_CS_INT           (1 << 2)
#define DMA_CS_WAITING_WRITES (1 << 6)
#define DMA_CS_ERROR         (1 << 8)
#define DMA_CS_RESET         (1 << 31)
#define DMA_CS_ABORT         (1 << 30)

/* Per-channel register offsets */
#define DMA_CH_CS            0x00
#define DMA_CH_CBADDR        0x04
#define DMA_CH_TI            0x08
#define DMA_CH_SRC           0x0C
#define DMA_CH_DST           0x10
#define DMA_CH_LEN           0x14
#define DMA_CH_STRIDE         0x18
#define DMA_CH_NEXTCB        0x1C
#define DMA_CH_DEBUG         0x20

/* BCM2712 dma32 brcm,dma-channel-mask = 0x35 => channels 0,2,4,5. */
#define DMA_CHAN_NET_TX      0
#define DMA_CHAN_NET_RX      0
#define DMA_CHAN_SD          2
#define DMA_CHAN_GPU         4
#define DMA_CHAN_MEMCPY      5
#define DMA_CHAN_SPARE       5
#define DMA_CHAN_MASK        ((1U << 0) | (1U << 2) | (1U << 4) | (1U << 5))

/* BCM2712 CB pointers are always PA >> 5; there is no direct/raw mode. */
bool dma_cb_encode(struct dma_cb *cb, u64 src, u64 dst, u32 len,
                   bool source_increment, u64 next);

/* Init the DMA engine */
void dma_init(void);
bool dma_selftest(void);
void dma_diag_snapshot(struct dma_diag_snapshot *out);

/* Core 0 owns hardware. Other cores use synchronous CPU copies/fills. */
/* Simple memcpy via bounded DMA quanta (blocks until complete). */
bool dma_memcpy(u32 channel, void *dst, const void *src, u32 len);

/* Async: start a DMA transfer (returns immediately) */
bool dma_start(u32 channel, struct dma_cb *cb);

/* Poll: is a DMA channel still active? */
bool dma_busy(u32 channel);

/* Wait for DMA completion on a channel */
void dma_wait(u32 channel);

/* Abort a running transfer */
void dma_abort(u32 channel);

/* Zero a memory region via DMA (faster than NEON for large regions) */
bool dma_zero(u32 channel, void *dst, u32 len);

/* Scatter-gather: chain multiple CBs and start */
bool dma_start_chain(u32 channel, struct dma_cb *first_cb);
