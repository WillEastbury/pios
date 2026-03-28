/*
 * dma.c - BCM2712 DMA Engine driver
 *
 * Scatter-gather DMA with 40-bit addressing.
 * 6 channels assigned: net TX/RX, SD, memcpy, GPU, spare.
 * All operations use control block chains in physical memory.
 */

#include "dma.h"
#include "mmio.h"
#include "simd.h"
#include "uart.h"

/* DMA channel register access */
static inline u64 dma_reg(u32 ch, u32 off) {
    return DMA_BASE + (u64)ch * DMA_CHAN_STRIDE + off;
}

/* DMA Enable register: one bit per channel */
#define DMA_ENABLE_REG  (DMA_BASE + 0xFF0)

/* Static control block pool — 16 CBs per channel, 32-byte aligned */
#define CBS_PER_CHAN    16
static struct dma_cb cb_pool[DMA_NUM_CHANNELS][CBS_PER_CHAN] ALIGNED(32);

/* A known zero word for DMA zero-fill source */
static const u32 zero_word ALIGNED(32) = 0;

void dma_init(void) {
    /* Enable all 6 channels */
    mmio_write(DMA_ENABLE_REG, (1 << DMA_NUM_CHANNELS) - 1);
    dsb();

    /* Reset each channel */
    for (u32 ch = 0; ch < DMA_NUM_CHANNELS; ch++) {
        mmio_write(dma_reg(ch, DMA_CH_CS), DMA_CS_RESET);
        delay_cycles(1000);
        /* Clear status bits */
        mmio_write(dma_reg(ch, DMA_CH_CS), DMA_CS_END | DMA_CS_INT | DMA_CS_ERROR);
    }

    /* Zero the CB pool */
    simd_zero(cb_pool, sizeof(cb_pool));
    dsb();

    uart_puts("[dma] 6 channels initialised\n");
}

bool dma_busy(u32 channel) {
    if (channel >= DMA_NUM_CHANNELS) return false;
    return (mmio_read(dma_reg(channel, DMA_CH_CS)) & DMA_CS_ACTIVE) != 0;
}

void dma_wait(u32 channel) {
    if (channel >= DMA_NUM_CHANNELS) return;
    while (mmio_read(dma_reg(channel, DMA_CH_CS)) & DMA_CS_ACTIVE)
        ;
    /* Clear end/int flags */
    mmio_write(dma_reg(channel, DMA_CH_CS), DMA_CS_END | DMA_CS_INT);
}

void dma_abort(u32 channel) {
    if (channel >= DMA_NUM_CHANNELS) return;
    mmio_write(dma_reg(channel, DMA_CH_CS), DMA_CS_ABORT);
    delay_cycles(1000);
    mmio_write(dma_reg(channel, DMA_CH_CS), DMA_CS_RESET);
    delay_cycles(1000);
    mmio_write(dma_reg(channel, DMA_CH_CS), DMA_CS_END | DMA_CS_INT | DMA_CS_ERROR);
}

bool dma_start(u32 channel, struct dma_cb *cb) {
    if (channel >= DMA_NUM_CHANNELS) return false;
    if (dma_busy(channel)) return false;

    dsb();  /* ensure CB is visible to DMA */

    /* Write CB address (physical) */
    mmio_write(dma_reg(channel, DMA_CH_CBADDR), (u32)(usize)cb);

    /* Activate */
    mmio_write(dma_reg(channel, DMA_CH_CS), DMA_CS_ACTIVE);

    return true;
}

bool dma_start_chain(u32 channel, struct dma_cb *first_cb) {
    return dma_start(channel, first_cb);
}

bool dma_memcpy(u32 channel, void *dst, const void *src, u32 len) {
    if (channel >= DMA_NUM_CHANNELS || len == 0) return false;
    if (dma_busy(channel)) return false;

    struct dma_cb *cb = &cb_pool[channel][0];

    cb->ti       = DMA_TI_SRC_INC | DMA_TI_DEST_INC |
                   DMA_TI_SRC_WIDTH | DMA_TI_DEST_WIDTH |
                   DMA_TI_BURST(4) | DMA_TI_WAIT_RESP;
    cb->src_addr = (u32)(usize)src;
    cb->dst_addr = (u32)(usize)dst;
    cb->xfer_len = len;
    cb->stride   = 0;
    cb->next_cb  = 0;  /* single transfer */

    dsb();

    if (!dma_start(channel, cb))
        return false;

    dma_wait(channel);

    /* Check for errors */
    u32 cs = mmio_read(dma_reg(channel, DMA_CH_CS));
    if (cs & DMA_CS_ERROR) {
        mmio_write(dma_reg(channel, DMA_CH_CS), DMA_CS_ERROR);
        return false;
    }

    return true;
}

bool dma_zero(u32 channel, void *dst, u32 len) {
    if (channel >= DMA_NUM_CHANNELS || len == 0) return false;
    if (dma_busy(channel)) return false;

    struct dma_cb *cb = &cb_pool[channel][0];

    /* Source does NOT increment (reads 0 repeatedly from zero_word).
     * Destination increments normally. */
    cb->ti       = DMA_TI_DEST_INC | DMA_TI_DEST_WIDTH |
                   DMA_TI_BURST(4) | DMA_TI_WAIT_RESP;
    cb->src_addr = (u32)(usize)&zero_word;
    cb->dst_addr = (u32)(usize)dst;
    cb->xfer_len = len;
    cb->stride   = 0;
    cb->next_cb  = 0;

    dsb();

    if (!dma_start(channel, cb))
        return false;

    dma_wait(channel);

    u32 cs = mmio_read(dma_reg(channel, DMA_CH_CS));
    return !(cs & DMA_CS_ERROR);
}

/* ---- Scatter-gather helpers ---- */

/* Build a chain of CBs for a multi-block memcpy.
 * Splits a large transfer into CBS_PER_CHAN chunks.
 * Returns pointer to first CB, or NULL if too many chunks. */
struct dma_cb *dma_build_sg_memcpy(u32 channel, void *dst, const void *src,
                                    u32 len, u32 chunk_size) {
    if (channel >= DMA_NUM_CHANNELS || len == 0) return NULL;

    u32 num_chunks = (len + chunk_size - 1) / chunk_size;
    if (num_chunks > CBS_PER_CHAN) return NULL;

    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;
    u32 remaining = len;

    for (u32 i = 0; i < num_chunks; i++) {
        struct dma_cb *cb = &cb_pool[channel][i];
        u32 this_len = (remaining > chunk_size) ? chunk_size : remaining;

        cb->ti       = DMA_TI_SRC_INC | DMA_TI_DEST_INC |
                       DMA_TI_SRC_WIDTH | DMA_TI_DEST_WIDTH |
                       DMA_TI_BURST(4) | DMA_TI_WAIT_RESP;
        cb->src_addr = (u32)(usize)s;
        cb->dst_addr = (u32)(usize)d;
        cb->xfer_len = this_len;
        cb->stride   = 0;

        if (i < num_chunks - 1)
            cb->next_cb = (u32)(usize)&cb_pool[channel][i + 1];
        else
            cb->next_cb = 0;

        d += this_len;
        s += this_len;
        remaining -= this_len;
    }

    dsb();
    return &cb_pool[channel][0];
}
