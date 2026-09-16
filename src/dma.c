/*
 * dma.c - BCM2712 DMA Engine driver
 *
 * Scatter-gather DMA with 40-bit addressing.
 * 6 channels assigned: net TX/RX, SD, memcpy, GPU, spare.
 * All operations use control block chains in physical memory.
 */

#include "dma.h"
#include "mmio.h"
#include "mmu.h"
#include "simd.h"
#include "uart.h"
#include "fb.h"
#include "timer.h"
#include "watchdog.h"
#include "net.h"

#define DMA_WAIT_MS 5U
#define DMA_WAIT_SPINS 100000U

/* Below this transfer size, DMA setup + cache maintenance + completion-wait
 * costs more than a NEON copy, so dma_memcpy()/dma_zero() shortcut straight to
 * simd_*(). Conservative initial value; tuned empirically by the IPC
 * DMA-crossover benchmark (the cross-over point where the engine wins). */
#define DMA_MIN_EFFECTIVE_BYTES 4096U

/* DMA channel register access */
static inline u64 dma_reg(u32 ch, u32 off) {
    return DMA_BASE + (u64)ch * DMA_CHAN_STRIDE + off;
}

static inline u32 dma_cb_addr(const struct dma_cb *cb)
{
    return (u32)((u64)(usize)cb >> 5);
}

/* Static control block pool — 16 CBs per channel, 32-byte aligned */
#define CBS_PER_CHAN    16
static struct dma_cb cb_pool[DMA_NUM_CHANNELS][CBS_PER_CHAN] ALIGNED(64);

static const u32 zero_word ALIGNED(32) = 0;
static u8 dma_test_src[16384] ALIGNED(64);
static u8 dma_test_dst[16384] ALIGNED(64);
static bool dma_hw_memcpy_enabled;
static u32 dma_selftest_runs;
static u32 dma_selftest_failures;
static u32 dma_last_error;
static u32 dma_last_channel;
static u32 dma_last_len;
static u32 dma_last_mismatch_off;
static u32 dma_last_got;
static u32 dma_last_expected;
static u32 dma_hw_copies;
static u32 dma_hw_zeroes;
static u32 dma_last_cs;
static u32 dma_last_debug;
static u32 dma_last_cbaddr;

#define DMA_ERR_NONE        0U
#define DMA_ERR_START       1U
#define DMA_ERR_TIMEOUT     2U
#define DMA_ERR_HW_ERROR    3U
#define DMA_ERR_MISMATCH    4U
#define DMA_ERR_ADDR_RANGE  5U

static bool dma_memcpy_hw(u32 channel, void *dst, const void *src, u32 len);
static bool dma_zero_hw(u32 channel, void *dst, u32 len);
static bool dma_selftest_transfer(void);

static bool dma_channel_allowed(u32 ch)
{
    return ch == 0 || ch == 2 || ch == 4 || ch == 5;
}

void dma_init(void) {
#if !PIOS_HAS_DMA
    dma_hw_memcpy_enabled = false;
    dma_last_error = DMA_ERR_NONE;
    return;
#else
    dma_hw_memcpy_enabled = false;
    dma_last_error = DMA_ERR_NONE;
    /* Reset usable dma32 channels. DT mask 0x35 exposes channels 0,2,4,5. */
    for (u32 ch = 0; ch < DMA_NUM_CHANNELS; ch++) {
        if (!dma_channel_allowed(ch))
            continue;
        dma_abort(ch);
    }

    /* Zero the CB pool */
    simd_zero(cb_pool, sizeof(cb_pool));
    dsb();

    uart_puts("[dma] 6 channels initialised\n");
#endif
}

static void dma_dump_channel(u32 ch, const char *tag)
{
    uart_puts("[dma] ");
    uart_puts(tag);
    uart_puts(" ch=");
    uart_hex(ch);
    uart_puts(" CS=");
    uart_hex(mmio_read(dma_reg(ch, DMA_CH_CS)));
    uart_puts(" CB=");
    uart_hex(mmio_read(dma_reg(ch, DMA_CH_CBADDR)));
    uart_puts(" TI=");
    uart_hex(mmio_read(dma_reg(ch, DMA_CH_TI)));
    uart_puts(" SRC=");
    uart_hex(mmio_read(dma_reg(ch, DMA_CH_SRC)));
    uart_puts(" DST=");
    uart_hex(mmio_read(dma_reg(ch, DMA_CH_DST)));
    uart_puts(" LEN=");
    uart_hex(mmio_read(dma_reg(ch, DMA_CH_LEN)));
    uart_puts(" DBG=");
    uart_hex(mmio_read(dma_reg(ch, DMA_CH_DEBUG)));
    uart_puts("\n");
}

void dma_diag_snapshot(struct dma_diag_snapshot *out)
{
    if (!out)
        return;
#if !PIOS_HAS_DMA
    simd_zero(out, sizeof(*out));
    return;
#else
    out->hw_memcpy_enabled = dma_hw_memcpy_enabled;
    out->direct_mode = false;
    out->cbaddr_shifted = true;
    out->selftest_runs = dma_selftest_runs;
    out->selftest_failures = dma_selftest_failures;
    out->last_error = dma_last_error;
    out->last_channel = dma_last_channel;
    out->last_len = dma_last_len;
    out->last_mismatch_off = dma_last_mismatch_off;
    out->last_got = dma_last_got;
    out->last_expected = dma_last_expected;
    out->channel_mask = DMA_CHAN_MASK;
    out->hw_copies = dma_hw_copies;
    out->hw_zeroes = dma_hw_zeroes;
    out->last_cs = dma_last_cs;
    out->last_debug = dma_last_debug;
    out->last_cbaddr = dma_last_cbaddr;
    for (u32 ch = 0; ch < DMA_NUM_CHANNELS; ch++) {
        out->channel[ch].cs = dma_channel_allowed(ch) ? mmio_read(dma_reg(ch, DMA_CH_CS)) : 0;
        out->channel[ch].cbaddr = dma_channel_allowed(ch) ? mmio_read(dma_reg(ch, DMA_CH_CBADDR)) : 0;
        out->channel[ch].ti = dma_channel_allowed(ch) ? mmio_read(dma_reg(ch, DMA_CH_TI)) : 0;
        out->channel[ch].src = dma_channel_allowed(ch) ? mmio_read(dma_reg(ch, DMA_CH_SRC)) : 0;
        out->channel[ch].dst = dma_channel_allowed(ch) ? mmio_read(dma_reg(ch, DMA_CH_DST)) : 0;
        out->channel[ch].len = dma_channel_allowed(ch) ? mmio_read(dma_reg(ch, DMA_CH_LEN)) : 0;
        out->channel[ch].debug = dma_channel_allowed(ch) ? mmio_read(dma_reg(ch, DMA_CH_DEBUG)) : 0;
    }
#endif
}

bool dma_selftest(void)
{
#if !PIOS_HAS_DMA
    return false;
#else
    if (core_id() != 0U) {
        uart_puts("[dma] selftest requires core 0\n");
        return false;
    }
    dma_selftest_runs++;
    dma_hw_memcpy_enabled = false;
    dma_last_error = DMA_ERR_NONE;
    if (dma_selftest_transfer()) {
        dma_hw_memcpy_enabled = true;
        uart_puts("[dma] selftest ok cbaddr=shifted\n");
        return true;
    }
    dma_selftest_failures++;
    dma_hw_memcpy_enabled = false;
    uart_puts("[dma] selftest failed\n");
    return false;
#endif
}

static bool dma_selftest_transfer(void)
{
    for (u32 i = 0; i < sizeof(dma_test_src); i++) {
        dma_test_src[i] = (u8)(0xA5U ^ (i * 37U) ^ (i >> 2));
        dma_test_dst[i] = 0;
    }
    dcache_clean_range((u64)(usize)dma_test_src, sizeof(dma_test_src));
    dcache_clean_invalidate_range((u64)(usize)dma_test_dst, sizeof(dma_test_dst));

    uart_puts("[dma] selftest src=");
    uart_hex((u32)(usize)dma_test_src);
    uart_puts(" dst=");
    uart_hex((u32)(usize)dma_test_dst);
    uart_puts(" cb=");
    uart_hex((u32)(usize)&cb_pool[DMA_CHAN_MEMCPY][0]);
    uart_puts(" mode=shifted\n");

    dma_abort(DMA_CHAN_MEMCPY);
    if (!dma_memcpy_hw(DMA_CHAN_MEMCPY, dma_test_dst, dma_test_src, sizeof(dma_test_src))) {
        uart_puts("[dma] selftest memcpy returned false\n");
        dma_dump_channel(DMA_CHAN_MEMCPY, "selftest-fail");
        return false;
    }

    dcache_invalidate_range((u64)(usize)dma_test_dst, sizeof(dma_test_dst));
    for (u32 i = 0; i < sizeof(dma_test_src); i++) {
        if (dma_test_dst[i] != dma_test_src[i]) {
            uart_puts("[dma] selftest mismatch off=");
            uart_hex(i);
            uart_puts(" got=");
            uart_hex(dma_test_dst[i]);
            uart_puts(" exp=");
            uart_hex(dma_test_src[i]);
            uart_puts("\n");
            dma_last_error = DMA_ERR_MISMATCH;
            dma_last_mismatch_off = i;
            dma_last_got = dma_test_dst[i];
            dma_last_expected = dma_test_src[i];
            dma_dump_channel(DMA_CHAN_MEMCPY, "selftest-mismatch");
            return false;
        }
    }

    if (!dma_zero_hw(DMA_CHAN_MEMCPY, dma_test_dst, sizeof(dma_test_dst)))
        return false;
    for (u32 i = 0; i < sizeof(dma_test_dst); i++) {
        if (dma_test_dst[i] != 0U) {
            dma_last_error = DMA_ERR_MISMATCH;
            dma_last_mismatch_off = i;
            dma_last_got = dma_test_dst[i];
            dma_last_expected = 0U;
            return false;
        }
    }
    return true;
}

bool dma_busy(u32 channel) {
    if (!PIOS_HAS_DMA || core_id() != 0U) return false;
    if (channel >= DMA_NUM_CHANNELS) return false;
    if (!dma_channel_allowed(channel)) return false;
    return mmio_read(dma_reg(channel, DMA_CH_CBADDR)) != 0U ||
           (mmio_read(dma_reg(channel, DMA_CH_CS)) &
            (DMA_CS_ACTIVE | DMA_CS_WAITING_WRITES)) != 0U;
}

static bool dma_wait_complete(u32 channel)
{
    u64 start = timer_monotonic_ms();
    for (u32 spin = 0; spin < DMA_WAIT_SPINS; spin++) {
        u32 cs = mmio_read(dma_reg(channel, DMA_CH_CS));
        if (cs & DMA_CS_ERROR) {
            dma_last_error = DMA_ERR_HW_ERROR;
            break;
        }
        if (!(cs & (DMA_CS_ACTIVE | DMA_CS_WAITING_WRITES)) && (cs & DMA_CS_END) &&
            mmio_read(dma_reg(channel, DMA_CH_CBADDR)) == 0U) {
            mmio_write(dma_reg(channel, DMA_CH_CS), DMA_CS_END | DMA_CS_INT);
            return true;
        }
        if (timer_monotonic_ms() - start >= DMA_WAIT_MS) {
            dma_last_error = DMA_ERR_TIMEOUT;
            break;
        }
    }
    if (dma_last_error == DMA_ERR_NONE)
        dma_last_error = DMA_ERR_TIMEOUT;
    dma_last_cs = mmio_read(dma_reg(channel, DMA_CH_CS));
    dma_last_debug = mmio_read(dma_reg(channel, DMA_CH_DEBUG));
    dma_last_cbaddr = mmio_read(dma_reg(channel, DMA_CH_CBADDR));
    dma_dump_channel(channel, "transfer-fail");
    dma_abort(channel);
    return false;
}

void dma_wait(u32 channel) {
    if (!PIOS_HAS_DMA || core_id() != 0U) return;
    if (channel >= DMA_NUM_CHANNELS || !dma_channel_allowed(channel)) return;
    (void)dma_wait_complete(channel);
}

void dma_abort(u32 channel) {
    if (!PIOS_HAS_DMA || core_id() != 0U) return;
    if (channel >= DMA_NUM_CHANNELS) return;
    if (!dma_channel_allowed(channel)) return;
    mmio_write(dma_reg(channel, DMA_CH_NEXTCB), 0U);
    u32 cs = mmio_read(dma_reg(channel, DMA_CH_CS));
    if (mmio_read(dma_reg(channel, DMA_CH_CBADDR)) != 0U || (cs & DMA_CS_ACTIVE)) {
        mmio_write(dma_reg(channel, DMA_CH_CS), cs | DMA_CS_ABORT | DMA_CS_ACTIVE);
        for (u32 spin = 0; spin < 100U; spin++) {
            if (!(mmio_read(dma_reg(channel, DMA_CH_CS)) & DMA_CS_ABORT))
                break;
        }
        mmio_write(dma_reg(channel, DMA_CH_CS), 0U);
    }
    mmio_write(dma_reg(channel, DMA_CH_CS), DMA_CS_RESET);
    dsb();
    for (u32 spin = 0; spin < 1000U; spin++) {
        if (!(mmio_read(dma_reg(channel, DMA_CH_CS)) &
              (DMA_CS_ACTIVE | DMA_CS_WAITING_WRITES)) &&
            mmio_read(dma_reg(channel, DMA_CH_CBADDR)) == 0U)
            return;
    }
    dma_dump_channel(channel, "reset-fail");
    watchdog_reboot_now(0x444D4101U);
}

bool dma_start(u32 channel, struct dma_cb *cb) {
    if (!PIOS_HAS_DMA || core_id() != 0U) return false;
    if (channel >= DMA_NUM_CHANNELS) return false;
    if (!dma_channel_allowed(channel)) return false;
    if (dma_busy(channel)) return false;
    if (!cb || ((u64)(usize)cb & 31U) != 0U ||
        (u64)(usize)cb > 0x40000000ULL - sizeof(*cb) ||
        cb->xfer_len == 0U || cb->xfer_len > DMA_MAX_CB_BYTES)
        return false;
    dma_abort(channel);

    /* Control blocks live in normal cacheable RAM. The DMA engine fetches
     * them directly, so clean the CB before handing its address to hardware. */
    dcache_clean_range((u64)(usize)cb, sizeof(*cb));
    dsb();

    mmio_write(dma_reg(channel, DMA_CH_CBADDR), dma_cb_addr(cb));

    /* Activate */
    mmio_write(dma_reg(channel, DMA_CH_CS), DMA_CS_ACTIVE);

    return true;
}

bool dma_start_chain(u32 channel, struct dma_cb *first_cb) {
    return dma_start(channel, first_cb);
}

static bool dma_memcpy_hw(u32 channel, void *dst, const void *src, u32 len) {
    if (channel >= DMA_NUM_CHANNELS || len == 0 || len > DMA_MAX_CB_BYTES) return false;
    if (!dma_channel_allowed(channel)) return false;
    if (dma_busy(channel)) return false;

    /* BCM2712 dma32 uses the SoC DMA address; for low RAM this is PA. */
    if ((u64)(usize)src >= 0x40000000ULL || (u64)(usize)dst >= 0x40000000ULL ||
        len > 0x40000000ULL - (u64)(usize)src ||
        len > 0x40000000ULL - (u64)(usize)dst) {
        uart_puts("[dma] addr outside dma-ranges\n");
        dma_last_error = DMA_ERR_ADDR_RANGE;
        dma_last_channel = channel;
        dma_last_len = len;
        return false;
    }

    /* Flush source to RAM so DMA engine sees current data, and evict the
     * destination first so dirty cached lines cannot later overwrite DMA. */
    dcache_clean_range((u64)(usize)src, len);
    dcache_clean_invalidate_range((u64)(usize)dst, len);

    struct dma_cb *cb = &cb_pool[channel][0];

    if (!dma_cb_encode(cb, (u64)(usize)src, (u64)(usize)dst, len, true, 0))
        return false;

    dma_last_channel = channel;
    dma_last_len = len;
    dma_last_error = DMA_ERR_NONE;
    bool started = dma_start(channel, cb);
    if (!started) {
        dma_last_error = DMA_ERR_START;
        return false;
    }

    if (!dma_wait_complete(channel))
        return false;

    /* Invalidate destination so CPU sees DMA-written data */
    dcache_invalidate_range((u64)(usize)dst, len);

    dma_last_error = DMA_ERR_NONE;
    dma_hw_copies++;
    return true;
}

bool dma_memcpy(u32 channel, void *dst, const void *src, u32 len) {
    if (!dst || !src || !len ||
        (u64)(usize)dst > ~0ULL - len || (u64)(usize)src > ~0ULL - len) {
        uart_puts("[dma] invalid copy span\n");
        return false;
    }
    /* Size fast-path: below the effective threshold, DMA setup + cache
     * maintenance + completion-wait costs more than a NEON copy, so go
     * straight to simd_memcpy and never touch the engine. Tuned empirically
     * by the IPC DMA-crossover benchmark. */
    if (core_id() != 0U || len < DMA_MIN_EFFECTIVE_BYTES) {
        simd_memcpy(dst, src, len);
        return len != 0;
    }
    bool low_ram = (u64)(usize)src < 0x40000000ULL &&
                   (u64)(usize)dst < 0x40000000ULL &&
                   len <= 0x40000000ULL - (u64)(usize)src &&
                   len <= 0x40000000ULL - (u64)(usize)dst;
    if (dma_hw_memcpy_enabled && low_ram) {
        for (u32 off = 0U; off < len;) {
            u32 part = len - off;
            if (part > 32768U) part = 32768U;
            if (!dma_memcpy_hw(channel, (u8 *)dst + off, (const u8 *)src + off, part))
                return false;
            off += part;
            if (off < len)
                net_dispatch_yield();
        }
        return true;
    }
    simd_memcpy(dst, src, len);
    return len != 0;
}

bool dma_zero(u32 channel, void *dst, u32 len) {
    if (!dst || !len || (u64)(usize)dst > ~0ULL - len) {
        uart_puts("[dma] invalid zero span\n");
        return false;
    }
    /* Size fast-path: small fills are cheaper with NEON than DMA setup. */
    if (core_id() != 0U || len < DMA_MIN_EFFECTIVE_BYTES) {
        (void)channel;
        simd_zero(dst, len);
        return len != 0;
    }
    bool low_ram = (u64)(usize)dst < 0x40000000ULL &&
                   len <= 0x40000000ULL - (u64)(usize)dst;
    if (!dma_hw_memcpy_enabled || !low_ram) {
        (void)channel;
        simd_zero(dst, len);
        return len != 0;
    }
    for (u32 off = 0U; off < len;) {
        u32 part = len - off;
        if (part > 32768U) part = 32768U;
        if (!dma_zero_hw(channel, (u8 *)dst + off, part))
            return false;
        off += part;
        if (off < len)
            net_dispatch_yield();
    }
    return true;
}

static bool dma_zero_hw(u32 channel, void *dst, u32 len) {
    if (channel >= DMA_NUM_CHANNELS || len == 0 || len > DMA_MAX_CB_BYTES) return false;
    if (!dma_channel_allowed(channel)) return false;
    if (dma_busy(channel)) return false;

    /* BCM2712 dma32 uses the SoC DMA address; for low RAM this is PA. */
    if ((u64)(usize)dst >= 0x40000000ULL ||
        len > 0x40000000ULL - (u64)(usize)dst) {
        uart_puts("[dma] addr outside dma-ranges\n");
        return false;
    }

    dcache_clean_range((u64)(usize)&zero_word, sizeof(zero_word));
    dcache_clean_invalidate_range((u64)(usize)dst, len);

    struct dma_cb *cb = &cb_pool[channel][0];

    /* Source does NOT increment (reads 0 repeatedly from zero_word).
     * Destination increments normally. */
    if (!dma_cb_encode(cb, (u64)(usize)&zero_word, (u64)(usize)dst, len, false, 0))
        return false;

    dma_last_error = DMA_ERR_NONE;
    dma_last_channel = channel;
    dma_last_len = len;
    bool started = dma_start(channel, cb);
    if (!started)
        return false;

    if (!dma_wait_complete(channel))
        return false;

    /* Invalidate destination so CPU sees DMA-zeroed data */
    dcache_invalidate_range((u64)(usize)dst, len);

    dma_hw_zeroes++;
    return true;
}

/* ---- Scatter-gather helpers ---- */

/* Build a chain of CBs for a multi-block memcpy.
 * Splits a large transfer into CBS_PER_CHAN chunks.
 * Returns pointer to first CB, or NULL if too many chunks. */
struct dma_cb *dma_build_sg_memcpy(u32 channel, void *dst, const void *src,
                                    u32 len, u32 chunk_size) {
    if (!PIOS_HAS_DMA || core_id() != 0U) return NULL;
    if (channel >= DMA_NUM_CHANNELS || len == 0 ||
        chunk_size == 0 || chunk_size > DMA_MAX_CB_BYTES) return NULL;
    if (!dma_channel_allowed(channel)) return NULL;

    if ((u64)(usize)src >= 0x40000000ULL || (u64)(usize)dst >= 0x40000000ULL ||
        len > 0x40000000ULL - (u64)(usize)src ||
        len > 0x40000000ULL - (u64)(usize)dst)
        return NULL;
    u32 num_chunks = len / chunk_size + (len % chunk_size != 0U);
    if (num_chunks > CBS_PER_CHAN) return NULL;

    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;
    u32 remaining = len;

    for (u32 i = 0; i < num_chunks; i++) {
        struct dma_cb *cb = &cb_pool[channel][i];
        u32 this_len = (remaining > chunk_size) ? chunk_size : remaining;

        u64 next = i + 1U < num_chunks ? (u64)(usize)&cb_pool[channel][i + 1U] : 0U;
        if (!dma_cb_encode(cb, (u64)(usize)s, (u64)(usize)d, this_len, true, next))
            return NULL;

        d += this_len;
        s += this_len;
        remaining -= this_len;
    }

    dcache_clean_range((u64)(usize)&cb_pool[channel][0],
                       num_chunks * (u32)sizeof(struct dma_cb));
    dsb();
    return &cb_pool[channel][0];
}
