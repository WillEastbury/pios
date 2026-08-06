/*
 * rp1_dma.c - guarded RP1 internal AXI-DMAC bring-up
 *
 * The first supported operation is an explicit 256-byte memory-copy selftest.
 * It uses one immutable 64-byte LLI, bounded polling, byte-exact verification,
 * and permanent quarantine after the first transfer failure.
 */

#include "rp1_dma.h"
#include "platform.h"
#include "rp1.h"
#include "rp1_clk.h"
#include "mmu.h"
#include "watchdog.h"

#define RP1_DMA_BASE          0x188000UL
#define RP1_DMA_PCIE_BASE     0x1000000000ULL

#define DMAC_ID               0x000U
#define DMAC_COMPVER          0x008U
#define DMAC_CFG              0x010U
#define DMAC_CHEN             0x018U
#define DMAC_INTSTATUS        0x030U
#define DMAC_RESET            0x058U

#define CH_BASE               0x100U
#define CH_LLP                0x028U
#define CH_STATUS             0x030U
#define CH_CFG_L              0x020U
#define CH_CFG_H              0x024U
#define CH_INTSTATUS_ENA      0x080U
#define CH_INTSTATUS          0x088U
#define CH_INTSIGNAL_ENA      0x090U
#define CH_INTCLEAR           0x098U

#define DMAC_CFG_ENABLE       (1U << 0)
#define CH_ENABLE             (1U << 0)
#define CH_ENABLE_WE          (1U << 8)

#define CH_CTL_H_ARLEN_EN     (1U << 6)
#define CH_CTL_H_ARLEN_POS    7U
#define CH_CTL_H_AWLEN_EN     (1U << 15)
#define CH_CTL_H_AWLEN_POS    16U
#define CH_CTL_H_LLI_LAST     (1U << 30)
#define CH_CTL_H_LLI_VALID    (1U << 31)

#define CH_CTL_L_DST_MSIZE_POS 18U
#define CH_CTL_L_SRC_MSIZE_POS 14U
#define CH_CTL_L_DST_WIDTH_POS 11U
#define CH_CTL_L_SRC_WIDTH_POS 8U

#define CH_CFG_L_DST_MULTBLK_POS 2U
#define CH_CFG_L_SRC_MULTBLK_POS 0U
#define CH_CFG_L_MULTBLK_LL      3U

#define IRQ_DMA_TRANSFER       (1U << 1)
#define IRQ_ALL_ERRORS         (0x3FU << 16 | 0x3FFU << 5)

#define RP1_DMA_TEST_BYTES     256U
#define RP1_DMA_TIMEOUT_US     100000U

struct rp1_dma_lli {
    u64 sar;
    u64 dar;
    u32 block_ts_lo;
    u32 block_ts_hi;
    u64 llp;
    u32 ctl_lo;
    u32 ctl_hi;
    u32 sstat;
    u32 dstat;
    u32 status_lo;
    u32 status_hi;
    u32 reserved_lo;
    u32 reserved_hi;
} PACKED ALIGNED(64);

_Static_assert(sizeof(struct rp1_dma_lli) == 64U,
               "RP1 DMA LLI wire format must be 64 bytes");

static struct rp1_dma_diag dma_diag ALIGNED(64);
#if PIOS_HAS_RP1
static struct rp1_dma_lli test_lli ALIGNED(64);
static u8 test_src[RP1_DMA_TEST_BYTES] ALIGNED(64);
static u8 test_dst[RP1_DMA_TEST_BYTES] ALIGNED(64);
static bool dma_busy;
static bool probe_watchdog_armed;
#endif

static inline u32 dma_read(u32 reg)
{
    return rp1_read32(RP1_DMA_BASE + reg);
}

static inline void dma_write(u32 reg, u32 value)
{
    rp1_write32(RP1_DMA_BASE + reg, value);
}

static inline u32 chan_read(u32 reg)
{
    return dma_read(CH_BASE + reg);
}

static inline void chan_write(u32 reg, u32 value)
{
    dma_write(CH_BASE + reg, value);
}

#if PIOS_HAS_RP1
static void chan_write64(u32 reg, u64 value)
{
    chan_write(reg, (u32)value);
    chan_write(reg + 4U, (u32)(value >> 32));
}
#endif

#if PIOS_HAS_RP1
static u64 dma_now_us(void)
{
    u64 count;
    u64 freq;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(count));
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    return freq ? (count * 1000000ULL) / freq : 0;
}
#endif

static void capture_registers(void)
{
#if PIOS_HAS_RP1
    if (!dma_diag.probed)
        return;
    dma_diag.cfg = dma_read(DMAC_CFG);
    dma_diag.chen = dma_read(DMAC_CHEN);
    dma_diag.int_status = dma_read(DMAC_INTSTATUS);
    dma_diag.chan_status = chan_read(CH_STATUS);
    dma_diag.chan_int_status = chan_read(CH_INTSTATUS);
#endif
}

static bool fail_probe(u32 error)
{
    dma_diag.last_error = error;
#if PIOS_HAS_RP1
    if (probe_watchdog_armed) {
        watchdog_hw_disable();
        probe_watchdog_armed = false;
    }
#endif
    return false;
}

#if PIOS_HAS_RP1
static bool fail_transfer(u32 error)
{
    dma_diag.last_error = error;
    dma_diag.selftest_failures++;
    dma_diag.quarantined = 1U;
    dma_diag.ready = 0U;
    capture_registers();
    watchdog_hw_disable();
    probe_watchdog_armed = false;
    return false;
}
#endif

bool rp1_dma_probe(void)
{
#if !PIOS_HAS_RP1
    return fail_probe(RP1_DMA_ERR_UNAVAILABLE);
#else
    if (dma_diag.probed)
        return dma_diag.ready != 0U && dma_diag.quarantined == 0U;
    if (!rp1_clk_enable(RP1_CLK_DMA) ||
        !rp1_clk_is_enabled(RP1_CLK_DMA))
        return fail_probe(RP1_DMA_ERR_CLOCK);

    watchdog_hw_arm_seconds(15U);
    probe_watchdog_armed = true;
    u32 id = dma_read(DMAC_ID);
    u32 version = dma_read(DMAC_COMPVER);
    dma_diag.dmac_id = id;
    dma_diag.comp_version = version;
    if (id == 0U || id == 0xFFFFFFFFU ||
        version == 0U || version == 0xFFFFFFFFU)
        return fail_probe(RP1_DMA_ERR_PROBE);

    dma_write(DMAC_RESET, 1U);
    u64 deadline = dma_now_us() + RP1_DMA_TIMEOUT_US;
    while (dma_read(DMAC_RESET) != 0U && dma_now_us() < deadline)
        ;
    if (dma_read(DMAC_RESET) != 0U)
        return fail_probe(RP1_DMA_ERR_RESET_TIMEOUT);

    dma_write(DMAC_CHEN, CH_ENABLE_WE);
    chan_write(CH_INTSTATUS_ENA, 0U);
    chan_write(CH_INTSIGNAL_ENA, 0U);
    chan_write(CH_INTCLEAR, 0xFFFFFFFFU);
    dma_write(DMAC_CFG, DMAC_CFG_ENABLE);
    dsb();

    dma_diag.probed = 1U;
    dma_diag.ready = 1U;
    dma_diag.last_error = RP1_DMA_ERR_NONE;
    capture_registers();
    watchdog_hw_disable();
    probe_watchdog_armed = false;
    return true;
#endif
}

bool rp1_dma_selftest(void)
{
#if !PIOS_HAS_RP1
    return fail_probe(RP1_DMA_ERR_UNAVAILABLE);
#else
    dma_diag.selftest_runs++;
    if (dma_diag.quarantined)
        return fail_probe(RP1_DMA_ERR_QUARANTINED);
    if (!rp1_dma_probe())
        return false;
    if (dma_busy)
        return fail_probe(RP1_DMA_ERR_BUSY);
    dma_busy = true;
    watchdog_hw_arm_seconds(15U);
    probe_watchdog_armed = true;

    for (u32 i = 0; i < RP1_DMA_TEST_BYTES; i++) {
        test_src[i] = (u8)((i * 37U + 0x5AU) & 0xFFU);
        test_dst[i] = 0xA5U;
    }
    for (u32 i = 0; i < sizeof(test_lli); i++)
        ((u8 *)&test_lli)[i] = 0U;

    const u32 transfer_width = 4U; /* 128-bit RP1 master data width */
    const u32 block_count = RP1_DMA_TEST_BYTES >> transfer_width;
    const u32 burst_len = 8U;
    const u32 burst_msize = 1U; /* four transfers */
    test_lli.sar = RP1_DMA_PCIE_BASE + (u64)(usize)test_src;
    test_lli.dar = RP1_DMA_PCIE_BASE + (u64)(usize)test_dst;
    test_lli.block_ts_lo = block_count - 1U;
    test_lli.ctl_lo =
        (burst_msize << CH_CTL_L_DST_MSIZE_POS) |
        (burst_msize << CH_CTL_L_SRC_MSIZE_POS) |
        (transfer_width << CH_CTL_L_DST_WIDTH_POS) |
        (transfer_width << CH_CTL_L_SRC_WIDTH_POS);
    test_lli.ctl_hi =
        CH_CTL_H_LLI_VALID | CH_CTL_H_LLI_LAST |
        CH_CTL_H_ARLEN_EN | (burst_len << CH_CTL_H_ARLEN_POS) |
        CH_CTL_H_AWLEN_EN | (burst_len << CH_CTL_H_AWLEN_POS);

    dcache_clean_range((u64)(usize)test_src, sizeof(test_src));
    dcache_clean_invalidate_range((u64)(usize)test_dst, sizeof(test_dst));
    dcache_clean_range((u64)(usize)&test_lli, sizeof(test_lli));
    dsb();

    dma_write(DMAC_CHEN, CH_ENABLE_WE);
    chan_write(CH_INTCLEAR, 0xFFFFFFFFU);
    chan_write(CH_CFG_L,
               (CH_CFG_L_MULTBLK_LL << CH_CFG_L_DST_MULTBLK_POS) |
               (CH_CFG_L_MULTBLK_LL << CH_CFG_L_SRC_MULTBLK_POS));
    chan_write(CH_CFG_H, 0U);
    chan_write(CH_INTSTATUS_ENA, IRQ_DMA_TRANSFER | IRQ_ALL_ERRORS);
    chan_write(CH_INTSIGNAL_ENA, 0U);
    chan_write64(CH_LLP, RP1_DMA_PCIE_BASE + (u64)(usize)&test_lli);
    dsb();
    dma_write(DMAC_CHEN, CH_ENABLE | CH_ENABLE_WE);

    u64 deadline = dma_now_us() + RP1_DMA_TIMEOUT_US;
    u32 irq;
    do {
        irq = chan_read(CH_INTSTATUS);
        if (irq & (IRQ_DMA_TRANSFER | IRQ_ALL_ERRORS))
            break;
    } while (dma_now_us() < deadline);

    dma_write(DMAC_CHEN, CH_ENABLE_WE);
    chan_write(CH_INTCLEAR, irq);
    dsb();
    dma_busy = false;
    dcache_invalidate_range((u64)(usize)test_dst, sizeof(test_dst));

    if ((irq & IRQ_ALL_ERRORS) != 0U)
        return fail_transfer(RP1_DMA_ERR_TRANSFER_HW);
    if ((irq & IRQ_DMA_TRANSFER) == 0U)
        return fail_transfer(RP1_DMA_ERR_TRANSFER_TIMEOUT);

    for (u32 i = 0; i < RP1_DMA_TEST_BYTES; i++) {
        if (test_dst[i] != test_src[i]) {
            dma_diag.last_mismatch = i;
            dma_diag.last_got = test_dst[i];
            dma_diag.last_expected = test_src[i];
            return fail_transfer(RP1_DMA_ERR_MISMATCH);
        }
    }

    dma_diag.last_error = RP1_DMA_ERR_NONE;
    capture_registers();
    watchdog_hw_disable();
    probe_watchdog_armed = false;
    return true;
#endif
}

void rp1_dma_diag_snapshot(struct rp1_dma_diag *out)
{
    if (!out)
        return;
    capture_registers();
    *out = dma_diag;
}
