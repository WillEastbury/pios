/*
 * rp1_adc.c - RP1 12-bit ADC and internal temperature sensor
 *
 * Single-shot polling follows Raspberry Pi's upstream RP1 ADC driver. The
 * command path owns the device on core 0; DMA/FIFO sampling is deliberately
 * deferred until the RP1 AXI DMAC has its own descriptor-safe driver.
 */

#include "rp1_adc.h"
#include "platform.h"
#include "rp1.h"
#include "rp1_clk.h"
#include "timer.h"

#define RP1_ADC_BASE       0x0C8000UL
#define ADC_CS             0x00U
#define ADC_RESULT         0x04U
#define ADC_FCS            0x08U
#define ADC_DIV            0x10U
#define ADC_INTE           0x18U
#define ADC_INTS           0x20U
#define ADC_SET            0x2000U
#define ADC_CLR            0x3000U

#define CS_AINSEL_MASK     (0x7U << 12)
#define CS_ERR_STICKY      (1U << 10)
#define CS_ERR             (1U << 9)
#define CS_READY           (1U << 8)
#define CS_START_ONCE      (1U << 2)
#define CS_TS_EN           (1U << 1)
#define CS_EN              (1U << 0)

#define ADC_RESULT_MASK    0xFFFU
#define ADC_TIMEOUT_US     1000U
#define ADC_POLL_US        10U

static struct rp1_adc_diag adc_diag ALIGNED(64);
#if PIOS_HAS_RP1
static bool adc_busy;
#endif

static inline u32 adc_read(u32 reg)
{
    return rp1_read32(RP1_ADC_BASE + reg);
}

static inline void adc_write(u32 reg, u32 value)
{
    rp1_write32(RP1_ADC_BASE + reg, value);
}

#if PIOS_HAS_RP1
static u64 adc_now_us(void)
{
    u64 count;
    u64 freq;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(count));
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    return freq ? (count * 1000000ULL) / freq : 0;
}
#endif

static void adc_capture_registers(void)
{
#if PIOS_HAS_RP1
    if (!adc_diag.initialized)
        return;
    adc_diag.cs = adc_read(ADC_CS);
    adc_diag.fcs = adc_read(ADC_FCS);
    adc_diag.div = adc_read(ADC_DIV);
    adc_diag.ints = adc_read(ADC_INTS);
#endif
}

static bool adc_fail(u32 error)
{
    adc_diag.last_error = error;
    adc_diag.failures++;
    adc_capture_registers();
    return false;
}

bool rp1_adc_init(void)
{
#if !PIOS_HAS_RP1
    return adc_fail(RP1_ADC_ERR_UNAVAILABLE);
#else
    if (adc_diag.initialized)
        return true;
    if (!rp1_clk_enable(RP1_CLK_ADC) ||
        !rp1_clk_is_enabled(RP1_CLK_ADC))
        return adc_fail(RP1_ADC_ERR_CLOCK);

    adc_write(ADC_INTE, 0U);
    adc_write(ADC_CS, CS_EN | CS_ERR_STICKY);
    dsb();
    u32 cs = adc_read(ADC_CS);
    adc_diag.cs = cs;
    if ((cs & CS_EN) == 0U)
        return adc_fail(RP1_ADC_ERR_ENABLE);

    adc_diag.initialized = 1U;
    adc_diag.last_error = RP1_ADC_ERR_NONE;
    adc_capture_registers();
    return true;
#endif
}

bool rp1_adc_read_raw(u32 channel, u32 *raw_out)
{
#if !PIOS_HAS_RP1
    (void)channel;
    (void)raw_out;
    return adc_fail(RP1_ADC_ERR_UNAVAILABLE);
#else
    if (!raw_out)
        return adc_fail(RP1_ADC_ERR_BAD_CHANNEL);
    if (!adc_diag.initialized)
        return adc_fail(RP1_ADC_ERR_NOT_INITIALIZED);
    if (channel >= RP1_ADC_CHANNEL_COUNT)
        return adc_fail(RP1_ADC_ERR_BAD_CHANNEL);
    if (adc_busy)
        return adc_fail(RP1_ADC_ERR_BUSY);

    adc_busy = true;
    adc_write(ADC_CLR + ADC_CS, CS_AINSEL_MASK | CS_TS_EN);
    adc_write(ADC_SET + ADC_CS,
              (channel << 12) |
              (channel == RP1_ADC_TEMP_CHANNEL ? CS_TS_EN : 0U));
    adc_write(ADC_SET + ADC_CS, CS_START_ONCE);
    dsb();

    u64 start = adc_now_us();
    u64 deadline = start + ADC_TIMEOUT_US;
    u32 cs;
    do {
        cs = adc_read(ADC_CS);
        if (cs & CS_READY)
            break;
        timer_delay_us(ADC_POLL_US);
    } while (adc_now_us() < deadline);

    if ((cs & CS_READY) == 0U) {
        adc_busy = false;
        adc_diag.timeouts++;
        return adc_fail(RP1_ADC_ERR_TIMEOUT);
    }
    if (cs & CS_ERR) {
        adc_busy = false;
        adc_diag.conversion_errors++;
        return adc_fail(RP1_ADC_ERR_CONVERSION);
    }

    u32 raw = adc_read(ADC_RESULT) & ADC_RESULT_MASK;
    adc_diag.reads++;
    adc_diag.last_channel = channel;
    adc_diag.last_raw = raw;
    adc_diag.last_mv = rp1_adc_raw_to_mv(raw);
    if (channel == RP1_ADC_TEMP_CHANNEL)
        adc_diag.last_temp_mc = rp1_adc_raw_to_millidegrees(raw);
    adc_diag.last_error = RP1_ADC_ERR_NONE;
    adc_busy = false;
    adc_capture_registers();
    *raw_out = raw;
    return true;
#endif
}

bool rp1_adc_read_mv(u32 channel, u32 *mv_out)
{
    u32 raw;
    if (!mv_out || !rp1_adc_read_raw(channel, &raw))
        return false;
    *mv_out = rp1_adc_raw_to_mv(raw);
    return true;
}

bool rp1_adc_read_temperature(i32 *millidegrees_out)
{
    u32 raw;
    if (!millidegrees_out ||
        !rp1_adc_read_raw(RP1_ADC_TEMP_CHANNEL, &raw))
        return false;
    *millidegrees_out = rp1_adc_raw_to_millidegrees(raw);
    return true;
}

void rp1_adc_diag_snapshot(struct rp1_adc_diag *out)
{
    if (!out)
        return;
    adc_capture_registers();
    *out = adc_diag;
}
