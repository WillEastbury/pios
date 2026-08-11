#include "rp1_i2c.h"
#include "platform.h"
#include "mmio.h"
#include "rp1_clk.h"
#include "rp1_gpio.h"
#include "timer.h"

#define I2C1_BASE       (RP1_BAR_BASE + 0x74000ULL)
#define RP1_RESET_CLR0  (RP1_BAR_BASE + 0x17000ULL)
#define RP1_RESET_I2C1  (1U << 8)
#define IC_CON          0x00U
#define IC_TAR          0x04U
#define IC_DATA_CMD     0x10U
#define IC_SS_SCL_HCNT  0x14U
#define IC_SS_SCL_LCNT  0x18U
#define IC_FS_SCL_HCNT  0x1CU
#define IC_FS_SCL_LCNT  0x20U
#define IC_RAW_INTR_STAT 0x34U
#define IC_CLR_INTR     0x40U
#define IC_ENABLE       0x6CU
#define IC_STATUS       0x70U
#define IC_TXFLR        0x74U
#define IC_RXFLR        0x78U
#define IC_TX_ABRT_SOURCE 0x80U
#define IC_ENABLE_STATUS 0x9CU
#define IC_COMP_VERSION 0xF8U
#define IC_COMP_TYPE    0xFCU

#define IC_COMP_TYPE_DW 0x44570140U
#define IC_STATUS_TFNF  (1U << 1)
#define IC_STATUS_RFNE  (1U << 3)
#define IC_STATUS_MST_ACTIVITY (1U << 5)
#define IC_INTR_TX_ABRT (1U << 6)
#define IC_DATA_READ    (1U << 8)
#define IC_DATA_STOP    (1U << 9)
#define IC_DATA_RESTART (1U << 10)

static struct rp1_i2c_diag i2c_diag;
static bool i2c_ready;

static inline u32 ir(u32 off) { return mmio_read(I2C1_BASE + off); }
static inline void iw(u32 off, u32 val) { mmio_write(I2C1_BASE + off, val); }

bool rp1_i2c_init(u32 bus_hz)
{
#if !PIOS_HAS_RP1
    (void)bus_hz;
    return false;
#else
    i2c_ready = false;
    if (bus_hz != 100000U && bus_hz != 400000U)
        return false;
    (void)rp1_clk_enable(RP1_CLK_SYS);
    mmio_write(RP1_RESET_CLR0, RP1_RESET_I2C1);
    timer_delay_us(10U);
    i2c_diag.comp_type = ir(IC_COMP_TYPE);
    i2c_diag.comp_version = ir(IC_COMP_VERSION);
    if (i2c_diag.comp_type != IC_COMP_TYPE_DW)
        return false;

    rp1_gpio_set_function(2U, RP1_FSEL_ALT3);
    rp1_gpio_set_function(3U, RP1_FSEL_ALT3);
    rp1_gpio_set_pull(2U, RP1_PULL_UP);
    rp1_gpio_set_pull(3U, RP1_PULL_UP);

    iw(IC_ENABLE, 0U);
    u32 period = 200000000U / bus_hz;
    u32 high = period * 4U / 10U;
    u32 low = period - high;
    if (bus_hz == 100000U) {
        iw(IC_SS_SCL_HCNT, high);
        iw(IC_SS_SCL_LCNT, low);
        iw(IC_CON, (1U << 0) | (1U << 5));
    } else {
        iw(IC_FS_SCL_HCNT, high);
        iw(IC_FS_SCL_LCNT, low);
        iw(IC_CON, (1U << 0) | (2U << 1) | (1U << 5));
    }
    iw(IC_ENABLE, 1U);
    i2c_ready = (ir(IC_ENABLE_STATUS) & 1U) != 0U;
    return i2c_ready;
#endif
}

bool rp1_i2c_write_read(u8 address, const u8 *write_data, u32 write_len,
                        u8 *read_data, u32 read_len)
{
    if (!i2c_ready || address > 0x7FU ||
        (write_len && !write_data) || (read_len && !read_data) ||
        (write_len == 0U && read_len == 0U))
        return false;
    iw(IC_ENABLE, 0U);
    iw(IC_TAR, address);
    iw(IC_ENABLE, 1U);
    (void)ir(IC_CLR_INTR);

    u32 wi = 0U, queued_reads = 0U, ri = 0U;
    u64 deadline = timer_monotonic_ms() + 1000ULL;
    while (wi < write_len || queued_reads < read_len || ri < read_len) {
        u32 raw = ir(IC_RAW_INTR_STAT);
        if (raw & IC_INTR_TX_ABRT) {
            i2c_diag.abort_source = ir(IC_TX_ABRT_SOURCE);
            (void)ir(IC_CLR_INTR);
            return false;
        }
        if (wi < write_len && (ir(IC_STATUS) & IC_STATUS_TFNF)) {
            u32 cmd = write_data[wi++];
            if (wi == write_len && read_len == 0U)
                cmd |= IC_DATA_STOP;
            iw(IC_DATA_CMD, cmd);
        } else if (queued_reads < read_len &&
                   (ir(IC_STATUS) & IC_STATUS_TFNF)) {
            u32 cmd = IC_DATA_READ;
            if (queued_reads == 0U && write_len)
                cmd |= IC_DATA_RESTART;
            if (++queued_reads == read_len)
                cmd |= IC_DATA_STOP;
            iw(IC_DATA_CMD, cmd);
        }
        while (ri < read_len && (ir(IC_STATUS) & IC_STATUS_RFNE))
            read_data[ri++] = (u8)ir(IC_DATA_CMD);
        if (timer_monotonic_ms() >= deadline) {
            i2c_diag.timeouts++;
            return false;
        }
    }
    while ((ir(IC_STATUS) & IC_STATUS_MST_ACTIVITY) || ir(IC_TXFLR)) {
        if (timer_monotonic_ms() >= deadline) {
            i2c_diag.timeouts++;
            return false;
        }
    }
    i2c_diag.status = ir(IC_STATUS);
    return true;
}

void rp1_i2c_diag_snapshot(struct rp1_i2c_diag *out)
{
    if (out) *out = i2c_diag;
}

bool rp1_i2c_probe(u8 address)
{
    if (!i2c_ready || address > 0x7FU)
        return false;
    /* A one-byte read is the least intrusive probe: it does not write to any
     * register on the target, so scanning cannot corrupt device state the way
     * a write probe can. An absent device NAKs its address and the transfer
     * aborts. */
    u8 scratch = 0U;
    u32 before = i2c_diag.timeouts;
    bool ok = rp1_i2c_write_read(address, NULL, 0U, &scratch, 1U);
    /* A timeout means the bus is wedged, not that the device is absent. */
    if (i2c_diag.timeouts != before)
        return false;
    return ok;
}

u32 rp1_i2c_scan(u8 *found, u32 found_len)
{
    if (!found || found_len < 16U)
        return 0U;
    for (u32 i = 0U; i < 16U; i++)
        found[i] = 0U;
    u32 count = 0U;
    /* 0x00-0x07 and 0x78-0x7F are reserved by the I2C specification; probing
     * them can trigger general-call or 10-bit addressing behaviour. */
    for (u8 addr = 0x08U; addr <= 0x77U; addr++) {
        if (rp1_i2c_probe(addr)) {
            found[addr >> 3] |= (u8)(1U << (addr & 7U));
            count++;
        }
    }
    return count;
}
