/*
 * sdio.c - BCM2712 SDIO2 host controller driver
 *
 * Minimal polling SDIO host for the native WiFi SDIO controller. Pi 5 uses
 * BCM2712 SDIO2; Pi 3/Zero 2 W use legacy BCM2837 SDIO1 at 0x3F300000.
 *
 * SDHCI-compatible register layout. Reuses the same command encoding
 * scheme as sd.c (EMMC2) but adds SDIO-specific CMD5/CMD52/CMD53.
 *
 * Polling mode, no DMA, no interrupts.
 *
 * Reference: SD Host Controller Simplified Spec v3.0
 *            SDIO Simplified Spec v3.0
 *            Linux drivers/mmc/host/sdhci.c
 */

#include "sdio.h"
#include "mmio.h"
#include "uart.h"
#include "timer.h"
#include "gic.h"
#include "fb.h"
#include "mailbox.h"
#include "exception.h"
#include "airq.h"
#include "core.h"
#include "net_dispatch.h"

/* ── SDHCI register offsets ── */
#define REG_ARG2            0x00
#define REG_BLKSIZECNT      0x04
#define REG_ARG1            0x08
#define REG_CMDTM           0x0C
#define REG_RESP0           0x10
#define REG_RESP1           0x14
#define REG_RESP2           0x18
#define REG_RESP3           0x1C
#define REG_DATA            0x20
#define REG_STATUS          0x24
#define REG_CONTROL0        0x28
#define REG_CONTROL1        0x2C
#define REG_INTERRUPT       0x30
#define REG_IRPT_MASK       0x34
#define REG_IRPT_EN         0x38
#define REG_CONTROL2        0x3C
#define REG_CAP0            0x40
#define REG_CAP1            0x44

/* Status register bits */
#define SR_CMD_INHIBIT      (1 << 0)
#define SR_DAT_INHIBIT      (1 << 1)
#define SR_READ_AVAILABLE   (1 << 11)
#define SR_WRITE_READY      (1 << 10)

/* Control1 bits */
#define C1_SRST_HC          (1 << 24)
#define C1_SRST_CMD         (1 << 25)
#define C1_SRST_DATA        (1 << 26)
#define C1_CLK_EN           (1 << 2)
#define C1_CLK_STABLE       (1 << 1)
#define C1_CLK_INTLEN       (1 << 0)
#define C1_TOUNIT(x)        ((x) << 16)

/* Interrupt bits */
#define INT_CMD_DONE        (1 << 0)
#define INT_DATA_DONE       (1 << 1)
#define INT_WRITE_RDY       (1 << 4)
#define INT_READ_RDY        (1 << 5)
#define INT_CARD            (1 << 8)    /* SDIO in-band card interrupt */
#define INT_ERROR_SUMMARY   (1U << 15)
#define INT_ERROR           0xFFFF8000U  /* summary + all error bits */
#define INT_ALL             0xFFFFFFFFU  /* clear everything */

/* Command encoding */
#define CMD(n)              ((n) << 24)
#define RSP_NONE            0
#define RSP_136             (1 << 16)
#define RSP_48              (2 << 16)
#define RSP_48_BUSY         (3 << 16)
#define CMD_ISDATA          (1 << 21)
#define CMD_IXCHK           (1 << 20)
#define CMD_CRCCHK          (1 << 19)
#define TM_READ             (1 << 4)
#define TM_WRITE            0
#define TM_MULTI            (1 << 5)
#define TM_BLKCNT           (1 << 1)

/* ── SDIO-specific command encodings ── */
#define SDIO_CMD0   (CMD(0)  | RSP_NONE)
#define SDIO_CMD3   (CMD(3)  | RSP_48  | CMD_IXCHK | CMD_CRCCHK)
#define SDIO_CMD5   (CMD(5)  | RSP_48)
#define SDIO_CMD7   (CMD(7)  | RSP_48_BUSY | CMD_IXCHK | CMD_CRCCHK)
#define SDIO_CMD52  (CMD(52) | RSP_48  | CMD_IXCHK | CMD_CRCCHK)
#define SDIO_CMD53_R (CMD(53) | RSP_48 | CMD_ISDATA | TM_READ  | CMD_IXCHK | CMD_CRCCHK)
#define SDIO_CMD53_W (CMD(53) | RSP_48 | CMD_ISDATA | TM_WRITE | CMD_IXCHK | CMD_CRCCHK)

/* CMD52 argument bits */
#define CMD52_WRITE         (1U << 31)
#define CMD52_FUNC(f)       ((u32)(f) << 28)
#define CMD52_RAW           (1U << 27)
#define CMD52_ADDR(a)       (((u32)(a) & 0x1FFFF) << 9)
#define CMD52_DATA(d)       ((u32)(d) & 0xFF)

/* CMD53 argument bits */
#define CMD53_WRITE         (1U << 31)
#define CMD53_FUNC(f)       ((u32)(f) << 28)
#define CMD53_BLOCK_MODE    (1U << 27)
#define CMD53_INCR_ADDR     (1U << 26)
#define CMD53_ADDR(a)       (((u32)(a) & 0x1FFFF) << 9)
#define CMD53_COUNT(c)      ((u32)(c) & 0x1FF)
#define R5_ERROR_MASK       0x0000CB00U

/* Card state */
static u32 sdio_rca;
static struct sdio_diag sdio_diag ALIGNED(64);
static bool sdio_initialized;

/* ── Register helpers ── */
/* 32-bit access (responses, data, present state, interrupts, caps) */
#if PIOS_HAS_WIFI_SDIO
static inline u32 sr(u32 off)           { return mmio_read(WIFI_SDIO_HOST_BASE + off); }
static inline void sw(u32 off, u32 val) { mmio_write(WIFI_SDIO_HOST_BASE + off, val); }
/* 16-bit access (clock control, block size/count, host control 2, transfer mode) */
static inline u16 sr16(u32 off)           { return mmio_read16(WIFI_SDIO_HOST_BASE + off); }
static inline void sw16(u32 off, u16 val) { mmio_write16(WIFI_SDIO_HOST_BASE + off, val); }
/* 8-bit access (host control, power control, timeout, software reset) */
static inline u8 sr8(u32 off)           { return mmio_read8(WIFI_SDIO_HOST_BASE + off); }
static inline void sw8(u32 off, u8 val) { mmio_write8(WIFI_SDIO_HOST_BASE + off, val); }
#else
static inline u32 sr(u32 off) { (void)off; return 0U; }
static inline void sw(u32 off, u32 val) { (void)off; (void)val; }
static inline u16 sr16(u32 off) { (void)off; return 0U; }
static inline void sw16(u32 off, u16 val) { (void)off; (void)val; }
static inline u8 sr8(u32 off) { (void)off; return 0U; }
static inline void sw8(u32 off, u8 val) { (void)off; (void)val; }
#endif
static volatile bool sdio_card_irq_latched;
static bool sdio_card_irq_level;

/* SDHCI spec-accurate sub-register offsets */
#define SDHCI_HOST_CONTROL    0x28  /* 8-bit */
#define SDHCI_POWER_CONTROL   0x29  /* 8-bit */
#define SDHCI_BLOCK_GAP       0x2A  /* 8-bit */
#define SDHCI_WAKEUP          0x2B  /* 8-bit */
#define SDHCI_CLOCK_CONTROL   0x2C  /* 16-bit */
#define SDHCI_TIMEOUT_CONTROL 0x2E  /* 8-bit */
#define SDHCI_SOFTWARE_RESET  0x2F  /* 8-bit */
#define SDHCI_HOST_CONTROL2   0x3E  /* 16-bit */

/* Software reset bits (8-bit register at 0x2F) */
#define SDHCI_RESET_ALL       0x01
#define SDHCI_RESET_CMD       0x02
#define SDHCI_RESET_DATA      0x04

/* Power control bits */
#define SDHCI_POWER_ON        0x01
#define SDHCI_POWER_330       0x0E  /* 3.3V */

/* ── Low-level helpers ── */

static bool sdio_wait_cmd(void)
{
    for (u32 i = 0; i < 1000000; i++) {
        if (!(sr(REG_STATUS) & SR_CMD_INHIBIT))
            return true;
        delay_cycles(10);
    }
    return false;
}

static bool sdio_wait_data(void)
{
    for (u32 i = 0; i < 100000; i++) {
        if (!(sr(REG_STATUS) & SR_DAT_INHIBIT))
            return true;
        delay_cycles(10);
    }
    /* Try DATA reset to recover */
    sw8(SDHCI_SOFTWARE_RESET, SDHCI_RESET_DATA);
    u32 t = 100000;
    while ((sr8(SDHCI_SOFTWARE_RESET) & SDHCI_RESET_DATA) && t--)
        delay_cycles(10);
    sw(REG_INTERRUPT, 0xFFFFFFFF);
    return !(sr(REG_STATUS) & SR_DAT_INHIBIT);
}

static bool sdio_reset_cmd_line(void)
{
    sw8(SDHCI_SOFTWARE_RESET, SDHCI_RESET_CMD);
    u32 timeout = 100000U;
    while (timeout != 0U && (sr8(SDHCI_SOFTWARE_RESET) &
                             SDHCI_RESET_CMD) != 0U) {
        timeout--;
        delay_cycles(10U);
    }
    sw(REG_INTERRUPT, INT_ALL);
    if ((sr8(SDHCI_SOFTWARE_RESET) & SDHCI_RESET_CMD) != 0U) {
        uart_puts("[sdio] CMD reset timeout\n");
        return false;
    }
    return true;
}

static bool sdio_send_cmd(u32 cmd, u32 arg, u32 *resp)
{
    if (!sdio_wait_cmd())
        return false;

    /* Circle pattern: clear DAT_INHIBIT before data/busy commands */
    if ((sr(REG_STATUS) & SR_DAT_INHIBIT) &&
        (cmd & CMD_ISDATA)) {
        sw8(SDHCI_SOFTWARE_RESET, SDHCI_RESET_DATA);
        u32 t = 100000;
        while ((sr8(SDHCI_SOFTWARE_RESET) & SDHCI_RESET_DATA) && t--)
            delay_cycles(10);
        t = 100000;
        while ((sr(REG_STATUS) & SR_DAT_INHIBIT) && t--)
            delay_cycles(10);
        sw(REG_INTERRUPT, 0xFFFFFFFF);
    }

    sw(REG_INTERRUPT, INT_ALL);
    sw(REG_ARG1, arg);
    sw(REG_CMDTM, cmd);

    u32 timeout = 1000000;
    u32 intr;
    do {
        intr = sr(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            sw(REG_INTERRUPT, INT_ERROR);
            return false;
        }
        delay_cycles(10);
    } while (!(intr & INT_CMD_DONE) && timeout--);

    if (!timeout)
        return false;

    sw(REG_INTERRUPT, INT_CMD_DONE);

    if (resp) {
        resp[0] = sr(REG_RESP0);
        resp[1] = sr(REG_RESP1);
        resp[2] = sr(REG_RESP2);
        resp[3] = sr(REG_RESP3);
    }

    /* For R1b (busy) responses, wait for DAT line to release */
    if (cmd & RSP_48_BUSY) {
        u32 busy_timeout = 1000000;
        while ((sr(REG_STATUS) & SR_DAT_INHIBIT) && busy_timeout--)
            delay_cycles(10);
        if (!busy_timeout) {
            uart_puts("[sdio] busy timeout\n");
            return false;
        }
    }

    return true;
}

static void sdio_set_clock(u32 freq_khz)
{
    /* Disable SD clock via 16-bit CLOCK_CONTROL */
    sw16(SDHCI_CLOCK_CONTROL, 0);
    delay_cycles(1000);

    /* Set timeout via 8-bit register */
    sw8(SDHCI_TIMEOUT_CONTROL, 0x0E);

    /* Derive base clock from SDHCI capabilities register */
    u32 cap = sr(REG_CAP0);
    u32 base_mhz = (cap >> 8) & 0xFF;
    if (base_mhz == 0) base_mhz = 50;  /* fallback */
    u32 base_khz = base_mhz * 1000;

    /* Calculate divider — must produce even real divisor per SDHCI v3 spec */
    u32 real_div = (base_khz + freq_khz - 1) / freq_khz;
    if (real_div & 1) real_div++;  /* round up to even */
    if (real_div < 2) real_div = 2;
    u32 encoded = real_div >> 1;
    if (encoded > 0x3FF) encoded = 0x3FF;

    /* CLOCK_CONTROL: [15:8]=SDCLK Freq Select, [7:6]=upper divider bits,
     * [2]=SD Clock Enable, [1]=Clock Stable (RO), [0]=Internal Clock Enable */
    u16 clk = ((encoded & 0xFF) << 8) | (((encoded >> 8) & 0x3) << 6) | 0x01;
    sw16(SDHCI_CLOCK_CONTROL, clk);
    delay_cycles(1000);

    /* Wait for internal clock stable (bit 1) */
    u32 timeout = 100000;
    while (!(sr16(SDHCI_CLOCK_CONTROL) & 0x02) && timeout--)
        delay_cycles(10);

    /* Enable SD clock (bit 2) */
    clk = sr16(SDHCI_CLOCK_CONTROL);
    clk |= 0x04;
    sw16(SDHCI_CLOCK_CONTROL, clk);
    delay_cycles(1000);
}

/* ── GPIO and power setup ── */

/* ── BCM2712 SoC GPIO/pinctrl helpers ── */

static u32 soc_stepping;

static void detect_soc_stepping(void)
{
    u32 val = mmio_read(BCM2712_SOC_STEPPING);
    uart_puts("[sdio] SOC_STEPPING=");
    uart_hex(val);
    if ((val >> 16) == 0x2712) {
        soc_stepping = val & 0xFF;
        uart_puts(" rev=");
        uart_hex(soc_stepping);
    } else {
        soc_stepping = 0xFF;  /* unknown — assume pre-D0 */
        uart_puts(" (unknown SoC)");
    }
    uart_puts("\n");
}

static bool is_d0_stepping(void)
{
    return soc_stepping >= SOC_STEPPING_D0;
}

/* BCM2712 pinctrl: set pin function via FSEL registers at BCM2712_PINCTRL_BASE.
 * Register layout depends on SoC stepping:
 *   Pre-D0: 8 GPIOs per 32-bit reg, 4 bits each: reg = (pin/8)*4, shift = (pin%8)*4
 *   D0+:    Pins 28-31 in reg 2, pins 32-35 in reg 3, shift = ((pin-24)%8)*4 */
static void bcm2712_gpio_set_fsel(u32 pin, u32 fsel)
{
    u32 reg_off, shift;
    if (is_d0_stepping()) {
        reg_off = (pin < 32 ? 2 : 3) * 4;
        shift = ((pin - 24) % 8) * 4;
    } else {
        reg_off = (pin / 8) * 4;
        shift = (pin % 8) * 4;
    }
    u32 val = mmio_read(BCM2712_PINCTRL_BASE + reg_off);
    val &= ~(0xFU << shift);
    val |= (fsel & 0xF) << shift;
    mmio_write(BCM2712_PINCTRL_BASE + reg_off, val);
}

static u32 bcm2712_gpio_get_fsel(u32 pin)
{
    u32 reg_off, shift;
    if (is_d0_stepping()) {
        reg_off = (pin < 32 ? 2 : 3) * 4;
        shift = ((pin - 24) % 8) * 4;
    } else {
        reg_off = (pin / 8) * 4;
        shift = (pin % 8) * 4;
    }
    return (mmio_read(BCM2712_PINCTRL_BASE + reg_off) >> shift) & 0xF;
}

/* BCM2712 GPIO pad pull-up/down control.
 * Layout depends on stepping:
 *   Pre-D0: offset = (pin+112)/15, bit = ((pin+112)%15)*2
 *   D0+:    pins 28-32 in reg 5, pins 33-35 in reg 6, bit = ((pin-18)%15)*2 */
static void bcm2712_gpio_set_pull(u32 pin, u32 mode)
{
    u32 pad_off, pad_bit;
    if (is_d0_stepping()) {
        pad_off = (pin < 33 ? 5 : 6) * 4;
        pad_bit = ((pin - 18) % 15) * 2;
    } else {
        u32 offset = pin + 112;
        pad_off = (offset / 15) * 4;
        pad_bit = (offset % 15) * 2;
    }
    u32 val = mmio_read(BCM2712_PINCTRL_BASE + pad_off);
    val &= ~(3U << pad_bit);
    val |= (mode << pad_bit);
    mmio_write(BCM2712_PINCTRL_BASE + pad_off, val);
}

#define PULL_NONE 0
#define PULL_UP   2

static void sdio_gpio_init(void)
{
#if PIOS_HAS_WIFI_SDIO2
    detect_soc_stepping();

    /* Read current FSEL for SDIO2 pins */
    uart_puts("[sdio] FSEL: ");
    for (u32 p = 30; p <= 35; p++) {
        uart_puts("g");
        uart_hex(p);
        uart_puts("=");
        uart_hex(bcm2712_gpio_get_fsel(p));
        uart_puts(" ");
    }
    uart_puts("\n");

    /* Pin function select per SoC stepping (from Circle ether4330.c):
     *   D0+:    All pins = Func1
     *   Pre-D0: Most = Func4, pins 33/35 = Func3 */
    u32 clk_func, cmd_func, d0_func, d1_func, d2_func, d3_func;
    if (is_d0_stepping()) {
        clk_func = cmd_func = d0_func = d1_func = d2_func = d3_func = 1;
    } else {
        clk_func = 4; cmd_func = 4; d0_func = 4;
        d1_func = 3;  d2_func = 4;  d3_func = 3;
    }

    uart_puts("[sdio] Configuring SDIO2 pins (");
    uart_puts(is_d0_stepping() ? "D0+" : "pre-D0");
    uart_puts(")...\n");

    bcm2712_gpio_set_fsel(30, clk_func);  bcm2712_gpio_set_pull(30, PULL_NONE);
    bcm2712_gpio_set_fsel(31, cmd_func);  bcm2712_gpio_set_pull(31, PULL_UP);
    bcm2712_gpio_set_fsel(32, d0_func);   bcm2712_gpio_set_pull(32, PULL_UP);
    bcm2712_gpio_set_fsel(33, d1_func);   bcm2712_gpio_set_pull(33, PULL_UP);
    bcm2712_gpio_set_fsel(34, d2_func);   bcm2712_gpio_set_pull(34, PULL_UP);
    bcm2712_gpio_set_fsel(35, d3_func);   bcm2712_gpio_set_pull(35, PULL_UP);

    /* Verify */
    uart_puts("[sdio] post-mux FSEL: ");
    for (u32 p = 30; p <= 35; p++) {
        uart_puts("g");
        uart_hex(p);
        uart_puts("=");
        uart_hex(bcm2712_gpio_get_fsel(p));
        uart_puts(" ");
    }
    uart_puts("\n");
#elif PIOS_HAS_WIFI_SDIO1
    /* BCM2837 SDIO1 uses the legacy GPIO block: GPIO34-39 are ALT3
     * (function-select value 7), with the controller's card-side pull-ups
     * enabled during identification. */
    static const u32 pins[] = { 34U, 35U, 36U, 37U, 38U, 39U };
    const u32 pin_mask = 0xFCU; /* GPIO34-39 in GPIO bank 1 */
    uart_puts("[sdio] configuring BCM2837 SDIO1 GPIO34-39\n");
    for (u32 i = 0U; i < sizeof(pins) / sizeof(pins[0]); i++) {
        u32 pin = pins[i];
        u32 off = (pin / 10U) * 4U;
        u32 shift = (pin % 10U) * 3U;
        u32 fsel = mmio_read(PIOS_PERIPH_BASE + 0x200000UL + off);
        fsel &= ~(7U << shift);
        fsel |= 7U << shift; /* ALT3 = SD1_CLK/CMD/DAT[0..3] */
        mmio_write(PIOS_PERIPH_BASE + 0x200000UL + off, fsel);
    }
    /* BCM2835/2837 GPIO pull sequence: select pull-up, clock it into the
     * bank-1 pins, then return the global pull selector to disabled. */
    mmio_write(PIOS_PERIPH_BASE + 0x200000UL + 0x94U, 2U);
    delay_cycles(150U);
    mmio_write(PIOS_PERIPH_BASE + 0x200000UL + 0x9CU, pin_mask);
    delay_cycles(150U);
    mmio_write(PIOS_PERIPH_BASE + 0x200000UL + 0x94U, 0U);
    mmio_write(PIOS_PERIPH_BASE + 0x200000UL + 0x9CU, 0U);
    uart_puts("[sdio] SDIO1 pins ready\n");
#else
    uart_puts("[sdio] no WiFi SDIO GPIO configuration\n");
#endif
}

bool sdio_power_on(void)
{
#if PIOS_HAS_WIFI_SDIO2
    /* Assert WL_REG_ON (GPIO 28) to power up CYW43455.
     * Circle: GPIO28 output HIGH + 150ms delay.
     * Uses BCM2712 SoC GPIO registers (not brcmstb-gpio base):
     *   IODIR0 at 0x107D508508 — bit clear = output
     *   DATA0  at 0x107D508504 — bit set = high */
    u32 bit = 1U << SDIO_WL_REG_ON_GPIO;  /* bit 28 */

    /* Set as output (clear IODIR bit) */
    u32 iodir = mmio_read(BCM2712_GPIO1_IODIR0);
    mmio_write(BCM2712_GPIO1_IODIR0, iodir & ~bit);

    /* Drive low (reset) */
    u32 data = mmio_read(BCM2712_GPIO1_DATA0);
    mmio_write(BCM2712_GPIO1_DATA0, data & ~bit);
    timer_delay_ms(20U);

    /* Drive high (power on) */
    data = mmio_read(BCM2712_GPIO1_DATA0);
    mmio_write(BCM2712_GPIO1_DATA0, data | bit);
    timer_delay_ms(150U);
    
    /* Verify GPIO state */
    u32 data_readback = mmio_read(BCM2712_GPIO1_DATA0);
    u32 iodir_readback = mmio_read(BCM2712_GPIO1_IODIR0);
    sdio_diag.wl_data = data_readback;
    sdio_diag.wl_iodir = iodir_readback;
    uart_puts("[sdio] WL_REG_ON GPIO28: DATA=");
    uart_hex(data_readback);
    uart_puts(" IODIR=");
    uart_hex(iodir_readback);
    uart_puts(" bit28=");
    uart_hex((data_readback >> 28) & 1);
    uart_puts("\n");
    return true;
#elif PIOS_HAS_WIFI_SDIO1 && PIOS_WIFI_WL_REG_ON_FIRMWARE
    if (!mbox_set_gpio_output(PIOS_WIFI_WL_REG_ON_GPIO, false)) {
        uart_puts("[sdio] firmware WL_ON low failed\n");
        return false;
    }
    timer_delay_ms(20U);
    if (!mbox_set_gpio_output(PIOS_WIFI_WL_REG_ON_GPIO, true)) {
        uart_puts("[sdio] firmware WL_ON high failed\n");
        return false;
    }
    timer_delay_ms(150U);
    sdio_diag.wl_data = 1U;
    sdio_diag.wl_iodir = 1U;
    uart_puts("[sdio] firmware WL_ON GPIO129 high\n");
    return true;
#elif PIOS_HAS_WIFI_SDIO1
    /* Zero 2 W WL_REG_ON is direct BCM2710 GPIO41, active high. */
    const u32 gpio = PIOS_WIFI_WL_REG_ON_GPIO;
    const u32 bit = 1U << (gpio - 32U);
    const u64 base = PIOS_PERIPH_BASE + 0x200000UL;
    u32 fsel = mmio_read(base + 0x10U); /* GPFSEL4, GPIO40-49 */
    fsel &= ~(7U << ((gpio - 40U) * 3U));
    fsel |= 1U << ((gpio - 40U) * 3U); /* output */
    mmio_write(base + 0x10U, fsel);
    mmio_write(base + 0x2CU, bit); /* GPCLR1: hold radio in reset */
    timer_delay_ms(20U);
    mmio_write(base + 0x20U, bit); /* GPSET1: release radio reset */
    timer_delay_ms(150U);
    sdio_diag.wl_data = mmio_read(base + 0x38U); /* GPLEV1 */
    sdio_diag.wl_iodir = mmio_read(base + 0x10U);
    uart_puts("[sdio] WL_REG_ON GPIO41: level=");
    uart_hex((sdio_diag.wl_data >> 9U) & 1U);
    uart_puts("\n");
    return true;
#else
    return false;
#endif
}

void sdio_power_off(void)
{
#if PIOS_HAS_WIFI_SDIO2
    u32 bit = 1U << SDIO_WL_REG_ON_GPIO;
    u32 data = mmio_read(BCM2712_GPIO1_DATA0);
    mmio_write(BCM2712_GPIO1_DATA0, data & ~bit);
    timer_delay_ms(20U);
#elif PIOS_HAS_WIFI_SDIO1 && PIOS_WIFI_WL_REG_ON_FIRMWARE
    (void)mbox_set_gpio_output(PIOS_WIFI_WL_REG_ON_GPIO, false);
    timer_delay_ms(20U);
#elif PIOS_HAS_WIFI_SDIO1
    const u32 bit = 1U << (PIOS_WIFI_WL_REG_ON_GPIO - 32U);
    mmio_write(PIOS_PERIPH_BASE + 0x200000UL + 0x2CU, bit);
    timer_delay_ms(20U);
#endif
}

/* ── SDIO card enumeration ── */

bool sdio_init(void)
{
    sdio_initialized = false;
    sdio_rca = 0;
    memset(&sdio_diag, 0, sizeof(sdio_diag));
    sdio_diag.attempted = 1U;
    sdio_diag.last_stage = 1U;

#if !PIOS_HAS_WIFI_SDIO
    /* No supported native WiFi SDIO controller -- fail closed rather than
     * touching MMIO at address 0. */
    uart_puts("[sdio] no WiFi SDIO controller on this platform\n");
    return false;
#else
    uart_puts(PIOS_HAS_WIFI_SDIO1 ? "[sdio] init BCM2837 SDIO1\n" :
                                    "[sdio] init BCM2712 SDIO2\n");

    /* Probe: read SDHCI capability register to verify controller is alive */
    sdio_diag.cap0 = sr(REG_CAP0);
    sdio_diag.cap1 = sr(REG_CAP1);
    uart_puts("[sdio] base=");
    uart_hex(WIFI_SDIO_HOST_BASE);
    uart_puts(" CAP0=");
    uart_hex(sdio_diag.cap0);
    uart_puts(" CAP1=");
    uart_hex(sdio_diag.cap1);
    uart_puts("\n");

    /* Configure GPIOs for SDIO */
    sdio_gpio_init();

    /* Power-cycle the WiFi chip */
    if (!sdio_power_on()) {
        uart_puts("[sdio] WiFi power-up failed\n");
        return false;
    }

    /* Probe PRESENT_STATE for card */
    u32 pstate = sr(REG_STATUS);
    sdio_diag.present_before = pstate;
    uart_puts("[sdio] PRESENT=");
    uart_hex(pstate);
    uart_puts(" card=");
    uart_hex((pstate >> 16) & 1);
    uart_puts("\n");

    /* Program BCM2712 CFG block (MUST happen before SDHCI init)
     * Linux sdhci-brcmstb.c: sdhci_brcmstb_cfginit_2712() */
#if PIOS_HAS_WIFI_SDIO2
    u64 cfg = WIFI_SDIO_HOST_BASE + BCM2712_SDIO2_CFG_OFFSET;

    /* Force card present for non-removable WiFi chip */
    u32 ctrl = mmio_read(cfg + SDIO_CFG_CTRL);
    uart_puts("[sdio] CFG_CTRL before=");
    uart_hex(ctrl);
    ctrl |= SDIO_CFG_CTRL_SDCD_N_TEST_EN;   /* enable card-detect override */
    ctrl &= ~SDIO_CFG_CTRL_SDCD_N_TEST_LEV;  /* level=0 = card present */
    mmio_write(cfg + SDIO_CFG_CTRL, ctrl);
    sdio_diag.cfg_ctrl = mmio_read(cfg + SDIO_CFG_CTRL);
    uart_puts(" after=");
    uart_hex(sdio_diag.cfg_ctrl);
    uart_puts("\n");

    /* MAX_50MHZ strap — leave as-is for basic 25MHz bring-up.
     * Only override if implementing UHS/tuning modes (>50MHz). */

    /* Set SD_PIN_SEL to SD mode (not eMMC) — BCM2712-specific */
    u32 pinsel = mmio_read(cfg + SDIO_CFG_SD_PIN_SEL);
    pinsel &= ~0x3U;
    pinsel |= 0x2;  /* SD mode */
    mmio_write(cfg + SDIO_CFG_SD_PIN_SEL, pinsel);
    sdio_diag.pin_sel = mmio_read(cfg + SDIO_CFG_SD_PIN_SEL);

    /* Re-check PRESENT_STATE after CFG */
    pstate = sr(REG_STATUS);
    sdio_diag.present_after_cfg = pstate;
    uart_puts("[sdio] post-CFG PRESENT=");
    uart_hex(pstate);
    uart_puts(" card=");
    uart_hex((pstate >> 16) & 1);
    uart_puts("\n");
#endif

    /* Reset the host controller using proper 8-bit software reset register */
    uart_puts("[sdio] HC reset...\n");
    sdio_diag.last_stage = 2U;
    sw8(SDHCI_SOFTWARE_RESET, SDHCI_RESET_ALL);
    u32 timeout = 100000;
    while ((sr8(SDHCI_SOFTWARE_RESET) & SDHCI_RESET_ALL) && timeout--)
        delay_cycles(100);
    if (!timeout) {
        uart_puts("[sdio] HC rst timeout\n");
        return false;
    }
    uart_puts("[sdio] HC reset ok\n");

    /* Power on: 3.3V via 8-bit power control register */
    sw8(SDHCI_POWER_CONTROL, SDHCI_POWER_330 | SDHCI_POWER_ON);
    delay_cycles(20000);

    /* Clear Host Control 2 via 16-bit register (not 32-bit at 0x3C!) */
    sw16(SDHCI_HOST_CONTROL2, 0);

    /* Set up clock for 400 kHz identification mode.
     * Circle: single write with divider + timeout + internal clock enable */
    uart_puts("[sdio] setting 400kHz clock...\n");
    sdio_diag.last_stage = 3U;
    sdio_set_clock(400);
    delay_cycles(500000);

    /* Set up interrupts per SDHCI spec */
    sw(REG_IRPT_EN, 0);                   /* disable signal enable */
    sw(REG_INTERRUPT, 0xFFFFFFFF);         /* clear ALL pending (was INT_ALL which missed bit 15) */
    sw(REG_IRPT_MASK, 0xFFFFFFFF);        /* enable all status bits */

    uart_puts("[sdio] CTRL1=");
    uart_hex(sr(REG_CONTROL1));
    uart_puts(" STATUS=");
    uart_hex(sr(REG_STATUS));
    uart_puts(" card=");
    uart_hex((sr(REG_STATUS) >> 16) & 1);
    uart_puts("\n");

    timer_delay_ms(20U);

    /* CMD0: GO_IDLE_STATE */
    sdio_send_cmd(SDIO_CMD0, 0, NULL);
    /* Reset CMD line and wait for auto-clear */
    sw8(SDHCI_SOFTWARE_RESET, SDHCI_RESET_CMD);
    timeout = 100000;
    while ((sr8(SDHCI_SOFTWARE_RESET) & SDHCI_RESET_CMD) && timeout--)
        delay_cycles(10);
    sw(REG_INTERRUPT, 0xFFFFFFFF);
    delay_cycles(100000);

    /* Re-check PRESENT_STATE after CMD0 + clock */
    pstate = sr(REG_STATUS);
    uart_puts("[sdio] post-CMD0 PRESENT=");
    uart_hex(pstate);
    uart_puts(" card_inserted=");
    uart_hex((pstate >> 16) & 1);
    uart_puts("\n");

    /* CMD5: IO_SEND_OP_COND — probe for SDIO card.
     * First CMD5 may return CRC/index errors which are normal for SDIO. */
    u32 resp[4];
    uart_puts("[sdio] CMD5 probe...\n");
    sdio_diag.last_stage = 4U;
    
    bool cmd5_ok = false;
    for (u32 retry = 0; retry < 10; retry++) {
        sdio_diag.cmd5_attempts = retry + 1U;
        sw(REG_INTERRUPT, INT_ALL);  /* clear all pending */
        
        /* Send CMD5 directly — tolerate errors on first attempts */
        if (!sdio_wait_cmd()) {
            uart_puts("[sdio] CMD5 wait_cmd fail\n");
            if (!sdio_reset_cmd_line())
                return false;
            timer_delay_ms(10U);
            continue;
        }
        
        sw(REG_INTERRUPT, INT_ALL);
        sw(REG_ARG1, 0);
        sw(REG_CMDTM, SDIO_CMD5);
        
        u32 tout = 1000000;
        u32 intr;
        do {
            intr = sr(REG_INTERRUPT);
            delay_cycles(10);
        } while (!(intr & (INT_CMD_DONE | INT_ERROR)) && tout--);
        
        uart_puts("[sdio] CMD5 intr=");
        uart_hex(intr);
        sdio_diag.cmd5_interrupt = intr;
        
        if (intr & INT_CMD_DONE) {
            /* Read response even if error bits are set */
            resp[0] = sr(REG_RESP0);
            sdio_diag.cmd5_response = resp[0];
            uart_puts(" R=");
            uart_hex(resp[0]);
            uart_puts("\n");
            sw(REG_INTERRUPT, INT_ALL);
            
            if (resp[0] != 0) {
                cmd5_ok = true;
                break;
            }
        } else {
            uart_puts(" (no CMD_DONE)\n");
        }
        
        sw(REG_INTERRUPT, 0xFFFFFFFF);
        /* Reset CMD line on error — proper 8-bit write + poll */
        if (!sdio_reset_cmd_line())
            return false;
        timer_delay_ms(100U);
    }
    
    if (!cmd5_ok) {
        uart_puts("[sdio] CMD5 no response\n");
        return false;
    }

    u32 ocr = resp[0];
    uart_puts("[sdio] OCR=");
    uart_hex(ocr);
    uart_puts("\n");

    u32 num_funcs = (ocr >> 28) & 0x7;
    uart_puts("[sdio] funcs=");
    uart_hex(num_funcs);
    uart_puts("\n");

    /* Send CMD5 with 3.3V voltage window until card ready (bit 31 = ready) */
    #define SDIO_OCR_V3_3  0x00300000  /* 3.2-3.4V window */
    u32 cmd5_arg = SDIO_OCR_V3_3;
    timeout = 100;
    do {
        if (!sdio_send_cmd(SDIO_CMD5, cmd5_arg, resp)) {
            uart_puts("[sdio] CMD5 retry fail\n");
            return false;
        }
        timer_delay_ms(100U);
    } while (!(resp[0] & (1U << 31)) && timeout--);

    if (!timeout) {
        uart_puts("[sdio] CMD5 rdy timeout\n");
        return false;
    }

    /* CMD3: SEND_RELATIVE_ADDR */
    if (!sdio_send_cmd(SDIO_CMD3, 0, resp)) {
        uart_puts("[sdio] CMD3 fail\n");
        return false;
    }
    sdio_rca = resp[0] >> 16;
    uart_puts("[sdio] RCA=");
    uart_hex(sdio_rca);
    uart_puts("\n");

    /* CMD7: SELECT_CARD */
    if (!sdio_send_cmd(SDIO_CMD7, sdio_rca << 16, resp)) {
        uart_puts("[sdio] CMD7 fail\n");
        return false;
    }

    /* Switch to higher clock (25 MHz) */
    sdio_set_clock(25000);

    /* Verify CCCR access */
    u8 cccr_rev;
    if (!sdio_cmd52_read(SDIO_FUNC_CIA, CCCR_SDIO_REV, &cccr_rev)) {
        uart_puts("[sdio] CCCR fail\n");
        return false;
    }
    uart_puts("[sdio] CCCR=");
    uart_hex(cccr_rev);
    uart_puts("\n");

    u8 high_speed = 0U;
    if (sdio_cmd52_read(SDIO_FUNC_CIA, CCCR_HIGH_SPEED, &high_speed) &&
        (high_speed & HIGH_SPEED_SHS) != 0U &&
        sdio_cmd52_write(SDIO_FUNC_CIA, CCCR_HIGH_SPEED,
                         high_speed | HIGH_SPEED_EHS)) {
        sdio_set_clock(50000U);
        uart_puts("[sdio] high speed 50MHz\n");
    }

    sdio_initialized = true;
    sdio_diag.initialized = 1U;
    sdio_diag.last_stage = 8U;
    sdio_diag.control1 = sr(REG_CONTROL1);
    sdio_diag.status = sr(REG_STATUS);
    uart_puts("[sdio] init OK\n");
    return true;
#endif /* PIOS_HAS_WIFI_SDIO */
}

void sdio_reset_data_line(void)
{
    sw8(SDHCI_SOFTWARE_RESET, SDHCI_RESET_DATA);
    u32 t = 100000;
    while ((sr8(SDHCI_SOFTWARE_RESET) & SDHCI_RESET_DATA) && t--)
        delay_cycles(10);
    sw(REG_INTERRUPT, 0xFFFFFFFF);
}

bool sdio_card_present(void)
{
    return sdio_initialized;
}

void sdio_diag_snapshot(struct sdio_diag *out)
{
    if (!out)
        return;
    *out = sdio_diag;
}

/* ── CMD52: IO_RW_DIRECT ── */

bool sdio_cmd52_read(u32 func, u32 addr, u8 *val)
{
    u32 arg = CMD52_FUNC(func) | CMD52_ADDR(addr);
    u32 resp[4];

    if (!sdio_send_cmd(SDIO_CMD52, arg, resp))
        return false;

    /* Response R5: bits [7:0] = read data, bits [15:8] = flags */
    u32 flags = (resp[0] >> 8) & 0xFF;
    if (flags & 0xCB) /* error bits */
        return false;

    if (val)
        *val = (u8)(resp[0] & 0xFF);
    return true;
}

bool sdio_cmd52_write(u32 func, u32 addr, u8 val)
{
    u32 arg = CMD52_WRITE | CMD52_FUNC(func) | CMD52_ADDR(addr) | CMD52_DATA(val);
    u32 resp[4];

    if (!sdio_send_cmd(SDIO_CMD52, arg, resp)) {
        uart_puts("[c52w] CMD fail f=");
        uart_hex(func);
        uart_puts(" a=");
        uart_hex(addr);
        uart_puts("\n");
        return false;
    }

    u32 flags = (resp[0] >> 8) & 0xFF;
    if (flags & 0xCB) {
        uart_puts("[c52w] R5 err f=");
        uart_hex(func);
        uart_puts(" a=");
        uart_hex(addr);
        uart_puts(" v=");
        uart_hex(val);
        uart_puts(" R5=");
        uart_hex(resp[0]);
        uart_puts("\n");
        return false;
    }

    return true;
}

/* ── CMD53: IO_RW_EXTENDED ── */

bool sdio_cmd53_read(u32 func, u32 addr, u8 *buf, u32 len, bool incr)
{
    if (len == 0 || len > 512)
        return false;

    if (!sdio_wait_data()) {
        uart_puts("[c53r] wait_data timeout\n");
        return false;
    }

    sw(REG_BLKSIZECNT, (1 << 16) | len);
    sw(REG_INTERRUPT, INT_ALL);

    u32 arg = CMD53_FUNC(func) | CMD53_ADDR(addr) | CMD53_COUNT(len);
    if (incr)
        arg |= CMD53_INCR_ADDR;

    u32 resp[4];
    if (!sdio_send_cmd(SDIO_CMD53_R, arg, resp)) {
        uart_puts("[c53r] CMD fail f=");
        uart_hex(func);
        uart_puts(" a=");
        uart_hex(addr);
        uart_puts(" len=");
        uart_hex(len);
        uart_puts("\n");
        return false;
    }

    /* Wait for read ready */
    u32 timeout = 1000000;
    u32 intr;
    while (timeout != 0U) {
        intr = sr(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            uart_puts("[c53r] INT_ERROR=");
            uart_hex(intr);
            uart_puts("\n");
            sw(REG_INTERRUPT, INT_ERROR);
            return false;
        }
        if (intr & INT_READ_RDY)
            break;
        timeout--;
        delay_cycles(10);
    }

    if (timeout == 0U) {
        uart_puts("[c53r] READ_RDY timeout intr=");
        uart_hex(intr);
        uart_puts("\n");
        sdio_reset_data_line();
        return false;
    }

    /* Read data — 32-bit words, use byte writes for alignment safety */
    u32 words = (len + 3) / 4;
    for (u32 i = 0; i < words; i++) {
        u32 val = sr(REG_DATA);
        u32 off = i * 4;
        buf[off]     = (u8)(val & 0xFF);
        if (off + 1 < len) buf[off + 1] = (u8)((val >> 8) & 0xFF);
        if (off + 2 < len) buf[off + 2] = (u8)((val >> 16) & 0xFF);
        if (off + 3 < len) buf[off + 3] = (u8)((val >> 24) & 0xFF);
    }

    /* Wait for transfer complete */
    timeout = 1000000;
    while (timeout != 0U) {
        intr = sr(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            uart_puts("[c53r] DATA error=");
            uart_hex(intr);
            uart_puts("\n");
            sw(REG_INTERRUPT, INT_ERROR | INT_DATA_DONE);
            sdio_reset_data_line();
            return false;
        }
        if (intr & INT_DATA_DONE)
            break;
        timeout--;
        delay_cycles(10);
    }
    if (timeout == 0U) {
        uart_puts("[c53r] DATA_DONE timeout intr=");
        uart_hex(intr);
        uart_puts("\n");
        sdio_reset_data_line();
        return false;
    }

    sw(REG_INTERRUPT, INT_DATA_DONE);
    return true;
}

bool sdio_cmd53_write(u32 func, u32 addr, const u8 *buf, u32 len, bool incr)
{
    if (len == 0 || len > 512)
        return false;

    if (!sdio_wait_data()) {
        uart_puts("[w53] wait data fail\n");
        return false;
    }

    sw(REG_BLKSIZECNT, (1 << 16) | len);
    sw(REG_INTERRUPT, INT_ALL);

    u32 arg = CMD53_WRITE | CMD53_FUNC(func) | CMD53_ADDR(addr) | CMD53_COUNT(len);
    if (incr)
        arg |= CMD53_INCR_ADDR;

    u32 resp[4];
    if (!sdio_send_cmd(SDIO_CMD53_W, arg, resp)) {
        uart_puts("[w53] cmd fail\n");
        return false;
    }

    /* Wait for write ready */
    u32 timeout = 1000000;
    u32 intr;
    while (timeout != 0U) {
        intr = sr(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            uart_puts("[w53] err i=");
            uart_hex(intr);
            uart_puts("\n");
            sw(REG_INTERRUPT, INT_ERROR);
            return false;
        }
        if (intr & INT_WRITE_RDY)
            break;
        timeout--;
        delay_cycles(10);
    }

    if (timeout == 0U) {
        uart_puts("[w53] wrdy timeout\n");
        return false;
    }

    /* Write data — 32-bit words, use byte reads for alignment safety */
    u32 words = (len + 3) / 4;
    for (u32 i = 0; i < words; i++) {
        u32 off = i * 4;
        u32 val = (u32)buf[off];
        if (off + 1 < len) val |= (u32)buf[off + 1] << 8;
        if (off + 2 < len) val |= (u32)buf[off + 2] << 16;
        if (off + 3 < len) val |= (u32)buf[off + 3] << 24;
        sw(REG_DATA, val);
    }

    /* Wait for transfer complete */
    timeout = 1000000;
    while (timeout != 0U) {
        intr = sr(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            uart_puts("[w53] data err i=");
            uart_hex(intr);
            uart_puts("\n");
            sw(REG_INTERRUPT, INT_ERROR | INT_DATA_DONE);
            sdio_reset_data_line();
            return false;
        }
        if (intr & INT_DATA_DONE)
            break;
        timeout--;
        delay_cycles(10);
    }

    if (timeout == 0U) {
        uart_puts("[w53] data done timeout i=");
        uart_hex(intr);
        uart_puts("\n");
        sdio_reset_data_line();
        return false;
    }

    sw(REG_INTERRUPT, INT_DATA_DONE);
    return true;
}

/* ── Block-mode CMD53 ── */

bool sdio_cmd53_read_blocks(u32 func, u32 addr, u8 *buf,
                            u32 blksz, u32 nblks, bool incr)
{
    if (nblks == 0U || nblks > 511U || blksz == 0U || blksz > 512U ||
        (blksz & 3U) != 0U)
        return false;

    if (!sdio_wait_data())
        return false;

    sw(REG_BLKSIZECNT, (nblks << 16) | blksz);
    sw(REG_INTERRUPT, INT_ALL);

    u32 cmd_flags = SDIO_CMD53_R;
    if (nblks > 1)
        cmd_flags |= TM_MULTI | TM_BLKCNT;

    u32 arg = CMD53_FUNC(func) | CMD53_BLOCK_MODE |
              CMD53_ADDR(addr) | CMD53_COUNT(nblks);
    if (incr)
        arg |= CMD53_INCR_ADDR;

    u32 resp[4];
    if (!sdio_send_cmd(cmd_flags, arg, resp) ||
        (resp[0] & R5_ERROR_MASK) != 0U)
        return false;

    for (u32 block = 0; block < nblks; block++) {
        u32 timeout = 1000000;
        u32 intr = 0U;
        while (timeout != 0U) {
            intr = sr(REG_INTERRUPT);
            if (intr & INT_ERROR) {
                sw(REG_INTERRUPT, 0xFFFFFFFFU);
                sdio_reset_data_line();
                return false;
            }
            if (intr & INT_READ_RDY)
                break;
            timeout--;
            delay_cycles(10);
        }
        if (timeout == 0U) {
            sdio_reset_data_line();
            return false;
        }
        sw(REG_INTERRUPT, INT_READ_RDY);
        for (u32 off = 0; off < blksz; off += 4U) {
            u32 val = sr(REG_DATA);
            u32 pos = block * blksz + off;
            buf[pos] = (u8)(val & 0xFF);
            buf[pos + 1U] = (u8)((val >> 8) & 0xFF);
            buf[pos + 2U] = (u8)((val >> 16) & 0xFF);
            buf[pos + 3U] = (u8)((val >> 24) & 0xFF);
        }
    }

    u32 timeout = 1000000;
    u32 intr = 0U;
    while (timeout != 0U) {
        intr = sr(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            sw(REG_INTERRUPT, 0xFFFFFFFFU);
            sdio_reset_data_line();
            return false;
        }
        if (intr & INT_DATA_DONE)
            break;
        timeout--;
        delay_cycles(10);
    }
    if (timeout == 0U) {
        sdio_reset_data_line();
        return false;
    }
    sw(REG_INTERRUPT, 0xFFFFFFFFU);
    return true;
}

bool sdio_cmd53_write_blocks(u32 func, u32 addr, const u8 *buf,
                             u32 blksz, u32 nblks, bool incr)
{
    if (nblks == 0U || nblks > 511U || blksz == 0U || blksz > 512U ||
        (blksz & 3U) != 0U)
        return false;

    if (!sdio_wait_data())
        return false;

    sw(REG_BLKSIZECNT, (nblks << 16) | blksz);
    sw(REG_INTERRUPT, INT_ALL);

    u32 cmd_flags = SDIO_CMD53_W;
    if (nblks > 1)
        cmd_flags |= TM_MULTI | TM_BLKCNT;

    u32 arg = CMD53_WRITE | CMD53_FUNC(func) | CMD53_BLOCK_MODE |
              CMD53_ADDR(addr) | CMD53_COUNT(nblks);
    if (incr)
        arg |= CMD53_INCR_ADDR;

    u32 resp[4];
    if (!sdio_send_cmd(cmd_flags, arg, resp) ||
        (resp[0] & R5_ERROR_MASK) != 0U)
        return false;

    for (u32 block = 0; block < nblks; block++) {
        u32 timeout = 1000000;
        u32 intr = 0U;
        while (timeout != 0U) {
            intr = sr(REG_INTERRUPT);
            if (intr & INT_ERROR) {
                sw(REG_INTERRUPT, 0xFFFFFFFFU);
                sdio_reset_data_line();
                return false;
            }
            if (intr & INT_WRITE_RDY)
                break;
            timeout--;
            delay_cycles(10);
        }
        if (timeout == 0U) {
            sdio_reset_data_line();
            return false;
        }
        sw(REG_INTERRUPT, INT_WRITE_RDY);
        for (u32 off = 0; off < blksz; off += 4U) {
            u32 pos = block * blksz + off;
            u32 val = (u32)buf[pos] |
                      ((u32)buf[pos + 1U] << 8) |
                      ((u32)buf[pos + 2U] << 16) |
                      ((u32)buf[pos + 3U] << 24);
            sw(REG_DATA, val);
        }
    }

    u32 timeout = 1000000;
    u32 intr = 0U;
    while (timeout != 0U) {
        intr = sr(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            sw(REG_INTERRUPT, 0xFFFFFFFFU);
            sdio_reset_data_line();
            return false;
        }
        if (intr & INT_DATA_DONE)
            break;
        timeout--;
        delay_cycles(10);
    }
    if (timeout == 0U) {
        sdio_reset_data_line();
        return false;
    }
    sw(REG_INTERRUPT, 0xFFFFFFFFU);
    return true;
}

/* ── Utility functions ── */

bool sdio_set_block_size(u32 func, u16 blksz)
{
    if (func == 0) {
        /* CCCR block size */
        if (!sdio_cmd52_write(0, CCCR_BLK_SIZE_LO, (u8)(blksz & 0xFF)))
            return false;
        return sdio_cmd52_write(0, CCCR_BLK_SIZE_HI, (u8)((blksz >> 8) & 0xFF));
    }

    /* Function-specific block size via FBR */
    if (!sdio_cmd52_write(0, FBR_BLK_SIZE_LO(func), (u8)(blksz & 0xFF)))
        return false;
    return sdio_cmd52_write(0, FBR_BLK_SIZE_HI(func), (u8)((blksz >> 8) & 0xFF));
}

bool sdio_enable_func(u32 func)
{
    if (func == 0 || func > 7)
        return false;

    u8 io_en;
    if (!sdio_cmd52_read(0, CCCR_IO_ENABLE, &io_en))
        return false;

    io_en |= (1 << func);
    if (!sdio_cmd52_write(0, CCCR_IO_ENABLE, io_en))
        return false;

    /* Poll IO_READY until function is ready */
    u32 timeout = 100000;
    u8 io_rdy;
    do {
        if (!sdio_cmd52_read(0, CCCR_IO_READY, &io_rdy))
            return false;
        if (io_rdy & (1 << func))
            return true;
        delay_cycles(1000);
    } while (timeout--);

    uart_puts("[sdio] func en timeout f=");
    uart_hex(func);
    uart_puts("\n");
    return false;
}

bool sdio_disable_func(u32 func)
{
    if (func == 0 || func > 7)
        return false;

    u8 io_en;
    if (!sdio_cmd52_read(0, CCCR_IO_ENABLE, &io_en))
        return false;

    io_en &= ~(1 << func);
    return sdio_cmd52_write(0, CCCR_IO_ENABLE, io_en);
}

bool sdio_enable_func_irq(u32 func)
{
    u8 int_en;
    if (!sdio_cmd52_read(0, CCCR_INT_ENABLE, &int_en))
        return false;

    int_en |= (1 << func) | 1;  /* master interrupt enable + function */
    return sdio_cmd52_write(0, CCCR_INT_ENABLE, int_en);
}

/* SDHCI "Card Interrupt" status. REG_IRPT_MASK is left wide open by
 * sdio_init(), so the bit latches in the status register even though
 * REG_IRPT_EN keeps the controller from signalling the GIC. That makes this a
 * cheap poll-side accelerator: one MMIO read tells us whether the CYW43455 is
 * asserting DAT1, instead of issuing CMD52s to find out there is nothing to do. */
bool sdio_card_irq_pending(void)
{
#if !PIOS_HAS_WIFI_SDIO
    return false;
#else
    return (sr16(REG_INTERRUPT) & INT_CARD) != 0U;
#endif
}

bool sdio_card_irq_edge(void)
{
    bool level = sdio_card_irq_pending();
    bool edge = level && !sdio_card_irq_level;
    sdio_card_irq_level = level;
    return edge;
}

/* Acknowledge the latched card-interrupt status. SDHCI treats this bit as
 * level-sensitive against DAT1, so if the card is still asserting it will come
 * straight back -- which is the correct signal to keep draining. Without the
 * ack a single stale assertion pins the RX backoff at zero forever. */
void sdio_card_irq_ack(void)
{
#if PIOS_HAS_WIFI_SDIO
    sw16(REG_INTERRUPT, (u16)INT_CARD);
#endif
}

void sdio_card_irq_mask(void)
{
#if PIOS_HAS_WIFI_SDIO
    u16 signal = sr16(REG_IRPT_EN);
    sw16(REG_IRPT_EN, (u16)(signal & ~(u16)INT_CARD));
#endif
}

void sdio_card_irq_unmask(void)
{
#if PIOS_HAS_WIFI_SDIO
    u16 signal = sr16(REG_IRPT_EN);
    sw16(REG_IRPT_EN, (u16)(signal | INT_CARD));
#endif
}

static void sdio_gic_irq_handler(void)
{
    /* Top half: mask the level line, ack, post. Drain is FIFO/AIRQ only. */
    sdio_card_irq_mask();
    if (sdio_card_irq_pending()) {
        sdio_card_irq_latched = true;
        sdio_card_irq_ack();
    }
    (void)airq_post_from(CORE_NET, AIRQ_SRC_WIFI, NET_DISPATCH_CAUSE_IRQ);
}

void sdio_card_irq_arm(void)
{
#if PIOS_HAS_WIFI_SDIO
    sdio_card_irq_latched = false;
    sdio_card_irq_level = false;
    sdio_card_irq_unmask();
#if PIOS_WIFI_SDIO_IRQ != 0
    irq_register(PIOS_WIFI_SDIO_IRQ, sdio_gic_irq_handler);
    gic_set_group1(PIOS_WIFI_SDIO_IRQ);
    gic_set_priority(PIOS_WIFI_SDIO_IRQ, 0x60U);
    gic_set_target(PIOS_WIFI_SDIO_IRQ, 1U);
    gic_clear_pending(PIOS_WIFI_SDIO_IRQ);
    gic_enable_irq(PIOS_WIFI_SDIO_IRQ);
    uart_puts("[sdio] card IRQ armed intid=");
    uart_hex(PIOS_WIFI_SDIO_IRQ);
    uart_puts("\n");
#else
    uart_puts("[sdio] card IRQ: no host INTID on this platform\n");
#endif
#endif
}

bool sdio_card_irq_take(void)
{
    bool pending = sdio_card_irq_latched;
    sdio_card_irq_latched = false;
    return pending;
}

void sdio_irq_snapshot(u32 *status, u32 *signal_enable, u32 *mask,
                       u32 *gic_enable, u32 *gic_pending,
                       u32 *gic_target)
{
#if PIOS_HAS_WIFI_SDIO
    if (status)
        *status = sr16(REG_INTERRUPT);
    if (signal_enable)
        *signal_enable = sr16(REG_IRPT_EN);
    if (mask)
        *mask = sr16(REG_IRPT_MASK);
#else
    if (status) *status = 0U;
    if (signal_enable) *signal_enable = 0U;
    if (mask) *mask = 0U;
#endif
#if PIOS_HAS_WIFI_SDIO2
    u64 gicd = gic_runtime_gicd_base();
    if (gic_enable)
        *gic_enable = mmio_read(gicd + 0x100U + (274U / 32U) * 4U);
    if (gic_pending)
        *gic_pending = mmio_read(gicd + 0x200U + (274U / 32U) * 4U);
    if (gic_target)
        *gic_target = mmio_read(gicd + 0x800U + (274U / 4U) * 4U);
#else
    if (gic_enable) *gic_enable = 0U;
    if (gic_pending) *gic_pending = 0U;
    if (gic_target) *gic_target = 0U;
#endif
}

bool sdio_set_bus_width_4bit(void)
{
    u8 bus_if;
    if (!sdio_cmd52_read(0, CCCR_BUS_IFACE, &bus_if))
        return false;

    bus_if = (bus_if & ~0x03) | BUS_WIDTH_4BIT;
    if (!sdio_cmd52_write(0, CCCR_BUS_IFACE, bus_if))
        return false;

    /* Update host controller for 4-bit mode */
    u32 c0 = sr(REG_CONTROL0);
    c0 |= (1 << 1);  /* 4-bit bus width */
    sw(REG_CONTROL0, c0);

    return true;
}
