/*
 * sdio.c - RP1 SDIO host controller driver
 *
 * Minimal polling SDIO host for the RP1's Synopsys controller at
 * RP1_BAR_BASE + 0x104000, talking to the CYW43455 WiFi/BT combo.
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
#include "rp1.h"
#include "rp1_gpio.h"
#include "rp1_clk.h"
#include "uart.h"
#include "timer.h"
#include "fb.h"

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
#define INT_ERROR           0xFFFF0000U
#define INT_ALL             0xFFFF00FFU

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

/* Card state */
static u32 sdio_rca;
static bool sdio_initialized;

/* ── Register helpers — target BCM2712 SDIO2 ── */
static inline u32 sr(u32 off)           { return mmio_read(BCM2712_SDIO2_BASE + off); }
static inline void sw(u32 off, u32 val) { mmio_write(BCM2712_SDIO2_BASE + off, val); }

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
    for (u32 i = 0; i < 1000000; i++) {
        if (!(sr(REG_STATUS) & SR_DAT_INHIBIT))
            return true;
        delay_cycles(10);
    }
    return false;
}

static bool sdio_send_cmd(u32 cmd, u32 arg, u32 *resp)
{
    if (!sdio_wait_cmd())
        return false;

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
    return true;
}

static void sdio_set_clock(u32 freq_khz)
{
    /* Disable clock */
    u32 c1 = sr(REG_CONTROL1);
    c1 &= ~C1_CLK_EN;
    sw(REG_CONTROL1, c1);
    delay_cycles(1000);

    /* Base clock ~50MHz from RP1 PLL */
    u32 base_khz = 50000;
    u32 div = base_khz / freq_khz;
    if (base_khz / div > freq_khz) div++;
    div = div >> 1;
    if (div > 0x3FF) div = 0x3FF;

    u32 divider = ((div & 0xFF) << 8) | ((div >> 8) << 6);

    c1 = (c1 & 0xFFFF001FU) | divider | C1_CLK_INTLEN | C1_TOUNIT(0xE);
    sw(REG_CONTROL1, c1);
    delay_cycles(1000);

    /* Wait for clock stable */
    u32 timeout = 100000;
    while (!(sr(REG_CONTROL1) & C1_CLK_STABLE) && timeout--)
        delay_cycles(10);

    /* Enable clock */
    c1 = sr(REG_CONTROL1);
    c1 |= C1_CLK_EN;
    sw(REG_CONTROL1, c1);
    delay_cycles(1000);
}

/* ── GPIO and power setup ── */

/* ── BCM2712 SoC GPIO/pinctrl helpers ── */

/* BCM2712 pinctrl: set pin function via FSEL registers at 0x107D504100.
 * Each GPIO has 4 bits in FSEL registers, 8 GPIOs per 32-bit register. */
static void bcm2712_gpio_set_fsel(u32 pin, u32 fsel)
{
    u32 reg_off = (pin / 8) * 4;
    u32 shift = (pin % 8) * 4;
    u32 val = mmio_read(BCM2712_PINCTRL_BASE + reg_off);
    val &= ~(0xFU << shift);
    val |= (fsel & 0xF) << shift;
    mmio_write(BCM2712_PINCTRL_BASE + reg_off, val);
}

static void sdio_gpio_init(void)
{
    /* WL_ON pin (GPIO 28) — set as plain GPIO for WL_REG_ON */
    bcm2712_gpio_set_fsel(SDIO_WL_REG_ON_GPIO, 0);  /* func 0 = GPIO */

    /* SDIO2 pins (GPIO 30-35) — set to SDIO2 function.
     * The firmware likely already configured these via sdio2_30_pins.
     * We re-assert to be safe. */
    u32 pins[] = { SDIO2_GPIO_CLK, SDIO2_GPIO_CMD,
                   SDIO2_GPIO_DAT0, SDIO2_GPIO_DAT1,
                   SDIO2_GPIO_DAT2, SDIO2_GPIO_DAT3 };

    for (u32 i = 0; i < 6; i++) {
        bcm2712_gpio_set_fsel(pins[i], 1);  /* func 1 = SDIO2 */
    }
    uart_puts("[sdio] GPIO 28=WL_ON, 30-35=SDIO2\n");
}

void sdio_power_on(void)
{
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
    delay_cycles(200000);

    /* Drive high (power on) */
    data = mmio_read(BCM2712_GPIO1_DATA0);
    mmio_write(BCM2712_GPIO1_DATA0, data | bit);
    delay_cycles(2000000);  /* 150ms+ startup delay */
    uart_puts("[sdio] WL_REG_ON GPIO28 HIGH\n");
}

void sdio_power_off(void)
{
    u32 bit = 1U << SDIO_WL_REG_ON_GPIO;
    u32 data = mmio_read(BCM2712_GPIO1_DATA0);
    mmio_write(BCM2712_GPIO1_DATA0, data & ~bit);
    delay_cycles(200000);
}

/* ── SDIO card enumeration ── */

bool sdio_init(void)
{
    sdio_initialized = false;
    sdio_rca = 0;

    uart_puts("[sdio] init BCM2712 SDIO2\n");

    /* Probe: read SDHCI capability register to verify controller is alive */
    uart_puts("[sdio] base=");
    uart_hex(BCM2712_SDIO2_BASE);
    uart_puts(" CAP0=");
    uart_hex(sr(REG_CAP0));
    uart_puts(" CAP1=");
    uart_hex(sr(REG_CAP1));
    uart_puts("\n");

    /* Configure GPIOs for SDIO */
    sdio_gpio_init();

    /* Power-cycle the WiFi chip */
    sdio_power_on();

    /* Reset the host controller */
    uart_puts("[sdio] HC reset...\n");
    sw(REG_CONTROL1, C1_SRST_HC);
    u32 timeout = 100000;
    while ((sr(REG_CONTROL1) & C1_SRST_HC) && timeout--)
        delay_cycles(10);
    if (!timeout) {
        uart_puts("[sdio] HC rst timeout\n");
        return false;
    }
    uart_puts("[sdio] HC reset ok\n");

    /* Power on: 3.3V */
    sw(REG_CONTROL0, 0x0F00);
    delay_cycles(10000);
    uart_puts("[sdio] power on, CTRL0=");
    uart_hex(sr(REG_CONTROL0));
    uart_puts("\n");

    /* Enable all interrupts */
    sw(REG_IRPT_MASK, INT_ALL);
    sw(REG_IRPT_EN, INT_ALL);

    /* Slow clock for identification (400 kHz) */
    uart_puts("[sdio] setting 400kHz clock...\n");
    sdio_set_clock(400);
    delay_cycles(500000);
    uart_puts("[sdio] CTRL1=");
    uart_hex(sr(REG_CONTROL1));
    uart_puts(" STATUS=");
    uart_hex(sr(REG_STATUS));
    uart_puts("\n");

    /* CMD0: GO_IDLE_STATE */
    sdio_send_cmd(SDIO_CMD0, 0, NULL);
    sw(REG_CONTROL1, sr(REG_CONTROL1) | C1_SRST_CMD);
    timeout = 100000;
    while ((sr(REG_CONTROL1) & C1_SRST_CMD) && timeout--)
        delay_cycles(10);
    sw(REG_INTERRUPT, INT_ALL);
    delay_cycles(100000);

    /* CMD5: IO_SEND_OP_COND — probe for SDIO card.
     * First CMD5 may return CRC/index errors which are normal for SDIO. */
    u32 resp[4];
    uart_puts("[sdio] CMD5 probe...\n");
    
    bool cmd5_ok = false;
    for (u32 retry = 0; retry < 10; retry++) {
        sw(REG_INTERRUPT, INT_ALL);  /* clear all pending */
        
        /* Send CMD5 directly — tolerate errors on first attempts */
        if (!sdio_wait_cmd()) {
            uart_puts("[sdio] CMD5 wait_cmd fail\n");
            sw(REG_CONTROL1, sr(REG_CONTROL1) | C1_SRST_CMD);
            delay_cycles(100000);
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
        
        if (intr & INT_CMD_DONE) {
            /* Read response even if error bits are set */
            resp[0] = sr(REG_RESP0);
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
        
        sw(REG_INTERRUPT, INT_ALL);
        /* Reset CMD line on error */
        sw(REG_CONTROL1, sr(REG_CONTROL1) | C1_SRST_CMD);
        delay_cycles(100000);
        while (sr(REG_CONTROL1) & C1_SRST_CMD) delay_cycles(10);
        delay_cycles(2000000);
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

    /* Send CMD5 with voltage window until card ready */
    u32 cmd5_arg = ocr & 0x00FFFFFFU;  /* echo back supported voltages */
    timeout = 100;
    do {
        if (!sdio_send_cmd(SDIO_CMD5, cmd5_arg, resp)) {
            uart_puts("[sdio] CMD5 retry fail\n");
            return false;
        }
        delay_cycles(100000);
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

    sdio_initialized = true;
    uart_puts("[sdio] init OK\n");
    return true;
}

bool sdio_card_present(void)
{
    return sdio_initialized;
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

    if (!sdio_send_cmd(SDIO_CMD52, arg, resp))
        return false;

    u32 flags = (resp[0] >> 8) & 0xFF;
    if (flags & 0xCB)
        return false;

    return true;
}

/* ── CMD53: IO_RW_EXTENDED ── */

bool sdio_cmd53_read(u32 func, u32 addr, u8 *buf, u32 len, bool incr)
{
    if (len == 0 || len > 512)
        return false;

    if (!sdio_wait_data())
        return false;

    sw(REG_BLKSIZECNT, (1 << 16) | len);
    sw(REG_INTERRUPT, INT_ALL);

    u32 arg = CMD53_FUNC(func) | CMD53_ADDR(addr) | CMD53_COUNT(len);
    if (incr)
        arg |= CMD53_INCR_ADDR;

    u32 resp[4];
    if (!sdio_send_cmd(SDIO_CMD53_R, arg, resp))
        return false;

    /* Wait for read ready */
    u32 timeout = 1000000;
    u32 intr;
    do {
        intr = sr(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            sw(REG_INTERRUPT, INT_ERROR);
            return false;
        }
        delay_cycles(10);
    } while (!(intr & INT_READ_RDY) && timeout--);

    if (!timeout)
        return false;

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
    do {
        intr = sr(REG_INTERRUPT);
        delay_cycles(10);
    } while (!(intr & INT_DATA_DONE) && timeout--);

    sw(REG_INTERRUPT, INT_DATA_DONE);
    return true;
}

bool sdio_cmd53_write(u32 func, u32 addr, const u8 *buf, u32 len, bool incr)
{
    if (len == 0 || len > 512)
        return false;

    if (!sdio_wait_data())
        return false;

    sw(REG_BLKSIZECNT, (1 << 16) | len);
    sw(REG_INTERRUPT, INT_ALL);

    u32 arg = CMD53_WRITE | CMD53_FUNC(func) | CMD53_ADDR(addr) | CMD53_COUNT(len);
    if (incr)
        arg |= CMD53_INCR_ADDR;

    u32 resp[4];
    if (!sdio_send_cmd(SDIO_CMD53_W, arg, resp))
        return false;

    /* Wait for write ready */
    u32 timeout = 1000000;
    u32 intr;
    do {
        intr = sr(REG_INTERRUPT);
        if (intr & INT_ERROR) {
            sw(REG_INTERRUPT, INT_ERROR);
            return false;
        }
        delay_cycles(10);
    } while (!(intr & INT_WRITE_RDY) && timeout--);

    if (!timeout)
        return false;

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
    do {
        intr = sr(REG_INTERRUPT);
        delay_cycles(10);
    } while (!(intr & INT_DATA_DONE) && timeout--);

    sw(REG_INTERRUPT, INT_DATA_DONE);
    return true;
}

/* ── Block-mode CMD53 ── */

bool sdio_cmd53_read_blocks(u32 func, u32 addr, u8 *buf,
                            u32 blksz, u32 nblks, bool incr)
{
    if (nblks == 0 || blksz == 0)
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
    if (!sdio_send_cmd(cmd_flags, arg, resp))
        return false;

    u32 total_bytes = blksz * nblks;
    u32 total_words = (total_bytes + 3) / 4;

    for (u32 w = 0; w < total_words; w++) {
        u32 timeout = 1000000;
        while (!(sr(REG_STATUS) & SR_READ_AVAILABLE) && timeout--)
            delay_cycles(10);
        if (!timeout)
            return false;
        u32 val = sr(REG_DATA);
        u32 off = w * 4;
        buf[off]     = (u8)(val & 0xFF);
        if (off + 1 < total_bytes) buf[off + 1] = (u8)((val >> 8) & 0xFF);
        if (off + 2 < total_bytes) buf[off + 2] = (u8)((val >> 16) & 0xFF);
        if (off + 3 < total_bytes) buf[off + 3] = (u8)((val >> 24) & 0xFF);
    }

    u32 timeout = 1000000;
    u32 intr;
    do {
        intr = sr(REG_INTERRUPT);
        delay_cycles(10);
    } while (!(intr & INT_DATA_DONE) && timeout--);

    sw(REG_INTERRUPT, INT_DATA_DONE);
    return true;
}

bool sdio_cmd53_write_blocks(u32 func, u32 addr, const u8 *buf,
                             u32 blksz, u32 nblks, bool incr)
{
    if (nblks == 0 || blksz == 0)
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
    if (!sdio_send_cmd(cmd_flags, arg, resp))
        return false;

    u32 total_bytes = blksz * nblks;
    u32 total_words = (total_bytes + 3) / 4;

    for (u32 w = 0; w < total_words; w++) {
        u32 timeout = 1000000;
        while (!(sr(REG_STATUS) & SR_WRITE_READY) && timeout--)
            delay_cycles(10);
        if (!timeout)
            return false;
        u32 off = w * 4;
        u32 val = (u32)buf[off];
        if (off + 1 < total_bytes) val |= (u32)buf[off + 1] << 8;
        if (off + 2 < total_bytes) val |= (u32)buf[off + 2] << 16;
        if (off + 3 < total_bytes) val |= (u32)buf[off + 3] << 24;
        sw(REG_DATA, val);
    }

    u32 timeout = 1000000;
    u32 intr;
    do {
        intr = sr(REG_INTERRUPT);
        delay_cycles(10);
    } while (!(intr & INT_DATA_DONE) && timeout--);

    sw(REG_INTERRUPT, INT_DATA_DONE);
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
