/*
 * sdio.h - BCM2712 SDIO2 host controller driver
 *
 * Drives the BCM2712 SoC's dedicated SDIO2 controller (a Synopsys-derived
 * SDHCI-compatible core, DTB compatible "brcm,bcm2712-sdhci"), which is
 * wired directly to the onboard CYW43455 WiFi/BT combo chip. This is a
 * SEPARATE controller from the RP1 southbridge's own SDIO1/MMC0/MMC1 blocks
 * (RP1 is not involved in WiFi at all on Pi 5 -- see PIOS_WIFI_SDIO2_BASE
 * in platform.h and the DTB /axi/mmc@1100000 node).
 *
 * SDIO pins use BCM2712 SoC GPIOs 30-35 (sdio2_30_pins in DTB), not RP1 GPIO.
 *
 * Polling mode only — no DMA, no interrupts.
 * 1-bit bus width initially; upgraded to 4-bit after card init.
 *
 * Reference: SD Host Controller Simplified Specification v3.0
 *            SDIO Simplified Specification v3.0
 *            Linux drivers/mmc/host/sdhci.c
 */

#pragma once
#include "types.h"
#include "platform.h"

/* WiFi SDIO host, resolved from platform.h. */
#if PIOS_HAS_WIFI_SDIO2
#define WIFI_SDIO_HOST_BASE  PIOS_WIFI_SDIO2_BASE
#elif PIOS_HAS_WIFI_SDIO1
#define WIFI_SDIO_HOST_BASE  PIOS_WIFI_SDIO1_BASE
#else
#define WIFI_SDIO_HOST_BASE  0UL
#endif

/* BCM2712 SDHCI CFG block — at SDIO2 base + 0x400 (second reg range in DTB)
 * DTB: reg = <0x10 0x01100000 0x0 0x260>, <0x10 0x01100400 0x0 0x200>
 * Key registers: CTRL(+0x00), SD_PIN_SEL(+0x44), MAX_50MHZ(+0x1AC) */
#define BCM2712_SDIO2_CFG_OFFSET  0x400

/* CFG register offsets */
#define SDIO_CFG_CTRL             0x00
#define SDIO_CFG_SD_PIN_SEL       0x44
#define SDIO_CFG_MAX_50MHZ_MODE   0x1AC

/* CFG_CTRL bits */
#define SDIO_CFG_CTRL_SDCD_N_TEST_EN   (1U << 31)  /* force card detect */
#define SDIO_CFG_CTRL_SDCD_N_TEST_LEV  (1U << 30)  /* card detect level */

/* CFG_MAX_50MHZ bits */
#define SDIO_CFG_MAX_50MHZ_STRAP_OVERRIDE  (1U << 31)
#define SDIO_CFG_MAX_50MHZ_ENABLE          (1U << 0)

/* BCM2712 SoC stepping register — upper 16 bits = 0x2712, lower 8 = stepping */
#define BCM2712_SOC_STEPPING  0x1001504004UL
#define SOC_STEPPING_D0       0x30

/* BCM2712 SoC pinctrl for SDIO2 pins (sdio2_30_pins)
 * SoC GPIO controller at 0x107d504100 */
#define BCM2712_PINCTRL_BASE 0x107D504100UL

/* BCM2712 SoC GPIO — brcmstb-gpio register layout
 * Per bank: ODEN(+0x00), DATA(+0x04), IODIR(+0x08), ..., stride=0x20
 * Bank 0 = GPIO 0-31, Bank 1 = GPIO 32-35
 * Circle uses dedicated offsets: DATA0=0x107D508504, IODIR0=0x107D508508 */
#define BCM2712_GPIO_BASE    0x107D517C00UL
#define BCM2712_GPIO1_DATA0  0x107D508504UL
#define BCM2712_GPIO1_IODIR0 0x107D508508UL

/* SDIO2 uses BCM2712 SoC GPIOs 30-35 (sdio2_30_pins in DTB) */
#define SDIO2_GPIO_CLK       30
#define SDIO2_GPIO_CMD       31
#define SDIO2_GPIO_DAT0      32
#define SDIO2_GPIO_DAT1      33
#define SDIO2_GPIO_DAT2      34
#define SDIO2_GPIO_DAT3      35

/* WL_REG_ON: BCM2712 SoC GPIO 28 (active HIGH, 150ms startup delay)
 * DTS: gpio = <&gio 28 GPIO_ACTIVE_HIGH> in wl_on_reg
 * Circle: GPIO28 set output high + 150ms delay */
#define SDIO_WL_REG_ON_GPIO 28

/* SDIO function numbers */
#define SDIO_FUNC_CIA       0   /* Common I/O Area (CCCR/FBR) */
#define SDIO_FUNC_BACKPLANE 1   /* Silicon Backplane */
#define SDIO_FUNC_WLAN      2   /* WLAN data */

struct sdio_diag {
    u32 attempted;
    u32 initialized;
    u32 last_stage;
    u32 cap0;
    u32 cap1;
    u32 present_before;
    u32 present_after_cfg;
    u32 cfg_ctrl;
    u32 pin_sel;
    u32 wl_data;
    u32 wl_iodir;
    u32 cmd5_attempts;
    u32 cmd5_interrupt;
    u32 cmd5_response;
    u32 control1;
    u32 status;
} ALIGNED(64);

_Static_assert(sizeof(struct sdio_diag) == 64U,
               "SDIO diagnostics must occupy one cache line");

/* CCCR (Card Common Control Registers) offsets */
#define CCCR_SDIO_REV       0x00
#define CCCR_SD_REV         0x01
#define CCCR_IO_ENABLE      0x02
#define CCCR_IO_READY       0x03
#define CCCR_INT_ENABLE     0x04
#define CCCR_INT_PENDING    0x05
#define CCCR_IO_ABORT       0x06
#define CCCR_BUS_IFACE      0x07
#define CCCR_CARD_CAPS      0x08
#define CCCR_CIS_PTR        0x09    /* 3 bytes: 0x09-0x0B */
#define CCCR_BUS_SUSPEND    0x0C
#define CCCR_FUNC_SEL       0x0D
#define CCCR_EXEC_FLAGS     0x0E
#define CCCR_READY_FLAGS    0x0F
#define CCCR_BLK_SIZE_LO    0x10
#define CCCR_BLK_SIZE_HI    0x11
#define CCCR_POWER_CTRL     0x12
#define CCCR_HIGH_SPEED     0x13

/* CCCR bus interface control bits */
#define BUS_WIDTH_1BIT      0x00
#define BUS_WIDTH_4BIT      0x02

/* CCCR high-speed bits */
#define HIGH_SPEED_SHS      0x01    /* Supports High Speed */
#define HIGH_SPEED_EHS      0x02    /* Enable High Speed */

/* FBR (Function Basic Registers) base per function */
#define FBR_BASE(fn)        (0x100 * (fn))
#define FBR_BLK_SIZE_LO(fn) (FBR_BASE(fn) + 0x10)
#define FBR_BLK_SIZE_HI(fn) (FBR_BASE(fn) + 0x11)

/* SDIO host controller API */
bool sdio_init(void);
bool sdio_card_present(void);
void sdio_diag_snapshot(struct sdio_diag *out);

/* CMD52: IO_RW_DIRECT — single byte read/write */
bool sdio_cmd52_read(u32 func, u32 addr, u8 *val);
bool sdio_cmd52_write(u32 func, u32 addr, u8 val);

/* CMD53: IO_RW_EXTENDED — multi-byte/block read/write */
bool sdio_cmd53_read(u32 func, u32 addr, u8 *buf, u32 len, bool incr);
bool sdio_cmd53_write(u32 func, u32 addr, const u8 *buf, u32 len, bool incr);

/* Block-mode CMD53 */
bool sdio_cmd53_read_blocks(u32 func, u32 addr, u8 *buf,
                            u32 blksz, u32 nblks, bool incr);
bool sdio_cmd53_write_blocks(u32 func, u32 addr, const u8 *buf,
                             u32 blksz, u32 nblks, bool incr);

/* Set function block size */
bool sdio_set_block_size(u32 func, u16 blksz);

/* Enable/disable a function */
bool sdio_enable_func(u32 func);
bool sdio_disable_func(u32 func);

/* Enable/disable function interrupt */
bool sdio_enable_func_irq(u32 func);

/* Bus width control */
/* Card interrupt (SDHCI status bit 8) — the SDIO in-band interrupt the card
 * raises on DAT1. Reading it is a single MMIO access with no bus transaction,
 * so it is safe to consult on every reactor pass. */
bool sdio_card_irq_pending(void);
bool sdio_card_irq_edge(void);
void sdio_card_irq_ack(void);
void sdio_card_irq_arm(void);
bool sdio_card_irq_take(void);
void sdio_irq_snapshot(u32 *status, u32 *signal_enable, u32 *mask,
                       u32 *gic_enable, u32 *gic_pending,
                       u32 *gic_target);
bool sdio_set_bus_width_4bit(void);

/* Power control */
bool sdio_power_on(void);
void sdio_power_off(void);

/* Reset DATA line (clears DAT_INHIBIT after stuck transfers) */
void sdio_reset_data_line(void);
