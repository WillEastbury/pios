/*
 * sdio.h - RP1 SDIO host controller driver
 *
 * Drives the Synopsys SD/SDIO controller on the RP1 southbridge,
 * used to communicate with the CYW43455 WiFi/BT combo chip.
 *
 * The controller is SDHCI-compatible at RP1 BAR + 0x104000.
 * SDIO pins use RP1 GPIO bank 2 (GPIOs 34-39) in ALT0 mode.
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

/* RP1 SDIO1 controller offset from RP1_BAR_BASE */
#define RP1_SDIO1_BASE      0x104000

/* SDIO GPIO pins — DTB: rp1_sdio1_28_33 (RP1 GPIOs 28-33) */
#define SDIO_GPIO_CLK       28
#define SDIO_GPIO_CMD       29
#define SDIO_GPIO_DAT0      30
#define SDIO_GPIO_DAT1      31
#define SDIO_GPIO_DAT2      32
#define SDIO_GPIO_DAT3      33

/* WL_REG_ON: WiFi chip power/reset — BCM2712 SoC GPIO (not RP1)
 * On Pi 5, this is typically handled by the firmware/regulator.
 * RP1 GPIO used as fallback. Check DTB 'wl-on-reg' for actual pin. */
#define SDIO_WL_REG_ON_GPIO 35

/* SDIO function numbers */
#define SDIO_FUNC_CIA       0   /* Common I/O Area (CCCR/FBR) */
#define SDIO_FUNC_BACKPLANE 1   /* Silicon Backplane */
#define SDIO_FUNC_WLAN      2   /* WLAN data */

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
bool sdio_set_bus_width_4bit(void);

/* Power control */
void sdio_power_on(void);
void sdio_power_off(void);
