/*
 * rp1.h - RP1 southbridge access layer
 *
 * The RP1 is the Pi 5's I/O controller (USB, GPIO, UART, SPI, I2C, PWM).
 * Accessed via PCIe BAR1 mapped at RP1_BAR_BASE by the outbound ATU window.
 *
 * Reference: RP1 Peripheral Datasheet, Linux drivers/mfd/rp1.c
 */

#pragma once
#include "types.h"

/* RP1 expected chip ID (read from SYSINFO offset 0x00) */
#define RP1_C0_CHIP_ID      0x20001927

/* RP1 peripheral block offsets from RP1_BAR_BASE */
#define RP1_SYSINFO         0x000000
#define RP1_SYSCFG          0x008000
#define RP1_IO_BANK0        0x0D0000
#define RP1_IO_BANK1        0x0D4000
#define RP1_IO_BANK2        0x0D8000
#define RP1_SYS_RIO0        0x0E0000
#define RP1_SYS_RIO1        0x0E4000
#define RP1_SYS_RIO2        0x0E8000
#define RP1_PADS_BANK0      0x0F0000
#define RP1_PADS_BANK1      0x0F4000
#define RP1_PADS_BANK2      0x0F8000
#define RP1_PCIE_APBS       0x108000

#define RP1_INT_ETH         6U

struct rp1_irq_snapshot {
    u32 intstat_l;
    u32 intstat_h;
    u32 mip_status_l;
    u32 mip_mask_l;
    u32 eth_msix_cfg;
    u32 eth_msix_addr_lo;
    u32 eth_msix_addr_hi;
    u32 eth_msix_data;
    u32 eth_msix_vector_ctrl;
    u32 eth_msix_pba;
} PACKED;

bool rp1_init(void);
u32  rp1_read32(u64 offset);
void rp1_write32(u64 offset, u32 val);
void rp1_irq_snapshot(struct rp1_irq_snapshot *out);
void rp1_eth_irq_arm(void);
u32  rp1_eth_irq_ack(void);
void rp1_eth_irq_raise_test(void);
