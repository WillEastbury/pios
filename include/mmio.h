#pragma once
#include "types.h"

/*
 * BCM2712 (Raspberry Pi 5) peripheral address map.
 * ARM peripherals base: 0x107C000000 in the 36-bit address space.
 * These addresses are set by the GPU firmware before handing off to us.
 * Adjust if your firmware/DTB uses a different mapping.
 */

#define PERIPH_BASE         0x107C000000UL

/* PL011 UART0 - pre-configured by firmware for serial console.
 * On BCM2712 this is at PERIPH_BASE + 0x201000 in the SoC address map,
 * but GPIO14/15 UART is routed through RP1. For early boot before our
 * PCIe driver runs, firmware maps RP1 at 0x1c00000000 with enable_rp1_uart=1.
 * After our PCIe init remaps RP1 to RP1_BAR_BASE, use rp1_uart.h instead. */
#define UART0_BASE          (PERIPH_BASE + 0x201000)

/* VideoCore Mailbox (confirmed from bcm2712.dtsi: mailbox@7c013880) */
#define MBOX_BASE           (PERIPH_BASE + 0x013880)

/* EMMC2 / SD Host Controller
 * Pi 5 uses 0x1000FFF000 (not PERIPH_BASE + 0x300000 like Pi 4) */
#define EMMC2_BASE          0x1000FFF000UL

/* GENET v5 Ethernet MAC */
#define GENET_BASE          0x107D580000UL

/* BCM2712 PCIe Root Complex (PCIe2, quad-lane, connected to RP1) */
#define PCIE_RC_BASE        0x1000120000UL

/* RP1 southbridge register window (mapped via PCIe outbound ATU) */
#define RP1_BAR_BASE        0x1F00000000UL

static inline void mmio_write(u64 addr, u32 val) {
    *(volatile u32 *)addr = val;
}

static inline u32 mmio_read(u64 addr) {
    return *(volatile u32 *)addr;
}

static inline void mmio_write16(u64 addr, u16 val) {
    *(volatile u16 *)addr = val;
}

static inline u16 mmio_read16(u64 addr) {
    return *(volatile u16 *)addr;
}
