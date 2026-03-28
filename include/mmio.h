#pragma once
#include "types.h"

/*
 * BCM2712 (Raspberry Pi 5) peripheral address map.
 * ARM peripherals base: 0x107C000000 in the 36-bit address space.
 * These addresses are set by the GPU firmware before handing off to us.
 * Adjust if your firmware/DTB uses a different mapping.
 */

#define PERIPH_BASE         0x107C000000UL

/* PL011 UART0 - pre-configured by firmware for serial console */
#define UART0_BASE          (PERIPH_BASE + 0x201000)

/* VideoCore Mailbox */
#define MBOX_BASE           (PERIPH_BASE + 0x00B880)

/* EMMC2 / SD Host Controller */
#define EMMC2_BASE          (PERIPH_BASE + 0x300000)

/* GENET v5 Ethernet MAC */
#define GENET_BASE          0x107D580000UL

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
