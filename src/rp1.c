/*
 * rp1.c - RP1 southbridge access layer
 *
 * Configures the RP1's PCIe BAR, verifies chip identity,
 * and provides register access functions for RP1 peripherals.
 *
 * The RP1 uses BAR1 (PCI config offset 0x14) as its main peripheral
 * register window. We program it to match the outbound ATU target address
 * so that CPU accesses to RP1_BAR_BASE reach the RP1's internal registers.
 *
 * Reference: Linux drivers/mfd/rp1.c
 *            RP1 Peripheral Datasheet (draft)
 */

#include "rp1.h"
#include "pcie.h"
#include "mmio.h"
#include "uart.h"
#include "timer.h"

/* RP1 endpoint: Bus 1, Device 0, Function 0 */
#define RP1_BUS     1
#define RP1_DEV     0
#define RP1_FN      0

/* PCI config space offsets (Type 0 header) */
#define PCICFG_ID           0x00    /* Vendor/Device ID */
#define PCICFG_CMD          0x04    /* Command/Status */
#define PCICFG_BAR1         0x14    /* BAR1: RP1 peripheral register window */

/* PCI command bits */
#define PCI_CMD_MEM         (1U << 1)
#define PCI_CMD_MASTER      (1U << 2)

/* SYSINFO register offsets (within RP1_SYSINFO block at BAR base) */
#define SYSINFO_CHIP_ID     0x00
#define SYSINFO_PLATFORM    0x04

/* ---- Public API ---- */

u32 rp1_read32(u64 offset) {
    return mmio_read(RP1_BAR_BASE + offset);
}

void rp1_write32(u64 offset, u32 val) {
    mmio_write(RP1_BAR_BASE + offset, val);
}

bool rp1_init(void) {
    uart_puts("[rp1] Enumerating RP1 on bus 1...\n");

    /* Read vendor/device ID */
    u32 id = pcie_cfg_read(RP1_BUS, RP1_DEV, RP1_FN, PCICFG_ID);

    if (id == 0xFFFFFFFF || id == 0x00000000) {
        uart_puts("[rp1] No device on bus 1\n");
        return false;
    }

    u16 vendor = (u16)(id & 0xFFFF);
    u16 device = (u16)((id >> 16) & 0xFFFF);

    if (vendor != RP1_VENDOR_ID || device != RP1_DEVICE_ID) {
        uart_puts("[rp1] Unknown device: ");
        uart_hex(id);
        uart_puts("\n");
        return false;
    }

    /*
     * Program BAR1 to the PCIe target address.
     * The outbound ATU maps CPU RP1_BAR_BASE → PCIe PCIE_TARGET_ADDR,
     * so BAR1 must be at PCIE_TARGET_ADDR for the addresses to line up.
     */
    pcie_cfg_write(RP1_BUS, RP1_DEV, RP1_FN, PCICFG_BAR1,
                   (u32)PCIE_TARGET_ADDR);
    dmb();

    /* Enable memory space access + bus mastering on RP1 */
    u32 cmd = pcie_cfg_read(RP1_BUS, RP1_DEV, RP1_FN, PCICFG_CMD);
    cmd |= PCI_CMD_MEM | PCI_CMD_MASTER;
    pcie_cfg_write(RP1_BUS, RP1_DEV, RP1_FN, PCICFG_CMD, cmd);
    dmb();
    timer_delay_us(100);

    /* Read chip ID from SYSINFO block at base of BAR */
    u32 chip_id = rp1_read32(RP1_SYSINFO + SYSINFO_CHIP_ID);
    uart_puts("[rp1] Chip ID: ");
    uart_hex(chip_id);

    if (chip_id == RP1_C0_CHIP_ID) {
        uart_puts(" (RP1-C0)\n");
    } else {
        uart_puts(" (unexpected, expected 0x20001927)\n");
    }

    u32 platform = rp1_read32(RP1_SYSINFO + SYSINFO_PLATFORM);
    uart_puts("[rp1] Platform: ");
    uart_hex(platform);
    uart_puts("\n");

    return true;
}
