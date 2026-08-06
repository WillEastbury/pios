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
#include "fb.h"

/* RP1 endpoint: Bus 1, Device 0, Function 0 */
#define RP1_BUS     1
#define RP1_DEV     0
#define RP1_FN      0

/* PCI config space offsets (Type 0 header) */
#define PCICFG_ID           0x00    /* Vendor/Device ID */
#define PCICFG_CMD          0x04    /* Command/Status */
#define PCICFG_BAR0         0x10    /* BAR0: RP1 MSI-X table/PBA window */
#define PCICFG_BAR1         0x14    /* BAR1: RP1 peripheral register window */
#define PCICFG_BAR2         0x18    /* BAR2: RP1 shared SRAM window */

/* PCI command bits */
#define PCI_CMD_MEM         (1U << 1)
#define PCI_CMD_MASTER      (1U << 2)

/* SYSINFO register offsets (within RP1_SYSINFO block at BAR base) */
#define SYSINFO_CHIP_ID     0x00
#define SYSINFO_PLATFORM    0x04

#define RP1_REG_SET         0x800
#define RP1_REG_CLR         0xC00
#define RP1_MSIX_CFG(n)     (0x8 + (4U * (n)))
#define RP1_MSIX_CFG_ENABLE (1U << 0)
#define RP1_MSIX_CFG_TEST   (1U << 1)
#define RP1_MSIX_CFG_IACK   (1U << 2)
#define RP1_MSIX_CFG_IACK_EN (1U << 3)
#define RP1_INTSTATL        0x108
#define RP1_INTSTATH        0x10C

#define BCM2712_MIP0_BASE       0x1000130000UL
#define MIP_INT_RAISE           0x00
#define MIP_INT_CLEAR           0x10
#define MIP_INT_CFGL_HOST       0x20
#define MIP_INT_CFGH_HOST       0x30
#define MIP_INT_MASKL_HOST      0x40
#define MIP_INT_MASKH_HOST      0x50
#define MIP_INT_MASKL_VPU       0x60
#define MIP_INT_MASKH_VPU       0x70
#define MIP_INT_STATUSL_HOST    0x80
#define MIP_INT_STATUSH_HOST    0x90
#define MIP_INT_STATUSL_VPU     0xA0
#define MIP_INT_STATUSH_VPU     0xB0

/* Keep BAR1 at PCIE_TARGET_ADDR for the RP1 peripheral aperture. Map BAR0
 * near the end of the existing 8MB outbound window for the MSI-X table/PBA
 * (MSI-X capability reports table BIR0 offset 0, PBA offset 0x2000). */
#define RP1_MSIX_TARGET_ADDR    (PCIE_TARGET_ADDR + 0x00700000U)
#define RP1_MSIX_CPU_BASE       (RP1_BAR_BASE + 0x00700000UL)
#define RP1_SRAM_TARGET_ADDR    (PCIE_TARGET_ADDR + RP1_SRAM_WINDOW)

/* ---- Public API ---- */

u32 rp1_read32(u64 offset) {
    return mmio_read(RP1_BAR_BASE + offset);
}

void rp1_write32(u64 offset, u32 val) {
    mmio_write(RP1_BAR_BASE + offset, val);
}

void rp1_irq_snapshot(struct rp1_irq_snapshot *out)
{
    if (!out)
        return;
    out->intstat_l = rp1_read32(RP1_PCIE_APBS + RP1_INTSTATL);
    out->intstat_h = rp1_read32(RP1_PCIE_APBS + RP1_INTSTATH);
    out->mip_status_l = mmio_read(BCM2712_MIP0_BASE + MIP_INT_STATUSL_HOST);
    out->mip_mask_l = mmio_read(BCM2712_MIP0_BASE + MIP_INT_MASKL_HOST);
    out->mip_vpu_status_l = mmio_read(BCM2712_MIP0_BASE + MIP_INT_STATUSL_VPU);
    out->mip_vpu_mask_l = mmio_read(BCM2712_MIP0_BASE + MIP_INT_MASKL_VPU);
    out->mip_cfgl_host = mmio_read(BCM2712_MIP0_BASE + MIP_INT_CFGL_HOST);
    out->eth_msix_cfg = rp1_read32(RP1_PCIE_APBS + RP1_MSIX_CFG(RP1_INT_ETH));
    u64 eth_vec = RP1_MSIX_CPU_BASE + (RP1_INT_ETH * 16U);
    out->eth_msix_addr_lo = mmio_read(eth_vec + 0x0);
    out->eth_msix_addr_hi = mmio_read(eth_vec + 0x4);
    out->eth_msix_data = mmio_read(eth_vec + 0x8);
    out->eth_msix_vector_ctrl = mmio_read(eth_vec + 0xC);
    out->eth_msix_pba = mmio_read(RP1_MSIX_CPU_BASE + 0x2000U + ((RP1_INT_ETH / 32U) * 4U));
}

void rp1_eth_irq_arm(void)
{
    const u32 eth_bit = 1U << RP1_INT_ETH;
    const u64 mip_msg = 0x000000FFFFFFF000ULL;
    u64 eth_vec = RP1_MSIX_CPU_BASE + (RP1_INT_ETH * 16U);
    u32 msix_cap = pcie_cfg_read(RP1_BUS, RP1_DEV, RP1_FN, 0xB0);

    /* MIP0 routes MSI data N to GIC SPI 128+N. Unmask host delivery,
     * mask VPU delivery, set edge-triggered host config, and clear stale
     * ETH vector state before unmasking the endpoint. */
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_MASKL_HOST, 0);
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_MASKH_HOST, 0);
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_MASKL_VPU, 0xFFFFFFFFU);
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_MASKH_VPU, 0xFFFFFFFFU);
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_CFGL_HOST, 0xFFFFFFFFU);
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_CFGH_HOST, 0xFFFFFFFFU);
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_CLEAR, eth_bit);

    /* Program RP1 MSI-X vector 6 (ETH): address = MIP0 message window,
     * data = vector number. Clear vector mask. */
    mmio_write(eth_vec + 0x0, (u32)mip_msg);
    mmio_write(eth_vec + 0x4, (u32)(mip_msg >> 32));
    mmio_write(eth_vec + 0x8, RP1_INT_ETH);
    mmio_write(eth_vec + 0xC, 0);

    /* Enable MSI-X globally in RP1's PCI capability (cap 0xB0, control
     * bits in upper halfword). Table size is already advertised there. */
    pcie_cfg_write(RP1_BUS, RP1_DEV, RP1_FN, 0xB0, msix_cap | (1U << 31));

    /* RP1 wrapper: enable the ETH MSI-X source and IACK for level source. */
    rp1_write32(RP1_PCIE_APBS + RP1_REG_SET + RP1_MSIX_CFG(RP1_INT_ETH),
                RP1_MSIX_CFG_ENABLE | RP1_MSIX_CFG_IACK_EN);
}

void rp1_eth_host_arm(void)
{
    const u32 eth_bit = 1U << RP1_INT_ETH;
    /* Linux-aligned MIP config (drivers/irqchip/irq-bcm2712-mip.c): host
     * unmasked, VPU masked, and CFG*_HOST = ~0 = EDGE. The MIP delivers a
     * momentary edge per MSI-X to the GIC; completion is the GIC EOI, not a
     * MIP status clear. (An earlier PIOS note claimed CFGL=0 was needed for
     * host delivery, but that mis-read the non-latching edge status register —
     * the true signal is the GIC IRQ count, and edge mode is what sustains.) */
    rp1_eth_irq_arm();
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_MASKL_HOST, 0);
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_MASKH_HOST, 0);
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_MASKL_VPU, 0xFFFFFFFFU);
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_MASKH_VPU, 0xFFFFFFFFU);
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_CFGL_HOST, 0xFFFFFFFFU);
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_CFGH_HOST, 0xFFFFFFFFU);
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_CLEAR, eth_bit);
    rp1_write32(RP1_PCIE_APBS + RP1_REG_CLR + RP1_MSIX_CFG(RP1_INT_ETH),
                RP1_MSIX_CFG_TEST | RP1_MSIX_CFG_IACK);
    rp1_write32(RP1_PCIE_APBS + RP1_REG_SET + RP1_MSIX_CFG(RP1_INT_ETH),
                RP1_MSIX_CFG_ENABLE | RP1_MSIX_CFG_IACK_EN);
}

/* Re-arm the RP1 ETH MSI-X after a drain: with IACK_EN set the RP1 endpoint
 * holds off re-sending until acked, so this is what lets the next received
 * frame generate a fresh edge into the MIP/GIC. Gating the IACK behind the
 * RX drain (rather than firing it in the handler) bounds the interrupt rate
 * to the packet rate and prevents the level-mode storm. */
void rp1_eth_irq_rearm(void)
{
    rp1_write32(RP1_PCIE_APBS + RP1_REG_SET + RP1_MSIX_CFG(RP1_INT_ETH),
                RP1_MSIX_CFG_IACK);
    dsb();
}

u32 rp1_mip_host_status_l(void)
{
    return mmio_read(BCM2712_MIP0_BASE + MIP_INT_STATUSL_HOST);
}

u32 rp1_eth_irq_ack(void)
{
    const u32 eth_bit = 1U << RP1_INT_ETH;
    u32 st = mmio_read(BCM2712_MIP0_BASE + MIP_INT_STATUSL_HOST);
    if (st & eth_bit) {
        bool raw_low = (rp1_read32(RP1_PCIE_APBS + RP1_INTSTATL) & eth_bit) == 0;
        if (raw_low && mmio_read(BCM2712_MIP0_BASE + MIP_INT_CFGL_HOST) == 0) {
            mmio_write(BCM2712_MIP0_BASE + MIP_INT_CFGL_HOST, 0xFFFFFFFFU);
            mmio_write(BCM2712_MIP0_BASE + MIP_INT_CLEAR, eth_bit);
            dsb();
            mmio_write(BCM2712_MIP0_BASE + MIP_INT_CFGL_HOST, 0);
        }
        mmio_write(BCM2712_MIP0_BASE + MIP_INT_CLEAR, eth_bit);
        dsb();
        rp1_write32(RP1_PCIE_APBS + RP1_REG_SET + RP1_MSIX_CFG(RP1_INT_ETH),
                    RP1_MSIX_CFG_IACK);
        dsb();
        if (raw_low) {
            mmio_write(BCM2712_MIP0_BASE + MIP_INT_CLEAR, eth_bit);
            dsb();
        }
    }
    return st;
}

u32 rp1_irq_status_l(void)
{
    return rp1_read32(RP1_PCIE_APBS + RP1_INTSTATL);
}

void rp1_eth_level_ack(void)
{
    rp1_write32(RP1_PCIE_APBS + RP1_REG_SET + RP1_MSIX_CFG(RP1_INT_ETH),
                RP1_MSIX_CFG_IACK);
}

void rp1_eth_irq_raise_test(void)
{
    mmio_write(BCM2712_MIP0_BASE + MIP_INT_RAISE, 1U << RP1_INT_ETH);
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
     * Program BAR0/BAR1/BAR2 to PCIe target addresses in our outbound window.
     * BAR0 hosts the MSI-X table/PBA. BAR1 hosts RP1 peripheral registers.
     * BAR2 hosts RP1's 64KB shared SRAM used by the M3 firmware mailbox.
     * The outbound ATU maps CPU RP1_BAR_BASE → PCIe PCIE_TARGET_ADDR,
     * so BAR1 must be at PCIE_TARGET_ADDR for the addresses to line up.
     */
    pcie_cfg_write(RP1_BUS, RP1_DEV, RP1_FN, PCICFG_BAR0,
                   (u32)RP1_MSIX_TARGET_ADDR);
    pcie_cfg_write(RP1_BUS, RP1_DEV, RP1_FN, PCICFG_BAR1,
                   (u32)PCIE_TARGET_ADDR);
    pcie_cfg_write(RP1_BUS, RP1_DEV, RP1_FN, PCICFG_BAR2,
                   (u32)RP1_SRAM_TARGET_ADDR);
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
