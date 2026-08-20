/*
 * pcie.h - BCM2712 PCIe Root Complex driver
 *
 * Initialises PCIe2 (quad-lane) connected to the RP1 southbridge.
 * Handles link training, outbound ATU window, and config space access.
 *
 * Reference: Linux drivers/pci/controller/pcie-brcmstb.c (BCM2712)
 *            Device tree: pcie2@120000 in bcm2712.dtsi
 */

#pragma once
#include "types.h"

/* RP1 PCIe identity */
#define RP1_VENDOR_ID       0x1de4
#define RP1_DEVICE_ID       0x0001

/* Outbound window: maps CPU addresses to PCIe memory space */
#define PCIE_CPU_WIN_BASE   0x1F00000000UL  /* CPU-side base */
#define PCIE_CPU_WIN_SIZE   0x00800000UL    /* 8MB window */
#define PCIE_TARGET_ADDR    0x80000000UL    /* PCIe-side target for RP1 BAR */

/* RP1 inbound DMA window: device address 0x10_00000000 maps CPU physical
 * address 0. Only low 4 GiB CPU addresses are representable by this window.
 * Every RP1 bus master must use this helper and fail closed on overflow. */
#define RP1_PCIE_DMA_BASE   0x1000000000ULL
#define RP1_PCIE_DMA_SIZE   0x100000000ULL

static inline bool rp1_pcie_dma_addr(const void *ptr, u64 size, u64 *out)
{
    if (!ptr || !out || size == 0ULL)
        return false;
    u64 pa = (u64)(usize)ptr;
    if (pa >= RP1_PCIE_DMA_SIZE || size > RP1_PCIE_DMA_SIZE - pa)
        return false;
    *out = RP1_PCIE_DMA_BASE + pa;
    return true;
}

bool pcie_init(void);
bool pcie_link_up(void);

/* Config space access (bus/dev/func addressing) */
u32  pcie_cfg_read(u32 bus, u32 dev, u32 func, u32 reg);
void pcie_cfg_write(u32 bus, u32 dev, u32 func, u32 reg, u32 val);

/* AER diagnostics */
struct pcie_aer_snapshot {
    u32 aer_offset;
    u32 uncorr;
    u32 corr;
    u32 hdr0;
    u32 hdr1;
    u32 hdr2;
    u32 hdr3;
} PACKED;

void pcie_aer_init(void);
void pcie_aer_dump(const char *tag);
void pcie_aer_snapshot(struct pcie_aer_snapshot *out, bool clear);
