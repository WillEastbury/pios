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

bool pcie_init(void);
bool pcie_link_up(void);

/* Config space access (bus/dev/func addressing) */
u32  pcie_cfg_read(u32 bus, u32 dev, u32 func, u32 reg);
void pcie_cfg_write(u32 bus, u32 dev, u32 func, u32 reg, u32 val);
