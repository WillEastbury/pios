#pragma once
#include "types.h"

#define PIOS_PLATFORM_PI5        1
#define PIOS_PLATFORM_QEMU_VIRT  2

#ifndef PIOS_PLATFORM
#define PIOS_PLATFORM PIOS_PLATFORM_PI5
#endif

#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
#define PIOS_PLATFORM_NAME          "qemu-virt"
#define PIOS_PLATFORM_CORE_COUNT    4U
#define PIOS_PERIPH_BASE            0x09000000UL
#define PIOS_UART0_BASE             0x09000000UL
#define PIOS_MBOX_BASE              0UL
#define PIOS_EMMC2_BASE             0UL
#define PIOS_GENET_BASE             0UL
#define PIOS_PCIE_RC_BASE           0UL
#define PIOS_RP1_BAR_BASE           0UL
#define PIOS_GIC_BASE               0x08000000UL
#define PIOS_GICD_BASE              0x08000000UL
#define PIOS_GICC_BASE              0x08010000UL
#define PIOS_HAS_RP1                0
#define PIOS_HAS_PCIE               0
#define PIOS_HAS_GENET              0
#define PIOS_HAS_SD                 0
#define PIOS_HAS_MAILBOX_FB         0
#else
#define PIOS_PLATFORM_NAME          "pi5-bcm2712"
#define PIOS_PLATFORM_CORE_COUNT    4U
#define PIOS_PERIPH_BASE            0x107C000000UL
#define PIOS_UART0_BASE             (PIOS_PERIPH_BASE + 0x201000UL)
#define PIOS_MBOX_BASE              (PIOS_PERIPH_BASE + 0x013880UL)
#define PIOS_EMMC2_BASE             0x1000FFF000UL
#define PIOS_GENET_BASE             0x107D580000UL
#define PIOS_PCIE_RC_BASE           0x1000120000UL
#define PIOS_RP1_BAR_BASE           0x1F00000000UL
#define PIOS_GIC_BASE               0x107FFF8000UL
#define PIOS_GICD_BASE              0x107FFF9000UL
#define PIOS_GICC_BASE              0x107FFFA000UL
#define PIOS_HAS_RP1                1
#define PIOS_HAS_PCIE               1
#define PIOS_HAS_GENET              1
#define PIOS_HAS_SD                 1
#define PIOS_HAS_MAILBOX_FB         1
#endif

#define PIOS_CORE_PRIV_SIZE         0x01000000UL
#define PIOS_CORE0_RAM_BASE         0x00800000UL
#define PIOS_CORE1_RAM_BASE         0x01800000UL
#define PIOS_CORE2_RAM_BASE         0x02800000UL
#define PIOS_CORE3_RAM_BASE         0x03800000UL
#define PIOS_SHARED_FIFO_BASE       0x04800000UL
#define PIOS_SHARED_FIFO_SIZE       0x00100000UL
#define PIOS_DMA_NET_BASE           0x04900000UL
#define PIOS_DMA_NET_SIZE           0x00200000UL
#define PIOS_DMA_DISK_BASE          0x04B00000UL
#define PIOS_DMA_DISK_SIZE          0x00200000UL
#define PIOS_IPC_SHM_BASE           0x04D00000UL
#define PIOS_IPC_SHM_SIZE           0x00100000UL
#define PIOS_FB_BACK_BASE           0x05000000UL
#define PIOS_FB_BACK_SIZE           0x01000000UL
