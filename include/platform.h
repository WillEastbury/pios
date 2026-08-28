#pragma once
#include "types.h"

#define PIOS_PLATFORM_PI5        1
#define PIOS_PLATFORM_QEMU_VIRT  2
#define PIOS_PLATFORM_UEFI       3
#define PIOS_PLATFORM_HYPERV_ARM 4
#define PIOS_PLATFORM_HYPERV_AMD64 5
#define PIOS_PLATFORM_PI3        6
#define PIOS_PLATFORM_PIZERO2W   7
#define PIOS_PLATFORM_FVP_A76_GICV2 8

#ifndef PIOS_PLATFORM
#define PIOS_PLATFORM PIOS_PLATFORM_PI5
#endif

/* The tensor/QPU backend contains V3D 7.1 code and register definitions.
 * A firmware mailbox framebuffer alone does not imply this capability:
 * BCM2837-family boards expose that mailbox through VideoCore IV. */
#if PIOS_PLATFORM == PIOS_PLATFORM_PI5
#define PIOS_HAS_V3D_71             1
#else
#define PIOS_HAS_V3D_71             0
#endif

#if PIOS_PLATFORM == PIOS_PLATFORM_FVP_A76_GICV2
/* Arm Base FVP: Cortex-A76 cluster with the GICv2-compatible legacy
 * CPU interface. This is an architectural CPU/MMU/GIC test target, not a
 * BCM2712/RP1 hardware model. */
#define PIOS_PLATFORM_NAME          "fvp-a76-gicv2"
#define PIOS_PLATFORM_CORE_COUNT    4U
#define PIOS_PERIPH_BASE            0x1C000000UL
#define PIOS_UART0_BASE             0x1C090000UL
#define PIOS_MBOX_BASE              0UL
#define PIOS_EMMC2_BASE             0UL
#define PIOS_GENET_BASE             0UL
#define PIOS_PCIE_RC_BASE           0UL
#define PIOS_RP1_BAR_BASE           0UL
#define PIOS_GIC_BASE               0x2C000000UL
#define PIOS_GICD_BASE              0x2C040000UL
#define PIOS_GICC_BASE              0x2C000000UL
#define PIOS_QA7_BASE               0UL
#define PIOS_HAS_GIC                1
#define PIOS_HAS_RP1                0
#define PIOS_HAS_PCIE               0
#define PIOS_HAS_GENET              0
#define PIOS_HAS_SD                 0
#define PIOS_HAS_MAILBOX_FB         0
#define PIOS_HAS_BOOTINFO_FB        0
#define PIOS_HAS_DMA                0
#define PIOS_HAS_PSCI_SECONDARIES   0
#define PIOS_PSCI_USE_HVC           0
#define PIOS_PSCI_AFF_SHIFT         0
#define PIOS_HAS_VIRTIO_NET         0
#define PIOS_ENABLE_NATIVE_VIDEOCORE 0
#define PIOS_HAS_HYPERV             0
#define PIOS_HAS_VMBUS              0
#define PIOS_HAS_WIFI_SDIO2         0
#define PIOS_WIFI_SDIO2_BASE        0UL
#elif PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
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
#define PIOS_QA7_BASE               0UL
#define PIOS_HAS_GIC                1
#define PIOS_HAS_RP1                0
#define PIOS_HAS_PCIE               0
#define PIOS_HAS_GENET              0
#define PIOS_HAS_SD                 0
#define PIOS_HAS_MAILBOX_FB         0
#define PIOS_HAS_BOOTINFO_FB        1
#define PIOS_HAS_DMA                0
#define PIOS_HAS_PSCI_SECONDARIES   1
#define PIOS_PSCI_USE_HVC           1
#define PIOS_PSCI_AFF_SHIFT         0
#define PIOS_HAS_VIRTIO_NET         1
#define PIOS_VIRTIO_MMIO_BASE       0x0A000000UL
#define PIOS_VIRTIO_MMIO_STRIDE     0x200UL
#define PIOS_VIRTIO_MMIO_COUNT      32U
#define PIOS_ENABLE_NATIVE_VIDEOCORE 0
#define PIOS_HAS_HYPERV             0
#define PIOS_HAS_VMBUS              0
#define PIOS_HAS_WIFI_SDIO2         0
#define PIOS_WIFI_SDIO2_BASE        0UL
#elif PIOS_PLATFORM == PIOS_PLATFORM_HYPERV_ARM
#define PIOS_PLATFORM_NAME          "hyperv-arm64"
#define PIOS_PLATFORM_CORE_COUNT    4U
#define PIOS_PERIPH_BASE            0UL
#define PIOS_UART0_BASE             0UL
#define PIOS_MBOX_BASE              0UL
#define PIOS_EMMC2_BASE             0UL
#define PIOS_GENET_BASE             0UL
#define PIOS_PCIE_RC_BASE           0UL
#define PIOS_RP1_BAR_BASE           0UL
#define PIOS_GIC_BASE               0UL
#define PIOS_GICD_BASE              0UL
#define PIOS_GICC_BASE              0UL
#define PIOS_QA7_BASE               0UL
#define PIOS_HAS_GIC                1
#define PIOS_HAS_RP1                0
#define PIOS_HAS_PCIE               0
#define PIOS_HAS_GENET              0
#define PIOS_HAS_SD                 0
#define PIOS_HAS_MAILBOX_FB         0
#define PIOS_HAS_BOOTINFO_FB        0
#define PIOS_HAS_DMA                0
#define PIOS_HAS_PSCI_SECONDARIES   0
#define PIOS_PSCI_USE_HVC           0
#define PIOS_PSCI_AFF_SHIFT         8
#define PIOS_HAS_VIRTIO_NET         0
#define PIOS_ENABLE_NATIVE_VIDEOCORE 0
#define PIOS_HAS_HYPERV             1
#define PIOS_HAS_VMBUS              1
#define PIOS_HAS_WIFI_SDIO2         0
#define PIOS_WIFI_SDIO2_BASE        0UL
#elif PIOS_PLATFORM == PIOS_PLATFORM_HYPERV_AMD64
#define PIOS_PLATFORM_NAME          "hyperv-amd64"
#define PIOS_PLATFORM_CORE_COUNT    4U
#define PIOS_PERIPH_BASE            0UL
#define PIOS_UART0_BASE             0UL
#define PIOS_MBOX_BASE              0UL
#define PIOS_EMMC2_BASE             0UL
#define PIOS_GENET_BASE             0UL
#define PIOS_PCIE_RC_BASE           0UL
#define PIOS_RP1_BAR_BASE           0UL
#define PIOS_GIC_BASE               0UL
#define PIOS_GICD_BASE              0UL
#define PIOS_GICC_BASE              0UL
#define PIOS_QA7_BASE               0UL
#define PIOS_HAS_GIC                0
#define PIOS_HAS_RP1                0
#define PIOS_HAS_PCIE               0
#define PIOS_HAS_GENET              0
#define PIOS_HAS_SD                 0
#define PIOS_HAS_MAILBOX_FB         0
#define PIOS_HAS_BOOTINFO_FB        0
#define PIOS_HAS_DMA                0
#define PIOS_HAS_PSCI_SECONDARIES   0
#define PIOS_PSCI_USE_HVC           0
#define PIOS_PSCI_AFF_SHIFT         8
#define PIOS_HAS_VIRTIO_NET         0
#define PIOS_ENABLE_NATIVE_VIDEOCORE 0
#define PIOS_HAS_HYPERV             1
#define PIOS_HAS_VMBUS              1
#define PIOS_HAS_WIFI_SDIO2         0
#define PIOS_WIFI_SDIO2_BASE        0UL
#define PIOS_HAS_WIFI_SDIO1         0
#define PIOS_WIFI_SDIO1_BASE        0UL
#define PIOS_WIFI_WL_REG_ON_GPIO    0U
#elif PIOS_PLATFORM == PIOS_PLATFORM_PI3 || PIOS_PLATFORM == PIOS_PLATFORM_PIZERO2W
/* BCM2837/BCM2837B0 (Pi3 B/B+) and BCM2710A1 (Pi Zero 2 W) share the same
 * die/peripheral generation and "low peripheral" memory map -- quad
 * Cortex-A53 (ARMv8-A), legacy VideoCore IV, no RP1 southbridge, and
 * critically NO GIC-400: interrupts are the legacy Broadcom local
 * interrupt controller + ARM-local "QA7" peripherals block (per-core
 * timer IRQ enables, IPI mailboxes, pending-status), a physically
 * separate fixed block from the main peripheral bus. See src/irqc_legacy.c.
 * Secondary cores also do NOT support the PSCI HVC mechanism Pi5/QEMU use
 * -- stock firmware expects a spin-table wakeup (per-core mailbox in low
 * memory), which is not yet implemented; PIOS_HAS_PSCI_SECONDARIES=0 here
 * documents that gap rather than silently assuming PSCI works. */
#if PIOS_PLATFORM == PIOS_PLATFORM_PI3
#define PIOS_PLATFORM_NAME          "pi3-bcm2837"
#else
#define PIOS_PLATFORM_NAME          "pizero2w-bcm2710a1"
#endif
#define PIOS_PLATFORM_CORE_COUNT    4U
#define PIOS_PERIPH_BASE            0x3F000000UL
#define PIOS_UART0_BASE             (PIOS_PERIPH_BASE + 0x201000UL)
#define PIOS_MBOX_BASE              (PIOS_PERIPH_BASE + 0x00B880UL)
#define PIOS_EMMC2_BASE             (PIOS_PERIPH_BASE + 0x300000UL)
#define PIOS_GENET_BASE             0UL
#define PIOS_PCIE_RC_BASE           0UL
#define PIOS_RP1_BAR_BASE           0UL
#define PIOS_GIC_BASE               0UL
#define PIOS_GICD_BASE              0UL
#define PIOS_GICC_BASE              0UL
#define PIOS_QA7_BASE               0x40000000UL
#define PIOS_HAS_GIC                0
#define PIOS_HAS_RP1                0
#define PIOS_HAS_PCIE               0
#define PIOS_HAS_GENET              0
#define PIOS_HAS_SD                 1
#define PIOS_HAS_MAILBOX_FB         1
#define PIOS_HAS_BOOTINFO_FB        0
#define PIOS_HAS_DMA                0
#define PIOS_HAS_PSCI_SECONDARIES   0
#define PIOS_PSCI_USE_HVC           0
#define PIOS_PSCI_AFF_SHIFT         8
#define PIOS_HAS_VIRTIO_NET         0
#define PIOS_ENABLE_NATIVE_VIDEOCORE 0
#define PIOS_HAS_HYPERV             0
#define PIOS_HAS_VMBUS              0
#define PIOS_HAS_WIFI_SDIO2         0
#define PIOS_WIFI_SDIO2_BASE        0UL
#if PIOS_PLATFORM == PIOS_PLATFORM_PI3
/* Pi 3 B/B+ onboard radio: legacy Arasan SDIO1 on GPIO34-39. WL_ON is
 * firmware expgpio line 1 (property GPIO 129), not SoC GPIO43. */
#define PIOS_HAS_WIFI_SDIO1         1
#define PIOS_WIFI_SDIO1_BASE        0x3F300000UL
#define PIOS_WIFI_WL_REG_ON_GPIO     129U
#define PIOS_WIFI_WL_REG_ON_FIRMWARE 1
#else
/* Zero 2 W routes WL_ON to direct SoC GPIO41. */
#define PIOS_HAS_WIFI_SDIO1         1
#define PIOS_WIFI_SDIO1_BASE        0x3F300000UL
#define PIOS_WIFI_WL_REG_ON_GPIO    41U
#define PIOS_WIFI_WL_REG_ON_FIRMWARE 0
#endif
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
#define PIOS_QA7_BASE               0UL
#define PIOS_HAS_GIC                1
#define PIOS_HAS_RP1                1
#define PIOS_HAS_PCIE               1
#define PIOS_HAS_GENET              1
#define PIOS_HAS_SD                 1
#define PIOS_HAS_MAILBOX_FB         1
#define PIOS_HAS_BOOTINFO_FB        0
#define PIOS_HAS_DMA                1
#define PIOS_HAS_PSCI_SECONDARIES   1
#define PIOS_PSCI_USE_HVC           0
#define PIOS_PSCI_AFF_SHIFT         8
#define PIOS_HAS_VIRTIO_NET         0
#define PIOS_HAS_HYPERV             0
#define PIOS_HAS_VMBUS              0
/* Onboard CYW43455 WiFi/BT combo chip, over the BCM2712 SoC's dedicated
 * SDIO2 controller (NOT RP1 -- see src/board_detect.c comment history and
 * spike/wifi/sdio.c for the DTB-derived address). Driver lives in
 * spike/wifi/ until re-enabled (see spike/wifi/README.md). */
#define PIOS_HAS_WIFI_SDIO2         1
#define PIOS_WIFI_SDIO2_BASE        0x1001100000UL
#define PIOS_HAS_WIFI_SDIO1         0
#define PIOS_WIFI_SDIO1_BASE        0UL
#define PIOS_WIFI_WL_REG_ON_GPIO    0U
#define PIOS_WIFI_WL_REG_ON_FIRMWARE 0
#endif

#ifndef PIOS_HAS_WIFI_SDIO1
#define PIOS_HAS_WIFI_SDIO1         0
#define PIOS_WIFI_SDIO1_BASE        0UL
#define PIOS_WIFI_WL_REG_ON_GPIO    0U
#endif

#define PIOS_HAS_WIFI_SDIO          (PIOS_HAS_WIFI_SDIO1 || PIOS_HAS_WIFI_SDIO2)

#define PIOS_CORE_PRIV_SIZE         0x01000000UL

#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
#define PIOS_CORE0_RAM_BASE         0x42300000UL
#define PIOS_CORE1_RAM_BASE         0x43200000UL
#define PIOS_CORE2_RAM_BASE         0x44200000UL
#define PIOS_CORE3_RAM_BASE         0x45200000UL
#define PIOS_SHARED_FIFO_BASE       0x46200000UL
#define PIOS_DMA_NET_BASE           0x46300000UL
#define PIOS_DMA_DISK_BASE          0x46500000UL
#define PIOS_IPC_SHM_BASE           0x46700000UL
#define PIOS_FB_BACK_BASE           0x46A00000UL
#else
#define PIOS_CORE0_RAM_BASE         0x00800000UL
#define PIOS_CORE1_RAM_BASE         0x01800000UL
#define PIOS_CORE2_RAM_BASE         0x02800000UL
#define PIOS_CORE3_RAM_BASE         0x03800000UL
#define PIOS_SHARED_FIFO_BASE       0x04800000UL
#define PIOS_DMA_NET_BASE           0x04900000UL
#define PIOS_DMA_DISK_BASE          0x04B00000UL
#define PIOS_IPC_SHM_BASE           0x04D00000UL
#define PIOS_FB_BACK_BASE           0x05000000UL
#endif

#define PIOS_SHARED_FIFO_SIZE       0x00100000UL
#define PIOS_DMA_NET_SIZE           0x00200000UL
#define PIOS_DMA_DISK_SIZE          0x00200000UL
#define PIOS_IPC_SHM_SIZE           0x00100000UL
#define PIOS_FB_BACK_SIZE           0x01000000UL

/*
 * Global process arena (ADR-024 / issue #84).
 *
 * Process memory used to be carved out of the owning core's fixed 16 MB private
 * region, which capped the whole system at ~6 slots and — more importantly —
 * made a process's memory *belong to a core*, so it could never migrate (#85).
 * Slots now come from one global arena instead. Only signalling and buffering
 * keep fixed per-core/shared reservations.
 *
 * Placement rules, both load-bearing:
 *   - clear of the stage0 staging/trampoline window (Pi5 0x08000000 /
 *     0x07FFF000, QEMU 0x48000000 / 0x47FFF000), or an OTA would overwrite live
 *     process memory;
 *   - clear of FB_BACK and every other reservation above.
 *
 * Pi5 sits at 256 MB, inside L1[0]/l2_table_low, and is given Normal-WB
 * attributes at the FIRST MMU enable (see mmu_init) — never mapped one way and
 * tightened later, which is the boot-time WB->NC transition that caused the RX
 * descriptor-hole bug. QEMU sits inside the L1[1] 1 GB block, above staging.
 */
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
#define PIOS_PROC_ARENA_BASE        0x50000000UL
#else
#define PIOS_PROC_ARENA_BASE        0x10000000UL
#endif
#define PIOS_PROC_ARENA_SIZE        0x02000000UL   /* 32 MB = 16 x 2 MB slots */

/* Experimental native VideoCore probe.
 * 0 = keep the stable firmware-mailbox framebuffer path only.
 * 1 = additionally run a read-only VC6/V3D register visibility probe.
 */
#ifndef PIOS_ENABLE_NATIVE_VIDEOCORE
#define PIOS_ENABLE_NATIVE_VIDEOCORE 1
#endif

/* Experimental native V3D compute dispatch.
 * Depends on PIOS_ENABLE_NATIVE_VIDEOCORE. Keep off until the Pi5 V3D 7.1
 * MMU/page-table and CSD queue path is validated on hardware.
 */
#ifndef PIOS_ENABLE_NATIVE_V3D_COMPUTE
#define PIOS_ENABLE_NATIVE_V3D_COMPUTE 1
#endif

/* Keep native V3D MMU programming opt-in while the fault/IRQ path is under
 * bring-up. Native probe can still prove hardware visibility and compute
 * capability without leaving the V3D MMU enabled after boot.
 */
#ifndef PIOS_ENABLE_NATIVE_V3D_MMU_BOOT
#define PIOS_ENABLE_NATIVE_V3D_MMU_BOOT 1
#endif

/* Experimental built-in tiny QPU kernels.
 * These are Mesa-emitted V3D 7.1 QPU words for one-element add/ReLU bring-up.
 * Keep off until hardware trials prove the mailbox/CSD submission path.
 */
#ifndef PIOS_ENABLE_TINY_QPU_KERNELS
#define PIOS_ENABLE_TINY_QPU_KERNELS 1
#endif
