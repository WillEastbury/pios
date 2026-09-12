/*
 * gic.h - ARM GIC-400 interrupt controller (BCM2712)
 */

#pragma once
#include "types.h"
#include "platform.h"

/*
 * BCM2712 GIC-400 base addresses.
 * Distributor and CPU interface are at fixed offsets from GIC base.
 */
#define GIC_BASE            PIOS_GIC_BASE
#define GICD_BASE           PIOS_GICD_BASE
#define GICC_BASE           PIOS_GICC_BASE

/* Distributor registers */
#define GICD_CTLR           (GICD_BASE + 0x000)
#define GICD_TYPER          (GICD_BASE + 0x004)
#define GICD_ISENABLER(n)   (GICD_BASE + 0x100 + (n) * 4)
#define GICD_ICENABLER(n)   (GICD_BASE + 0x180 + (n) * 4)
#define GICD_ISPENDR(n)     (GICD_BASE + 0x200 + (n) * 4)
#define GICD_ICPENDR(n)     (GICD_BASE + 0x280 + (n) * 4)
#define GICD_IPRIORITYR(n)  (GICD_BASE + 0x400 + (n) * 4)
#define GICD_ITARGETSR(n)   (GICD_BASE + 0x800 + (n) * 4)
#define GICD_ICFGR(n)       (GICD_BASE + 0xC00 + (n) * 4)

/* CPU interface registers */
#define GICC_CTLR           (GICC_BASE + 0x000)
#define GICC_PMR            (GICC_BASE + 0x004)
#define GICC_IAR            (GICC_BASE + 0x00C)
#define GICC_EOIR           (GICC_BASE + 0x010)

/* Interrupt IDs */
#define GIC_INTID_SPURIOUS  1023
#define GIC_TIMER_NS_PHYS   30      /* Non-secure physical timer PPI */
#define GIC_TIMER_VIRT      27      /* Virtual timer PPI */
#define GIC_RP1_ETH_MSI     166     /* GIC_SPI 128 => INTID 160, plus RP1_INT_ETH(6) */
#define GIC_SGI_WAKE        9       /* SW-generated inter-core wake doorbell (SGI) */
/* BCM2837 ARMCTRL GPU IRQ62, exposed only as a private compatibility intid by
 * irqc_legacy. It is neither a GIC SPI nor Linux's remapped IRQ domain. */
#define LEGACY_GPU_IRQ_SDHCI PIOS_BCM2837_GPU_IRQ_SDIO1
#define LEGACY_GPU_IRQ_DWC2   PIOS_BCM2837_GPU_IRQ_DWC2

/* Max interrupts */
#define GIC_MAX_IRQ         320

void gic_init(void);
void gic_select_bases(u32 id, u64 gicd_base, u64 gicc_base);
void gic_restore_default_bases(void);
u32  gic_runtime_id(void);
u64  gic_runtime_gicd_base(void);
u64  gic_runtime_gicc_base(void);
void gic_enable_irq(u32 intid);
void gic_disable_irq(u32 intid);
void gic_set_priority(u32 intid, u8 priority);
void gic_set_target(u32 intid, u8 cpu_mask);
void gic_set_group1(u32 intid);
void gic_set_edge_triggered(u32 intid);
void gic_clear_pending(u32 intid);
u32  gic_acknowledge(void);
void gic_end_of_interrupt(u32 intid);
/* Send a software-generated interrupt (SGI 0-15) to the cores in target_mask
 * (bit N = core N). Inter-core doorbell IPI; from non-secure EL1 it is
 * delivered Group1-NS. A dsb ish before the trigger orders prior memory
 * writes ahead of the doorbell so the target observes posted work first. */
void gic_send_sgi(u8 target_mask, u32 sgi_id);
/* Enable the calling core's GIC CPU interface (banked). MUST be called on each
 * secondary core or it will never receive any interrupt. */
void gic_cpu_init(void);

#if !PIOS_HAS_GIC
/* Registering a known ARMCTRL source never unmasks it. A source can be
 * enabled/acknowledged only after its IRQ handler owns this registration. */
bool gic_legacy_register_gpu_irq(u32 intid);
bool gic_legacy_unregister_gpu_irq(u32 intid);
/* Route the normal GPU cascade to core 0 before any registered source is
 * enabled. The FIQ routing field is preserved. */
bool gic_legacy_route_gpu_core0(void);
#endif
