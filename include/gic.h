/*
 * gic.h - ARM GIC-400 interrupt controller (BCM2712)
 */

#pragma once
#include "types.h"

/*
 * BCM2712 GIC-400 base addresses.
 * Distributor and CPU interface are at fixed offsets from GIC base.
 */
#define GIC_BASE            (PERIPH_BASE + 0x01840000UL)
#define GICD_BASE           (GIC_BASE + 0x1000)
#define GICC_BASE           (GIC_BASE + 0x2000)

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

/* Max interrupts */
#define GIC_MAX_IRQ         256

void gic_init(void);
void gic_enable_irq(u32 intid);
void gic_disable_irq(u32 intid);
void gic_set_priority(u32 intid, u8 priority);
void gic_set_target(u32 intid, u8 cpu_mask);
u32  gic_acknowledge(void);
void gic_end_of_interrupt(u32 intid);
