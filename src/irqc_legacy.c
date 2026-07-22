/*
 * irqc_legacy.c - legacy Broadcom local interrupt controller driver
 *
 * Implements the exact same public API as gic.c (see include/gic.h) for
 * BCM2837-family boards (Pi3 B/B+, Pi Zero 2 W / BCM2710A1) that have no
 * GIC-400 at all. Only compiled when PIOS_HAS_GIC=0.
 *
 * These SoCs route the ARM architected timer PPIs and inter-core doorbells
 * through the ARM-local "QA7" peripherals block (a fixed, per-core register
 * window physically separate from the main peripheral bus), documented in
 * the BCM2836 ARM-local peripherals manual (QA7_rev3.4). Register offsets
 * below are cross-checked against the upstream Linux kernel driver
 * (drivers/irqchip/irq-bcm2836.c, LOCAL_* constants) rather than derived
 * from scratch, since a wrong MMIO offset here would silently misbehave
 * exactly like this session's earlier Pi5 hardware bugs.
 *
 * QA7_BASE + register layout (per core, stride 4 bytes unless noted):
 *   0x00 LOCAL_CONTROL
 *   0x08 LOCAL_PRESCALER
 *   0x0C LOCAL_GPU_ROUTING
 *   0x10 LOCAL_PM_ROUTING_SET
 *   0x14 LOCAL_PM_ROUTING_CLR
 *   0x40 + 4*cpu  LOCAL_TIMER_INT_CONTROLn  (per-cpu; bits0-3 = IRQ enable
 *                 for {CNTPS,CNTPNS,CNTHP,CNTV}IRQ; bits4-7 = FIQ enable)
 *   0x50 + 4*cpu  LOCAL_MAILBOX_INT_CONTROLn (per-cpu; bits0-3 = mailbox
 *                 0-3 IRQ enable; bits4-7 = FIQ enable)
 *   0x60 + 4*cpu  LOCAL_IRQ_PENDINGn (per-cpu; read-only status, bits per
 *                 LOCAL_IRQ_* below)
 *   0x70 + 4*cpu  LOCAL_FIQ_PENDINGn
 *   0x80 + 16*cpu + 4*mbox  LOCAL_MAILBOXn_SETm (write bits to raise doorbell)
 *   0xC0 + 16*cpu + 4*mbox  LOCAL_MAILBOXn_CLRm (write bits to ack/clear)
 *
 * LOCAL_IRQ_PENDING bit layout: 0=CNTPSIRQ 1=CNTPNSIRQ 2=CNTHPIRQ 3=CNTVIRQ
 * 4-7=MAILBOX0-3 8=GPU_FAST (peripheral IRQs routed from the separate legacy
 * Broadcom interrupt controller -- not yet wired here, see follow-up notes
 * at the bottom of this file) 9=PMU_FAST.
 *
 * We translate these local bit positions into the SAME intid numbering
 * space PIOS already uses on GIC (GIC_TIMER_NS_PHYS=30, GIC_TIMER_VIRT=27,
 * GIC_SGI_WAKE=9 -- see include/gic.h), so exception.c's dispatch table and
 * every existing caller (timer.c, proc.c, fifo.c, kernel.c) work completely
 * unchanged regardless of which controller is actually linked in.
 */
#include "gic.h"
#if !PIOS_HAS_GIC

#include "mmio.h"
#include "uart.h"

#define QA7_LOCAL_TIMER_INT_CONTROL0   0x40U
#define QA7_LOCAL_MAILBOX_INT_CONTROL0 0x50U
#define QA7_LOCAL_IRQ_PENDING0         0x60U
#define QA7_LOCAL_MAILBOX0_SET0        0x80U
#define QA7_LOCAL_MAILBOX0_CLR0        0xC0U

#define QA7_BIT_CNTPSIRQ   0U
#define QA7_BIT_CNTPNSIRQ  1U
#define QA7_BIT_CNTHPIRQ   2U
#define QA7_BIT_CNTVIRQ    3U
#define QA7_BIT_MAILBOX0   4U

static inline u64 qa7_reg(u32 off, u32 core)
{
    return QA7_BASE + off + (u64)(core * 4U);
}

/* sgi_id is used directly as the mailbox0 doorbell bit (0..31); PIOS only
 * ever sends GIC_SGI_WAKE (9), well within range. Unlike GIC's 4-bit SGI ID
 * space this has no hard 0..15 limit, but we keep callers using the same
 * GIC_SGI_WAKE constant so behaviour is identical either way. */

void gic_init(void)
{
    uart_puts("[irqc] legacy BCM local interrupt controller initialised (QA7 base=");
    uart_hex(QA7_BASE);
    uart_puts(")\n");
}

/* No multi-candidate-address probing on this platform -- QA7_BASE is a
 * single fixed, known-good address (unlike GIC-400 whose exact base can
 * vary across firmware/ATU remaps on Pi5). Kept as a harmless no-op so the
 * shared "irq probe"/"irq status" console commands still link and run. */
void gic_select_bases(u32 id, u64 gicd_base, u64 gicc_base)
{
    (void)id; (void)gicd_base; (void)gicc_base;
}

void gic_restore_default_bases(void)
{
}

u32 gic_runtime_id(void)
{
    return 1U;
}

u64 gic_runtime_gicd_base(void)
{
    return QA7_BASE;
}

u64 gic_runtime_gicc_base(void)
{
    return QA7_BASE;
}

/* The ARM generic-timer PPIs and the SGI wake doorbell are inherently
 * per-calling-core on real GIC too (PPI/SGI banked CPU interface), so we
 * target core_id() here exactly like gic_cpu_init()/gic_enable_irq() do
 * implicitly via the banked GICC/GICD registers on the real driver. */
void gic_enable_irq(u32 intid)
{
    u32 core = core_id() & 3U;
    u32 bit;
    if (intid == GIC_TIMER_NS_PHYS) bit = QA7_BIT_CNTPNSIRQ;
    else if (intid == GIC_TIMER_VIRT) bit = QA7_BIT_CNTVIRQ;
    else return; /* no peripheral/SPI routing on this platform yet */
    u32 v = mmio_read(qa7_reg(QA7_LOCAL_TIMER_INT_CONTROL0, core));
    mmio_write(qa7_reg(QA7_LOCAL_TIMER_INT_CONTROL0, core), v | (1U << bit));
}

void gic_disable_irq(u32 intid)
{
    u32 core = core_id() & 3U;
    u32 bit;
    if (intid == GIC_TIMER_NS_PHYS) bit = QA7_BIT_CNTPNSIRQ;
    else if (intid == GIC_TIMER_VIRT) bit = QA7_BIT_CNTVIRQ;
    else return;
    u32 v = mmio_read(qa7_reg(QA7_LOCAL_TIMER_INT_CONTROL0, core));
    mmio_write(qa7_reg(QA7_LOCAL_TIMER_INT_CONTROL0, core), v & ~(1U << bit));
}

/* No per-IRQ priority/target/group/edge concepts on the legacy local
 * controller -- each local IRQ line is already inherently per-core and
 * fixed-function (timer vs mailbox), so these are harmless no-ops kept
 * only so every existing GIC caller still links unchanged. */
void gic_set_priority(u32 intid, u8 priority) { (void)intid; (void)priority; }
void gic_set_target(u32 intid, u8 cpu_mask) { (void)intid; (void)cpu_mask; }
void gic_set_group1(u32 intid) { (void)intid; }
void gic_set_edge_triggered(u32 intid) { (void)intid; }
void gic_clear_pending(u32 intid) { (void)intid; }

u32 gic_acknowledge(void)
{
    u32 core = core_id() & 3U;
    u32 pending = mmio_read(qa7_reg(QA7_LOCAL_IRQ_PENDING0, core));
    if (pending & (1U << QA7_BIT_MAILBOX0))
        return GIC_SGI_WAKE;
    if (pending & (1U << QA7_BIT_CNTPNSIRQ))
        return GIC_TIMER_NS_PHYS;
    if (pending & (1U << QA7_BIT_CNTVIRQ))
        return GIC_TIMER_VIRT;
    return GIC_INTID_SPURIOUS;
}

void gic_end_of_interrupt(u32 iar_value)
{
    /* Timer IRQs need no controller-level ack: the pending bit tracks the
     * live ARM generic-timer comparator state, and timer.c's own handler
     * already reprograms CNTP_CTL_EL0/CNTP_TVAL_EL0 to clear that condition
     * (identical to the GIC path -- gic_end_of_interrupt() there doesn't
     * touch the timer peripheral either). Only the mailbox/SGI doorbell has
     * a controller-side latch that must be explicitly cleared. */
    if (iar_value == GIC_SGI_WAKE) {
        u32 core = core_id() & 3U;
        mmio_write(QA7_BASE + QA7_LOCAL_MAILBOX0_CLR0 + (u64)(core * 16U),
                   0xFFFFFFFFU);
    }
}

void gic_send_sgi(u8 target_mask, u32 sgi_id)
{
    if (sgi_id >= 32U)
        return;
    __asm__ volatile("dsb ish" ::: "memory");
    for (u32 core = 0; core < 4U; core++) {
        if (!(target_mask & (1U << core)))
            continue;
        mmio_write(QA7_BASE + QA7_LOCAL_MAILBOX0_SET0 + (u64)(core * 16U),
                   1U << sgi_id);
    }
}

/* Enable the CALLING core's local timer PPI line and mailbox0 doorbell.
 * Mirrors gic_cpu_init()'s role: gic_init() only touches core 0's state
 * (there is no shared distributor here either -- every register is
 * per-core), so secondary cores must call this once before they can
 * receive their own timer tick or a wake doorbell. */
void gic_cpu_init(void)
{
    u32 core = core_id() & 3U;
    /* Enable this core's mailbox0 IRQ line (bit0 of the per-core mailbox
     * interrupt-control register) so it can receive the wake doorbell, and
     * clear any stale doorbell bits left over from a warm reset before
     * unmasking -- mirrors gic_cpu_init()'s SGI/PPI pending-clear on GIC. */
    mmio_write(QA7_BASE + QA7_LOCAL_MAILBOX0_CLR0 + (u64)(core * 16U), 0xFFFFFFFFU);
    u32 v = mmio_read(qa7_reg(QA7_LOCAL_MAILBOX_INT_CONTROL0, core));
    mmio_write(qa7_reg(QA7_LOCAL_MAILBOX_INT_CONTROL0, core), v | 1U);
    __asm__ volatile("dsb sy; isb" ::: "memory");
}

#endif /* !PIOS_HAS_GIC */
