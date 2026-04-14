/*
 * gic.c - ARM GIC-400 interrupt controller driver
 */

#include "gic.h"
#include "mmio.h"
#include "uart.h"
#include "fb.h"

void gic_init(void) {
    /* Disable distributor during config */
    mmio_write(GICD_CTLR, 0);

    /* Find number of IRQ lines */
    u32 typer = mmio_read(GICD_TYPER);
    u32 num_irqs = ((typer & 0x1F) + 1) * 32;
    if (num_irqs > GIC_MAX_IRQ) num_irqs = GIC_MAX_IRQ;

    /* Disable all interrupts */
    /* Disable all interrupts */
    for (u32 i = 0; i < num_irqs / 32; i++)
        mmio_write(GICD_ICENABLER(i), 0xFFFFFFFF);

    /* Clear all pending */
    for (u32 i = 0; i < num_irqs / 32; i++)
        mmio_write(GICD_ICPENDR(i), 0xFFFFFFFF);

    /* Set all priorities to a default (0xA0) */
    for (u32 i = 0; i < num_irqs / 4; i++)
        mmio_write(GICD_IPRIORITYR(i), 0xA0A0A0A0);

    /* Target all SPIs to CPU 0 */
    for (u32 i = 8; i < num_irqs / 4; i++)
        mmio_write(GICD_ITARGETSR(i), 0x01010101);

    /* All SPIs: level-triggered */
    for (u32 i = 2; i < num_irqs / 16; i++)
        mmio_write(GICD_ICFGR(i), 0x00000000);

    /* Enable distributor */
    mmio_write(GICD_CTLR, 1);

    /* CPU Interface */
    mmio_write(GICC_PMR, 0xFF);    /* Accept all priorities */
    mmio_write(GICC_CTLR, 1);      /* Enable CPU interface */

    uart_puts("[gic] GIC-400 initialised, ");
    uart_hex(num_irqs);
    uart_puts(" IRQ lines\n");
}

void gic_enable_irq(u32 intid) {
    if (intid >= 1020) return;
    u32 reg = intid / 32;
    u32 bit = intid % 32;
    mmio_write(GICD_ISENABLER(reg), 1U << bit);
}

void gic_disable_irq(u32 intid) {
    if (intid >= 1020) return;
    u32 reg = intid / 32;
    u32 bit = intid % 32;
    mmio_write(GICD_ICENABLER(reg), 1U << bit);
}

void gic_set_priority(u32 intid, u8 priority) {
    if (intid >= 1020) return;
    u32 reg = intid / 4;
    u32 shift = (intid % 4) * 8;
    u32 val = mmio_read(GICD_IPRIORITYR(reg));
    val &= ~(0xFF << shift);
    val |= ((u32)priority << shift);
    mmio_write(GICD_IPRIORITYR(reg), val);
}

void gic_set_target(u32 intid, u8 cpu_mask) {
    if (intid >= 1020) return;
    u32 reg = intid / 4;
    u32 shift = (intid % 4) * 8;
    u32 val = mmio_read(GICD_ITARGETSR(reg));
    val &= ~(0xFF << shift);
    val |= ((u32)cpu_mask << shift);
    mmio_write(GICD_ITARGETSR(reg), val);
}

u32 gic_acknowledge(void) {
    return mmio_read(GICC_IAR) & 0x3FF;
}

void gic_end_of_interrupt(u32 intid) {
    mmio_write(GICC_EOIR, intid);
}
