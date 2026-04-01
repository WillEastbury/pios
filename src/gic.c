/*
 * gic.c - ARM GIC-400 interrupt controller driver
 */

#include "gic.h"
#include "mmio.h"
#include "uart.h"
#include "fb.h"

void gic_init(void) {
    fb_puts("  [gic] Disabling distributor\n");
    /* Disable distributor during config */
    mmio_write(GICD_CTLR, 0);

    /* Find number of IRQ lines */
    u32 typer = mmio_read(GICD_TYPER);
    u32 num_irqs = ((typer & 0x1F) + 1) * 32;
    if (num_irqs > GIC_MAX_IRQ) num_irqs = GIC_MAX_IRQ;
    fb_printf("  [gic] Detected %u IRQ lines\n", num_irqs);

    fb_puts("  [gic] Disabling all interrupts\n");
    /* Disable all interrupts */
    for (u32 i = 0; i < num_irqs / 32; i++)
        mmio_write(GICD_ICENABLER(i), 0xFFFFFFFF);

    fb_puts("  [gic] Clearing all pending\n");
    /* Clear all pending */
    for (u32 i = 0; i < num_irqs / 32; i++)
        mmio_write(GICD_ICPENDR(i), 0xFFFFFFFF);

    fb_puts("  [gic] Setting default priorities\n");
    /* Set all priorities to a default (0xA0) */
    for (u32 i = 0; i < num_irqs / 4; i++)
        mmio_write(GICD_IPRIORITYR(i), 0xA0A0A0A0);

    fb_puts("  [gic] Targeting all SPIs to CPU 0\n");
    /* Target all SPIs to CPU 0 */
    for (u32 i = 8; i < num_irqs / 4; i++)
        mmio_write(GICD_ITARGETSR(i), 0x01010101);

    fb_puts("  [gic] Configuring SPIs level-triggered\n");
    /* All SPIs: level-triggered */
    for (u32 i = 2; i < num_irqs / 16; i++)
        mmio_write(GICD_ICFGR(i), 0x00000000);

    fb_puts("  [gic] Enabling distributor\n");
    /* Enable distributor */
    mmio_write(GICD_CTLR, 1);

    fb_puts("  [gic] Configuring CPU interface\n");
    /* CPU Interface */
    mmio_write(GICC_PMR, 0xFF);    /* Accept all priorities */
    mmio_write(GICC_CTLR, 1);      /* Enable CPU interface */

    fb_puts("  [gic] GIC-400 ready\n");
    uart_puts("[gic] GIC-400 initialised, ");
    uart_hex(num_irqs);
    uart_puts(" IRQ lines\n");
}

void gic_enable_irq(u32 intid) {
    u32 reg = intid / 32;
    u32 bit = intid % 32;
    mmio_write(GICD_ISENABLER(reg), 1 << bit);
}

void gic_disable_irq(u32 intid) {
    u32 reg = intid / 32;
    u32 bit = intid % 32;
    mmio_write(GICD_ICENABLER(reg), 1 << bit);
}

void gic_set_priority(u32 intid, u8 priority) {
    u32 reg = intid / 4;
    u32 shift = (intid % 4) * 8;
    u32 val = mmio_read(GICD_IPRIORITYR(reg));
    val &= ~(0xFF << shift);
    val |= ((u32)priority << shift);
    mmio_write(GICD_IPRIORITYR(reg), val);
}

void gic_set_target(u32 intid, u8 cpu_mask) {
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
