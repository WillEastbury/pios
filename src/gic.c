/*
 * gic.c - ARM GIC-400 interrupt controller driver
 */

#include "gic.h"
#include "mmio.h"
#include "uart.h"
#include "fb.h"

static u64 gicd_runtime_base = GICD_BASE;
static u64 gicc_runtime_base = GICC_BASE;
static u32 gic_runtime_base_id = 1;

static u64 gicd_reg(u32 off)
{
    return gicd_runtime_base + off;
}

static u64 gicc_reg(u32 off)
{
    return gicc_runtime_base + off;
}

void gic_select_bases(u32 id, u64 gicd_base, u64 gicc_base)
{
    gic_runtime_base_id = id;
    gicd_runtime_base = gicd_base;
    gicc_runtime_base = gicc_base;
}

void gic_restore_default_bases(void)
{
    gic_select_bases(1, GICD_BASE, GICC_BASE);
}

u32 gic_runtime_id(void)
{
    return gic_runtime_base_id;
}

u64 gic_runtime_gicd_base(void)
{
    return gicd_runtime_base;
}

u64 gic_runtime_gicc_base(void)
{
    return gicc_runtime_base;
}

void gic_init(void) {
    u32 old_c_ctlr = mmio_read(gicc_reg(0x000));

    /* Disable distributor during config */
    mmio_write(gicd_reg(0x000), 0);

    /* Find number of IRQ lines */
    u32 typer = mmio_read(gicd_reg(0x004));
    u32 num_irqs = ((typer & 0x1F) + 1) * 32;
    if (num_irqs > GIC_MAX_IRQ) num_irqs = GIC_MAX_IRQ;

    /* Disable all interrupts */
    /* Disable all interrupts */
    for (u32 i = 0; i < num_irqs / 32; i++)
        mmio_write(gicd_reg(0x180 + i * 4), 0xFFFFFFFF);

    /* Clear all pending */
    for (u32 i = 0; i < num_irqs / 32; i++)
        mmio_write(gicd_reg(0x280 + i * 4), 0xFFFFFFFF);

    /* Set all priorities to a default (0xA0) */
    for (u32 i = 0; i < num_irqs / 4; i++)
        mmio_write(gicd_reg(0x400 + i * 4), 0xA0A0A0A0);

    /* Target all SPIs to CPU 0 */
    for (u32 i = 8; i < num_irqs / 4; i++)
        mmio_write(gicd_reg(0x800 + i * 4), 0x01010101);

    /* All SPIs: level-triggered */
    for (u32 i = 2; i < num_irqs / 16; i++)
        mmio_write(gicd_reg(0xC00 + i * 4), 0x00000000);

    /* Enable distributor */
    mmio_write(gicd_reg(0x000), 1);

    /* CPU Interface. Preserve firmware-programmed bypass/security bits; on
     * Pi 5 ATF leaves the GICC_CTLR at 0x60, and the validated live path is
     * old_c_ctlr|1 == 0x61. */
    mmio_write(gicc_reg(0x004), 0xF0);    /* Accept normal kernel IRQ priority */
    mmio_write(gicc_reg(0x000), old_c_ctlr | 1U);

    uart_puts("[gic] GIC-400 initialised id=");
    uart_hex(gic_runtime_base_id);
    uart_puts(" lines=");
    uart_hex(num_irqs);
    uart_puts("\n");
}

void gic_enable_irq(u32 intid) {
    if (intid >= 1020) return;
    u32 reg = intid / 32;
    u32 bit = intid % 32;
    mmio_write(gicd_reg(0x100 + reg * 4), 1U << bit);
}

void gic_disable_irq(u32 intid) {
    if (intid >= 1020) return;
    u32 reg = intid / 32;
    u32 bit = intid % 32;
    mmio_write(gicd_reg(0x180 + reg * 4), 1U << bit);
}

void gic_set_priority(u32 intid, u8 priority) {
    if (intid >= 1020) return;
    u32 reg = intid / 4;
    u32 shift = (intid % 4) * 8;
    u32 val = mmio_read(gicd_reg(0x400 + reg * 4));
    val &= ~(0xFF << shift);
    val |= ((u32)priority << shift);
    mmio_write(gicd_reg(0x400 + reg * 4), val);
}

void gic_set_group1(u32 intid) {
    if (intid >= 1020) return;
    u32 reg = intid / 32;
    u32 bit = intid % 32;
    mmio_write(gicd_reg(0x080 + reg * 4), 1U << bit);
}

void gic_set_edge_triggered(u32 intid) {
    if (intid >= 1020) return;
    u32 reg = intid / 16;
    u32 shift = (intid % 16) * 2;
    u32 val = mmio_read(gicd_reg(0xC00 + reg * 4));
    val &= ~(3U << shift);
    val |= (2U << shift);
    mmio_write(gicd_reg(0xC00 + reg * 4), val);
}

void gic_clear_pending(u32 intid) {
    if (intid >= 1020) return;
    u32 reg = intid / 32;
    u32 bit = intid % 32;
    mmio_write(gicd_reg(0x280 + reg * 4), 1U << bit);
}

void gic_set_target(u32 intid, u8 cpu_mask) {
    if (intid >= 1020) return;
    u32 reg = intid / 4;
    u32 shift = (intid % 4) * 8;
    u32 val = mmio_read(gicd_reg(0x800 + reg * 4));
    val &= ~(0xFF << shift);
    val |= ((u32)cpu_mask << shift);
    mmio_write(gicd_reg(0x800 + reg * 4), val);
}

u32 gic_acknowledge(void) {
    /* Return the full IAR value including CPUID bits. The caller masks
     * with 0x3FF to extract the intid for routing, but must pass the
     * unmodified IAR value back to gic_end_of_interrupt(). GIC-400 spec
     * says EOIR write must exactly match the last IAR read, otherwise
     * behaviour is UNPREDICTABLE (in practice the interrupt stays active
     * and re-fires forever). */
    return mmio_read(gicc_reg(0x00C));
}

void gic_end_of_interrupt(u32 iar_value) {
    mmio_write(gicc_reg(0x010), iar_value);
}
