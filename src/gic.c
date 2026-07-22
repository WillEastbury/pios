/*
 * gic.c - ARM GIC-400 interrupt controller driver
 *
 * Only compiled when the platform actually has a GIC-400 (PIOS_HAS_GIC=1 --
 * Pi5/QEMU/Hyper-V-ARM). BCM2837-family boards (Pi3, Pi Zero 2 W) have no
 * GIC at all; see src/irqc_legacy.c for that platform's implementation of
 * this exact same public API (gic_init/gic_send_sgi/gic_acknowledge/...),
 * backed by the legacy Broadcom local interrupt controller + QA7 ARM-local
 * peripherals block instead. Callers (exception.c, timer.c, proc.c, fifo.c,
 * kernel.c) never need to know which one is linked in.
 */
#include "gic.h"
#if PIOS_HAS_GIC

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

    /* Clear active state too. A watchdog warm reset can preserve GIC
     * distributor state; an SGI left active can never be delivered again. */
    for (u32 i = 0; i < num_irqs / 32; i++)
        mmio_write(gicd_reg(0x380 + i * 4), 0xFFFFFFFF);

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
    u32 val = mmio_read(gicd_reg(0x080 + reg * 4));
    mmio_write(gicd_reg(0x080 + reg * 4), val | (1U << bit));
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

void gic_send_sgi(u8 target_mask, u32 sgi_id) {
    if (sgi_id > 15U)
        return;
    /* Ensure any work posted to memory before the doorbell is globally visible
     * before the target core takes the SGI and re-scans its wake ring. */
    __asm__ volatile("dsb ish" ::: "memory");
    /* GICD_SGIR @ 0xF00: TargetListFilter[25:24]=00 (use explicit list),
     * CPUTargetList[23:16]=target_mask, SGIINTID[3:0]=sgi_id. From non-secure
     * EL1 the SGI is forced Group1-NS (same deliverable path as the timer PPI). */
    mmio_write(gicd_reg(0xF00), ((u32)target_mask << 16) | (sgi_id & 0xFU));
}

/* Enable the CALLING core's GIC CPU interface (banked GICC_CTLR/PMR). gic_init()
 * runs only on core 0 and enables only core 0's interface, so the secondary
 * cores never receive ANY interrupt (timer PPI, SGI, ...) until they call this.
 * Mirrors gic_init()'s CPU-interface bring-up: preserve firmware bypass/security
 * bits (ATF leaves GICC_CTLR ~0x60 on Pi 5; the validated enable is old|1). */
void gic_cpu_init(void) {
    u32 old_c_ctlr = mmio_read(gicc_reg(0x000));
    /* SGI/PPI pending and active registers are banked per CPU interface. */
    mmio_write(gicd_reg(0x280), 0xFFFFFFFFU);
    mmio_write(gicd_reg(0x380), 0xFFFFFFFFU);
    mmio_write(gicc_reg(0x004), 0xF0);          /* PMR: accept normal priorities */
    mmio_write(gicc_reg(0x000), old_c_ctlr | 1U);
    __asm__ volatile("dsb sy; isb" ::: "memory");
}

#endif /* PIOS_HAS_GIC */
