/*
 * qemu_virt_min.c - standalone QEMU virt smoke boot.
 *
 * Not the full PIOS kernel yet: this proves the first non-Pi platform constants,
 * PL011 console, generic timer registers, and GICv2 MMIO visibility.
 */
#include "types.h"
#include "platform.h"
#include "mmio.h"

#define QEMU_UART_DR  (PIOS_UART0_BASE + 0x00)
#define QEMU_UART_FR  (PIOS_UART0_BASE + 0x18)
#define QEMU_UART_TXFF (1U << 5)

static void qemu_putc(char c)
{
    if (c == '\n')
        qemu_putc('\r');
    while (mmio_read(QEMU_UART_FR) & QEMU_UART_TXFF) { }
    mmio_write(QEMU_UART_DR, (u32)c);
}

static void qemu_puts(const char *s)
{
    while (*s)
        qemu_putc(*s++);
}

static void qemu_hex(u64 v)
{
    static const char h[] = "0123456789ABCDEF";
    qemu_puts("0x");
    for (i32 i = 60; i >= 0; i -= 4)
        qemu_putc(h[(v >> (u32)i) & 0xFULL]);
}

static u64 qemu_cntfrq(void)
{
    u64 v;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(v));
    return v;
}

static u64 qemu_cntvct(void)
{
    u64 v;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(v));
    return v;
}

static void qemu_delay_ticks(u64 ticks)
{
    u64 start = qemu_cntvct();
    while ((qemu_cntvct() - start) < ticks) { }
}

void qemu_virt_main(void)
{
    u64 el;
    __asm__ volatile("mrs %0, CurrentEL" : "=r"(el));
    qemu_puts("\nPIOS qemu-virt minimal boot\n");
    qemu_puts("platform=");
    qemu_puts(PIOS_PLATFORM_NAME);
    qemu_puts("\nCurrentEL=");
    qemu_hex((el >> 2) & 3U);
    qemu_puts("\nPL011=");
    qemu_hex(PIOS_UART0_BASE);
#if PIOS_PLATFORM == PIOS_PLATFORM_FVP_A76_GICV2
    /*
     * The Base RevC model's GICv2-compatibility MMIO map is probed
     * separately. Do not let an unresolved legacy-map access prevent the
     * CPU/EL/UART/timer smoke target from proving the architectural platform.
     */
    qemu_puts("\nGICv2 MMIO probe deferred\n");
#else
    qemu_puts("\nGICD_TYPER=");
    qemu_hex(mmio_read(PIOS_GICD_BASE + 0x004));
#endif
    qemu_puts("\nCNTFRQ=");
    qemu_hex(qemu_cntfrq());
    qemu_puts("\nCNTVCT=");
    qemu_hex(qemu_cntvct());
    qemu_puts("\n");

    u64 freq = qemu_cntfrq();
    for (u32 i = 0; i < 5; i++) {
        qemu_puts("tick ");
        qemu_hex(i);
        qemu_puts("\n");
        qemu_delay_ticks(freq / 2U);
    }
#if PIOS_PLATFORM == PIOS_PLATFORM_FVP_A76_GICV2
    qemu_puts("fvp-a76 smoke boot complete; parking\n");
#else
    qemu_puts("qemu-virt smoke boot complete; parking\n");
#endif
    for (;;)
        __asm__ volatile("wfe");
}
