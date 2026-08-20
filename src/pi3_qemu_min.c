#include "types.h"
#include "mmio.h"
#include "platform.h"

#define UART_DR  (PIOS_UART0_BASE + 0x00)
#define UART_FR  (PIOS_UART0_BASE + 0x18)
#define UART_TXFF (1U << 5)

static void putc_uart(char c)
{
    if (c == '\n')
        putc_uart('\r');
    while (mmio_read(UART_FR) & UART_TXFF) { }
    mmio_write(UART_DR, (u32)c);
}

static void puts_uart(const char *s)
{
    while (*s)
        putc_uart(*s++);
}

void qemu_virt_main(void)
{
    puts_uart("\nPIOS qemu-pi3 minimal boot\n");
    puts_uart("platform=pi3-bcm2837\n");
    puts_uart("uart=0x3F201000\n");
    puts_uart("qemu-pi3 smoke boot complete; parking\n");
    for (;;)
        __asm__ volatile("wfe");
}
