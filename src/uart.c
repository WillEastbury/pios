/*
 * uart.c - PL011 UART driver (TX + RX)
 * On Pi 5, GPIO14/15 serial is on RP1 UART0, not the BCM2712 SoC UART.
 * After RP1 init, we redirect all I/O to the RP1 UART.
 */

#include "uart.h"
#include "rp1_uart.h"
#include "rp1_gpio.h"
#include "mmio.h"
#include "fb.h"

/* Use RP1 UART0 for GPIO14/15 serial header */
static bool use_rp1;

#define UART_DR     (UART0_BASE + 0x00)
#define UART_FR     (UART0_BASE + 0x18)
#define UART_IBRD   (UART0_BASE + 0x24)
#define UART_FBRD   (UART0_BASE + 0x28)
#define UART_LCRH   (UART0_BASE + 0x2C)
#define UART_CR     (UART0_BASE + 0x30)
#define UART_ICR    (UART0_BASE + 0x44)

#define FR_TXFF     (1 << 5)    /* TX FIFO full */
#define FR_RXFE     (1 << 4)    /* RX FIFO empty */

void uart_init(void) {
    /* Pi 5: GPIO14 TX = ALT4, GPIO15 RX = ALT4 (from Circle) */
    rp1_gpio_set_function(14, RP1_FSEL_ALT4);
    rp1_gpio_set_function(15, RP1_FSEL_ALT4);

    /* GPIO15 RX needs pull-up (Circle: SetPullMode(GPIOPullModeUp)) */
    {
        u64 pad15 = 0x1F000F0000UL + 0x04 + 15 * 4;  /* PADS_BANK0 + pin offset */
        u32 pad = *(volatile u32 *)pad15;
        pad |= (1 << 3);   /* PUE — pull-up enable */
        pad &= ~(1 << 2);  /* PDE — clear pull-down */
        pad |= (1 << 6);   /* IE  — input enable */
        *(volatile u32 *)pad15 = pad;
    }

    /* Just enable RXE without touching baud/LCRH */
    u64 cr_addr = 0x1F00030000UL + 0x30;
    u32 cr = *(volatile u32 *)cr_addr;
    cr |= (1 << 9);  /* RXE */
    *(volatile u32 *)cr_addr = cr;

    use_rp1 = true;
}

void uart_putc(char c) {
    if (use_rp1) {
        rp1_uart_putc(0, c);
    } else {
        while (mmio_read(UART_FR) & FR_TXFF) ;
        mmio_write(UART_DR, (u32)c);
    }
}

void uart_puts(const char *s) {
    while (*s) {
        if (*s == '\n')
            uart_putc('\r');
        uart_putc(*s++);
    }
}

void uart_hex(u64 val) {
    static const char hex[] = "0123456789ABCDEF";
    uart_puts("0x");
    for (int i = 60; i >= 0; i -= 4)
        uart_putc(hex[(val >> i) & 0xF]);
}

/* ---- RX ---- */

bool uart_rx_ready(void) {
    if (use_rp1) {
        /* Check RP1 UART0 flag register directly — don't consume data */
        return !(mmio_read(RP1_BAR_BASE + 0x030000 + 0x18) & (1 << 4));
    }
    return !(mmio_read(UART_FR) & FR_RXFE);
}

char uart_getc(void) {
    if (use_rp1)
        return rp1_uart_getc(0);
    while (mmio_read(UART_FR) & FR_RXFE) ;
    return (char)(mmio_read(UART_DR) & 0xFF);
}

i32 uart_try_getc(void) {
    if (use_rp1)
        return rp1_uart_try_getc(0);
    if (mmio_read(UART_FR) & FR_RXFE)
        return -1;
    return (i32)(mmio_read(UART_DR) & 0xFF);
}

u32 uart_getline(char *buf, u32 max) {
    u32 i = 0;
    while (i < max - 1) {
        char c = uart_getc();
        if (c == '\r' || c == '\n') {
            uart_puts("\r\n");
            break;
        }
        if (c == 0x7F || c == '\b') {   /* backspace / DEL */
            if (i > 0) {
                i--;
                uart_puts("\b \b");
            }
            continue;
        }
        if (c >= 0x20) {
            buf[i++] = c;
            uart_putc(c);   /* echo */
        }
    }
    buf[i] = '\0';
    return i;
}
