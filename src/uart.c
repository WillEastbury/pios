/*
 * uart.c - PL011 UART driver (TX + RX)
 * Pi 5 firmware pre-configures UART0 on GPIO14/15.
 * BCM2712-native UART — no PCIe / RP1 required.
 */

#include "uart.h"
#include "mmio.h"
#include "fb.h"

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
    fb_puts("  [uart] Disabling PL011\n");
    mmio_write(UART_CR, 0);
    mmio_write(UART_ICR, 0x7FF);

    fb_puts("  [uart] Setting baud 115200 (48MHz clock)\n");
    /* 115200 baud @ 48MHz UART clock
     * IBRD = 26, FBRD = 3 */
    mmio_write(UART_IBRD, 26);
    mmio_write(UART_FBRD, 3);

    fb_puts("  [uart] Configuring 8N1 + FIFO\n");
    /* 8N1, enable FIFO */
    mmio_write(UART_LCRH, (3 << 5) | (1 << 4));

    fb_puts("  [uart] Enabling UART + TX + RX\n");
    /* Enable UART + TX + RX */
    mmio_write(UART_CR, (1 << 0) | (1 << 8) | (1 << 9));
    fb_puts("  [uart] PL011 ready\n");
}

void uart_putc(char c) {
    while (mmio_read(UART_FR) & FR_TXFF)
        ;
    mmio_write(UART_DR, (u32)c);
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
    return !(mmio_read(UART_FR) & FR_RXFE);
}

char uart_getc(void) {
    while (mmio_read(UART_FR) & FR_RXFE)
        ;
    return (char)(mmio_read(UART_DR) & 0xFF);
}

i32 uart_try_getc(void) {
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
