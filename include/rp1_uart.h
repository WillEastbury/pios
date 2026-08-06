/*
 * rp1_uart.h - RP1 PL011 UART driver
 *
 * The RP1 has 6 PL011-compatible UARTs (UART0-5).
 * UART0 on GPIOs 14/15 is commonly used for serial console on the header.
 *
 * Reference: rp1.dtsi serial@30000..serial@44000
 */

#pragma once
#include "types.h"

/* RP1 UART indices */
#define RP1_UART0   0
#define RP1_UART1   1
#define RP1_UART2   2
#define RP1_UART3   3
#define RP1_UART4   4
#define RP1_UART5   5

bool rp1_uart_init(u32 index, u32 baud);
void rp1_uart_putc(u32 index, char c);
void rp1_uart_puts(u32 index, const char *s);
i32  rp1_uart_try_getc(u32 index);
char rp1_uart_getc(u32 index);
