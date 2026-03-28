#pragma once
#include "types.h"

/* PL011 UART - pre-configured by Pi 5 firmware, TX + RX */
void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
void uart_hex(u64 val);

/* RX - polling (non-blocking) and blocking */
bool uart_rx_ready(void);   /* true if at least one byte available */
char uart_getc(void);        /* blocking: waits for a byte */
i32  uart_try_getc(void);    /* non-blocking: returns char or -1 */

/* Read a line into buf (with echo). Returns length. */
u32  uart_getline(char *buf, u32 max);
