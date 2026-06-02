#pragma once
#include "types.h"

/* PL011 UART - pre-configured by Pi 5 firmware, TX + RX */
void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
void uart_hex(u64 val);

/* ANSI/VT terminal helpers for serial consoles such as PuTTY. */
#define UART_COLOR_BLACK    0
#define UART_COLOR_RED      1
#define UART_COLOR_GREEN    2
#define UART_COLOR_YELLOW   3
#define UART_COLOR_BLUE     4
#define UART_COLOR_MAGENTA  5
#define UART_COLOR_CYAN     6
#define UART_COLOR_WHITE    7

void uart_vt_reset(void);
void uart_vt_clear(void);
void uart_vt_home(void);
void uart_vt_move(u32 row, u32 col);
void uart_vt_color(u8 fg, u8 bg, bool bright);
void uart_vt_bell(void);
void uart_vt_hline(u32 n);
void uart_vt_box(u32 width, u32 height, const char *title);

/* RX - polling (non-blocking) and blocking */
bool uart_rx_ready(void);   /* true if at least one byte available */
char uart_getc(void);        /* blocking: waits for a byte */
i32  uart_try_getc(void);    /* non-blocking: returns char or -1 */

/* Read a line into buf (with echo). Returns length. */
u32  uart_getline(char *buf, u32 max);
