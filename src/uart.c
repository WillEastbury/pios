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
#include "platform.h"

/* Use RP1 UART0 for GPIO14/15 serial header */
static bool use_rp1;

#define UART_DIAG_VERBOSE 0
#define UART_VT_UTF8_BOXES 1

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
#if PIOS_HAS_RP1
    /* Debug: dump GPIO14/15 config to serial to see firmware state */
    volatile u32 *gpio14_ctrl = (volatile u32 *)(0x1F000D0000UL + 14 * 8 + 0x04);
    volatile u32 *gpio15_ctrl = (volatile u32 *)(0x1F000D0000UL + 15 * 8 + 0x04);
    volatile u32 *pad14 = (volatile u32 *)(0x1F000F0000UL + 0x04 + 14 * 4);
    volatile u32 *pad15 = (volatile u32 *)(0x1F000F0000UL + 0x04 + 15 * 4);
    volatile u32 *uart_cr = (volatile u32 *)(0x1F00030000UL + 0x30);

#if UART_DIAG_VERBOSE
    /* Print firmware state before we touch anything */
    uart_puts("[uart] FW: G14 ctrl=");
    uart_hex(*gpio14_ctrl);
    uart_puts(" pad=");
    uart_hex(*pad14);
    uart_puts("\r\n[uart] FW: G15 ctrl=");
    uart_hex(*gpio15_ctrl);
    uart_puts(" pad=");
    uart_hex(*pad15);
    uart_puts(" CR=");
    uart_hex(*uart_cr);
    uart_puts("\r\n");
#endif
    (void)gpio14_ctrl;
    (void)pad14;

    /* GPIO15 RX: explicit ALT4 + pull-up-enable + pull-down-clear +
     * input-enable, per Circle's proven Pi5 UART research (commit 20ec8f4).
     * A later "debug" commit (acfcc67) replaced this with a "copy GPIO14's
     * pad/funcsel config to GPIO15" strategy that silently DROPPED the
     * pull-up-enable (PUE) bit: GPIO14 is an output (TX) pin and has no
     * reason to carry pull-up/pull-down configuration, so blindly copying
     * its pad bits onto GPIO15 (an input/RX pin, which genuinely needs a
     * pull-up to avoid a floating line) left RX non-functional -- this was a
     * real regression, not a working alternative approach. Restored the
     * explicit, known-good configuration here. */
    u32 ctrl = *gpio15_ctrl;
    ctrl = (ctrl & ~0x1FU) | 4U;  /* funcsel = 4 (ALT4 = UART0 RX) */
    *gpio15_ctrl = ctrl;

    u32 pad = *pad15;
    pad |= (1U << 6);   /* IE  — input enable */
    pad |= (1U << 3);   /* PUE — pull-up enable */
    pad &= ~(1U << 2);  /* PDE — clear pull-down */
    *pad15 = pad;

    /* Enable RXE in UART control register */
    *uart_cr = *uart_cr | (1 << 9);

    /* Enable FIFO in LCRH (bit 4) — preserves firmware baud rate */
    volatile u32 *uart_lcrh = (volatile u32 *)(0x1F00030000UL + 0x2C);
    *uart_lcrh = *uart_lcrh | (1 << 4);

#if UART_DIAG_VERBOSE
    /* Print state after config */
    uart_puts("[uart] SET: G15 ctrl=");
    uart_hex(*gpio15_ctrl);
    uart_puts(" pad=");
    uart_hex(*pad15);
    uart_puts(" CR=");
    uart_hex(*uart_cr);
    uart_puts("\r\n");
#endif

    use_rp1 = true;
#else
    use_rp1 = false;
#endif
}

/* Mirror all uart output to HDMI framebuffer when enabled */
static bool hdmi_mirror;
static bool hdmi_scrolling;  /* guard against DMA scroll re-entry */

void uart_enable_hdmi_mirror(void) { hdmi_mirror = true; }

void uart_putc(char c) {
    if (use_rp1) {
        rp1_uart_putc(0, c);
    } else {
        while (mmio_read(UART_FR) & FR_TXFF) ;
        mmio_write(UART_DR, (u32)c);
    }
    if (hdmi_mirror && c != '\r' && !hdmi_scrolling) {
        hdmi_scrolling = true;
        fb_putc(c);
        hdmi_scrolling = false;
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

static void uart_put_dec(u32 v)
{
    char tmp[10];
    u32 n = 0;
    if (v == 0) {
        uart_putc('0');
        return;
    }
    while (v && n < sizeof(tmp)) {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (n)
        uart_putc(tmp[--n]);
}

void uart_vt_reset(void) { uart_puts("\x1b[0m"); }
void uart_vt_clear(void) { uart_puts("\x1b[2J"); }
void uart_vt_home(void) { uart_puts("\x1b[H"); }

void uart_vt_move(u32 row, u32 col)
{
    uart_puts("\x1b[");
    uart_put_dec(row);
    uart_putc(';');
    uart_put_dec(col);
    uart_putc('H');
}

void uart_vt_color(u8 fg, u8 bg, bool bright)
{
    uart_puts("\x1b[");
    uart_putc(bright ? '1' : '0');
    uart_putc(';');
    uart_put_dec(30U + (fg & 7U));
    uart_putc(';');
    uart_put_dec(40U + (bg & 7U));
    uart_putc('m');
}

void uart_vt_title(const char *title)
{
    uart_puts("\x1b]0;");
    if (title)
        uart_puts(title);
    uart_putc('\a');
}

void uart_vt_bell(void) { uart_putc('\a'); }

#if UART_VT_UTF8_BOXES
static const char *VT_TL = "\xE2\x94\x8C";
static const char *VT_TR = "\xE2\x94\x90";
static const char *VT_BL = "\xE2\x94\x94";
static const char *VT_BR = "\xE2\x94\x98";
static const char *VT_H  = "\xE2\x94\x80";
static const char *VT_V  = "\xE2\x94\x82";
#else
static const char *VT_TL = "+";
static const char *VT_TR = "+";
static const char *VT_BL = "+";
static const char *VT_BR = "+";
static const char *VT_H  = "-";
static const char *VT_V  = "|";
#endif

void uart_vt_hline(u32 n)
{
    for (u32 i = 0; i < n; i++)
        uart_puts(VT_H);
}

void uart_vt_box(u32 width, u32 height, const char *title)
{
    if (width < 4 || height < 2)
        return;
    uart_puts(VT_TL);
    uart_vt_hline(width - 2);
    uart_puts(VT_TR);
    uart_putc('\n');
    for (u32 row = 1; row + 1 < height; row++) {
        uart_puts(VT_V);
        if (row == 1 && title) {
            uart_putc(' ');
            u32 t = 0;
            while (title[t] && t + 3 < width) {
                uart_putc(title[t]);
                t++;
            }
            while (t + 3 < width) {
                uart_putc(' ');
                t++;
            }
            uart_putc(' ');
        } else {
            for (u32 col = 0; col < width - 2; col++)
                uart_putc(' ');
        }
        uart_puts(VT_V);
        uart_putc('\n');
    }
    uart_puts(VT_BL);
    uart_vt_hline(width - 2);
    uart_puts(VT_BR);
    uart_putc('\n');
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
