/*
 * rp1_uart.c - RP1 PL011 UART driver
 *
 * 6 PL011-compatible UARTs at known offsets within the RP1 BAR.
 * On Pi 5, UART0 (GPIOs 14/15) is the default header serial port.
 *
 * We reuse the same PL011 register knowledge as the BCM2712 UART
 * driver (uart.c), but targeting the RP1 peripheral addresses.
 *
 * Reference: rp1.dtsi serial@30000..serial@44000
 *            ARM PL011 Technical Reference Manual
 */

#include "rp1_uart.h"
#include "mmio.h"
#include "timer.h"

/* UART base offsets within RP1 BAR (from rp1.dtsi) */
static const u64 uart_base[6] = {
    0x030000,   /* UART0 */
    0x034000,   /* UART1 */
    0x038000,   /* UART2 */
    0x03C000,   /* UART3 */
    0x040000,   /* UART4 */
    0x044000,   /* UART5 */
};

/* PL011 register offsets */
#define UART_DR         0x00    /* Data register */
#define UART_FR         0x18    /* Flag register */
#define UART_IBRD       0x24    /* Integer baud rate divisor */
#define UART_FBRD       0x28    /* Fractional baud rate divisor */
#define UART_LCRH       0x2C    /* Line control */
#define UART_CR         0x30    /* Control register */
#define UART_ICR        0x44    /* Interrupt clear */

/* Flag register bits */
#define FR_TXFF         (1U << 5)   /* TX FIFO full */
#define FR_RXFE         (1U << 4)   /* RX FIFO empty */
#define FR_BUSY         (1U << 3)

/* Control register bits */
#define CR_UARTEN       (1U << 0)
#define CR_TXE          (1U << 8)
#define CR_RXE          (1U << 9)

/* Line control: 8N1 + FIFO enable */
#define LCRH_WLEN8      (3U << 5)
#define LCRH_FEN        (1U << 4)

/* Register helpers */
static inline u64 ub(u32 index, u32 reg) {
    return RP1_BAR_BASE + uart_base[index] + reg;
}

static inline void uw(u32 index, u32 reg, u32 val) { mmio_write(ub(index, reg), val); }
static inline u32  ur(u32 index, u32 reg)           { return mmio_read(ub(index, reg)); }

bool rp1_uart_init(u32 index, u32 baud) {
    if (index > 5) return false;

    /* Disable UART */
    uw(index, UART_CR, 0);

    /* Clear pending interrupts */
    uw(index, UART_ICR, 0x7FF);

    /*
     * Baud rate divisor for PL011.
     * UART clock = 48MHz (firmware default for RP1 CLK_UART).
     * Divisor = UART_CLK / (16 * baud)
     * For 115200: 48000000 / (16 * 115200) = 26.041...
     *   IBRD = 26, FBRD = round(0.041 * 64) = 3
     */
    u32 uart_clk = 48000000;
    u32 div16 = (uart_clk * 4) / baud;     /* 4x fixed-point */
    u32 ibrd = div16 / 64;
    u32 fbrd = div16 % 64;

    uw(index, UART_IBRD, ibrd);
    uw(index, UART_FBRD, fbrd);

    /* 8N1, FIFO enabled */
    uw(index, UART_LCRH, LCRH_WLEN8 | LCRH_FEN);

    /* Enable UART + TX + RX */
    uw(index, UART_CR, CR_UARTEN | CR_TXE | CR_RXE);

    return true;
}

void rp1_uart_putc(u32 index, char c) {
    if (index > 5) return;
    /* Wait for TX FIFO not full */
    while (ur(index, UART_FR) & FR_TXFF)
        ;
    uw(index, UART_DR, (u32)c);
}

void rp1_uart_puts(u32 index, const char *s) {
    while (*s) {
        if (*s == '\n')
            rp1_uart_putc(index, '\r');
        rp1_uart_putc(index, *s++);
    }
}

i32 rp1_uart_try_getc(u32 index) {
    if (index > 5) return -1;
    if (ur(index, UART_FR) & FR_RXFE)
        return -1;
    return (i32)(ur(index, UART_DR) & 0xFF);
}

char rp1_uart_getc(u32 index) {
    while (ur(index, UART_FR) & FR_RXFE)
        ;
    return (char)(ur(index, UART_DR) & 0xFF);
}
