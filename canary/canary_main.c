/*
 * canary_main.c - Pi 5 bare-metal UART test
 *
 * UART address: 0x1F00030000 (RP1 UART0 via firmware PCIe BAR)
 * Firmware reports: rp1_uart 0000001f00030000
 * Requires enable_rp1_uart=1 in config.txt
 */
typedef unsigned char uint8_t;
typedef unsigned int uint32_t;
typedef unsigned long uint64_t;

#define MMIO_BASE       0x1F00000000UL
#define PL011_BASE      (MMIO_BASE + 0x30000)

#define PL011_DR        0x00
#define PL011_FR        0x18
#define PL011_LCRH      0x2c
#define PL011_CR        0x30

#define PL011_FR_TXFF   (1 << 5)
#define PL011_LCRH_8BIT (3 << 5)
#define PL011_CR_UARTEN (1 << 0)
#define PL011_CR_TXE    (1 << 8)
#define PL011_CR_RXE    (1 << 9)

static inline void pl011_write(uint32_t offset, uint32_t val) {
    *(volatile uint32_t *)(PL011_BASE + offset) = val;
}
static inline uint32_t pl011_read(uint32_t offset) {
    return *(volatile uint32_t *)(PL011_BASE + offset);
}

static void pl011_init(void) {
    pl011_write(PL011_CR, 0);
    pl011_write(PL011_LCRH, PL011_LCRH_8BIT);
    pl011_write(PL011_CR, PL011_CR_UARTEN | PL011_CR_TXE | PL011_CR_RXE);
}

static void pl011_putc(uint8_t c) {
    while (pl011_read(PL011_FR) & PL011_FR_TXFF) {}
    pl011_write(PL011_DR, c);
}

static void pl011_puts(const char *s) {
    while (*s) {
        if (*s == '\n') pl011_putc('\r');
        pl011_putc((uint8_t)*s++);
    }
}

void main(void) {
    pl011_init();
    pl011_puts("PIOS CANARY ALIVE\n");

    while (1) {
        for (volatile uint32_t i = 0; i < 5000000; i++) {}
        pl011_putc('.');
    }
}
