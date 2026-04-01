/*
 * rp1_gpio.c - RP1 GPIO driver
 *
 * Register layout (from pinctrl-rp1.c):
 *   IO_BANKn + pin*8 + 0x00 = STATUS (read-only)
 *   IO_BANKn + pin*8 + 0x04 = CTRL   (funcsel, overrides, IRQ enables)
 *   SYS_RIOn + 0x00 = OUT, +0x04 = OE, +0x08 = IN
 *   PADS_BANKn + 0x04 + pin*4 = PAD (slew, schmitt, pull, drive, IE, OD)
 *
 * Atomic set/clear via offset: +0x2000 = SET, +0x3000 = CLR
 *
 * Bank layout:
 *   Bank 0: GPIO  0-27, IO_BANK0 (0xD0000), RIO0 (0xE0000), PADS0 (0xF0000)
 *   Bank 1: GPIO 28-33, offsets +0x4000 within each block
 *   Bank 2: GPIO 34-53, offsets +0x8000 within each block
 */

#include "rp1_gpio.h"
#include "rp1.h"
#include "mmio.h"
#include "uart.h"
#include "fb.h"

/* Atomic access offsets */
#define SET_OFFSET  0x2000
#define CLR_OFFSET  0x3000

/* Per-pin GPIO register offsets */
#define GPIO_STATUS 0x00
#define GPIO_CTRL   0x04

/* CTRL field masks */
#define CTRL_FUNCSEL_MASK   0x0000001F
#define CTRL_OUTOVER_MASK   0x00003000
#define CTRL_OUTOVER_LSB    12
#define CTRL_OEOVER_MASK    0x0000C000
#define CTRL_OEOVER_LSB     14

/* Override values */
#define OUTOVER_PERI    0
#define OEOVER_PERI     0
#define OEOVER_DISABLE  2
#define OEOVER_ENABLE   3

/* RIO register offsets */
#define RIO_OUT 0x00
#define RIO_OE  0x04
#define RIO_IN  0x08

/* PAD register fields */
#define PAD_SLEWFAST_MASK   0x01
#define PAD_SCHMITT_MASK    0x02
#define PAD_PULL_MASK       0x0C
#define PAD_PULL_LSB        2
#define PAD_DRIVE_MASK      0x30
#define PAD_DRIVE_LSB       4
#define PAD_IN_ENABLE       0x40
#define PAD_OUT_DISABLE     0x80

/* ---- Bank geometry ---- */

struct gpio_bank {
    u64 io_base;    /* IO_BANKn offset from RP1_BAR_BASE */
    u64 rio_base;   /* SYS_RIOn offset */
    u64 pads_base;  /* PADS_BANKn offset */
    u32 base_pin;   /* first GPIO in this bank */
    u32 num_pins;
};

static const struct gpio_bank banks[3] = {
    { RP1_IO_BANK0, RP1_SYS_RIO0, RP1_PADS_BANK0,  0, 28 },
    { RP1_IO_BANK1, RP1_SYS_RIO1, RP1_PADS_BANK1, 28,  6 },
    { RP1_IO_BANK2, RP1_SYS_RIO2, RP1_PADS_BANK2, 34, 20 },
};

static const struct gpio_bank *pin_bank(u32 pin) {
    if (pin < 28) return &banks[0];
    if (pin < 34) return &banks[1];
    if (pin < 54) return &banks[2];
    return NULL;
}

static u32 pin_offset(u32 pin, const struct gpio_bank *b) {
    return pin - b->base_pin;
}

/* ---- Register helpers ---- */

static inline u32 rr(u64 off) { return mmio_read(RP1_BAR_BASE + off); }
static inline void rw(u64 off, u32 val) { mmio_write(RP1_BAR_BASE + off, val); }

/* ---- Public API ---- */

void rp1_gpio_init(void) {
    fb_puts("  [gpio] Initialising 54 GPIOs (3 banks)\n");
    fb_puts("  [gpio] Bank0: GPIO 0-27, Bank1: GPIO 28-33, Bank2: GPIO 34-53\n");
    uart_puts("[rp1_gpio] 54 GPIOs (3 banks) ready\n");
    fb_puts("  [gpio] GPIO controller ready\n");
}

void rp1_gpio_set_function(u32 pin, u32 fsel) {
    const struct gpio_bank *b = pin_bank(pin);
    if (!b) return;
    u32 off = pin_offset(pin, b);

    /* Enable input and output on pad */
    u64 pad_addr = b->pads_base + 0x04 + off * 4;
    u32 pad = rr(pad_addr);
    pad |= PAD_IN_ENABLE;
    pad &= ~PAD_OUT_DISABLE;
    rw(pad_addr, pad);

    /* Set function select + output enable override */
    u64 ctrl_addr = b->io_base + off * 8 + GPIO_CTRL;
    u32 ctrl = rr(ctrl_addr);
    ctrl &= ~CTRL_FUNCSEL_MASK;
    ctrl |= (fsel & CTRL_FUNCSEL_MASK);

    if (fsel == RP1_FSEL_NONE) {
        ctrl = (ctrl & ~CTRL_OEOVER_MASK) | (OEOVER_DISABLE << CTRL_OEOVER_LSB);
    } else {
        ctrl = (ctrl & ~CTRL_OUTOVER_MASK) | (OUTOVER_PERI << CTRL_OUTOVER_LSB);
        ctrl = (ctrl & ~CTRL_OEOVER_MASK) | (OEOVER_PERI << CTRL_OEOVER_LSB);
    }
    rw(ctrl_addr, ctrl);
}

u32 rp1_gpio_get_function(u32 pin) {
    const struct gpio_bank *b = pin_bank(pin);
    if (!b) return RP1_FSEL_NONE;
    u32 off = pin_offset(pin, b);
    u32 ctrl = rr(b->io_base + off * 8 + GPIO_CTRL);
    return ctrl & CTRL_FUNCSEL_MASK;
}

void rp1_gpio_set_dir_output(u32 pin) {
    const struct gpio_bank *b = pin_bank(pin);
    if (!b) return;
    u32 off = pin_offset(pin, b);
    rw(b->rio_base + RIO_OE + SET_OFFSET, 1U << off);
}

void rp1_gpio_set_dir_input(u32 pin) {
    const struct gpio_bank *b = pin_bank(pin);
    if (!b) return;
    u32 off = pin_offset(pin, b);
    rw(b->rio_base + RIO_OE + CLR_OFFSET, 1U << off);
}

void rp1_gpio_write(u32 pin, bool val) {
    const struct gpio_bank *b = pin_bank(pin);
    if (!b) return;
    u32 off = pin_offset(pin, b);
    u64 addr = b->rio_base + RIO_OUT + (val ? SET_OFFSET : CLR_OFFSET);
    rw(addr, 1U << off);
}

bool rp1_gpio_read(u32 pin) {
    const struct gpio_bank *b = pin_bank(pin);
    if (!b) return false;
    u32 off = pin_offset(pin, b);
    return (rr(b->rio_base + RIO_IN) >> off) & 1;
}

void rp1_gpio_set_pull(u32 pin, u32 pull) {
    const struct gpio_bank *b = pin_bank(pin);
    if (!b) return;
    u32 off = pin_offset(pin, b);
    u64 pad_addr = b->pads_base + 0x04 + off * 4;
    u32 pad = rr(pad_addr);
    pad = (pad & ~PAD_PULL_MASK) | ((pull & 0x3) << PAD_PULL_LSB);
    rw(pad_addr, pad);
}

void rp1_gpio_set_drive(u32 pin, u32 drive) {
    const struct gpio_bank *b = pin_bank(pin);
    if (!b) return;
    u32 off = pin_offset(pin, b);
    u64 pad_addr = b->pads_base + 0x04 + off * 4;
    u32 pad = rr(pad_addr);
    pad = (pad & ~PAD_DRIVE_MASK) | ((drive & 0x3) << PAD_DRIVE_LSB);
    rw(pad_addr, pad);
}
