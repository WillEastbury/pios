#include "types.h"

#define PERIPH_BASE   0x107C000000UL
#define UART0_BASE    (PERIPH_BASE + 0x201000)
#define MBOX_BASE     (PERIPH_BASE + 0x00B880)

#define UART_DR     (UART0_BASE + 0x00)
#define UART_FR     (UART0_BASE + 0x18)
#define UART_IBRD   (UART0_BASE + 0x24)
#define UART_FBRD   (UART0_BASE + 0x28)
#define UART_LCRH   (UART0_BASE + 0x2C)
#define UART_CR     (UART0_BASE + 0x30)
#define UART_ICR    (UART0_BASE + 0x44)

#define MBOX_READ   (MBOX_BASE + 0x00)
#define MBOX_STATUS (MBOX_BASE + 0x18)
#define MBOX_WRITE  (MBOX_BASE + 0x20)
#define MBOX_FULL   0x80000000U
#define MBOX_EMPTY  0x40000000U
#define MBOX_CH_PROP 8U

#define TAG_SET_PHYS_WH      0x00048003
#define TAG_SET_VIRT_WH      0x00048004
#define TAG_SET_DEPTH        0x00048005
#define TAG_SET_PIXEL_ORDER  0x00048006
#define TAG_ALLOCATE_BUFFER  0x00040001
#define TAG_GET_PITCH        0x00040008
#define TAG_END              0x00000000

static inline void mmio_write(u64 addr, u32 val) { *(volatile u32 *)addr = val; }
static inline u32  mmio_read(u64 addr) { return *(volatile u32 *)addr; }

static void uart_init_raw(void)
{
    mmio_write(UART_CR, 0);
    mmio_write(UART_ICR, 0x7FF);
    mmio_write(UART_IBRD, 26);
    mmio_write(UART_FBRD, 3);
    mmio_write(UART_LCRH, (3U << 5) | (1U << 4));
    mmio_write(UART_CR, (1U << 0) | (1U << 8) | (1U << 9));
}

static void uart_putc_raw(char c)
{
    while (mmio_read(UART_FR) & (1U << 5)) {}
    mmio_write(UART_DR, (u32)c);
}

static void uart_puts_raw(const char *s)
{
    while (*s) {
        if (*s == '\n') uart_putc_raw('\r');
        uart_putc_raw(*s++);
    }
}

static volatile u32 mbox[36] ALIGNED(16);

static bool mailbox_call(u8 ch)
{
    u32 addr = ((u32)(usize)mbox & ~0xFU) | (u32)(ch & 0xFU);
    while (mmio_read(MBOX_STATUS) & MBOX_FULL) {}
    mmio_write(MBOX_WRITE, addr);
    for (;;) {
        while (mmio_read(MBOX_STATUS) & MBOX_EMPTY) {}
        u32 r = mmio_read(MBOX_READ);
        if (r == addr) return mbox[1] == 0x80000000U;
    }
}

static bool fb_try_green(void)
{
    mbox[0] = 35 * 4;
    mbox[1] = 0;
    mbox[2] = TAG_SET_PHYS_WH; mbox[3] = 8; mbox[4] = 0; mbox[5] = 1280; mbox[6] = 720;
    mbox[7] = TAG_SET_VIRT_WH; mbox[8] = 8; mbox[9] = 0; mbox[10] = 1280; mbox[11] = 720;
    mbox[12] = TAG_SET_DEPTH; mbox[13] = 4; mbox[14] = 0; mbox[15] = 32;
    mbox[16] = TAG_SET_PIXEL_ORDER; mbox[17] = 4; mbox[18] = 0; mbox[19] = 0;
    mbox[20] = TAG_ALLOCATE_BUFFER; mbox[21] = 8; mbox[22] = 0; mbox[23] = 4096; mbox[24] = 0;
    mbox[25] = TAG_GET_PITCH; mbox[26] = 4; mbox[27] = 0; mbox[28] = 0;
    mbox[29] = TAG_END;
    if (!mailbox_call(MBOX_CH_PROP) || mbox[23] == 0) return false;
    u32 *fb = (u32 *)(usize)(mbox[23] & 0x3FFFFFFFU);
    u32 pitch = mbox[28] / 4U;
    for (u32 y = 0; y < 720; y++) {
        for (u32 x = 0; x < 1280; x++) fb[y * pitch + x] = 0x0000FF00U;
    }
    return true;
}

void canary_main(void)
{
    bool fb = fb_try_green();
    /* Only touch UART after framebuffer attempt to avoid early UART faults masking HDMI signal. */
    uart_init_raw();
    uart_puts_raw("\nCANARY: start\n");
    uart_puts_raw(fb ? "CANARY: fb ok\n" : "CANARY: fb fail\n");
    for (;;) {
        for (volatile u32 i = 0; i < 8000000U; i++) {}
        uart_putc_raw('.');
    }
}
