/*
 * canary_main.c - Pi 5 UART + HDMI0 mailbox canary
 *
 * Prints READY through the firmware-configured RP1 UART before touching the
 * mailbox, then prints the same word into the allocated framebuffer.
 */
typedef unsigned char u8;
typedef unsigned int u32;
typedef unsigned long u64;

static inline void mmio_write(u64 addr, u32 val) {
    *(volatile u32 *)addr = val;
}
static inline u32 mmio_read(u64 addr) {
    return *(volatile u32 *)addr;
}

/* Firmware's pre-kernel RP1 BAR. PIOS remaps RP1 to 0x1F... only later. */
#define PL011_BASE      0x1C00030000UL
#define PL011_DR        (PL011_BASE + 0x00)
#define PL011_FR        (PL011_BASE + 0x18)
#define PL011_LCRH      (PL011_BASE + 0x2C)
#define PL011_CR        (PL011_BASE + 0x30)
#define PL011_FR_TXFF   (1U << 5)
#define PL011_LCRH_8BIT (3U << 5)
#define PL011_CR_UARTEN (1U << 0)
#define PL011_CR_TXE    (1U << 8)
#define PL011_CR_RXE    (1U << 9)

#define MBOX_BASE       0x107C013880UL
#define MBOX0_READ      (MBOX_BASE + 0x00)
#define MBOX0_STATUS    (MBOX_BASE + 0x18)
#define MBOX1_WRITE     (MBOX_BASE + 0x20)
#define MBOX1_STATUS    (MBOX_BASE + 0x38)
#define MBOX_FULL       0x80000000
#define MBOX_EMPTY      0x40000000
#define MBOX_CH         8
#define MBOX_REQUEST    0x00000000
#define MBOX_RESPONSE   0x80000000
#define MBOX_POLL_LIMIT 100000000U

#define FB_WIDTH 1920U

/* Aligned mailbox buffer — must be 16-byte aligned, in first 1GB for VC */
static volatile u32 __attribute__((aligned(16))) mbox[64];

static void uart_init(void) {
    mmio_write(PL011_CR, 0);
    mmio_write(PL011_LCRH, PL011_LCRH_8BIT);
    mmio_write(PL011_CR, PL011_CR_UARTEN | PL011_CR_TXE | PL011_CR_RXE);
}

static void uart_putc(char c) {
    while (mmio_read(PL011_FR) & PL011_FR_TXFF) {}
    mmio_write(PL011_DR, (u32)(u8)c);
}

static void uart_puts(const char *s) {
    while (*s) {
        if (*s == '\n')
            uart_putc('\r');
        uart_putc(*s++);
    }
}

static void uart_hex(u32 value) {
    static const char digits[] = "0123456789ABCDEF";
    uart_puts("0x");
    for (int shift = 28; shift >= 0; shift -= 4)
        uart_putc(digits[(value >> (u32)shift) & 0xFU]);
}

static void cache_flush(volatile void *addr, u32 size) {
    u64 p = (u64)addr;
    for (u32 i = 0; i < size; i += 64)
        __asm__ volatile("dc civac, %0" :: "r"(p + i));
    __asm__ volatile("dsb sy");
}

static int mbox_call(void) {
    cache_flush(mbox, sizeof(mbox));

    u32 addr = ((u32)(u64)mbox) | MBOX_CH;
    u32 polls = MBOX_POLL_LIMIT;
    while ((mmio_read(MBOX1_STATUS) & MBOX_FULL) && polls)
        polls--;
    if (!polls) {
        uart_puts("MBOX WRITE TIMEOUT\n");
        return 0;
    }
    mmio_write(MBOX1_WRITE, addr);

    polls = MBOX_POLL_LIMIT;
    while (polls--) {
        if (mmio_read(MBOX0_STATUS) & MBOX_EMPTY)
            continue;
        if (mmio_read(MBOX0_READ) == addr) {
            cache_flush(mbox, sizeof(mbox));
            if (mbox[1] == MBOX_RESPONSE)
                return 1;
            uart_puts("MBOX RESPONSE ");
            uart_hex(mbox[1]);
            uart_putc('\n');
            return 0;
        }
    }
    uart_puts("MBOX READ TIMEOUT\n");
    return 0;
}

static u8 ready_glyph(char c, u32 row) {
    static const u8 r[7] = {30, 17, 17, 30, 20, 18, 17};
    static const u8 e[7] = {31, 16, 16, 30, 16, 16, 31};
    static const u8 a[7] = {14, 17, 17, 31, 17, 17, 17};
    static const u8 d[7] = {30, 17, 17, 17, 17, 17, 30};
    static const u8 y[7] = {17, 17, 10, 4, 4, 4, 4};
    const u8 *glyph = 0;
    if (c == 'R') glyph = r;
    else if (c == 'E') glyph = e;
    else if (c == 'A') glyph = a;
    else if (c == 'D') glyph = d;
    else if (c == 'Y') glyph = y;
    return glyph ? glyph[row] : 0;
}

static void fb_ready(volatile u32 *fb, u32 pitch) {
    const u32 rows = 96U;
    const u32 stride = pitch / 4U;
    for (u32 y = 0; y < rows; y++)
        for (u32 x = 0; x < FB_WIDTH; x++)
            fb[y * stride + x] = 0x00000000U;

    static const char text[] = "READY";
    const u32 scale = 6U;
    u32 cursor = 32U;
    for (u32 n = 0; text[n]; n++) {
        for (u32 gy = 0; gy < 7U; gy++) {
            u8 bits = ready_glyph(text[n], gy);
            for (u32 gx = 0; gx < 5U; gx++) {
                if ((bits & (1U << (4U - gx))) == 0U)
                    continue;
                for (u32 sy = 0; sy < scale; sy++)
                    for (u32 sx = 0; sx < scale; sx++)
                        fb[(24U + gy * scale + sy) * stride +
                           cursor + gx * scale + sx] = 0x0000FF00U;
            }
        }
        cursor += 6U * scale;
    }
    cache_flush((volatile void *)fb, pitch * rows);
}

void main(void) {
    uart_init();
    uart_puts("READY\n");

    /* Allocate the framebuffer for the mode start4.elf already configured.
     * Pi 5's firmware path is allocation-only; do not renegotiate geometry. */
    mbox[0] = 8U * 4U;
    mbox[1] = MBOX_REQUEST;
    mbox[2] = 0x00040001U;
    mbox[3] = 8U;
    mbox[4] = 4U;
    mbox[5] = 16U;
    mbox[6] = 0U;
    mbox[7] = 0U;

    if (!mbox_call()) {
        uart_puts("HDMI0 FB FAIL\n");
        while (1) __asm__ volatile("wfe");
    }

    /* Read results */
    u32 fb_addr = mbox[5] & 0x3FFFFFFF;
    u32 fb_size = mbox[6];
    u32 pitch = FB_WIDTH * 4U;

    if (!fb_addr || fb_size < pitch * 96U) {
        uart_puts("HDMI0 FB INVALID ");
        uart_hex(mbox[5]);
        uart_putc('/');
        uart_hex(fb_size);
        uart_putc('\n');
        while (1) __asm__ volatile("wfe");
    }

    volatile u32 *fb = (volatile u32 *)(u64)fb_addr;
    uart_puts("HDMI0 FB READY\n");
    fb_ready(fb, pitch);

    while (1) __asm__ volatile("wfe");
}
