/*
 * canary_main.c - Absolute minimal proof-of-life for Pi 5
 *
 * Strategy: try every possible output path:
 * 1. BCM2712 debug UART at 0x107D001000 (what TF-A uses, pre-configured)
 * 2. RP1 UART0 at 0x1c00030000 (firmware PCIe BAR, if enable_rp1_uart=1)
 * 3. VideoCore mailbox framebuffer
 * 4. PM watchdog reboot (visible reboot loop as proof of life)
 */
#include "types.h"

static inline void mmio_write(u64 addr, u32 val) { *(volatile u32 *)addr = val; }
static inline u32  mmio_read(u64 addr) { return *(volatile u32 *)addr; }

/* BCM2712 internal debug UART (PL011, configured by TF-A at 115200) */
#define DBG_UART_BASE  0x107D001000UL
#define DBG_UART_DR    (DBG_UART_BASE + 0x00)
#define DBG_UART_FR    (DBG_UART_BASE + 0x18)

/* RP1 UART0 (via firmware PCIe BAR mapping) */
#define RP1_UART_BASE  0x1c00030000UL
#define RP1_UART_DR    (RP1_UART_BASE + 0x00)
#define RP1_UART_FR    (RP1_UART_BASE + 0x18)

static void uart_send(char c)
{
    /* Try debug UART — wait with timeout */
    for (u32 t = 0; t < 100000; t++) {
        if (!(mmio_read(DBG_UART_FR) & (1U << 5))) {
            mmio_write(DBG_UART_DR, (u32)c);
            break;
        }
    }
    /* Try RP1 UART — fire and forget (may not be mapped) */
    for (u32 t = 0; t < 100000; t++) {
        if (!(mmio_read(RP1_UART_FR) & (1U << 5))) {
            mmio_write(RP1_UART_DR, (u32)c);
            break;
        }
    }
}

static void uart_puts(const char *s)
{
    while (*s) {
        if (*s == '\n') uart_send('\r');
        uart_send(*s++);
    }
}

static void delay(u32 n)
{
    for (volatile u32 i = 0; i < n; i++) {}
}

/* PM watchdog */
#define PM_BASE    0x107D200000UL
#define PM_RSTC    (PM_BASE + 0x1C)
#define PM_WDOG    (PM_BASE + 0x24)
#define PM_PASSWD  0x5A000000U

/* Mailbox */
#define MBOX_BASE       0x107C013880UL
#define MBOX0_READ      (MBOX_BASE + 0x00)
#define MBOX0_STATUS    (MBOX_BASE + 0x18)
#define MBOX1_WRITE     (MBOX_BASE + 0x20)
#define MBOX1_STATUS    (MBOX_BASE + 0x38)

static volatile u32 __attribute__((aligned(16))) mbox[36];

static bool mailbox_call(void)
{
    u32 r = ((u32)(usize)&mbox[0] & ~0xFU) | 8U;
    __asm__ volatile("dsb sy" ::: "memory");
    for (u32 t = 0; t < 1000000; t++)
        if (!(mmio_read(MBOX1_STATUS) & 0x80000000U)) goto ok;
    return false;
ok:
    mmio_write(MBOX1_WRITE, r);
    for (u32 t = 0; t < 10000000; t++) {
        if (!(mmio_read(MBOX0_STATUS) & 0x40000000U)) {
            if (mmio_read(MBOX0_READ) == r)
                return mbox[1] == 0x80000000U;
        }
    }
    return false;
}

void canary_main(void)
{
    /* === STAGE 1: UART proof of life === */
    uart_puts("\n\nPIOS CANARY ALIVE\n");

    /* === STAGE 2: Try framebuffer === */
    uart_puts("FB: trying...\n");

    mbox[0]  = 35 * 4;
    mbox[1]  = 0;
    mbox[2]  = 0x00048003; mbox[3] = 8; mbox[4] = 0; mbox[5] = 1280; mbox[6] = 720;
    mbox[7]  = 0x00048004; mbox[8] = 8; mbox[9] = 0; mbox[10] = 1280; mbox[11] = 720;
    mbox[12] = 0x00048005; mbox[13] = 4; mbox[14] = 0; mbox[15] = 32;
    mbox[16] = 0x00048006; mbox[17] = 4; mbox[18] = 0; mbox[19] = 0;
    mbox[20] = 0x00040001; mbox[21] = 8; mbox[22] = 0; mbox[23] = 4096; mbox[24] = 0;
    mbox[25] = 0x00040008; mbox[26] = 4; mbox[27] = 0; mbox[28] = 0;
    mbox[29] = 0;

    bool fb_ok = false;
    if (mailbox_call() && mbox[23] != 0) {
        u32 *fb = (u32 *)(usize)(mbox[23] & 0x3FFFFFFFU);
        u32 pitch = mbox[28] / 4;
        for (u32 y = 0; y < 720; y++)
            for (u32 x = 0; x < 1280; x++)
                fb[y * pitch + x] = 0xFF00FF00U;
        fb_ok = true;
        uart_puts("FB: OK green\n");
    } else {
        uart_puts("FB: FAIL\n");
    }

    /* === STAGE 3: Heartbeat on UART (proves we're alive) === */
    uart_puts("Heartbeat:\n");
    for (u32 i = 0; i < 30; i++) {
        uart_send('.');
        delay(50000000);
    }

    /* === STAGE 4: If no FB, reboot as proof of life === */
    if (!fb_ok) {
        uart_puts("\nREBOOT\n");
        delay(10000000);
        mmio_write(PM_WDOG, PM_PASSWD | 10);
        mmio_write(PM_RSTC, PM_PASSWD | 0x20);
    }

    for (;;) __asm__ volatile("wfe");
}
