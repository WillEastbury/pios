/*
 * canary_main.c - Minimal HDMI green-screen test for Pi 5
 *
 * Mailbox base: 0x107C013880 (from ARM TF-A rpi_hw.h)
 * Read-side status at offset 0x18, write-side status at offset 0x38.
 * Caches disabled in start_canary.S so GPU sees RAM writes directly.
 */
#include "types.h"

#define MBOX_BASE       0x107C013880UL

#define MBOX0_READ      (MBOX_BASE + 0x00)
#define MBOX0_STATUS    (MBOX_BASE + 0x18)  /* read-side: EMPTY flag */
#define MBOX1_WRITE     (MBOX_BASE + 0x20)
#define MBOX1_STATUS    (MBOX_BASE + 0x38)  /* write-side: FULL flag */

#define MBOX_FULL       0x80000000U
#define MBOX_EMPTY      0x40000000U
#define MBOX_CH_PROP    8U
#define MBOX_RESPONSE   0x80000000U

/* BCM2712 debug UART (used by ARM TF-A, directly on SoC) */
#define DEBUG_UART_DR   0x107D001000UL

static inline void mmio_write(u64 addr, u32 val) { *(volatile u32 *)addr = val; }
static inline u32  mmio_read(u64 addr) { return *(volatile u32 *)addr; }

static void debug_char(char c)
{
    /* Write to BCM2712 internal debug UART - may be visible on Pi 5's
     * 3-pin UART header. Also try RP1 UART at firmware's PCIe mapping. */
    mmio_write(DEBUG_UART_DR, (u32)c);
    mmio_write(0x1c00030000UL, (u32)c);  /* RP1 UART via firmware PCIe BAR */
}

static volatile u32 __attribute__((aligned(16))) mbox[36];

static bool mailbox_call(u8 ch)
{
    u32 r = ((u32)(usize)&mbox[0] & ~0xFU) | (u32)(ch & 0xFU);

    /* DSB to ensure all mbox[] writes are visible in RAM */
    __asm__ volatile("dsb sy" ::: "memory");

    /* Wait for write mailbox not full (write-side status) */
    for (u32 t = 0; t < 1000000; t++) {
        if (!(mmio_read(MBOX1_STATUS) & MBOX_FULL)) goto write_ok;
    }
    return false;  /* timeout */
write_ok:
    mmio_write(MBOX1_WRITE, r);

    /* Wait for response (read-side status for EMPTY check) */
    for (u32 t = 0; t < 10000000; t++) {
        if (!(mmio_read(MBOX0_STATUS) & MBOX_EMPTY)) {
            if (mmio_read(MBOX0_READ) == r)
                return mbox[1] == MBOX_RESPONSE;
        }
    }
    return false;  /* timeout */
}

void canary_main(void)
{
    debug_char('C');

    /* Request framebuffer allocation from GPU */
    mbox[0] = 8 * 4;
    mbox[1] = 0;
    mbox[2] = 0x00040001;   /* ALLOCATE_BUFFER */
    mbox[3] = 8;
    mbox[4] = 4;             /* alignment */
    mbox[5] = 0;
    mbox[6] = 0;
    mbox[7] = 0;             /* TAG_END */

    if (!mailbox_call(MBOX_CH_PROP) || mbox[5] == 0) {
        debug_char('F');     /* mailbox failed */
        goto hang;
    }

    debug_char('G');         /* got framebuffer */

    u32 *fb = (u32 *)(usize)(mbox[5] & 0x3FFFFFFFU);
    u32 fb_size = mbox[6];
    u32 pixels = fb_size / 4;

    for (u32 i = 0; i < pixels; i++)
        fb[i] = 0xFF00FF00U;

    debug_char('D');         /* draw done */

hang:
    for (;;) __asm__ volatile("wfe");
}
