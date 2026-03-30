/*
 * canary_main.c - Minimal HDMI green-screen test for Pi 5
 *
 * Mailbox base: 0x107C013880 (confirmed in bcm2712.dtsi + Linux driver)
 * Uses full tag set (SET_PHYSICAL_WH etc.) to force GPU to create an FB.
 * Caches disabled in start_canary.S so GPU sees RAM writes directly.
 *
 * Proof-of-life fallback: if FB fails, triggers PM watchdog reset so
 * the user sees the Pi reboot-looping (proves code executes).
 */
#include "types.h"

#define MBOX_BASE       0x107C013880UL
#define MBOX0_READ      (MBOX_BASE + 0x00)
#define MBOX0_STATUS    (MBOX_BASE + 0x18)
#define MBOX1_WRITE     (MBOX_BASE + 0x20)
#define MBOX1_STATUS    (MBOX_BASE + 0x38)
#define MBOX_FULL       0x80000000U
#define MBOX_EMPTY      0x40000000U
#define MBOX_CH_PROP    8U
#define MBOX_RESPONSE   0x80000000U

/* PM watchdog registers (proof-of-life: triggers visible reboot) */
#define PM_BASE         0x107D200000UL
#define PM_RSTC         (PM_BASE + 0x1C)
#define PM_WDOG         (PM_BASE + 0x24)
#define PM_PASSWORD     0x5A000000U
#define PM_RSTC_WRCFG_FULL_RESET 0x00000020U

static inline void mmio_write(u64 addr, u32 val) { *(volatile u32 *)addr = val; }
static inline u32  mmio_read(u64 addr) { return *(volatile u32 *)addr; }

static void pm_reboot(void)
{
    /* Trigger hardware watchdog reset — Pi will visibly reboot */
    mmio_write(PM_WDOG, PM_PASSWORD | 10);      /* ~160µs timeout */
    mmio_write(PM_RSTC, PM_PASSWORD | PM_RSTC_WRCFG_FULL_RESET);
    for (;;) __asm__ volatile("wfe");
}

static void delay(u32 count)
{
    for (volatile u32 i = 0; i < count; i++) {}
}

static volatile u32 __attribute__((aligned(16))) mbox[36];

static bool mailbox_call(void)
{
    u32 r = ((u32)(usize)&mbox[0] & ~0xFU) | (MBOX_CH_PROP & 0xFU);

    __asm__ volatile("dsb sy" ::: "memory");

    /* Wait for write side not full */
    for (u32 t = 0; t < 1000000; t++) {
        if (!(mmio_read(MBOX1_STATUS) & MBOX_FULL))
            goto write_ok;
    }
    return false;
write_ok:
    mmio_write(MBOX1_WRITE, r);

    /* Wait for read side not empty, then check response */
    for (u32 t = 0; t < 10000000; t++) {
        if (!(mmio_read(MBOX0_STATUS) & MBOX_EMPTY)) {
            if (mmio_read(MBOX0_READ) == r)
                return mbox[1] == MBOX_RESPONSE;
        }
    }
    return false;
}

void canary_main(void)
{
    /* Full framebuffer request: set resolution, depth, then allocate */
    mbox[0]  = 35 * 4;          /* total message size */
    mbox[1]  = 0;               /* request code */

    mbox[2]  = 0x00048003;      /* SET_PHYSICAL_WH */
    mbox[3]  = 8;               /* value buffer size */
    mbox[4]  = 0;               /* request/response indicator */
    mbox[5]  = 1280;            /* width */
    mbox[6]  = 720;             /* height */

    mbox[7]  = 0x00048004;      /* SET_VIRTUAL_WH */
    mbox[8]  = 8;
    mbox[9]  = 0;
    mbox[10] = 1280;
    mbox[11] = 720;

    mbox[12] = 0x00048005;      /* SET_DEPTH */
    mbox[13] = 4;
    mbox[14] = 0;
    mbox[15] = 32;              /* bits per pixel */

    mbox[16] = 0x00048006;      /* SET_PIXEL_ORDER (RGB) */
    mbox[17] = 4;
    mbox[18] = 0;
    mbox[19] = 0;               /* 0=BGR, 1=RGB */

    mbox[20] = 0x00040001;      /* ALLOCATE_BUFFER */
    mbox[21] = 8;
    mbox[22] = 0;
    mbox[23] = 4096;            /* alignment */
    mbox[24] = 0;

    mbox[25] = 0x00040008;      /* GET_PITCH */
    mbox[26] = 4;
    mbox[27] = 0;
    mbox[28] = 0;

    mbox[29] = 0;               /* TAG_END */

    if (!mailbox_call() || mbox[23] == 0) {
        /* Mailbox failed — trigger visible reboot as proof of life.
         * Wait 3 seconds first so user can see the pattern. */
        delay(100000000);
        pm_reboot();
    }

    /* GPU returns bus address; mask to ARM physical */
    u32 *fb = (u32 *)(usize)(mbox[23] & 0x3FFFFFFFU);
    u32 pitch = mbox[28] / 4;

    /* Fill screen bright green */
    for (u32 y = 0; y < 720; y++)
        for (u32 x = 0; x < 1280; x++)
            fb[y * pitch + x] = 0xFF00FF00U;

    for (;;) __asm__ volatile("wfe");
}
