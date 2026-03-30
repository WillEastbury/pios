/*
 * canary_main.c - Minimal HDMI green-screen test for Pi 5
 *
 * Matched against working main.lv bare-metal framebuffer example.
 * Mailbox base: 0x107C000000 + 0x13880 = 0x107C013880
 *
 * Key: BCM2712 mailbox has SEPARATE read/write status registers:
 *   MBOX0 (read side):  READ=+0x00, STATUS=+0x18
 *   MBOX1 (write side): WRITE=+0x20, STATUS=+0x38
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

static inline void mmio_write(u64 addr, u32 val) { *(volatile u32 *)addr = val; }
static inline u32  mmio_read(u64 addr) { return *(volatile u32 *)addr; }

static volatile u32 __attribute__((aligned(16))) mbox[36];

static bool mailbox_call(u8 ch)
{
    u32 r = ((u32)(usize)&mbox[0] & ~0xFU) | (u32)(ch & 0xFU);

    /* Wait for write mailbox not full (MBOX1 status) */
    while (mmio_read(MBOX1_STATUS) & MBOX_FULL) {}
    mmio_write(MBOX1_WRITE, r);

    /* Wait for response on read mailbox (MBOX1 status for empty check) */
    for (;;) {
        while (mmio_read(MBOX1_STATUS) & MBOX_EMPTY) {}
        if (mmio_read(MBOX0_READ) == r)
            return mbox[1] == MBOX_RESPONSE;
    }
}

void canary_main(void)
{
    /*
     * Simple approach from working example: just allocate the existing
     * firmware framebuffer without trying to set resolution/depth.
     * The firmware's config.txt framebuffer_depth=32 handles that.
     */
    mbox[0] = 8 * 4;       /* message size: 8 u32s = 32 bytes */
    mbox[1] = 0;            /* request code */
    mbox[2] = 0x00040001;   /* ALLOCATE_BUFFER */
    mbox[3] = 8;            /* value buffer size */
    mbox[4] = 4;            /* request size (alignment) */
    mbox[5] = 0;            /* output: fb base address */
    mbox[6] = 0;            /* output: fb size */
    mbox[7] = 0;            /* TAG_END */

    if (!mailbox_call(MBOX_CH_PROP) || mbox[5] == 0)
        goto hang;

    /* GPU returns bus address; mask to ARM physical */
    u32 *fb = (u32 *)(usize)(mbox[5] & 0x3FFFFFFFU);
    u32 fb_size = mbox[6];

    /* Fill entire framebuffer green (ARGB) */
    u32 pixels = fb_size / 4;
    for (u32 i = 0; i < pixels; i++)
        fb[i] = 0xFF00FF00U;

hang:
    for (;;) __asm__ volatile("wfe");
}
