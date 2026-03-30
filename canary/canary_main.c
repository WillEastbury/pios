/*
 * canary_main.c - Minimal HDMI green-screen test for Pi 5
 *
 * Uses VideoCore mailbox to allocate a framebuffer and fill it green.
 * No UART, no RP1, no PCIe — just the BCM2712 mailbox at its correct address.
 *
 * Mailbox base confirmed from ARM Trusted Firmware (rpi_hw.h):
 *   RPI_IO_BASE + 0x7c013880 = 0x107C013880
 */
#include "types.h"

#define MBOX_BASE     0x107C013880UL

#define MBOX_READ     (MBOX_BASE + 0x00)
#define MBOX_STATUS   (MBOX_BASE + 0x18)
#define MBOX_WRITE    (MBOX_BASE + 0x20)
#define MBOX_FULL     0x80000000U
#define MBOX_EMPTY    0x40000000U
#define MBOX_CH_PROP  8U

static inline void mmio_write(u64 addr, u32 val) { *(volatile u32 *)addr = val; }
static inline u32  mmio_read(u64 addr) { return *(volatile u32 *)addr; }

static volatile u32 __attribute__((aligned(16))) mbox[36];

static bool mailbox_call(u8 ch)
{
    u32 addr = ((u32)(usize)&mbox[0] & ~0xFU) | (u32)(ch & 0xFU);
    while (mmio_read(MBOX_STATUS) & MBOX_FULL) {}
    mmio_write(MBOX_WRITE, addr);
    for (;;) {
        while (mmio_read(MBOX_STATUS) & MBOX_EMPTY) {}
        u32 r = mmio_read(MBOX_READ);
        if (r == addr) return mbox[1] == 0x80000000U;
    }
}

void canary_main(void)
{
    /* Request 1280x720x32 framebuffer via mailbox property interface */
    mbox[0]  = 35 * 4;          /* total size */
    mbox[1]  = 0;               /* request */
    mbox[2]  = 0x00048003;      /* SET_PHYSICAL_WH */
    mbox[3]  = 8; mbox[4] = 0; mbox[5] = 1280; mbox[6] = 720;
    mbox[7]  = 0x00048004;      /* SET_VIRTUAL_WH */
    mbox[8]  = 8; mbox[9] = 0; mbox[10] = 1280; mbox[11] = 720;
    mbox[12] = 0x00048005;      /* SET_DEPTH */
    mbox[13] = 4; mbox[14] = 0; mbox[15] = 32;
    mbox[16] = 0x00048006;      /* SET_PIXEL_ORDER (BGR) */
    mbox[17] = 4; mbox[18] = 0; mbox[19] = 0;
    mbox[20] = 0x00040001;      /* ALLOCATE_BUFFER */
    mbox[21] = 8; mbox[22] = 0; mbox[23] = 4096; mbox[24] = 0;
    mbox[25] = 0x00040008;      /* GET_PITCH */
    mbox[26] = 4; mbox[27] = 0; mbox[28] = 0;
    mbox[29] = 0;               /* TAG_END */

    if (!mailbox_call(MBOX_CH_PROP) || mbox[23] == 0)
        goto hang;

    /* GPU returns bus address; mask to physical */
    u32 *fb = (u32 *)(usize)(mbox[23] & 0x3FFFFFFFU);
    u32 pitch = mbox[28] / 4U;

    /* Fill entire screen bright green */
    for (u32 y = 0; y < 720; y++)
        for (u32 x = 0; x < 1280; x++)
            fb[y * pitch + x] = 0xFF00FF00U;  /* ARGB green */

hang:
    for (;;) __asm__ volatile("wfe");
}
