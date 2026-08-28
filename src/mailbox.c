/*
 * mailbox.c - VideoCore mailbox interface
 * Used to negotiate framebuffer allocation with the GPU.
 *
 * Register layout (BCM2712, base = MBOX_BASE from mmio.h):
 *   MBOX0_READ:    base + 0x00  (ARM reads from VC)
 *   MBOX0_STATUS:  base + 0x18  (check EMPTY bit 30 before reading)
 *   MBOX1_WRITE:   base + 0x20  (ARM writes to VC)
 *   MBOX1_STATUS:  base + 0x38  (check FULL bit 31 before writing)
 */

#include "mailbox.h"
#include "mmio.h"

#define MBOX0_READ      (MBOX_BASE + 0x00)
#define MBOX0_STATUS    (MBOX_BASE + 0x18)
#define MBOX1_WRITE     (MBOX_BASE + 0x20)
#define MBOX1_STATUS    (MBOX_BASE + 0x38)

#define MBOX_FULL       0x80000000
#define MBOX_EMPTY      0x40000000
#define MBOX_RESPONSE   0x80000000
#define MBOX_POLL_LIMIT 1000000U

static void mbox_cache_flush(volatile u32 *buf, u32 size) {
    u64 p = (u64)(usize)buf & ~63UL;
    u64 end = ((u64)(usize)buf + size + 63) & ~63UL;
    while (p < end) {
        __asm__ volatile("dc civac, %0" :: "r"(p));
        p += 64;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

bool mbox_call(u8 channel, volatile u32 *mbox_buf) {
#if PIOS_MBOX_BASE == 0
    (void)channel;
    (void)mbox_buf;
    return false;
#else
    u64 addr = (u64)(usize)mbox_buf;

    /* Ensure buffer is 16-byte aligned */
    if (addr & 0xF)
        return false;

    /* Flush cache so VideoCore sees our request */
    mbox_cache_flush(mbox_buf, mbox_buf[0]);

    u32 msg = (u32)(addr & 0xFFFFFFF0) | (channel & 0xF);

    /* Wait for MBOX1 (write side) to be not full. */
    for (u32 polls = 0; polls < MBOX_POLL_LIMIT; polls++) {
        if (!(mmio_read(MBOX1_STATUS) & MBOX_FULL))
            goto write_ready;
    }
    return false;

write_ready:
    /* Write message to MBOX1 */
    mmio_write(MBOX1_WRITE, msg);

    /* Wait for response on MBOX0 (read side) */
    for (u32 polls = 0; polls < MBOX_POLL_LIMIT; polls++) {
        if (mmio_read(MBOX0_STATUS) & MBOX_EMPTY)
            continue;
        if (mmio_read(MBOX0_READ) == msg) {
            /* Invalidate cache to see VideoCore's response */
            mbox_cache_flush(mbox_buf, mbox_buf[0]);
            return (mbox_buf[1] == MBOX_RESPONSE);
        }
    }
    return false;
#endif
}

bool mbox_set_gpio_output(u32 gpio, bool high)
{
#if PIOS_MBOX_BASE == 0
    (void)gpio;
    (void)high;
    return false;
#else
    static volatile u32 gpio_mbox[13] __attribute__((aligned(16)));
    gpio_mbox[0] = sizeof(gpio_mbox);
    gpio_mbox[1] = 0;
    gpio_mbox[2] = TAG_SET_GPIO_CONFIG;
    gpio_mbox[3] = 8;
    gpio_mbox[4] = 8;
    gpio_mbox[5] = gpio;
    gpio_mbox[6] = 1; /* output */
    gpio_mbox[7] = TAG_SET_GPIO_STATE;
    gpio_mbox[8] = 8;
    gpio_mbox[9] = 8;
    gpio_mbox[10] = gpio;
    gpio_mbox[11] = high ? 1U : 0U;
    gpio_mbox[12] = TAG_END;
    return mbox_call(MBOX_CH_PROP, gpio_mbox);
#endif
}
