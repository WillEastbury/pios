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

#if defined(__aarch64__)
static void mbox_cache_flush(volatile u32 *buf, u32 size) {
    u64 p = (u64)(usize)buf & ~63UL;
    u64 end = ((u64)(usize)buf + size + 63) & ~63UL;
    while (p < end) {
        __asm__ volatile("dc civac, %0" :: "r"(p));
        p += 64;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}
#endif

bool mbox_call(u8 channel, volatile u32 *mbox_buf) {
#if PIOS_MBOX_BASE == 0 || !defined(__aarch64__)
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

bool mbox_build_gpio_output_request(volatile u32 *buf, u32 words,
                                    u32 gpio, bool high)
{
    if (!buf || words < MBOX_GPIO_OUTPUT_REQUEST_WORDS)
        return false;

    for (u32 i = 0U; i < MBOX_GPIO_OUTPUT_REQUEST_WORDS; i++)
        buf[i] = 0U;
    buf[0] = MBOX_GPIO_OUTPUT_REQUEST_WORDS * sizeof(buf[0]);
    buf[1] = 0U;
    buf[2] = TAG_SET_GPIO_CONFIG;
    buf[3] = 20U;
    buf[4] = 20U;
    buf[5] = gpio;
    buf[6] = 1U; /* output */
    buf[7] = 0U; /* active-high */
    buf[8] = 0U; /* no firmware terminator override */
    buf[9] = 0U; /* no pull-up override */
    buf[10] = TAG_SET_GPIO_STATE;
    buf[11] = 8U;
    buf[12] = 8U;
    buf[13] = gpio;
    buf[14] = high ? 1U : 0U;
    buf[15] = TAG_END;
    return true;
}

bool mbox_set_gpio_output(u32 gpio, bool high, u32 *response_code)
{
    static volatile u32 gpio_mbox[MBOX_GPIO_OUTPUT_REQUEST_WORDS]
        __attribute__((aligned(16)));
    if (response_code)
        *response_code = 0U;
    if (!mbox_build_gpio_output_request(gpio_mbox,
                                        MBOX_GPIO_OUTPUT_REQUEST_WORDS,
                                        gpio, high))
        return false;
    if (!mbox_call(MBOX_CH_PROP, gpio_mbox)) {
        if (response_code)
            *response_code = gpio_mbox[1];
        return false;
    }
    if (gpio_mbox[4] != (MBOX_RESPONSE | 20U) ||
        gpio_mbox[12] != (MBOX_RESPONSE | 8U)) {
        if (response_code)
            *response_code = gpio_mbox[4] != (MBOX_RESPONSE | 20U) ?
                             gpio_mbox[4] : gpio_mbox[12];
        return false;
    }
    if (response_code)
        *response_code = MBOX_RESPONSE;
    return true;
}
