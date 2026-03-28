/*
 * mailbox.c - VideoCore mailbox interface
 * Used to negotiate framebuffer allocation with the GPU.
 */

#include "mailbox.h"
#include "mmio.h"

#define MBOX_READ       (MBOX_BASE + 0x00)
#define MBOX_STATUS     (MBOX_BASE + 0x18)
#define MBOX_WRITE      (MBOX_BASE + 0x20)

#define MBOX_FULL       0x80000000
#define MBOX_EMPTY      0x40000000
#define MBOX_RESPONSE   0x80000000

bool mbox_call(u8 channel, volatile u32 *mbox_buf) {
    u64 addr = (u64)(usize)mbox_buf;

    /* Ensure buffer is 16-byte aligned */
    if (addr & 0xF)
        return false;

    u32 msg = (u32)(addr & 0xFFFFFFF0) | (channel & 0xF);

    /* Wait for mailbox to be not full */
    while (mmio_read(MBOX_STATUS) & MBOX_FULL)
        ;

    /* Write message */
    mmio_write(MBOX_WRITE, msg);

    /* Wait for response */
    for (;;) {
        while (mmio_read(MBOX_STATUS) & MBOX_EMPTY)
            ;
        if (mmio_read(MBOX_READ) == msg)
            return (mbox_buf[1] == MBOX_RESPONSE);
    }
}
