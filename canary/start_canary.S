.section ".text.boot"
.global _start

/*
 * Canary v13: mailbox framebuffer ONLY — no UART, no PCIe addresses.
 * All registers are on BCM2712 SoC (0x107C013880).
 *
 * Requests framebuffer via VideoCore mailbox, fills screen green.
 * If the screen turns green, we have confirmed output.
 */

.equ MBOX_BASE,     0x107C013880
.equ MBOX_FULL,     0x80000000
.equ MBOX_EMPTY,    0x40000000
.equ MBOX_CH,       8

_start:
    mrs     x1, mpidr_el1
    and     x1, x1, #3
    cbnz    x1, park

    /* Stack just below our code */
    ldr     x1, =_start
    mov     sp, x1

    /* --- Build mailbox framebuffer request --- */
    ldr     x20, =fb_request

    mov     w0, #32                /* total buffer size (8 * 4) */
    str     w0, [x20, #0]
    str     wzr, [x20, #4]         /* request code = 0 */
    ldr     w0, =0x00040001        /* TAG: Allocate Buffer */
    str     w0, [x20, #8]
    mov     w0, #8                 /* value buffer size */
    str     w0, [x20, #12]
    mov     w0, #4                 /* request length */
    str     w0, [x20, #16]
    mov     w0, #16                /* alignment = 16 bytes */
    str     w0, [x20, #20]
    str     wzr, [x20, #24]        /* size (response) */
    str     wzr, [x20, #28]        /* end tag */

    /* Flush cache for request buffer */
    dc      civac, x20
    add     x1, x20, #32
    dc      civac, x1
    dsb     sy

    /* Mailbox: send request */
    ldr     x21, =MBOX_BASE

    /* Construct write value: (addr & ~0xF) | channel */
    mov     w1, w20
    orr     w1, w1, #MBOX_CH

    /* Wait for not full */
1:  ldr     w2, [x21, #0x38]      /* MBOX1_STATUS */
    tbnz    w2, #31, 1b

    /* Write to MBOX1 */
    str     w1, [x21, #0x20]      /* MBOX1_WRITE */

    /* Wait for response (not empty), then read */
2:  ldr     w2, [x21, #0x18]      /* MBOX0_STATUS */
    tbnz    w2, #30, 2b
    ldr     w2, [x21, #0x00]      /* MBOX0_READ (discard) */

    /* Invalidate cache to see response */
    dc      civac, x20
    dsb     sy

    /* Read FB address from response */
    ldr     w22, [x20, #20]
    and     w22, w22, #0x3fffffff
    cbz     w22, no_fb

    /* Fill 1920x1080 pixels with bright green (0xFF00FF00) */
    mov     x0, x22
    ldr     w1, =(1920*1080)
    ldr     w3, =0xFF00FF00
fill:
    str     w3, [x0], #4
    subs    w1, w1, #1
    b.ne    fill

no_fb:
park:
    wfe
    b       park

.balign 16
fb_request:
    .space 64, 0
