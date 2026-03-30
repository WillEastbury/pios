.section ".text.boot"
.global _start

/*
 * Canary v14: mailbox framebuffer with proper tag set.
 * Queries actual pitch, sets resolution, fills solid color.
 * Uses full tag set like the main.lv example.
 */
_start:
    mrs     x1, mpidr_el1
    and     x1, x1, #3
    cbnz    x1, park

    ldr     x1, =_start
    mov     sp, x1

    /* Build full framebuffer request with all tags */
    ldr     x20, =fb_request

    /* Header */
    mov     w0, #(35*4)            /* total buffer size */
    str     w0, [x20, #0]
    str     wzr, [x20, #4]         /* request code */

    /* Tag 1: Set physical width/height */
    ldr     w0, =0x00048003
    str     w0, [x20, #8]
    mov     w0, #8
    str     w0, [x20, #12]         /* value buf size */
    mov     w0, #8
    str     w0, [x20, #16]         /* request size */
    mov     w0, #1024
    str     w0, [x20, #20]         /* width */
    mov     w0, #768
    str     w0, [x20, #24]         /* height */

    /* Tag 2: Set virtual width/height */
    ldr     w0, =0x00048004
    str     w0, [x20, #28]
    mov     w0, #8
    str     w0, [x20, #32]
    mov     w0, #8
    str     w0, [x20, #36]
    mov     w0, #1024
    str     w0, [x20, #40]
    mov     w0, #768
    str     w0, [x20, #44]

    /* Tag 3: Set depth */
    ldr     w0, =0x00048005
    str     w0, [x20, #48]
    mov     w0, #4
    str     w0, [x20, #52]
    mov     w0, #4
    str     w0, [x20, #56]
    mov     w0, #32
    str     w0, [x20, #60]         /* 32bpp */

    /* Tag 4: Allocate buffer */
    ldr     w0, =0x00040001
    str     w0, [x20, #64]
    mov     w0, #8
    str     w0, [x20, #68]
    mov     w0, #4
    str     w0, [x20, #72]
    mov     w0, #16                /* alignment */
    str     w0, [x20, #76]
    str     wzr, [x20, #80]        /* size (response) */

    /* Tag 5: Get pitch */
    ldr     w0, =0x00040008
    str     w0, [x20, #84]
    mov     w0, #4
    str     w0, [x20, #88]
    mov     w0, #4
    str     w0, [x20, #92]
    str     wzr, [x20, #96]        /* pitch (response) */

    /* End tag */
    str     wzr, [x20, #100]

    /* Flush entire request from cache */
    mov     x4, x20
    mov     x5, #160               /* flush 160 bytes */
flush_req:
    dc      civac, x4
    add     x4, x4, #64
    subs    x5, x5, #64
    b.gt    flush_req
    dsb     sy

    /* Send to mailbox channel 8 */
    ldr     x21, =0x107C013880
    mov     w1, w20
    orr     w1, w1, #8             /* channel 8 */

1:  ldr     w2, [x21, #0x38]
    tbnz    w2, #31, 1b
    str     w1, [x21, #0x20]

2:  ldr     w2, [x21, #0x18]
    tbnz    w2, #30, 2b
    ldr     w2, [x21, #0x00]

    /* Invalidate cache for response */
    mov     x4, x20
    mov     x5, #160
flush_resp:
    dc      civac, x4
    add     x4, x4, #64
    subs    x5, x5, #64
    b.gt    flush_resp
    dsb     sy

    /* Read framebuffer address and pitch */
    ldr     w22, [x20, #76]        /* FB base */
    and     w22, w22, #0x3fffffff
    cbz     w22, park

    ldr     w23, [x20, #96]        /* pitch (bytes per row) */
    cbz     w23, use_default_pitch
    b       got_pitch
use_default_pitch:
    mov     w23, #(1024*4)         /* fallback: 1024 * 4bpp */
got_pitch:

    /* Fill entire screen solid blue */
    mov     x0, x22
    mov     w24, #768              /* rows */
row_loop:
    mov     x1, x0                 /* row start */
    mov     w25, #1024             /* cols */
col_loop:
    ldr     w3, =0xFF0040FF        /* ARGB: bright blue */
    str     w3, [x1], #4
    subs    w25, w25, #1
    b.ne    col_loop

    /* Advance by pitch (not width*4) */
    add     x0, x0, x23
    subs    w24, w24, #1
    b.ne    row_loop

park:
    wfe
    b       park

.balign 16
fb_request:
    .space 256, 0
