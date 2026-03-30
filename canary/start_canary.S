.section ".text.boot"
.global _start

/*
 * Canary v12: UART + framebuffer dual output test.
 *
 * With pciex4_reset=0 in config.txt, the PCIe BAR should survive.
 * Writes "PIOS" to RP1 UART (0x1F00030000), then fills screen white
 * via VideoCore mailbox framebuffer.
 *
 * If UART still fails (PCIe really gone), framebuffer should work
 * since mailbox is on-SoC at 0x107C013880.
 */

/* Mailbox registers (BCM2712 SoC — no PCIe needed) */
.equ MBOX_BASE,     0x107C013880
.equ MBOX1_WRITE,   0x20
.equ MBOX1_STATUS,  0x38
.equ MBOX0_READ,    0x00
.equ MBOX0_STATUS,  0x18
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

    /* --- Try RP1 UART (may work with pciex4_reset=0) --- */
    ldr     x9, =0x1F00030000
    mov     w0, #'P'
    str     w0, [x9]
    mov     w0, #'I'
    str     w0, [x9]
    mov     w0, #'O'
    str     w0, [x9]
    mov     w0, #'S'
    str     w0, [x9]
    mov     w0, #'\r'
    str     w0, [x9]
    mov     w0, #'\n'
    str     w0, [x9]

    /* --- Mailbox framebuffer request --- */
    /* Build request buffer (must be 16-byte aligned) */
    ldr     x20, =fb_request
    /* Size = 8 * 4 = 32 bytes */
    mov     w0, #32
    str     w0, [x20, #0]          /* total size */
    str     wzr, [x20, #4]         /* request code = 0 */
    ldr     w0, =0x00040001        /* MBOX_TAG_GETFB */
    str     w0, [x20, #8]
    mov     w0, #8                 /* value buffer size */
    str     w0, [x20, #12]
    mov     w0, #4                 /* request size (alignment) */
    str     w0, [x20, #16]
    mov     w0, #16                /* alignment = 16 */
    str     w0, [x20, #20]
    str     wzr, [x20, #24]        /* fb size (filled by response) */
    str     wzr, [x20, #28]        /* end tag */

    /* Clean cache for the request buffer (dcache writeback) */
    dc      civac, x20
    add     x1, x20, #32
    dc      civac, x1
    dsb     sy

    /* Send to mailbox: write (addr | channel) to MBOX1_WRITE */
    ldr     x21, =MBOX_BASE
    mov     w1, w20                /* low 32 bits of address */
    orr     w1, w1, #MBOX_CH       /* channel 8 */

    /* Wait for mailbox not full */
1:  ldr     w2, [x21, #MBOX1_STATUS]
    tbnz    w2, #31, 1b           /* bit 31 = FULL */

    /* Write request */
    str     w1, [x21, #MBOX1_WRITE]

    /* Wait for response: poll until not empty, then read */
2:  ldr     w2, [x21, #MBOX0_STATUS]
    tbnz    w2, #30, 2b           /* bit 30 = EMPTY */
    ldr     w2, [x21, #MBOX0_READ]

    /* Invalidate cache to see response */
    dc      civac, x20
    dsb     sy

    /* Get framebuffer address from response */
    ldr     w22, [x20, #20]        /* FB base address */
    and     w22, w22, #0x3fffffff  /* mask to bus address */
    cbz     w22, no_fb             /* if 0, no framebuffer */

    /* Fill screen with white (1920*1080*4 = ~8MB) */
    mov     x0, x22                /* FB start */
    ldr     w1, =(1920*1080)       /* pixel count */
    ldr     w3, =0xFF00FF00        /* green — visible! */
fill:
    str     w3, [x0], #4
    subs    w1, w1, #1
    b.ne    fill

no_fb:
    /* Infinite loop */
3:  wfe
    b       3b

park:
    wfe
    b       park

.balign 16
fb_request:
    .space 64, 0
