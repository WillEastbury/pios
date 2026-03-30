.section ".text.boot"
.global _start

/*
 * Canary v9: blind writes to TWO UARTs, no FIFO status check.
 * PL011 TX FIFO is 16-deep so a few chars won't overflow.
 *
 * UART A: 0x1F00030000 (RP1 UART0 via PCIe BAR — firmware reported address)
 * UART B: 0x107D001000 (BCM2712 debug UART — SoC-internal, no PCIe needed)
 */
_start:
    mrs     x1, mpidr_el1
    and     x1, x1, #3
    cbnz    x1, park

    /* Load both UART base addresses */
    ldr     x10, =0x107D001000     /* BCM2712 debug UART — try first (on-SoC) */
    ldr     x9, =0x1F00030000      /* RP1 UART via PCIe BAR — try second */

    /* Blind write to debug UART first (guaranteed on-SoC, no PCIe) */
    mov     w0, #'P'
    str     w0, [x10]
    mov     w0, #'I'
    str     w0, [x10]
    mov     w0, #'O'
    str     w0, [x10]
    mov     w0, #'S'
    str     w0, [x10]
    mov     w0, #'1'
    str     w0, [x10]
    mov     w0, #'\r'
    str     w0, [x10]
    mov     w0, #'\n'
    str     w0, [x10]

    /* Now try RP1 UART (may fault if PCIe BAR not mapped) */
    mov     w0, #'P'
    str     w0, [x9]
    mov     w0, #'I'
    str     w0, [x9]
    mov     w0, #'O'
    str     w0, [x9]
    mov     w0, #'S'
    str     w0, [x9]
    mov     w0, #'2'
    str     w0, [x9]
    mov     w0, #'\r'
    str     w0, [x9]
    mov     w0, #'\n'
    str     w0, [x9]

    /* Heartbeat dots to both */
dot_loop:
    mov     w0, #'.'
    str     w0, [x9]
    str     w0, [x10]
    mov     x3, #0x800000
delay:
    sub     x3, x3, #1
    cbnz    x3, delay
    b       dot_loop

park:
    wfe
    b       park
