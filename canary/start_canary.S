.section ".text.boot"
.global _start

/*
 * Ultra-minimal canary: pure assembly, no C, no init.
 * Firmware already configured RP1 UART at 0x1F00030000.
 * Just write chars directly to DR and loop.
 */
_start:
    mrs     x1, mpidr_el1
    and     x1, x1, #3
    cbnz    x1, 1f

    /* UART DR address — confirmed from firmware output */
    ldr     x9, =0x1F00030000

    /* Wait for TX FIFO not full (FR bit 5), then send char */
.macro PUTC ch
5:  ldr     w2, [x9, #0x18]    /* PL011_FR */
    tst     w2, #(1 << 5)      /* TXFF? */
    b.ne    5b
    mov     w0, #\ch
    str     w0, [x9]           /* PL011_DR */
.endm

    PUTC 'P'
    PUTC 'I'
    PUTC 'O'
    PUTC 'S'
    PUTC '\r'
    PUTC '\n'

    /* Heartbeat dots forever */
2:  PUTC '.'
    mov     x3, #0x800000
3:  sub     x3, x3, #1
    cbnz    x3, 3b
    b       2b

1:  wfe
    b       1b
