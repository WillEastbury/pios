.section .text.trampoline
.global bootstrap_trampoline
.global bootstrap_trampoline_end

/*
 * x0 = destination
 * x1 = source
 * x2 = byte count
 * x3 = entry address
 */
.balign 16
bootstrap_trampoline:
    mov     x4, x0
    mov     x5, x1
    mov     x6, x2
1:
    cbz     x6, 2f
    cmp     x6, #8
    b.lo    3f
    ldr     x7, [x5], #8
    str     x7, [x4], #8
    sub     x6, x6, #8
    b       1b
3:
    ldrb    w7, [x5], #1
    strb    w7, [x4], #1
    sub     x6, x6, #1
    b       1b
2:
    dsb     sy
    ic      iallu
    dsb     sy
    isb
    mrs     x8, sctlr_el1
    bic     x8, x8, #(1 << 0)   /* M: MMU off */
    bic     x8, x8, #(1 << 2)   /* C: D-cache off */
    bic     x8, x8, #(1 << 12)  /* I: I-cache off */
    msr     sctlr_el1, x8
    dsb     sy
    isb
    br      x3
bootstrap_trampoline_end:
