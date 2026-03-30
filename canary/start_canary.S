.section .text.boot
.global _start
.extern canary_main

_start:
    mrs     x0, mpidr_el1
    and     x0, x0, #0x3
    cbnz    x0, .Lpark

    ldr     x1, =__stack_top
    mov     sp, x1

    /* Clear BSS (mbox buffer lives here) */
    ldr     x1, =__bss_start
    ldr     x2, =__bss_end
.Lbss:
    cmp     x1, x2
    b.ge    .Lgo
    str     xzr, [x1], #8
    b       .Lbss
.Lgo:
    bl      canary_main

.Lhang:
    wfe
    b       .Lhang

.Lpark:
    wfe
    b       .Lpark
