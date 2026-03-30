.section .text.boot
.global _start
.extern canary_main

_start:
    mrs     x0, mpidr_el1
    and     x0, x0, #0x3
    cbnz    x0, .Lpark

    ldr     x1, =__stack_top
    mov     sp, x1
    bl      canary_main

.Lhang:
    wfe
    b       .Lhang

.Lpark:
    wfe
    b       .Lpark
