.section .text.boot
.global _start
.global bootstrap_trampoline
.global bootstrap_trampoline_end

_start:
    mrs     x0, mpidr_el1
    and     x0, x0, #0x3
    cbnz    x0, .Lpark

    ldr     x1, =__stack_top_core0
    mov     sp, x1

    ldr     x1, =__bss_start
    ldr     x2, =__bss_end
.Lbss:
    cmp     x1, x2
    b.hs    .Lbss_done
    str     xzr, [x1], #8
    b       .Lbss
.Lbss_done:
    bl      bootstrap_main

.Lhalt:
    wfi
    b       .Lhalt

.Lpark:
    wfe
    b       .Lpark

/*
 * Runs from high RAM after bootstrap_main copies it there.
 * x0 = destination (0x80000)
 * x1 = source staging address
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
    ldr     x7, [x5], #8
    str     x7, [x4], #8
    sub     x6, x6, #8
    b       1b
2:
    dsb     sy
    ic      iallu
    dsb     sy
    isb
    br      x3
bootstrap_trampoline_end:
