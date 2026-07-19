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

    mrs     x0, CurrentEL
    lsr     x0, x0, #2
    cmp     x0, #2
    b.ne    .Lel1

    /* Minimal firmware EL2 -> EL1 handoff. Stage0 does not need an MMU, but
     * the SD/timer path must run under the same EL1 timer and SCTLR baseline
     * as the proven full kernel/provisioner. */
    mrs     x0, cnthctl_el2
    orr     x0, x0, #3
    msr     cnthctl_el2, x0
    msr     cntvoff_el2, xzr
    mov     x0, #0x33FF
    msr     cptr_el2, x0
    msr     hstr_el2, xzr
    mov     x0, #(3 << 20)
    msr     cpacr_el1, x0
    mov     x0, #(1 << 31)
    msr     hcr_el2, x0
    mov     x0, #0x0800
    movk    x0, #0x30D0, lsl #16
    msr     sctlr_el1, x0
    mov     x0, #0x3C5
    msr     spsr_el2, x0
    adr     x0, .Lel1
    msr     elr_el2, x0
    eret

.Lel1:
    msr     SPSel, #1
    mrs     x0, cpacr_el1
    orr     x0, x0, #(3 << 20)
    msr     cpacr_el1, x0
    isb
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
    br      x3
bootstrap_trampoline_end:
