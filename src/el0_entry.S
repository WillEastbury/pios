/*
 * el0_entry.S - minimal EL1 -> EL0 entry helper.
 *
 * x0 = EL0 entry PC
 * x1 = EL0 stack pointer
 */
.section .text
.global proc_el0_enter
proc_el0_enter:
    msr     sp_el0, x1
    msr     elr_el1, x0
    mov     x2, #0x3c0        /* EL0t + DAIF masked */
    msr     spsr_el1, x2
    mov     x0, xzr           /* no direct-KPI table in EL0 */
    isb
    eret
