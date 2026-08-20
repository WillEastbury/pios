/*
 * el0_entry.S - minimal EL1 -> EL0 entry helper.
 *
 * x0 = EL0 entry PC
 * x1 = EL0 stack pointer (SP_EL0 -- NOT the EL1 exception stack; see ADR-021)
 * x2 = SPSR_EL1 to eret with (PROC_ENTRY_SPSR_EL0, supplied by the caller so the
 *      C entry contract and the actual eret can never drift apart)
 * x3 = EL0 scheduler shared-slot index
 *
 * On entry SP is the process's private EL1 exception stack, which must NOT be
 * the same region as x1: once IRQs are enabled at EL0 the PE forces SPSel=1 on
 * exception entry and pushes the trap frame onto SP_EL1. That frame carries
 * SPSR, so a process able to reach it could eret itself to EL1.
 */
.section .text
.global proc_el0_enter
proc_el0_enter:
    msr     sp_el0, x1
    msr     elr_el1, x0
    msr     spsr_el1, x2
    mov     x21, x3
    mov     x0, xzr           /* no direct-KPI table in EL0 */
    isb
    eret
