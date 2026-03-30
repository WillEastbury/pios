/*
 * vectors.S - AArch64 exception vector table + EL2→EL1 drop
 *
 * Vector table: 16 entries × 0x80 bytes each = 0x800 bytes.
 * We handle IRQs; everything else dumps diagnostics and halts.
 */

.section .text
.extern el2_hvc_trap
.extern el2_sync_fault_trap
.extern el2_boot_el_state

/* ---- EL2 → EL1 transition (called from _start before kernel_main) ---- */
.global el2_to_el1
el2_to_el1:
    /* Check current EL */
    mrs     x0, CurrentEL
    lsr     x0, x0, #2
    cmp     x0, #2
    b.ne    .Lfrom_el1

    /* Record that we booted at EL2 and install EL2 vectors */
    ldr     x1, =el2_boot_el_state
    mov     w2, #2
    str     w2, [x1]
    ldr     x1, =el2_vector_table
    msr     vbar_el2, x1
    mov     x1, sp
    msr     sp_el2, x1
    isb

    /* HCR_EL2: RW=1 (EL1 is AArch64), no trapping */
    mov     x0, #(1 << 31)         /* RW bit */
    msr     hcr_el2, x0

    /* Enable timer access from EL1 */
    mrs     x0, cnthctl_el2
    orr     x0, x0, #3             /* EL1PCTEN + EL1PCEN */
    msr     cnthctl_el2, x0
    msr     cntvoff_el2, xzr       /* no virtual offset */

    /* Enable NEON/FPU at EL2 (CPTR_EL2) */
    mov     x0, #0x33FF
    msr     cptr_el2, x0

    /* SPSR_EL2: return to EL1h with IRQs masked initially */
    mov     x0, #0x3C5             /* EL1h, DAIF masked */
    msr     spsr_el2, x0

    /* ELR_EL2: return address = label after eret */
    adr     x0, .Lel1_entry
    msr     elr_el2, x0

    eret

.Lel1_entry:
    /* Now at EL1 — enable NEON */
    mrs     x0, cpacr_el1
    orr     x0, x0, #(3 << 20)
    msr     cpacr_el1, x0
    isb
    b       .Lret

.Lfrom_el1:
    ldr     x1, =el2_boot_el_state
    mov     w2, #1
    str     w2, [x1]
.Lret:
    ret

/* ---- EL2 exception vectors (HVC trap path) ---- */
.balign 0x800
.global el2_vector_table
el2_vector_table:
.balign 0x80
    b       el2_sync_handler
.balign 0x80
    b       el2_irq_handler
.balign 0x80
    b       el2_irq_handler
.balign 0x80
    b       el2_irq_handler
.balign 0x80
    b       el2_sync_handler
.balign 0x80
    b       el2_irq_handler
.balign 0x80
    b       el2_irq_handler
.balign 0x80
    b       el2_irq_handler
.balign 0x80
    b       el2_sync_handler
.balign 0x80
    b       el2_irq_handler
.balign 0x80
    b       el2_irq_handler
.balign 0x80
    b       el2_irq_handler
.balign 0x80
    b       el2_sync_handler
.balign 0x80
    b       el2_irq_handler
.balign 0x80
    b       el2_irq_handler
.balign 0x80
    b       el2_irq_handler

el2_irq_handler:
    eret

el2_sync_handler:
    sub     sp, sp, #32
    stp     x1, x2, [sp, #0]
    stp     x3, x4, [sp, #16]
    mrs     x9, esr_el2
    ubfx    x10, x9, #26, #6
    cmp     x10, #0x16            /* HVC from lower EL */
    b.ne    .Lel2_sync_bad
    mrs     x11, elr_el2
    add     x11, x11, #4
    msr     elr_el2, x11
    bl      el2_hvc_trap          /* x0=fid, x1..x4 args, x0 return */
    b       .Lel2_sync_out
.Lel2_sync_bad:
    mrs     x0, esr_el2
    mrs     x1, elr_el2
    bl      el2_sync_fault_trap
.Lel2_sync_out:
    ldp     x1, x2, [sp, #0]
    ldp     x3, x4, [sp, #16]
    add     sp, sp, #32
    eret

/* ---- Exception Vector Table ---- */

.balign 0x800
.global vector_table
vector_table:

/* ---- Current EL, SP_EL0 ---- */
.balign 0x80
    b       sync_handler        /* Synchronous */
.balign 0x80
    b       irq_handler         /* IRQ */
.balign 0x80
    b       fiq_handler         /* FIQ */
.balign 0x80
    b       serror_handler      /* SError */

/* ---- Current EL, SP_ELx ---- */
.balign 0x80
    b       sync_handler
.balign 0x80
    b       irq_handler
.balign 0x80
    b       fiq_handler
.balign 0x80
    b       serror_handler

/* ---- Lower EL, AArch64 ---- */
.balign 0x80
    b       sync_handler
.balign 0x80
    b       irq_handler
.balign 0x80
    b       fiq_handler
.balign 0x80
    b       serror_handler

/* ---- Lower EL, AArch32 ---- */
.balign 0x80
    b       sync_handler
.balign 0x80
    b       irq_handler
.balign 0x80
    b       fiq_handler
.balign 0x80
    b       serror_handler

/* ---- Common handlers ---- */

/* Save all registers, call C handler, restore */
.macro  SAVE_CONTEXT
    sub     sp, sp, #(34 * 8)
    stp     x0, x1, [sp, #(0 * 16)]
    stp     x2, x3, [sp, #(1 * 16)]
    stp     x4, x5, [sp, #(2 * 16)]
    stp     x6, x7, [sp, #(3 * 16)]
    stp     x8, x9, [sp, #(4 * 16)]
    stp     x10, x11, [sp, #(5 * 16)]
    stp     x12, x13, [sp, #(6 * 16)]
    stp     x14, x15, [sp, #(7 * 16)]
    stp     x16, x17, [sp, #(8 * 16)]
    stp     x18, x19, [sp, #(9 * 16)]
    stp     x20, x21, [sp, #(10 * 16)]
    stp     x22, x23, [sp, #(11 * 16)]
    stp     x24, x25, [sp, #(12 * 16)]
    stp     x26, x27, [sp, #(13 * 16)]
    stp     x28, x29, [sp, #(14 * 16)]
    mrs     x0, elr_el1
    mrs     x1, spsr_el1
    stp     x30, x0, [sp, #(15 * 16)]
    str     x1, [sp, #(16 * 16)]
.endm

.macro  RESTORE_CONTEXT
    ldr     x1, [sp, #(16 * 16)]
    ldp     x30, x0, [sp, #(15 * 16)]
    msr     spsr_el1, x1
    msr     elr_el1, x0
    ldp     x28, x29, [sp, #(14 * 16)]
    ldp     x26, x27, [sp, #(13 * 16)]
    ldp     x24, x25, [sp, #(12 * 16)]
    ldp     x22, x23, [sp, #(11 * 16)]
    ldp     x20, x21, [sp, #(10 * 16)]
    ldp     x18, x19, [sp, #(9 * 16)]
    ldp     x16, x17, [sp, #(8 * 16)]
    ldp     x14, x15, [sp, #(7 * 16)]
    ldp     x12, x13, [sp, #(6 * 16)]
    ldp     x10, x11, [sp, #(5 * 16)]
    ldp     x8, x9, [sp, #(4 * 16)]
    ldp     x6, x7, [sp, #(3 * 16)]
    ldp     x4, x5, [sp, #(2 * 16)]
    ldp     x2, x3, [sp, #(1 * 16)]
    ldp     x0, x1, [sp, #(0 * 16)]
    add     sp, sp, #(34 * 8)
.endm

irq_handler:
    SAVE_CONTEXT
    mov     x0, sp
    bl      irq_dispatch        /* C function */
    RESTORE_CONTEXT
    eret

sync_handler:
    SAVE_CONTEXT
    mrs     x0, esr_el1
    mrs     x1, elr_el1
    mrs     x2, far_el1
    bl      sync_exception      /* C function */
    RESTORE_CONTEXT
    eret

fiq_handler:
    b       fiq_handler         /* spin — unused */

serror_handler:
    SAVE_CONTEXT
    mrs     x0, esr_el1
    bl      serror_exception    /* C function */
    RESTORE_CONTEXT
    eret
