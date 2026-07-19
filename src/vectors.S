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
.extern irq_vector_minimal_mode
.extern irq_vector_minimal_count
.extern irq_vector_minimal_last_iar
.extern core_asm_stage
.weak debug_freeze_cooperative_dispatch

.section .bss
.balign 4
.global irq_vector_minimal_mode
irq_vector_minimal_mode:
    .word 0
.global irq_vector_minimal_count
irq_vector_minimal_count:
    .word 0
.global irq_vector_minimal_last_iar
irq_vector_minimal_last_iar:
    .word 0

/* ---- EL2 → EL1 transition (called from _start before kernel_main) ---- */
.section .text
.global el2_to_el1
el2_to_el1:
    mrs     x3, mpidr_el1
    ubfx    x3, x3, #8, #8
    ldr     x4, =core_asm_stage
    mov     w5, #0x30
    str     w5, [x4, x3, lsl #2]

    /* Check current EL */
    mrs     x0, CurrentEL
    lsr     x0, x0, #2
    cmp     x0, #2
    b.ne    .Lfrom_el1

    mov     w5, #0x31
    str     w5, [x4, x3, lsl #2]

    /* Record that we booted at EL2 and install EL2 vectors */
    ldr     x1, =el2_boot_el_state
    mov     w2, #2
    str     w2, [x1]
    mov     w5, #0x32
    str     w5, [x4, x3, lsl #2]
    /* Secondary cores only need a minimal EL2->EL1 drop here. Avoid
     * touching VBAR_EL2/SP_EL2 in this shared path; live Pi 5 testing
     * shows secondary cores can wedge there before the EL1 transition. */
    isb
    mov     w5, #0x33
    str     w5, [x4, x3, lsl #2]

    /* HCR_EL2: RW=1 (EL1 is AArch64), no trapping */
    mov     x0, #(1 << 31)         /* RW bit */
    msr     hcr_el2, x0
    mov     w5, #0x34
    str     w5, [x4, x3, lsl #2]

    /* Enable timer access from EL1 */
    mrs     x0, cnthctl_el2
    orr     x0, x0, #3             /* EL1PCTEN + EL1PCEN */
    msr     cnthctl_el2, x0
    msr     cntvoff_el2, xzr       /* no virtual offset */
    mov     w5, #0x35
    str     w5, [x4, x3, lsl #2]

    /* Enable NEON/FPU at EL2: clear TFP (bit 10) so EL1 SIMD/FP is not trapped.
     * RES1 bits [13:12] and [9:0] must stay set → 0x33BF. */
    mov     x0, #0x33BF
    msr     cptr_el2, x0
    mov     w5, #0x36
    str     w5, [x4, x3, lsl #2]

    /* SPSR_EL2: return to EL1h with IRQs masked initially */
    mov     x0, #0x3C5             /* EL1h, DAIF masked */
    msr     spsr_el2, x0
    mov     w5, #0x37
    str     w5, [x4, x3, lsl #2]

    /* ELR_EL2: return address = label after eret */
    adr     x0, .Lel1_entry
    msr     elr_el2, x0

    mov     w5, #0x38
    str     w5, [x4, x3, lsl #2]

    eret

.Lel1_entry:
    mrs     x3, mpidr_el1
    ubfx    x3, x3, #8, #8
    ldr     x4, =core_asm_stage
    mov     w5, #0x39
    str     w5, [x4, x3, lsl #2]
    /* Now at EL1 — enable NEON */
    mrs     x0, cpacr_el1
    orr     x0, x0, #(3 << 20)
    msr     cpacr_el1, x0
    isb
    b       .Lret

.Lfrom_el1:
    mrs     x3, mpidr_el1
    ubfx    x3, x3, #8, #8
    ldr     x4, =core_asm_stage
    mov     w5, #0x3A
    str     w5, [x4, x3, lsl #2]
    ldr     x1, =el2_boot_el_state
    mov     w2, #1
    str     w2, [x1]
.Lret:
    /* Both entry paths return to C at EL1. QEMU PSCI starts secondaries
     * directly at EL1, so enabling FP/SIMD only after the EL2 eret left that
     * path unable to execute compiler-generated NEON (core_env_init traps on
     * its first `movi`). Establish the same CPACR_EL1 contract here for both. */
    mrs     x0, cpacr_el1
    orr     x0, x0, #(3 << 20)
    msr     cpacr_el1, x0
    isb
    mrs     x3, mpidr_el1
    ubfx    x3, x3, #8, #8
    ldr     x4, =core_asm_stage
    mov     w5, #0x3B
    str     w5, [x4, x3, lsl #2]
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
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    ldr     x12, =0x09000000
#else
    ldr     x12, =0x107C201000
#endif
1:  ldr     w13, [x12, #0x18]
    tbnz    w13, #5, 1b
    mov     w13, #'2'
    str     w13, [x12, #0x00]
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

.global debug_freeze_cooperative_point
debug_freeze_cooperative_point:
    SAVE_CONTEXT
    ldr     x1, [sp, #(15 * 16)]       /* saved x30 = caller return PC */
    str     x1, [sp, #(15 * 16 + 8)]   /* frame.elr */
    mrs     x1, daif
    str     x1, [sp, #(16 * 16)]       /* diagnostic PSTATE subset */
    mov     x0, sp
    ldr     x1, =debug_freeze_cooperative_dispatch
    cbz     x1, 1f                     /* stage0/provisioner do not link it */
    blr     x1
1:
    RESTORE_CONTEXT
    ret

irq_handler:
    stp     x16, x17, [sp, #-16]!
    ldr     x16, =irq_vector_minimal_mode
    ldr     w17, [x16]
    cbnz    w17, .Lirq_minimal

    ldp     x16, x17, [sp], #16
    SAVE_CONTEXT
    mov     x0, sp
    bl      irq_dispatch        /* C function */
    RESTORE_CONTEXT
    eret

.Lirq_minimal:
    ldr     x16, =irq_vector_minimal_count
    ldr     w17, [x16]
    add     w17, w17, #1
    str     w17, [x16]

    mov     x16, #2
    msr     cntp_ctl_el0, x16          /* IMASK=1, ENABLE=0 */
    isb

    ldr     x16, =0x107FFFA000         /* GICC base */
    ldr     w17, [x16, #0x00C]         /* IAR */
    ldr     x16, =irq_vector_minimal_last_iar
    str     w17, [x16]
    cmp     w17, #0x3ff
    b.eq    .Lirq_minimal_done
    ldr     x16, =0x107FFFA000
    str     w17, [x16, #0x010]         /* EOIR must echo full IAR */
    dsb     sy
    isb
.Lirq_minimal_done:
    ldp     x16, x17, [sp], #16
    eret

sync_handler:
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    ldr     x12, =0x09000000
#else
    ldr     x12, =0x107C201000
#endif
1:  ldr     w13, [x12, #0x18]
    tbnz    w13, #5, 1b
    mov     w13, #'1'
    str     w13, [x12, #0x00]
    SAVE_CONTEXT
    mov     x0, sp
    mrs     x1, esr_el1
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
