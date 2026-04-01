/*
 * start.S - aarch64 boot for Pi 5 bare metal
 *
 * Core 0: BSS clear, fb init, EL2 crash vectors, EL2→EL1 eret, kernel_main
 * Cores 1-3: parked by firmware, brought up via PSCI from core.c
 */

.section .text.boot
.global _start
.global secondary_entry

_start:
    mrs     x0, mpidr_el1
    and     x0, x0, #0x3
    cbnz    x0, .Lpark          /* only core 0 continues */

    /* ── EL2 phase: stack, NEON, BSS ── */
    ldr     x1, =_start
    mov     sp, x1

    mov     x0, #0x33BF            /* CPTR_EL2: NEON enabled (TFP=0) */
    msr     cptr_el2, x0
    isb

    /* Clear BSS */
    ldr     x1, =__bss_start
    ldr     w2, =__bss_size
.Lbss_loop:
    cbz     w2, .Lbss_done
    str     xzr, [x1], #8
    sub     w2, w2, #1
    cbnz    w2, .Lbss_loop
.Lbss_done:

    /* Init framebuffer at EL2 */
    bl      kernel_fb_early

    /* Stay at EL2 — go straight to kernel_main */
    bl      kernel_main
    b       .Lhalt

/* EL1 landing pad (unused for now — EL2-only boot) */
.Lel1_entry:
    ldr     x1, =_start
    mov     sp, x1
    bl      kernel_main
    b       .Lhalt

/*
 * ── EL2 crash vector table ──
 * All 16 entries land in the same handler that dumps ESR/ELR/FAR to
 * the already-initialised framebuffer, then halts.
 */
.balign 0x800
el2_crash_vectors:
/* Current EL, SP_EL0 — Synchronous */
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
/* Current EL, SP_ELx — Synchronous */
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
/* Lower EL, AArch64 — Synchronous */
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
/* Lower EL, AArch32 — Synchronous */
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash

.Lel2_crash:
    /* Save fault registers and call C crash dumper.
     * x0 = ESR_EL2, x1 = ELR_EL2, x2 = FAR_EL2, x3 = SPSR_EL2 */
    mrs     x0, esr_el2
    mrs     x1, elr_el2
    mrs     x2, far_el2
    mrs     x3, spsr_el2
    bl      kernel_el2_crash
    b       .Lhalt

/*
 * Secondary core entry point.
 */
secondary_entry:
    cmp     x0, #1
    b.eq    .Lstack1
    cmp     x0, #2
    b.eq    .Lstack2
    cmp     x0, #3
    b.eq    .Lstack3
    b       .Lhalt

.macro SECONDARY_SETUP core_id, stack_sym, entry_fn
    mov     x19, #\core_id
    ldr     x1, =\stack_sym
    mov     sp, x1
    bl      el2_to_el1
    mrs     x1, sctlr_el1
    bic     x1, x1, #(1 << 0)
    bic     x1, x1, #(1 << 2)
    bic     x1, x1, #(1 << 12)
    orr     x1, x1, #(1 << 1)
    orr     x1, x1, #(1 << 3)
    msr     sctlr_el1, x1
    isb
    mrs     x1, cpacr_el1
    orr     x1, x1, #(3 << 20)
    msr     cpacr_el1, x1
    isb
    ldr     x1, =\stack_sym
    mov     sp, x1
    sub     x1, x1, #0x4000
    msr     sp_el1, x1
    ldr     x1, =vector_table
    msr     vbar_el1, x1
    isb
    ldr     x1, =shared_ttbr0
    ldr     x1, [x1]
    msr     ttbr0_el1, x1
    ldr     x1, =shared_mair
    ldr     x1, [x1]
    msr     mair_el1, x1
    ldr     x1, =shared_tcr
    ldr     x1, [x1]
    msr     tcr_el1, x1
    tlbi    vmalle1
    dsb     sy
    isb
    mrs     x1, sctlr_el1
    orr     x1, x1, #(1 << 0)
    orr     x1, x1, #(1 << 2)
    orr     x1, x1, #(1 << 12)
    msr     sctlr_el1, x1
    isb
    msr     daifclr, #2
    bl      \entry_fn
    b       .Lhalt
.endm

.Lstack1:
    SECONDARY_SETUP 1, __stack_top_core1, core1_main
.Lstack2:
    SECONDARY_SETUP 2, __stack_top_core2, core2_main
.Lstack3:
    SECONDARY_SETUP 3, __stack_top_core3, core3_main

.Lpark:
    wfe
    b       .Lpark
.Lhalt:
    wfi
    b       .Lhalt
