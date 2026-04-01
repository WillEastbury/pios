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

    /* Init framebuffer at EL2 (proven working) */
    bl      kernel_fb_early

    /* Install EL2 crash vectors so we catch any eret faults */
    adr     x0, el2_crash_vectors
    msr     vbar_el2, x0
    isb

    /* ── EL2 → EL1 drop (matching Circle's proven sequence) ── */

    /* Timers */
    mrs     x0, cnthctl_el2
    orr     x0, x0, #3
    msr     cnthctl_el2, x0
    msr     cntvoff_el2, xzr

    /* Copy MIDR/MPIDR to virtual regs */
    mrs     x0, midr_el1
    mrs     x1, mpidr_el1
    msr     vpidr_el2, x0
    msr     vmpidr_el2, x1

    /* Disable coprocessor traps */
    mov     x0, #0x33FF
    msr     cptr_el2, x0           /* no traps to EL2 (TFP doesn't matter, we're leaving) */
    msr     hstr_el2, xzr          /* no hypervisor system traps */
    mov     x0, #(3 << 20)
    msr     cpacr_el1, x0          /* NEON at EL1 */

    /* HCR_EL2: 64-bit EL1 */
    mov     x0, #(1 << 31)
    msr     hcr_el2, x0

    /* SCTLR_EL1: RES1 bits set (Circle's value: 0x30D00800) */
    mov     x0, #0x0800
    movk    x0, #0x30D0, lsl #16
    msr     sctlr_el1, x0

    /* Return to EL1_SP0 (EL1t) */
    mov     x0, #0x3C4
    msr     spsr_el2, x0
    ldr     x0, =.Lel1_entry
    msr     elr_el2, x0
    eret

/* ── EL1 landing pad ── */
.Lel1_entry:
    /* Stack */
    ldr     x1, =_start
    mov     sp, x1

    /* NEON (confirm) */
    mrs     x1, cpacr_el1
    orr     x1, x1, #(3 << 20)
    msr     cpacr_el1, x1
    isb

    /* Exception vectors */
    ldr     x1, =vector_table
    msr     vbar_el1, x1
    isb

    /* ── EL1 MMU setup (same page table, EL1 registers) ── */

    /* Zero L1 table */
    ldr     x0, =l1_table
    mov     x1, #512
.Lzero_l1:
    str     xzr, [x0], #8
    subs    x1, x1, #1
    b.ne    .Lzero_l1

    /* L1[0]: Non-Cacheable RAM (FB at 0x3F400000) — ATTR(1) = 0x705 */
    ldr     x0, =l1_table
    mov     x2, #0x705
    str     x2, [x0, #0]

    /* L1[1..3]: Cacheable RAM — ATTR(2) = 0x709 */
    mov     x2, #0x709
    mov     x3, #0x40000000
    orr     x4, x3, x2
    str     x4, [x0, #8]
    ldr     x3, =0x80000000
    orr     x4, x3, x2
    str     x4, [x0, #16]
    ldr     x3, =0xC0000000
    orr     x4, x3, x2
    str     x4, [x0, #24]

    /* L1[64..67]: Device (BCM2712 peripherals at 0x10_00000000+) */
    mov     x2, #0x401
    movk    x2, #0x0060, lsl #48
    mov     x5, #64
.Lperiph:
    lsl     x3, x5, #30
    orr     x4, x3, x2
    lsl     x6, x5, #3
    str     x4, [x0, x6]
    add     x5, x5, #1
    cmp     x5, #68
    b.lt    .Lperiph

    /* L1[124..127]: Device (RP1) */
    mov     x5, #124
.Lrp1:
    lsl     x3, x5, #30
    orr     x4, x3, x2
    lsl     x6, x5, #3
    str     x4, [x0, x6]
    add     x5, x5, #1
    cmp     x5, #128
    b.lt    .Lrp1

    dsb     sy

    /* MAIR_EL1 */
    ldr     x1, =0xBBFF4400
    msr     mair_el1, x1
    isb

    /* TCR_EL1: T0SZ=25, IRGN0=WB, ORGN0=WB, SH0=Inner, TG0=4K, IPS=36b, EPD1=1 */
    ldr     x1, =0x200803519
    msr     tcr_el1, x1
    isb

    /* TTBR0_EL1 */
    ldr     x1, =l1_table
    msr     ttbr0_el1, x1
    msr     ttbr1_el1, xzr
    isb

    /* TLB flush */
    tlbi    vmalle1
    dsb     sy
    isb

    /* Enable EL1 MMU + caches */
    mrs     x1, sctlr_el1
    orr     x1, x1, #(1 << 0)   /* M  = MMU */
    orr     x1, x1, #(1 << 2)   /* C  = D-cache */
    orr     x1, x1, #(1 << 12)  /* I  = I-cache */
    bic     x1, x1, #(1 << 1)   /* no A */
    bic     x1, x1, #(1 << 3)   /* no SA */
    msr     sctlr_el1, x1
    isb

    /* ── EL1 + MMU ON — move stack and enter C ── */
    ldr     x1, =__stack_top_core0
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
