/*
 * start.S - aarch64 boot for Pi 5 bare metal
 *
 * Core 0: clear BSS, call kernel_main()
 * Cores 1-3: parked by firmware, brought up via PSCI from core.c
 */

.section .text.boot
.global _start
.global secondary_entry

_start:
    mrs     x0, mpidr_el1
    and     x0, x0, #0x3
    cbnz    x0, .Lpark          /* only core 0 continues */

    /* Stack — use same location as canary: just below kernel load address.
     * __stack_top_core0 is deep in BSS (~3MB), which may be unmapped by
     * firmware.  _start (0x80000) grows downward into safe low RAM. */
    ldr     x1, =_start
    mov     sp, x1

    /* Skip EL2→EL1 for now — run at whatever EL firmware gave us,
     * exactly like the working canary kernel. */

    /* Clear BSS */
    ldr     x1, =__bss_start
    ldr     w2, =__bss_size
.Lbss_loop:
    cbz     w2, .Lbss_done
    str     xzr, [x1], #8
    sub     w2, w2, #1
    cbnz    w2, .Lbss_loop
.Lbss_done:

    bl      kernel_main
    b       .Lhalt

/*
 * Secondary core entry point.
 * Called by PSCI CPU_ON with x0 = context_id (core number).
 */
secondary_entry:
    cmp     x0, #1
    b.eq    .Lstack1
    cmp     x0, #2
    b.eq    .Lstack2
    cmp     x0, #3
    b.eq    .Lstack3
    b       .Lhalt

/*
 * Secondary core setup macro:
 *   1. EL2→EL1 transition
 *   2. SCTLR_EL1 safe baseline
 *   3. NEON enable
 *   4. Per-core SP (main + exception)
 *   5. VBAR_EL1 (exception vectors)
 *   6. TTBR0_EL1 (share same page tables as core 0)
 *   7. Jump to per-core C entry
 */
.macro SECONDARY_SETUP core_id, stack_sym, entry_fn
    /* Save core_id across bl */
    mov     x19, #\core_id
    ldr     x1, =\stack_sym
    mov     sp, x1

    /* EL2 → EL1 */
    bl      el2_to_el1

    /* SCTLR_EL1: safe baseline */
    mrs     x1, sctlr_el1
    bic     x1, x1, #(1 << 0)
    bic     x1, x1, #(1 << 2)
    bic     x1, x1, #(1 << 12)
    orr     x1, x1, #(1 << 1)
    orr     x1, x1, #(1 << 3)
    msr     sctlr_el1, x1
    isb

    /* Enable NEON */
    mrs     x1, cpacr_el1
    orr     x1, x1, #(3 << 20)
    msr     cpacr_el1, x1
    isb

    /* Per-core SP_EL0 (main stack) */
    ldr     x1, =\stack_sym
    mov     sp, x1

    /* Per-core SP_EL1 (exception stack, 16KB below) */
    sub     x1, x1, #0x4000
    msr     sp_el1, x1

    /* VBAR_EL1: same vector table as core 0 */
    ldr     x1, =vector_table
    msr     vbar_el1, x1
    isb

    /* TTBR0_EL1: point to shared L1 page table (set by core 0's mmu_init).
     * We read the symbol directly — core 0 has already populated it. */
    ldr     x1, =shared_ttbr0
    ldr     x1, [x1]
    msr     ttbr0_el1, x1

    /* MAIR_EL1 + TCR_EL1: replicate core 0's values */
    ldr     x1, =shared_mair
    ldr     x1, [x1]
    msr     mair_el1, x1

    ldr     x1, =shared_tcr
    ldr     x1, [x1]
    msr     tcr_el1, x1

    /* TLB invalidate before enabling MMU */
    tlbi    vmalle1
    dsb     sy
    isb

    /* Enable MMU + caches (M=1, C=1, I=1) */
    mrs     x1, sctlr_el1
    orr     x1, x1, #(1 << 0)
    orr     x1, x1, #(1 << 2)
    orr     x1, x1, #(1 << 12)
    msr     sctlr_el1, x1
    isb

    /* Unmask IRQs (clear DAIF.I bit) */
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
