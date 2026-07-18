/*
 * start.S - aarch64 boot for Pi 5 bare metal
 *
 * Core 0: BSS clear, fb init, EL2 crash vectors, EL2→EL1 eret, kernel_main
 * Cores 1-3: parked by firmware, brought up via PSCI from core.c
 */

.section .text.boot
.global _start
.global secondary_entry

.section .bss
.balign 4
.global core_asm_stage
core_asm_stage:
    .space 16

.section .text.boot

#ifndef PIOS_PLATFORM
#define PIOS_PLATFORM 1
#endif
#ifndef PIOS_PLATFORM_QEMU_VIRT
#define PIOS_PLATFORM_QEMU_VIRT 2
#endif

/* GATE (reversible): canonical AArch64 cache bring-up before enabling SCTLR.C/I.
 * Set to 0 and rebuild to revert. A/B health-gated OTA backstops a bad boot.
 * Diagnosing the long-standing "caches enabled but not allocating" A76 bug. */
#ifndef PIOS_CACHE_BRINGUP_FIX
#define PIOS_CACHE_BRINGUP_FIX 1
#endif

/* GATE (reversible): map the low 1GB (kernel RAM) Write-Back cacheable from the
 * boot translation table instead of Non-Cacheable, so the kernel never runs a
 * Non-Cacheable phase that is later swapped to WB (Circle maps RAM WB single-
 * stage; PIOS's NC-then-swap may be poisoning the A76 cache so it won't
 * allocate). Side effect: the scanout framebuffer at 0x3F400000 becomes WB
 * (HDMI may show stale frames) — acceptable for a network-accessed test. Set to
 * 0 and rebuild to revert. */
#ifndef PIOS_CACHE_WB_FROM_BOOT
#define PIOS_CACHE_WB_FROM_BOOT 1
#endif

_start:
    mrs     x0, mpidr_el1
    and     x0, x0, #0x3
    cbnz    x0, .Lpark          /* only core 0 continues */

    /* Stage0 handoff enters here at EL1 with MMU/caches disabled. */
    mrs     x0, CurrentEL
    lsr     x0, x0, #2
    cmp     x0, #1
    b.eq    .Lstage1_from_el1

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

    /* Return to EL1h (SP_EL1). Exception handlers use SP_EL1, so keep the
     * kernel on the same stack selection instead of EL1t/SP_EL0. */
    mov     x0, #0x3C5
    msr     spsr_el2, x0
    ldr     x0, =.Lel1_entry
    msr     elr_el2, x0
    eret

/* ── Stage1 handoff from stage0: already at EL1, must do BSS+FB ourselves. */
.Lstage1_from_el1:
    ldr     x1, =_start
    mov     sp, x1
    mrs     x1, cpacr_el1
    orr     x1, x1, #(3 << 20)
    msr     cpacr_el1, x1
    isb
    ldr     x1, =__bss_start
    ldr     w2, =__bss_size
.Lstage1_bss_loop:
    cbz     w2, .Lstage1_bss_done
    str     xzr, [x1], #8
    sub     w2, w2, #1
    cbnz    w2, .Lstage1_bss_loop
.Lstage1_bss_done:
    bl      kernel_fb_early
    b       .Lel1_entry

/* ── EL1 landing pad ── */
.Lel1_entry:
    msr     SPSel, #1
    isb

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

    /* L1[0]: first 1GB. */
    ldr     x0, =l1_table
#if PIOS_CACHE_WB_FROM_BOOT
    /* Map block 0 (which holds the kernel code window) Write-Back from the very
     * first cache enable. This is the proven breakthrough fix: it eliminates the
     * Non-Cacheable boot window that otherwise poisons the A76 cache so it never
     * allocates. The runtime mmu_enable_caching() then splits block 0 (code WB,
     * rest NC) and maps blocks 1-3 NC so all DMA driver .bss stays coherent.
     * Reverts to the original NC block-0 when the gate is 0. */
    mov     x2, #0x709                 /* ATTR(2) = WB cacheable          */
    str     x2, [x0, #0]
#else
    mov     x2, #0x705                 /* original: NC 1GB block          */
    str     x2, [x0, #0]
#endif

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

#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    /* QEMU ramfb/GOP can be placed at 0x140000000; map that 1GiB block NC. */
    mov     x2, #0x705
    mov     x5, #5
    lsl     x3, x5, #30
    orr     x4, x3, x2
    lsl     x6, x5, #3
    str     x4, [x0, x6]
#endif

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

#if PIOS_CACHE_BRINGUP_FIX
    /* Canonical AArch64 bring-up: clean+invalidate ALL data/unified caches by
     * set/way over every level to PoC, then invalidate the I-cache, BEFORE
     * enabling SCTLR.C/I. PIOS previously skipped this (Circle/Linux do it).
     * clean+invalidate (not plain invalidate) so any dirty BSS-clear lines the
     * firmware's EL2 caches may hold are written back, not lost. Standard Linux
     * __flush_dcache_all algorithm. Clobbers x0-x11,x16,x17 (all scratch here;
     * the stack is reloaded before entering C). Gated by PIOS_CACHE_BRINGUP_FIX
     * so it reverts with a one-line change. */
    mrs     x0, clidr_el1
    and     w3, w0, #0x07000000        /* LoC = CLIDR[26:24]            */
    lsr     w3, w3, #23                /* LoC * 2 (level index step)    */
    cbz     w3, .Lci_done
    mov     w10, #0                    /* w10 = 2 * current level       */
    mov     w8, #1
.Lci_level:
    add     w2, w10, w10, lsr #1       /* w2 = 3 * level (ctype shift)  */
    lsr     w1, w0, w2
    and     w1, w1, #7                 /* cache type at this level      */
    cmp     w1, #2
    b.lt    .Lci_next                  /* <2 => no D/unified cache here  */
    msr     csselr_el1, x10            /* select cache level            */
    isb
    mrs     x1, ccsidr_el1
    and     w2, w1, #7
    add     w2, w2, #4                 /* w2 = log2(line bytes)         */
    ubfx    w4, w1, #3, #10            /* w4 = max way index            */
    clz     w5, w4                     /* w5 = way field bit position   */
    lsl     w9, w4, w5                 /* w9 = way << waybit            */
    lsl     w16, w8, w5
.Lci_way:
    ubfx    w7, w1, #13, #15           /* w7 = max set index            */
    lsl     w7, w7, w2
    lsl     w17, w8, w2
.Lci_set:
    orr     w11, w10, w9               /* level | way                   */
    orr     w11, w11, w7               /* | set                         */
    dc      cisw, x11                  /* clean+invalidate by set/way   */
    subs    w7, w7, w17
    b.ge    .Lci_set
    subs    x9, x9, x16
    b.ge    .Lci_way
.Lci_next:
    add     w10, w10, #2
    cmp     w3, w10
    dsb     sy
    b.gt    .Lci_level
.Lci_done:
    ic      iallu                      /* invalidate all I-cache to PoU */
    dsb     sy
    isb
#endif

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
/* Current EL, SP_EL0 - Synchronous */
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
/* Current EL, SP_ELx - Synchronous */
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
/* Lower EL, AArch64 - Synchronous */
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
.balign 0x80
    b       .Lel2_crash
/* Lower EL, AArch32 - Synchronous */
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
 *
 * Extract the core index from MPIDR_EL1 using the SAME platform-aware
 * convention as core_id() in types.h: real Pi 5 hardware (Cortex-A76)
 * carries the core index in Aff1 (bits 15:8); QEMU's virt machine carries
 * it in Aff0 (bits 1:0) with Aff1=0 for every vCPU (matching
 * PIOS_PSCI_AFF_SHIFT=0 for the QEMU PSCI CPU_ON target_mpidr in core.c).
 * Previously this unconditionally read Aff1 only -- under QEMU every
 * secondary vCPU read Aff1=0, failed all three cmp checks below, and fell
 * straight through to .Lhalt without ever reaching SECONDARY_SETUP: PSCI
 * CPU_ON reported success (the vCPU genuinely started), but the core wrote
 * no core_asm_stage/core_stage progress and could receive no interrupts
 * (GIC CPU interface never brought up), because it parked itself in the
 * first few instructions. Real Pi 5 hardware was unaffected since Aff1 is
 * genuinely nonzero there. Confirmed via 'sgi test'/'core status' showing
 * zero delivery and zero stage progress on cores 1-3 under QEMU even after
 * fixing core_status_snapshot()'s cache-invalidate-before-read (ruling out
 * a stale-read diagnostic artifact) -- the cores were never truly running
 * kernel code at all.
 */
secondary_entry:
    mrs     x0, mpidr_el1
    ubfx    x1, x0, #8, #8      /* Aff1 */
    cbnz    x1, .Lhave_core_id
    and     x1, x0, #3          /* Aff0 fallback (QEMU) */
.Lhave_core_id:
    mov     x0, x1
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
    ldr     x2, =core_asm_stage
    mov     w3, #0x10
    str     w3, [x2, x19, lsl #2]
    bl      el2_to_el1
    ldr     x2, =core_asm_stage
    mov     w3, #0x11
    str     w3, [x2, x19, lsl #2]
    mrs     x1, sctlr_el1
    bic     x1, x1, #(1 << 0)
    bic     x1, x1, #(1 << 2)
    bic     x1, x1, #(1 << 12)
    bic     x1, x1, #(1 << 1)   /* no A  (match core 0: allow unaligned Normal accesses) */
    bic     x1, x1, #(1 << 3)   /* no SA (match core 0) */
    msr     sctlr_el1, x1
    isb
    ldr     x2, =core_asm_stage
    mov     w3, #0x12
    str     w3, [x2, x19, lsl #2]
    ldr     x1, =\stack_sym
    mov     sp, x1
    ldr     x1, =vector_table
    msr     vbar_el1, x1
    isb
    ldr     x2, =core_asm_stage
    mov     w3, #0x13
    str     w3, [x2, x19, lsl #2]
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
    ldr     x2, =core_asm_stage
    mov     w3, #0x14
    str     w3, [x2, x19, lsl #2]
    mrs     x1, sctlr_el1
    orr     x1, x1, #(1 << 0)
    orr     x1, x1, #(1 << 2)
    orr     x1, x1, #(1 << 12)
    bic     x1, x1, #(1 << 1)   /* no A  (match core 0) */
    bic     x1, x1, #(1 << 3)   /* no SA (match core 0) */
    msr     sctlr_el1, x1
    isb
    ldr     x2, =core_asm_stage
    mov     w3, #0x15
    str     w3, [x2, x19, lsl #2]
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
