.section .text.boot
.global _start

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

    /* Minimal firmware EL2 -> EL1 handoff. The SD/timer path must run under the
     * same EL1 timer and SCTLR baseline as the proven full kernel/provisioner;
     * the MMU is then enabled below (see .Lbss_done) before any SD access. */
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
    /* ── EL1 MMU bring-up (required before touching SD/EMMC) ──
     * sd.c's EMMC PIO bring-up and the VideoCore mailbox are only proven
     * correct with the MMU enabled and the low 1 GiB mapped Normal-NC — the
     * exact configuration the one-shot provisioner uses (build_provisioner.bat
     * forces PIOS_CACHE_WB_FROM_BOOT=0). With the MMU off every RAM access is
     * Device-nGnRnE and the CPU runs uncached; under that the EMMC card
     * identification handshake (CMD55 + ACMD41) never completes. Reuse the
     * proven start.S MAIR/TCR/SCTLR/attr magic. Keep the whole low 1 GiB NC so
     * no L2 split table (l2_table_boot, not linked here) is needed and the SD
     * PIO buffers, staging area, trampoline, and framebuffer stay coherent.
     *
     *   L1[0]        first 1 GiB    Normal-NC  code/stacks/SD buffers/FB (<1 GiB)
     *   L1[1..3]     next 3 GiB     Normal-WB  (unused by stage0)
     *   L1[64..67]   0x10_00000000  Device     UART, mailbox, EMMC2, PM watchdog
     *   L1[124..127] 0x1F_00000000  Device     RP1 (unused by stage0)
     */
    ldr     x0, =l1_table
    mov     x1, #512
.Lb_zero_l1:
    str     xzr, [x0], #8
    subs    x1, x1, #1
    b.ne    .Lb_zero_l1

    ldr     x0, =l1_table
    mov     x2, #0x705                 /* ATTR(1) = Normal-NC             */
    str     x2, [x0, #0]               /* L1[0]: first 1 GiB Normal-NC    */

    mov     x2, #0x709                 /* ATTR(2) = Normal-WB             */
    mov     x3, #0x40000000
    orr     x4, x3, x2
    str     x4, [x0, #8]               /* L1[1]                           */
    ldr     x3, =0x80000000
    orr     x4, x3, x2
    str     x4, [x0, #16]              /* L1[2]                           */
    ldr     x3, =0xC0000000
    orr     x4, x3, x2
    str     x4, [x0, #24]              /* L1[3]                           */

    mov     x2, #0x401                 /* Device-nGnRnE, ATTR(0)          */
    movk    x2, #0x0060, lsl #48
    mov     x5, #64
.Lb_periph:
    lsl     x3, x5, #30
    orr     x4, x3, x2
    lsl     x6, x5, #3
    str     x4, [x0, x6]
    add     x5, x5, #1
    cmp     x5, #68
    b.lt    .Lb_periph

    mov     x5, #124
.Lb_rp1:
    lsl     x3, x5, #30
    orr     x4, x3, x2
    lsl     x6, x5, #3
    str     x4, [x0, x6]
    add     x5, x5, #1
    cmp     x5, #128
    b.lt    .Lb_rp1

    dsb     sy

    ldr     x1, =0xBBFF4400            /* MAIR_EL1 (matches start.S)      */
    msr     mair_el1, x1
    isb
    ldr     x1, =0x200803519           /* TCR_EL1 (matches start.S)       */
    msr     tcr_el1, x1
    isb
    ldr     x1, =l1_table
    msr     ttbr0_el1, x1
    msr     ttbr1_el1, xzr
    isb
    tlbi    vmalle1
    dsb     sy
    isb

    /* Enable MMU + caches. NC low 1 GiB means the SD PIO buffers never
     * allocate dirty D-cache lines, so the provisioner's proven no-set/way
     * bring-up path is safe here too. */
    mrs     x1, sctlr_el1
    orr     x1, x1, #(1 << 0)          /* M  = MMU                        */
    orr     x1, x1, #(1 << 2)          /* C  = D-cache                    */
    orr     x1, x1, #(1 << 12)         /* I  = I-cache                    */
    bic     x1, x1, #(1 << 1)          /* no A                            */
    bic     x1, x1, #(1 << 3)          /* no SA                           */
    msr     sctlr_el1, x1
    isb

    ldr     x1, =__stack_top_core0
    mov     sp, x1
    bl      bootstrap_main

.Lhalt:
    wfi
    b       .Lhalt

.Lpark:
    wfe
    b       .Lpark
