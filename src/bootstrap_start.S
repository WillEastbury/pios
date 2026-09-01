.section .text.boot
.global _start

#ifndef PIOS_PLATFORM
#define PIOS_PLATFORM 1
#endif
#ifndef PIOS_PLATFORM_QEMU_VIRT
#define PIOS_PLATFORM_QEMU_VIRT 2
#endif

_start:
    msr     daifset, #0xf

    mrs     x0, mpidr_el1
    and     x0, x0, #0x3
    cbnz    x0, .Lpark

    ldr     x1, =__stack_top_core0
    mov     sp, x1

#if PIOS_PLATFORM == PIOS_PLATFORM_PI5
    bl      stage0_firmware_hello
#endif

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
    /* ── Runtime board detection (before ANY MMU/peripheral setup) ──
     * MIDR_EL1 is a CPU identification system register, not an MMIO
     * peripheral, so it's always safe to read here regardless of which
     * board this is -- unlike any peripheral address, which we don't yet
     * know how to map correctly until AFTER this decision. The L1 page
     * table itself must differ by board: Pi5's peripherals live at a high
     * 36-bit address entirely outside the low 4 GiB, while the BCM2837
     * family's (Pi3/Pi Zero 2W) peripherals live INSIDE the low 1-2 GiB
     * that would otherwise be a simple Normal-NC RAM block, so giving them
     * Device memory type requires a genuinely different table layout, not
     * just different addresses. board_detect_init() (src/board_detect.c)
     * also populates g_board_bases/g_board_family for later C-side use
     * (bootstrap.c/sd.c/fb.c's runtime peripheral base resolution). */
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    /* A QEMU-targeted stage0 build only ever runs under QEMU -cpu cortex-a53,
     * which reports the SAME MIDR as a real BCM2837 -- runtime detection
     * cannot (and need not) disambiguate the two. Skip board_detect_init()
     * entirely and go straight to the QEMU-specific table below. */
    b       .Lb_qemu_table
#else
    bl      board_detect_init
    ldr     x0, =g_board_family
    ldr     w0, [x0]
    cmp     w0, #1                      /* BOARD_FAMILY_PI5     */
    b.eq    .Lb_pi5_table
    cmp     w0, #2                      /* BOARD_FAMILY_BCM2837 */
    b.eq    .Lb_bcm2837_table
    cmp     w0, #3                      /* BOARD_FAMILY_PI4     */
    b.eq    .Lb_pi4_table
    /* BOARD_FAMILY_UNKNOWN (0) or anything else: neither peripheral map is
     * known to be safe. Fail closed rather than guess -- an unrecognized
     * core here means genuinely unanticipated hardware, and silently
     * assuming either board's addresses risks an external abort or worse
     * on whatever this actually is. */
    b       .Lhalt
#endif

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
.Lb_pi5_table:
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

    b       .Lb_mmu_enable

    /* ── BCM2711 (Pi 4): UART/mailbox/EMMC2 live at 0xFE00_0000 and GENET
     * at 0xFD58_0000, in the top of the first 4 GiB. Stage0 only needs
     * kernel/staging in L1[0] and Device access to that hole, so L1[3]
     * is a single 1 GiB Device block. Stage2 splits L1[3] so 0xC0000000-
     * 0xFBFFFFFF stays RAM. */
.Lb_pi4_table:
    ldr     x0, =l1_table
    mov     x1, #512
.Lb_zero_l1_pi4:
    str     xzr, [x0], #8
    subs    x1, x1, #1
    b.ne    .Lb_zero_l1_pi4

    ldr     x0, =l1_table
    mov     x2, #0x705                 /* ATTR(1) = Normal-NC             */
    str     x2, [x0, #0]               /* L1[0]: first 1 GiB Normal-NC    */

    mov     x2, #0x401                 /* Device-nGnRnE, ATTR(0)          */
    movk    x2, #0x0060, lsl #48
    ldr     x3, =0xC0000000
    orr     x4, x3, x2
    str     x4, [x0, #24]              /* L1[3]: 0xC0000000-0xFFFFFFFF Device */
    b       .Lb_mmu_enable

    /* ── BCM2837-family (Pi3 B/B+/A+, Pi Zero 2 W): peripherals fall inside
     * the low 4 GiB, so L1[0] and L1[1] cannot be simple 1 GiB blocks with
     * a single memory type each the way Pi5's can:
     *   L1[0] -> l2_table_bcm2837_boot (2 MiB granularity):
     *     L2[0..503]   0x00000000-0x3EFFFFFF (1008 MiB) Normal-NC   RAM
     *     L2[504..511] 0x3F000000-0x3FFFFFFF (16 MiB)   Device      low periph
     *       (UART0/mailbox/EMMC/PM watchdog -- see board_detect.c)
     *   L1[1] 0x40000000-0x7FFFFFFF: single 1 GiB Device block. QA7
     *     (ARM-local peripherals: per-core timer IRQ enables, IPI
     *     mailboxes -- see src/irqc_legacy.c) lives at its very start
     *     (0x40000000); stage0 never touches anything else in this range,
     *     so there is no need to split it further.
     *   L1[2]/L1[3]: left unmapped (fault on any access). Stage0's own RAM
     *     usage (BOOT_STAGING_ADDR/BOOT_TRAMP_ADDR) is entirely within
     *     L1[0]; this loader has no reason to touch, and no confident
     *     knowledge of, anything past 2 GiB on this SoC. */
.Lb_bcm2837_table:
    ldr     x0, =l1_table
    mov     x1, #512
.Lb_zero_l1_bcm:
    str     xzr, [x0], #8
    subs    x1, x1, #1
    b.ne    .Lb_zero_l1_bcm

    ldr     x7, =l2_table_bcm2837_boot
    mov     x1, #512
.Lb_zero_l2_bcm:
    str     xzr, [x7], #8
    subs    x1, x1, #1
    b.ne    .Lb_zero_l2_bcm

    ldr     x0, =l1_table
    ldr     x7, =l2_table_bcm2837_boot
    mov     x5, #0
    mov     x2, #0x705                 /* ATTR(1) = Normal-NC             */
.Lb_bcm_l2_ram:
    lsl     x3, x5, #21
    orr     x4, x3, x2
    str     x4, [x7, x5, lsl #3]
    add     x5, x5, #1
    cmp     x5, #504
    b.lt    .Lb_bcm_l2_ram

    mov     x2, #0x401                 /* Device-nGnRnE, ATTR(0)          */
    movk    x2, #0x0060, lsl #48
.Lb_bcm_l2_periph:
    lsl     x3, x5, #21
    orr     x4, x3, x2
    str     x4, [x7, x5, lsl #3]
    add     x5, x5, #1
    cmp     x5, #512
    b.lt    .Lb_bcm_l2_periph

    ldr     x2, =l2_table_bcm2837_boot
    orr     x2, x2, #3                 /* valid table descriptor          */
    str     x2, [x0, #0]               /* L1[0] -> l2_table_bcm2837_boot  */

    mov     x2, #0x401                 /* Device-nGnRnE, ATTR(0)          */
    movk    x2, #0x0060, lsl #48
    mov     x3, #0x40000000
    orr     x4, x3, x2
    str     x4, [x0, #8]               /* L1[1]: QA7 + rest, Device       */
    b       .Lb_mmu_enable

    /* ── QEMU -M virt: peripherals (GICv2 @0x08000000, PL011 @0x09000000,
     * virtio-mmio @0x0A000000+) all live inside the first 1 GiB, and RAM
     * starts at 0x40000000 (NOT 0, unlike real hardware) -- this is where
     * stage0 itself is linked/loaded and where BOOT_STAGING_ADDR/
     * BOOT_TRAMP_ADDR/BOOT_DST_ADDR all live (see bootstrap.c). QEMU's TCG
     * does not model CPU caches, so mapping the low 1 GiB (including the
     * MMIO peripherals) as plain Normal-NC is safe here -- start.S already
     * does exactly this for the existing direct-boot QEMU kernel, and it
     * works today, so stage0 reuses the identical, proven layout:
     *   L1[0]     first 1 GiB   Normal-NC  GIC/UART/virtio-mmio + low RAM
     *   L1[1..3]  next 3 GiB    Normal-WB  RAM (stage0/stage2/staging/tramp)
     */
.Lb_qemu_table:
    ldr     x0, =l1_table
    mov     x1, #512
.Lb_zero_l1_qemu:
    str     xzr, [x0], #8
    subs    x1, x1, #1
    b.ne    .Lb_zero_l1_qemu

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

    b       .Lb_mmu_enable

.Lb_mmu_enable:
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
