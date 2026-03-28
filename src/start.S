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

    /* Enable NEON / FPU (CPACR_EL1 bits [21:20] = 0b11) */
    mrs     x1, cpacr_el1
    orr     x1, x1, #(3 << 20)
    msr     cpacr_el1, x1
    isb

    /* Core 0: set stack */
    ldr     x1, =__stack_top_core0
    mov     sp, x1

    /* Clear BSS */
    ldr     x1, =__bss_start
    ldr     x2, =__bss_end
.Lbss_loop:
    cmp     x1, x2
    b.ge    .Lbss_done
    str     xzr, [x1], #8
    b       .Lbss_loop
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

.Lstack1:
    /* Enable NEON on secondary core */
    mrs     x1, cpacr_el1
    orr     x1, x1, #(3 << 20)
    msr     cpacr_el1, x1
    isb
    ldr     x1, =__stack_top_core1
    mov     sp, x1
    bl      core1_main
    b       .Lhalt

.Lstack2:
    mrs     x1, cpacr_el1
    orr     x1, x1, #(3 << 20)
    msr     cpacr_el1, x1
    isb
    ldr     x1, =__stack_top_core2
    mov     sp, x1
    bl      core2_main
    b       .Lhalt

.Lstack3:
    mrs     x1, cpacr_el1
    orr     x1, x1, #(3 << 20)
    msr     cpacr_el1, x1
    isb
    ldr     x1, =__stack_top_core3
    mov     sp, x1
    bl      core3_main
    b       .Lhalt

.Lpark:
    wfe
    b       .Lpark

.Lhalt:
    wfi
    b       .Lhalt
