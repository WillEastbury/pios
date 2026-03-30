.section .text.boot
.global _start
.extern canary_main

/*
 * ARM64 Linux image header (64 bytes).
 */
_start:
    b       _real_start             /* code0: branch over header */
    .word   0                       /* code1: reserved */
    .quad   0                       /* text_offset */
    .quad   0                       /* image_size (0 = unknown) */
    .quad   0                       /* flags */
    .quad   0                       /* reserved */
    .quad   0                       /* reserved */
    .quad   0                       /* reserved */
    .ascii  "ARM\x64"              /* magic: 0x644d5241 */
    .word   0                       /* PE offset (unused) */

_real_start:
    /* Cortex-A76: core ID in Aff1 (bits [15:8]). Park non-zero cores. */
    mrs     x0, mpidr_el1
    ubfx    x0, x0, #8, #8         /* extract Aff1 */
    cbnz    x0, .Lpark

    /* Disable caches at whatever EL we're at */
    mrs     x0, CurrentEL
    lsr     x0, x0, #2
    cmp     x0, #2
    b.ne    .Lel1_cache
    mrs     x1, sctlr_el2
    bic     x1, x1, #(1 << 2)
    bic     x1, x1, #(1 << 12)
    msr     sctlr_el2, x1
    isb
    b       .Lcache_done
.Lel1_cache:
    mrs     x1, sctlr_el1
    bic     x1, x1, #(1 << 2)
    bic     x1, x1, #(1 << 12)
    msr     sctlr_el1, x1
    isb

.Lcache_done:
    ldr     x1, =__stack_top
    mov     sp, x1

    /* Clear BSS */
    ldr     x1, =__bss_start
    ldr     x2, =__bss_end
.Lbss:
    cmp     x1, x2
    b.ge    .Lgo
    str     xzr, [x1], #8
    b       .Lbss
.Lgo:
    dsb     sy
    bl      canary_main

.Lhang:
    wfe
    b       .Lhang

.Lpark:
    wfe
    b       .Lpark
