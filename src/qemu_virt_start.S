/*
 * qemu_virt_start.S - minimal AArch64 QEMU virt entry.
 *
 * This is intentionally separate from the Pi5 start path while the platform
 * split is underway. It runs with MMU off, drops EL2->EL1 if needed, clears BSS,
 * and enters qemu_virt_main().
 */

.section .text.boot
.global _start

_start:
    mrs     x0, mpidr_el1
    and     x0, x0, #0xff
    cbnz    x0, 1f

    ldr     x1, =__stack_top
    mov     sp, x1

    mrs     x0, CurrentEL
    lsr     x0, x0, #2
    cmp     x0, #2
    b.ne    2f

    mrs     x0, cnthctl_el2
    orr     x0, x0, #3
    msr     cnthctl_el2, x0
    msr     cntvoff_el2, xzr
    mov     x0, #(1 << 31)
    msr     hcr_el2, x0
    mov     x0, #0x33ff
    msr     cptr_el2, x0
    mov     x0, #(3 << 20)
    msr     cpacr_el1, x0
    mov     x0, #0x3c5
    msr     spsr_el2, x0
    adr     x0, 2f
    msr     elr_el2, x0
    eret

2:
    msr     SPSel, #1
    isb
    ldr     x1, =__stack_top
    mov     sp, x1
    ldr     x1, =__bss_start
    ldr     x2, =__bss_end
3:
    cmp     x1, x2
    b.hs    4f
    str     xzr, [x1], #8
    b       3b
4:
    bl      qemu_virt_main

1:
    wfe
    b       1b

