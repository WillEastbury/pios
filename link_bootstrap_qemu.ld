ENTRY(_start)

/*
 * link_bootstrap_qemu.ld - stage0 linker script for the QEMU-virt platform.
 * Identical in structure to link_bootstrap.ld, but QEMU's -M virt machine
 * has NO RAM below 0x40000000 (that address range is GIC/UART/virtio-mmio),
 * so this stage0 must be linked/loaded inside the actual RAM window at
 * 0x40080000 -- matching link_qemu_full.ld's existing stage2 load address
 * and bootstrap.c's QEMU-specific BOOT_DST_ADDR (the self-overwriting
 * trampoline jump requires the two to match exactly; see bootstrap.c).
 */
PHDRS
{
    text PT_LOAD FLAGS(5); /* R|X */
    data PT_LOAD FLAGS(6); /* R|W */
}

SECTIONS
{
    . = 0x40080000;

    .text.boot : { *(.text.boot) } :text
    .text      : { *(.text*) } :text
    .rodata    : { *(.rodata*) } :text
    __rodata_end = .;
    .data      : { *(.data*) } :data

    . = ALIGN(16);
    .bss (NOLOAD) : {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        __bss_end = .;
    } :data

    . = ALIGN(4096);
    . += 0x10000;
    __stack_top_core0 = .;
}
