ENTRY(_start)

PHDRS
{
    text PT_LOAD FLAGS(5); /* R|X */
    data PT_LOAD FLAGS(6); /* R|W */
}

SECTIONS
{
    . = 0x80000;
    __image_start = .;

    __text_start = .;
    .text.boot : { *(.text.boot) } :text
    .text      : { *(.text*) } :text
    __text_end = .;
    .stage2_manifest : {
        __stage2_manifest_start = .;
        KEEP(*(.stage2_manifest))
        __stage2_manifest_end = .;
    } :text
    .rodata    : { *(.rodata*) } :text
    __rodata_end = .;
    .data      : { *(.data*) } :data

    . = ALIGN(16);
    .bss . (NOLOAD) : {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        __bss_end = .;
    } :data

    /* 256KB stack per core, growing downward */
    . = ALIGN(4096);
    . += 0x40000;
    __stack_top_core0 = .;
    . += 0x40000;
    __stack_top_core1 = .;
    . += 0x40000;
    __stack_top_core2 = .;
    . += 0x40000;
    __stack_top_core3 = .;

    . = ALIGN(4096); /* inside RW segment for static reservations */
    __heap_start = .;
    __image_end = .;
}
__bss_size = (__bss_end - __bss_start) >> 3;
