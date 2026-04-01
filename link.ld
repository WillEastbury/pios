ENTRY(_start)

PHDRS
{
    text PT_LOAD FLAGS(5); /* R|X */
    data PT_LOAD FLAGS(6); /* R|W */
}

SECTIONS
{
    . = 0x80000;

    __text_start = .;
    .text.boot : { *(.text.boot) } :text
    .text      : { *(.text*) } :text
    __text_end = .;
    .rodata    : { *(.rodata*) } :text
    .data      : { *(.data*) } :data

    . = ALIGN(16);
    .bss (NOLOAD) : {
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
}
__bss_size = (__bss_end - __bss_start) >> 3;
