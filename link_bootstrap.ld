ENTRY(_start)

PHDRS
{
    text PT_LOAD FLAGS(5); /* R|X */
    data PT_LOAD FLAGS(6); /* R|W */
}

SECTIONS
{
    . = 0x80000;

    .text.boot : { *(.text.boot) } :text
    .text      : { *(.text*) } :text
    .rodata    : { *(.rodata*) } :text
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
