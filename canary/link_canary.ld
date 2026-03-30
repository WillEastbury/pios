ENTRY(_start)

SECTIONS
{
    . = 0x00080000;

    .text.boot : { *(.text.boot) }
    .text      : { *(.text*) }
    .rodata    : { *(.rodata*) }
    .data      : { *(.data*) }

    . = ALIGN(16);
    __bss_start = .;
    .bss       : { *(.bss*) *(COMMON) }
    __bss_end = .;

    . = ALIGN(16);
    __stack_top = 0x00100000;
}
