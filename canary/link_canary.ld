ENTRY(_start)

SECTIONS
{
    . = 0x00080000;

    .text.boot : { *(.text.boot) }
    .text      : { *(.text*) }
    .rodata    : { *(.rodata*) }
    .data      : { *(.data*) }
    .bss       : { *(.bss*) *(COMMON) }

    . = ALIGN(16);
    __stack_top = 0x00100000;
}
