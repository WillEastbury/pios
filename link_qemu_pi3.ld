ENTRY(_start)
SECTIONS
{
    . = 0x00080000;
    .text : { *(.text.boot) *(.text*) }
    .rodata : { *(.rodata*) }
    .data : { *(.data*) }
    .bss (NOLOAD) : {
        . = ALIGN(8);
        __bss_start = .;
        *(.bss*) *(COMMON)
        . = ALIGN(8);
        __bss_end = .;
    }
    . = ALIGN(16);
    . += 0x4000;
    __stack_top = .;
}
