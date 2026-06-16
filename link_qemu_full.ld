ENTRY(_start)

SECTIONS
{
    . = 0x40080000;
    __image_start = .;

    __text_start = .;
    .text.boot : { *(.text.boot) }
    .text      : { *(.text*) }
    __text_end = .;
    .stage2_manifest : {
        __stage2_manifest_start = .;
        KEEP(*(.stage2_manifest))
        __stage2_manifest_end = .;
    }
    .rodata    : { *(.rodata*) }
    .data      : { *(.data*) }

    . = ALIGN(16);
    .bss . (NOLOAD) : {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        __bss_end = .;
    }

    . = ALIGN(4096);
    . += 0x40000;
    __stack_top_core0 = .;
    . += 0x40000;
    __stack_top_core1 = .;
    . += 0x40000;
    __stack_top_core2 = .;
    . += 0x40000;
    __stack_top_core3 = .;

    . = ALIGN(4096);
    __heap_start = .;
    __image_end = .;

    /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
__bss_size = (__bss_end - __bss_start) >> 3;
