ENTRY(_start)

SECTIONS
{
    . = 0x40080000;
    __image_start = .;

    .text : {
        *(.text.boot)
        *(.text*)
    }
    .stage2_manifest : {
        KEEP(*(.stage2_manifest))
    }
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
    . = . + 0x4000;
    __stack_top = .;
    __image_end = .;

    /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
