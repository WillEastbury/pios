ENTRY(_start)

PHDRS
{
    text PT_LOAD FLAGS(5); /* R|X */
    data PT_LOAD FLAGS(6); /* R|W */
}

SECTIONS
{
    . = 0x40080000;
    __image_start = .;

    .text : {
        *(.text.boot)
        *(.text*)
    } :text
    .stage2_manifest : {
        KEEP(*(.stage2_manifest))
    } :text
    .rodata : { *(.rodata*) } :text
    __rodata_end = .;
    .data : { *(.data*) } :data
    .bss (NOLOAD) : {
        . = ALIGN(8);
        __bss_start = .;
        *(.bss*) *(COMMON)
        . = ALIGN(8);
        __bss_end = .;
    } :data

    . = ALIGN(16);
    . = . + 0x4000;
    __stack_top = .;
    __image_end = .;

    /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
