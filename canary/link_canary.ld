ENTRY(_start)

PHDRS
{
    text PT_LOAD FLAGS(5); /* R|X */
    data PT_LOAD FLAGS(6); /* R|W */
}

SECTIONS
{
    . = 0x80000;

    .text.boot : { KEEP(*(.text.boot)) *(.text .text.* .gnu.linkonce.t*) } :text
    .rodata    : { *(.rodata .rodata.* .gnu.linkonce.r*) } :text
    PROVIDE(_data = .);
    .data      : { *(.data .data.* .gnu.linkonce.d*) } :data
    .bss (NOLOAD) : {
        . = ALIGN(16);
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_end = .;
    } :data
    _end = .;

    /DISCARD/ : { *(.comment) *(.gnu*) *(.note*) *(.eh_frame*) }
}
__bss_size = (__bss_end - __bss_start) >> 3;
