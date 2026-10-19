ENTRY(module_entry)
SECTIONS
{
    . = 0;
    .text : { *(.text.module_entry) *(.text*) }
    .rodata : { *(.rodata*) }
    /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
    .data : { *(.data*) *(.got*) }
    .bss : { *(.bss*) *(COMMON) }
}
ASSERT(SIZEOF(.data) == 0, "Tensor PMOD must not contain writable data or GOT")
ASSERT(SIZEOF(.bss) == 0, "Tensor PMOD state must be caller/kernel owned")
