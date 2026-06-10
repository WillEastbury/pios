/*
 * Link address for the EL0 probe: core3 slot1.
 * CORE3_RAM_BASE 0x03800000 + PROC_SLOT_OFFSET 0x100000 + slot1*2MB.
 */
ENTRY(_ustart)

SECTIONS
{
    . = 0x03B00000;

    .text : {
        *(.text.ustart)
        *(.text*)
    }
    .rodata : { *(.rodata*) }
    .data : { *(.data*) }
    .bss (NOLOAD) : { *(.bss*) *(COMMON) }

    /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
