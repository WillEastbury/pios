/*
 * Link script for the EL0 port-81 httpd image.
 *
 * High user VA avoids stale/global identity mappings while the physical payload is
 * still loaded into core2 slot0 (0x02900000).
 */
ENTRY(_ustart)

SECTIONS
{
    . = 0x2001000000;

    .text : {
        *(.text.ustart)
        *(.text*)
    }
    .rodata : { *(.rodata*) }
    .data : { *(.data*) }
    .bss (NOLOAD) : { *(.bss*) *(COMMON) }

    /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
