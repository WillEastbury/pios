/*
 * Link address for the EL0 PicoScript VM proof: high user VA, backed by core3
 * physical slot1. This avoids stale/global identity-map TLB entries for the
 * physical slot VA while testing EL0 page permissions.
 */
ENTRY(_ustart)

SECTIONS
{
    . = 0x2002000000;

    .text : {
        *(.text.ustart)
        *(.text*)
    }
    .rodata : { *(.rodata*) }
    .data : { *(.data*) }
    .bss (NOLOAD) : { *(.bss*) *(COMMON) }

    /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
