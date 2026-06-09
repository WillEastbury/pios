/*
 * user.ld - link script for PIOS flat userland binaries.
 *
 * The image is linked at the fixed address of core 2, process slot 0
 * (core_ram_bases[2] 0x02800000 + PROC_SLOT_OFFSET 0x100000). The kernel
 * loads the flat image there and the MMU maps [base, base+code] read-execute,
 * so the binary must contain no writable globals (.data/.bss) — keep all
 * mutable state on the stack. objcopy -O binary emits text+rodata only.
 */
ENTRY(_ustart)

SECTIONS
{
    . = 0x02900000;

    .text : {
        *(.text.ustart)
        *(.text*)
    }
    .rodata : { *(.rodata*) }

    /* These should be empty for a well-formed flat app; flag drift loudly. */
    .data : { *(.data*) }
    .bss (NOLOAD) : { *(.bss*) *(COMMON) }

    /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
