ENTRY(efi_main)
SECTIONS
{
    . = 0;
    .text : {
        *(.text*)
        *(.rodata*)
    }
    .data : {
        *(.data*)
        *(.sdata*)
    }
    .dynamic : { *(.dynamic*) }
    .rela : {
        *(.rela*)
        *(.rel*)
    }
    .dynsym : { *(.dynsym*) }
    .dynstr : { *(.dynstr*) }
    .hash : { *(.hash*) }
    .gnu.hash : { *(.gnu.hash*) }
    .eh_frame : { *(.eh_frame*) }
    .reloc : { BYTE(0) }
}

