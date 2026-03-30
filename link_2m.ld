ENTRY(_start)
PHDRS { text PT_LOAD FLAGS(5); data PT_LOAD FLAGS(6); }
SECTIONS {
  . = 0x200000;
  __text_start = .;
  .text.boot : { *(.text.boot) } :text
  .text      : { *(.text*) } :text
  __text_end = .;
  .rodata    : { *(.rodata*) } :text
  .data      : { *(.data*) } :data
  . = ALIGN(16);
  .bss (NOLOAD) : { __bss_start = .; *(.bss*) *(COMMON) __bss_end = .; } :data
  . = ALIGN(4096); . += 0x40000; __stack_top_core0 = .;
  . += 0x40000; __stack_top_core1 = .;
  . += 0x40000; __stack_top_core2 = .;
  . += 0x40000; __stack_top_core3 = .;
  . = ALIGN(4096); __heap_start = .;
}
