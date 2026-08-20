ENTRY(_start)
PHDRS { text PT_LOAD FLAGS(5); data PT_LOAD FLAGS(6); }
SECTIONS {
  . = 0x200000;
  __text_start = .;
  .text.boot : { *(.text.boot) } :text
  .text      : { *(.text*) } :text
  __text_end = .;
  .rodata    : { *(.rodata*) } :text
  __rodata_end = .;
  .data      : { *(.data*) } :data
  . = ALIGN(16);
  .bss (NOLOAD) : { __bss_start = .; *(.bss*) *(COMMON) __bss_end = .; } :data
  . = ALIGN(4096); . += 0x40000; __stack_top_core0 = .;
  . += 0x40000; __stack_top_core1 = .;
  . += 0x40000; __stack_top_core2 = .;
  . += 0x40000; __stack_top_core3 = .;
  . = ALIGN(4096);
  .pgtbl_pool (NOLOAD) : { __pgtbl_pool_start = .; *(.pgtbl_pool*) __pgtbl_pool_end = .; } :data
  . = ALIGN(4096); __heap_start = .;
}
__bss_size = (__bss_end - __bss_start) >> 3;
__pgtbl_pool_size = (__pgtbl_pool_end - __pgtbl_pool_start) >> 3;
