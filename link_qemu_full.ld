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

    __text_start = .;
    .text.boot : { *(.text.boot) } :text
    .text      : { *(.text*) } :text
    __text_end = .;
    .stage2_manifest : {
        __stage2_manifest_start = .;
        KEEP(*(.stage2_manifest))
        __stage2_manifest_end = .;
    } :text
    .rodata    : { *(.rodata*) } :text
    __rodata_end = .;
    .data      : { *(.data*) } :data

    . = ALIGN(16);
    .bss . (NOLOAD) : {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        __bss_end = .;
    } :data

    /* Fixed-size per-core stacks. Sized only from the small, slot-count-
     * independent .bss above -- the large per-slot MMU page-table pool
     * (.pgtbl_pool, below) is deliberately kept OUT of .bss so raising
     * MAX_PROCS_PER_CORE can never again push these stacks (and therefore
     * __heap_start) upward. See issue #86: growing those per-slot arrays
     * inside .bss displaced the stacks into CORE0_RAM_BASE, tripping the
     * stack-boundary canary at boot with far==CORE0_RAM_BASE. */
    . = ALIGN(4096);
    . += 0x40000;
    __stack_top_core0 = .;
    . += 0x40000;
    __stack_top_core1 = .;
    . += 0x40000;
    __stack_top_core2 = .;
    . += 0x40000;
    __stack_top_core3 = .;

    /* Per-process MMU page-table pool (mmu.c: user_l1, user_l2_low,
     * user_l2_phys, user_l2_high, user_l3_proc, user_l3_ipc, user_table_valid).
     * NOLOAD like .bss; zeroed by start.S's second clear loop
     * (__pgtbl_pool_start/__pgtbl_pool_size), not the .bss one, since it is
     * no longer contiguous with .bss. */
    . = ALIGN(4096);
    .pgtbl_pool . (NOLOAD) : {
        __pgtbl_pool_start = .;
        *(.pgtbl_pool*)
        __pgtbl_pool_end = .;
    } :data

    . = ALIGN(4096);
    __heap_start = .;
    __image_end = .;

    /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
__bss_size = (__bss_end - __bss_start) >> 3;
__pgtbl_pool_size = (__pgtbl_pool_end - __pgtbl_pool_start) >> 3;

/* Build-time guard (issue #86): fail the link instead of tripping a boot-time
 * stack canary. CORE0_RAM_BASE for QEMU is 0x42000000 (include/platform.h);
 * keep in sync. 512KB margin covers early-boot stack usage below __image_end. */
ASSERT(__image_end + 0x20000 <= 0x42000000,
       "kernel image (text+data+bss+stacks+pgtbl_pool) leaves less than 128KB before CORE0_RAM_BASE (0x42000000) -- reduce MAX_PROCS_PER_CORE or shrink another static, see issue #86")
