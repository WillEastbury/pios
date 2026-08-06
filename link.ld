ENTRY(_start)

PHDRS
{
    text PT_LOAD FLAGS(5); /* R|X */
    data PT_LOAD FLAGS(6); /* R|W */
}

SECTIONS
{
    . = 0x80000;
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
     * inside .bss displaced the QEMU stacks into CORE0_RAM_BASE, tripping the
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

    . = ALIGN(4096); /* inside RW segment for static reservations */
    __heap_start = .;
    __image_end = .;
}
__bss_size = (__bss_end - __bss_start) >> 3;
__pgtbl_pool_size = (__pgtbl_pool_end - __pgtbl_pool_start) >> 3;

/* Build-time guard (issue #86): the Pi5 kernel image already legitimately
 * extends past CORE0_RAM_BASE (0x00800000) at baseline -- that boundary is
 * not a hard "must stay below" wall on this platform the way it is on QEMU,
 * so this asserts against CORE1_RAM_BASE (0x01800000) instead: the image
 * (text+data+bss+stacks+pgtbl_pool) must never spill into core 1's private
 * RAM. Keep the constant in sync with include/platform.h. */
ASSERT(__image_end + 0x100000 <= 0x01800000,
       "kernel image (text+data+bss+stacks+pgtbl_pool) leaves less than 1MB before CORE1_RAM_BASE (0x01800000) -- reduce MAX_PROCS_PER_CORE or shrink another static, see issue #86")
