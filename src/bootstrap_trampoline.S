.section .text.trampoline
.global bootstrap_trampoline
.global bootstrap_trampoline_end

#ifndef PIOS_PLATFORM
#define PIOS_PLATFORM 1
#endif
#ifndef PIOS_PLATFORM_QEMU_VIRT
#define PIOS_PLATFORM_QEMU_VIRT 2
#endif

/*
 * x0 = destination
 * x1 = source
 * x2 = byte count
 * x3 = entry address
 */
.balign 16
bootstrap_trampoline:
    mov     x4, x0
    mov     x5, x1
    mov     x6, x2

#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    /* QEMU-only: disable the MMU/caches BEFORE the copy loop, not after.
     * BOOT_DST_ADDR equals stage0's own link address (0x40080000), so the
     * copy below overwrites the very pages holding stage0's live L1 page
     * table/code while the MMU is still walking that same table. Real Pi5
     * silicon tolerates this (the already-cached block-level TLB entry for
     * the destination range survives the whole tight copy loop), but QEMU's
     * TCG treats a write to a page it has translated code from as
     * self-modifying code and invalidates the softTLB/translations for that
     * region mid-copy, so a later iteration's translation walk sees the
     * corrupted, partially-overwritten table and takes a translation fault
     * (observed: Data Abort, DFSC=translation fault level 1, mid-copy).
     * Disabling the MMU first sidesteps the hazard entirely -- copying with
     * MMU off is functionally identical (just uncached/Device-ordered),
     * and this whole payload is copied only once at boot. Pi5 keeps its
     * proven original order (copy first, then disable) untouched below. */
    dsb     sy
    ic      iallu
    dsb     sy
    isb
    mrs     x8, sctlr_el1
    bic     x8, x8, #(1 << 0)   /* M: MMU off */
    bic     x8, x8, #(1 << 2)   /* C: D-cache off */
    bic     x8, x8, #(1 << 12)  /* I: I-cache off */
    msr     sctlr_el1, x8
    dsb     sy
    isb
#endif

1:
    cbz     x6, 2f
    cmp     x6, #8
    b.lo    3f
    ldr     x7, [x5], #8
    str     x7, [x4], #8
    sub     x6, x6, #8
    b       1b
3:
    ldrb    w7, [x5], #1
    strb    w7, [x4], #1
    sub     x6, x6, #1
    b       1b
2:
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    dsb     sy
    ic      iallu
    dsb     sy
    isb
#else
    dsb     sy
    ic      iallu
    dsb     sy
    isb
    mrs     x8, sctlr_el1
    bic     x8, x8, #(1 << 0)   /* M: MMU off */
    bic     x8, x8, #(1 << 2)   /* C: D-cache off */
    bic     x8, x8, #(1 << 12)  /* I: I-cache off */
    msr     sctlr_el1, x8
    dsb     sy
    isb
#endif
    br      x3
bootstrap_trampoline_end:
