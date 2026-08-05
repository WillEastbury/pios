/*
 * board_detect.c - runtime CPU/board detection. See include/board_detect.h.
 */
#include "types.h"
#include "board_detect.h"
#include "platform.h"

u32 board_family_from_midr(u64 midr)
{
    u32 partnum = (u32)((midr >> 4) & 0xFFFU);
    if (partnum == MIDR_PARTNUM_CORTEX_A76)
        return BOARD_FAMILY_PI5;
    if (partnum == MIDR_PARTNUM_CORTEX_A53)
        return BOARD_FAMILY_BCM2837;
    return BOARD_FAMILY_UNKNOWN;
}

#ifdef PIOS_HOST_TYPES_SHIM
/* Host unit tests never call this (no real MIDR_EL1 to read); provide a
 * harmless stub so the file still compiles when linked into a host test
 * binary that only wants board_family_from_midr(). */
u32 board_detect_family(void) { return BOARD_FAMILY_UNKNOWN; }
#else
u32 board_detect_family(void)
{
    u64 midr;
    __asm__ volatile("mrs %0, midr_el1" : "=r"(midr));
    return board_family_from_midr(midr);
}
#endif

void board_runtime_bases_for(u32 family, struct board_runtime_bases *out)
{
    if (!out)
        return;
    if (family == BOARD_FAMILY_BCM2837) {
        /* Pi3 B/B+/A+ and Pi Zero 2 W (BCM2837(B0)/BCM2710A1) share the
         * same "low peripheral" memory map -- see include/platform.h
         * PIOS_PLATFORM_PI3/_PIZERO2W, which these values are cross-checked
         * against verbatim. */
        out->periph_base = 0x3F000000ULL;
        out->uart0_base  = 0x3F000000ULL + 0x201000ULL;
        out->mbox_base   = 0x3F000000ULL + 0x00B880ULL;
        out->emmc_base   = 0x3F000000ULL + 0x300000ULL;
        out->pm_base     = 0x3F000000ULL + 0x00100000ULL;
        out->qa7_base    = 0x40000000ULL;
    } else {
        /* Default/fallback: Pi5 (BCM2712). Also used for BOARD_FAMILY_UNKNOWN
         * -- fail toward the most hardware-validated, proven path rather
         * than an all-zero/undefined one. */
        out->periph_base = 0x107C000000ULL;
        out->uart0_base  = 0x107C000000ULL + 0x201000ULL;
        out->mbox_base   = 0x107C000000ULL + 0x013880ULL;
        out->emmc_base   = 0x1000FFF000ULL;
        out->pm_base     = 0x107C000000ULL + 0x00100000ULL;
        out->qa7_base    = 0ULL;
    }
}

struct board_runtime_bases g_board_bases;
u32 g_board_family = BOARD_FAMILY_UNKNOWN;

void board_detect_init(void)
{
#if PIOS_PLATFORM == PIOS_PLATFORM_QEMU_VIRT
    /* A QEMU-compiled stage0 must NEVER populate g_board_bases from MIDR
     * detection: QEMU's -cpu cortex-a53 reports the SAME MIDR PartNum as
     * real BCM2837 hardware, so board_detect_family() would misclassify it
     * as BCM2837 and fill g_board_bases with real Broadcom peripheral
     * addresses (UART0 0x3F201000, mailbox 0x3F00B880, etc.) that don't
     * exist under QEMU -- any PIOS_RUNTIME_MMIO_BOOTSTRAP-gated code (fb.c/
     * sd.c/uart.c) reading those would silently hang polling a status
     * register that never responds, producing zero boot output. Populate
     * g_board_bases from platform.h's already-correct QEMU_VIRT compile-time
     * constants instead. */
    g_board_family = BOARD_FAMILY_UNKNOWN;
    g_board_bases.periph_base = PIOS_PERIPH_BASE;
    g_board_bases.uart0_base  = PIOS_UART0_BASE;
    g_board_bases.mbox_base   = PIOS_MBOX_BASE;
    g_board_bases.emmc_base   = PIOS_EMMC2_BASE;
    g_board_bases.pm_base     = 0;
    g_board_bases.qa7_base    = PIOS_QA7_BASE;
#else
    g_board_family = board_detect_family();
    board_runtime_bases_for(g_board_family, &g_board_bases);
#endif
}
