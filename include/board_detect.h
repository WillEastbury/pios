/*
 * board_detect.h - runtime CPU/board detection for the multi-platform
 * stage0 bootstrap loader.
 *
 * PIOS ships separate, compile-time-configured main kernel images per
 * platform (kernel8_pi3.img, kernel8_pizero2w.img, real_kernel.img for Pi5,
 * PIOS_QEMU_FULL.BIN for QEMU -- see include/platform.h). Rebuilding the
 * ~20k-line main kernel to be runtime-multi-platform is not worthwhile: it
 * has deep, pervasive platform coupling and each image already boots
 * correctly once loaded.
 *
 * The tiny stage0 bootstrap loader (src/bootstrap.c, ~1000 lines) is a much
 * smaller surface and CAN be made genuinely runtime-multi-platform: read the
 * ARM MIDR_EL1 register (a CPU identification register, not an MMIO
 * peripheral -- safe to read before any board-specific address is known) to
 * distinguish Cortex-A76 (Pi5), Cortex-A72 (Pi4 / BCM2711), and Cortex-A53
 * (BCM2837-family: Pi3 B/B+/A+, Pi Zero 2 W), then select the matching
 * peripheral base addresses and the matching payload entry from the
 * multi-platform PIOSSTG2.PKG (see include/stage2_manifest.h). One
 * bootstrap kernel8.img can then boot correctly regardless of which of
 * these boards it's inserted into.
 *
 * MIDR_EL1 PartNum values (bits [15:4]), cross-checked against the Linux
 * kernel's arch/arm64/include/asm/cputype.h:
 *   Cortex-A76: 0xD0B
 *   Cortex-A72: 0xD08
 *   Cortex-A53: 0xD03
 */
#pragma once
#include "types.h"

/* Detected board family. Deliberately distinct from PIOS_STAGE2_PLATFORM_*
 * (which also covers QEMU/UEFI/Hyper-V -- targets stage0 never runs on,
 * since QEMU boots its own binary directly and UEFI/Hyper-V have their own
 * boot paths). Only the two real, physical, SD-booted board families that
 * can plausibly share one stage0 image are represented here (Pi 5, Pi 4,
 * BCM2837-family). */
#define BOARD_FAMILY_UNKNOWN   0U
#define BOARD_FAMILY_PI5       1U   /* BCM2712, Cortex-A76 */
#define BOARD_FAMILY_BCM2837   2U   /* BCM2837(B0)/BCM2710A1, Cortex-A53 */
#define BOARD_FAMILY_PI4       3U   /* BCM2711, Cortex-A72 */

#define BOARD_MODEL_UNKNOWN    0U
#define BOARD_MODEL_PI3_B      0x08U
#define BOARD_MODEL_PI3_B_PLUS 0x0DU
#define BOARD_MODEL_PI4_B      0x11U
#define BOARD_MODEL_ZERO2W     0x12U

#define MIDR_PARTNUM_CORTEX_A76 0xD0BU
#define MIDR_PARTNUM_CORTEX_A72 0xD08U
#define MIDR_PARTNUM_CORTEX_A53 0xD03U

/* Pure decode: given a raw MIDR_EL1 value, return the board family. No MMIO,
 * no asm -- host-testable (see tests/test_board_detect.c). */
u32 board_family_from_midr(u64 midr);
u32 board_model_from_revision(u32 revision);

/* Read MIDR_EL1 and decode it. AArch64-only (inline asm); not compiled/
 * callable on the host test build. */
u32 board_detect_family(void);

/* Resolved runtime peripheral base addresses for the given family. Returns
 * 0 for any field that doesn't apply (e.g. RP1/GENET bases on BCM2837). */
struct board_runtime_bases {
    u64 periph_base;   /* main "low peripheral" MMIO window */
    u64 uart0_base;    /* PL011 UART0 */
    u64 mbox_base;     /* VideoCore mailbox */
    u64 emmc_base;     /* SD/EMMC host controller */
    u64 pm_base;        /* power-management/watchdog block */
    u64 qa7_base;       /* ARM-local peripherals (BCM2837-family only) */
};

void board_runtime_bases_for(u32 family, struct board_runtime_bases *out);

/* Populated once, very early in bootstrap_main() (before any peripheral
 * access), by calling board_detect_init(). sd.c and fb.c reference this
 * directly (guarded by PIOS_RUNTIME_MMIO_BOOTSTRAP) instead of their normal
 * compile-time PIOS_*_BASE macros, so the SAME bootstrap.o/sd.o/fb.o work
 * correctly regardless of which of these boards they end up running on. */
extern struct board_runtime_bases g_board_bases;
extern u32 g_board_family;

/* Detects the board family (board_detect_family()) and populates
 * g_board_bases/g_board_family. Must be called before any UART/mailbox/SD/
 * watchdog access in the bootstrap image. */
void board_detect_init(void);
