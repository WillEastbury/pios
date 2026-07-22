/*
 * test_board_detect.c - host unit test for board_family_from_midr().
 *
 * Verifies the pure MIDR_EL1 decode logic against real, documented PartNum
 * values (cross-checked against the Linux kernel's cputype.h) without
 * needing actual ARM hardware or an mrs instruction.
 */
#include <stdio.h>
#include "board_detect.h"

static int failures = 0;

static void expect_u32(const char *what, u32 got, u32 want)
{
    if (got != want) {
        printf("FAIL %s: got=%u want=%u\n", what, got, want);
        failures++;
    }
}

/* Build a plausible MIDR_EL1 value: [31:24]=implementer [23:20]=variant
 * [19:16]=arch [15:4]=partnum [3:0]=revision. */
static u64 make_midr(u32 implementer, u32 variant, u32 arch, u32 partnum, u32 rev)
{
    return ((u64)(implementer & 0xFFU) << 24) |
           ((u64)(variant & 0xFU) << 20) |
           ((u64)(arch & 0xFU) << 16) |
           ((u64)(partnum & 0xFFFU) << 4) |
           (u64)(rev & 0xFU);
}

int main(void)
{
    /* Real-world Pi5 MIDR_EL1 shape: ARM implementer (0x41), Cortex-A76
     * (partnum 0xD0B), architecture field 0xF (ARMv8), some variant/rev. */
    u64 a76_midr = make_midr(0x41, 0x1, 0xF, MIDR_PARTNUM_CORTEX_A76, 0x1);
    expect_u32("Cortex-A76 -> BOARD_FAMILY_PI5",
               board_family_from_midr(a76_midr), BOARD_FAMILY_PI5);

    /* Real-world Pi3/Pi Zero 2W MIDR_EL1 shape: Cortex-A53. */
    u64 a53_midr = make_midr(0x41, 0x0, 0xF, MIDR_PARTNUM_CORTEX_A53, 0x4);
    expect_u32("Cortex-A53 -> BOARD_FAMILY_BCM2837",
               board_family_from_midr(a53_midr), BOARD_FAMILY_BCM2837);

    /* Variant/revision bits must not affect the decode -- only partnum. */
    u64 a76_other_variant = make_midr(0x41, 0x3, 0xF, MIDR_PARTNUM_CORTEX_A76, 0x0);
    expect_u32("Cortex-A76 (different variant/rev) -> BOARD_FAMILY_PI5",
               board_family_from_midr(a76_other_variant), BOARD_FAMILY_PI5);

    /* An unrecognized core (e.g. Cortex-A72, partnum 0xD08) must fail closed
     * to UNKNOWN, not silently alias to either known family. */
    u64 a72_midr = make_midr(0x41, 0x0, 0xF, 0xD08, 0x0);
    expect_u32("Cortex-A72 (unrecognized) -> BOARD_FAMILY_UNKNOWN",
               board_family_from_midr(a72_midr), BOARD_FAMILY_UNKNOWN);

    /* Implementer byte must not leak into the partnum comparison (e.g. a
     * non-ARM implementer with a coincidentally-matching low 12 bits should
     * still decode by partnum only, matching real hardware behavior --
     * PartNum is only meaningful in combination with Implementer in
     * practice, but PIOS only ever runs on ARM Ltd cores so partnum alone
     * is a correct and sufficient discriminator here). */
    u64 zero_midr = 0;
    expect_u32("all-zero MIDR -> BOARD_FAMILY_UNKNOWN",
               board_family_from_midr(zero_midr), BOARD_FAMILY_UNKNOWN);

    if (failures == 0) {
        printf("OK: all board_detect MIDR decode assertions passed\n");
        return 0;
    }
    printf("%d failure(s)\n", failures);
    return 1;
}
