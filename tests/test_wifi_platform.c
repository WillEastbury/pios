/*
 * Host checks for WiFi board/profile selection.  These values select firmware
 * and GPIO layouts before any SDIO transaction, so no MMIO is needed here.
 */
#include <stdio.h>
#include "board_detect.h"
#include "cyw43.h"
#include "mailbox.h"
#include "platform.h"
#include "wifi_platform.h"

static int failures;

static void expect(bool condition, const char *what)
{
    if (!condition) {
        printf("FAIL %s\n", what);
        failures++;
    }
}

static void expect_profile(u32 platform, u32 model, const char *name,
                           u16 chip)
{
    struct wifi_firmware_paths paths;
    expect(wifi_firmware_paths_for(platform, model, &paths), name);
    expect(paths.expected_chip_id == chip, name);
}

static void test_zero2w_manifest(void)
{
    const struct wifi_zero2w_firmware_manifest *small;
    const struct wifi_zero2w_firmware_manifest *full;
    struct wifi_zero2w_firmware_manifest malformed;
    struct wifi_firmware_paths paths;
    struct wifi_firmware_artifact abc = {
        "test", 3U,
        { 0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea,
          0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
          0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c,
          0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad }
    };
    static const u8 abc_data[] = { 'a', 'b', 'c' };

    expect(wifi_zero2w_firmware_manifest_count() == 2U,
           "Zero2W has exactly two preload records");
    small = wifi_zero2w_firmware_manifest_at(0U);
    full = wifi_zero2w_firmware_manifest_at(1U);
    expect(small && full && wifi_zero2w_firmware_manifest_valid(small) &&
           wifi_zero2w_firmware_manifest_valid(full),
           "Zero2W records are valid");
    expect(!small->clm_required && !small->clm.path &&
           small->clm.bytes == 0U && small->firmware.bytes == 442211U &&
           small->nvram.bytes == 1185U,
           "43436s candidate explicitly has no CLM");
    expect(full->clm_required && full->clm.path && full->firmware.bytes == 416101U &&
           full->nvram.bytes == 1706U && full->clm.bytes == 11209U,
           "43436 candidate requires its CLM");

    expect(wifi_zero2w_firmware_manifest_for_chip(
               (1U << 16) | CYW43430_CHIP_ID) == small,
           "raw ChipCommon revision one selects 43436s");
    expect(wifi_zero2w_firmware_manifest_for_chip(
               (2U << 16) | CYW43430_CHIP_ID) == full,
           "raw ChipCommon revision two selects 43436");
    expect(wifi_zero2w_firmware_manifest_for_chip(
               (15U << 16) | CYW43430_CHIP_ID) == full,
           "raw ChipCommon revision fifteen selects 43436");
    expect(!wifi_zero2w_firmware_manifest_for_chip(CYW43430_CHIP_ID) &&
           !wifi_zero2w_firmware_manifest_for_chip((1U << 16) | 0x4345U) &&
           !wifi_zero2w_firmware_manifest_for_chip(
               (16U << 16) | CYW43430_CHIP_ID),
           "unknown raw chip/revision fails closed");

    expect(!wifi_firmware_paths_for(PIOS_PLATFORM_PIZERO2W,
                                    BOARD_MODEL_PI3_B, &paths) &&
           !wifi_firmware_paths_for(PIOS_PLATFORM_PIZERO2W,
                                    BOARD_MODEL_PI3_B_PLUS, &paths) &&
           !wifi_firmware_paths_for(PIOS_PLATFORM_PIZERO2W,
                                    BOARD_MODEL_ZERO2W, &paths),
           "Zero2W never selects firmware from PCB board revision");

    malformed = *small;
    malformed.version++;
    expect(!wifi_zero2w_firmware_manifest_valid(&malformed),
           "bad manifest version rejected");
    malformed = *small;
    malformed.clm_required = true;
    expect(!wifi_zero2w_firmware_manifest_valid(&malformed),
           "no-CLM policy mismatch rejected");
    malformed = *full;
    malformed.clm.path = 0;
    expect(!wifi_zero2w_firmware_manifest_valid(&malformed),
           "required CLM missing from manifest rejected");
    malformed = *full;
    malformed.revision_min = 1U;
    expect(!wifi_zero2w_firmware_manifest_valid(&malformed),
           "malformed revision policy rejected");

    expect(wifi_firmware_artifact_matches(&abc, abc_data, sizeof(abc_data)),
           "matching manifest artifact accepted");
    expect(!wifi_firmware_artifact_matches(&abc, abc_data, 2U),
           "wrong artifact length rejected");
    abc.sha256[0] ^= 1U;
    expect(!wifi_firmware_artifact_matches(&abc, abc_data, sizeof(abc_data)),
           "wrong artifact hash rejected");
    abc.path = 0;
    expect(!wifi_firmware_artifact_matches(&abc, 0, 0U),
           "malformed missing artifact rejected");
}

int main(void)
{
    struct wifi_firmware_paths paths;
    u32 offset, shift;
    u32 mbox[MBOX_GPIO_OUTPUT_REQUEST_WORDS];

    expect_profile(PIOS_PLATFORM_PI5, BOARD_MODEL_UNKNOWN,
                   "Pi 5 CYW43455 profile", CYW43455_CHIP_ID);
    expect_profile(PIOS_PLATFORM_PI4, BOARD_MODEL_PI4_B,
                   "Pi 4 CYW43455 profile", CYW43455_CHIP_ID);
    expect_profile(PIOS_PLATFORM_PI3, BOARD_MODEL_PI3_B,
                   "Pi 3 B CYW43430 profile", CYW43430_CHIP_ID);
    expect_profile(PIOS_PLATFORM_PI3, BOARD_MODEL_PI3_B_PLUS,
                   "Pi 3 B+ CYW43455 profile", CYW43455_CHIP_ID);
    test_zero2w_manifest();
    expect(wifi_firmware_paths_for(PIOS_PLATFORM_PI5, BOARD_MODEL_UNKNOWN,
                                   &paths) && paths.firmware[0] == '/',
           "profile paths are absolute FAT paths");

    expect(!wifi_firmware_paths_for(PIOS_PLATFORM_PI3,
                                    BOARD_MODEL_UNKNOWN, &paths),
           "unknown Pi 3 model rejected");
    expect(!wifi_firmware_paths_for(PIOS_PLATFORM_QEMU_VIRT,
                                    BOARD_MODEL_UNKNOWN, &paths),
           "QEMU radio profile rejected");

    expect(wifi_bcm2712_stepping_from_register(0x2712002FU) ==
           WIFI_BCM2712_STEPPING_PRE_D0, "BCM2712 pre-D0 decode");
    expect(wifi_bcm2712_stepping_from_register(0x27120030U) ==
           WIFI_BCM2712_STEPPING_D0, "BCM2712 D0 decode");
    expect(wifi_bcm2712_stepping_from_register(0x00000000U) ==
           WIFI_BCM2712_STEPPING_UNKNOWN, "unknown stepping rejected");

    expect(wifi_bcm2711_pull_register(34U, &offset, &shift) &&
           offset == 0xECU && shift == 4U,
           "BCM2711 GPIO34 pull field");
    expect(wifi_bcm2711_pull_register(39U, &offset, &shift) &&
           offset == 0xECU && shift == 14U,
           "BCM2711 GPIO39 pull field");
    expect(!wifi_bcm2711_pull_register(54U, &offset, &shift),
           "out-of-range BCM2711 GPIO rejected");

    expect(mbox_build_gpio_output_request(mbox,
                                          MBOX_GPIO_OUTPUT_REQUEST_WORDS,
                                          129U, true),
           "mailbox GPIO request builds");
    expect(mbox[0] == 64U && mbox[2] == TAG_SET_GPIO_CONFIG &&
           mbox[3] == 20U && mbox[4] == 20U && mbox[5] == 129U &&
           mbox[6] == 1U && mbox[7] == 0U && mbox[8] == 0U &&
           mbox[9] == 0U,
           "mailbox GPIO config has five fields");
    expect(mbox[10] == TAG_SET_GPIO_STATE && mbox[11] == 8U &&
           mbox[12] == 8U && mbox[13] == 129U && mbox[14] == 1U &&
           mbox[15] == TAG_END,
           "mailbox GPIO state follows complete config");
    expect(!mbox_build_gpio_output_request(mbox,
                                           MBOX_GPIO_OUTPUT_REQUEST_WORDS - 1U,
                                           129U, false),
           "short mailbox GPIO request rejected");

    if (failures == 0) {
        printf("OK: all WiFi platform assertions passed\n");
        return 0;
    }
    printf("%d failure(s)\n", failures);
    return 1;
}
