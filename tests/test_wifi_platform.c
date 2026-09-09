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
    expect_profile(PIOS_PLATFORM_PIZERO2W, BOARD_MODEL_ZERO2W,
                   "Zero 2 W CYW43436 profile", CYW43430_CHIP_ID);
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
