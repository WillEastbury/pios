#include "board_detect.h"
#include "cyw43.h"
#include "platform.h"
#include "wifi_platform.h"

bool wifi_firmware_paths_for(u32 platform, u32 board_model,
                             struct wifi_firmware_paths *out)
{
    static const struct wifi_firmware_paths pi5 = {
        "pi5-cyw43455", "/wifi/firmware.bin", "/wifi/nvram.txt",
        "/wifi/clm.bin", CYW43455_CHIP_ID
    };
    static const struct wifi_firmware_paths pi4 = {
        "pi4-cyw43455", "/wifi/pi4/firmware.bin", "/wifi/pi4/nvram.txt",
        "/wifi/pi4/clm.bin", CYW43455_CHIP_ID
    };
    static const struct wifi_firmware_paths pi3b = {
        "pi3b-cyw43430", "/wifi/pi3b/firmware.bin", "/wifi/pi3b/nvram.txt",
        "/wifi/pi3b/clm.bin", CYW43430_CHIP_ID
    };
    static const struct wifi_firmware_paths pi3bp = {
        "pi3bp-cyw43455", "/wifi/pi3bp/firmware.bin",
        "/wifi/pi3bp/nvram.txt", "/wifi/pi3bp/clm.bin",
        CYW43455_CHIP_ID
    };
    static const struct wifi_firmware_paths zero2w = {
        "zero2w-cyw43436", "/wifi/zero2w/firmware.bin",
        "/wifi/zero2w/nvram.txt", "/wifi/zero2w/clm.bin",
        CYW43430_CHIP_ID
    };
    const struct wifi_firmware_paths *paths = 0;

    if (platform == PIOS_PLATFORM_PI5)
        paths = &pi5;
    else if (platform == PIOS_PLATFORM_PI4)
        paths = &pi4;
    else if (platform == PIOS_PLATFORM_PI3) {
        if (board_model == BOARD_MODEL_PI3_B)
            paths = &pi3b;
        else if (board_model == BOARD_MODEL_PI3_B_PLUS)
            paths = &pi3bp;
    } else if (platform == PIOS_PLATFORM_PIZERO2W)
        paths = &zero2w;

    if (!paths || !out)
        return false;
    *out = *paths;
    return true;
}

u32 wifi_bcm2712_stepping_from_register(u32 value)
{
    if ((value >> 16U) != 0x2712U)
        return WIFI_BCM2712_STEPPING_UNKNOWN;
    return (value & 0xFFU) >= 0x30U ?
           WIFI_BCM2712_STEPPING_D0 : WIFI_BCM2712_STEPPING_PRE_D0;
}

bool wifi_bcm2711_pull_register(u32 pin, u32 *offset, u32 *shift)
{
    if (!offset || !shift || pin > 53U)
        return false;
    *offset = 0xE4U + (pin / 16U) * 4U;
    *shift = (pin % 16U) * 2U;
    return true;
}
