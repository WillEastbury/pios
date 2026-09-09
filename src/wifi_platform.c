#include "board_detect.h"
#include "crypto.h"
#include "cyw43.h"
#include "platform.h"
#include "wifi_platform.h"

static const struct wifi_zero2w_firmware_manifest zero2w_43436 = {
    WIFI_ZERO2W_MANIFEST_VERSION, CYW43430_CHIP_ID, 2U, 15U, true, 0U,
    "zero2w-cyw43436",
    { "/wifi/zero2w/43436/firmware.bin", 416101U,
      { 0x51, 0x0a, 0x7d, 0xd1, 0xe0, 0x56, 0x19, 0x9b,
        0x30, 0x94, 0x25, 0x54, 0x8e, 0xe0, 0xbd, 0x84,
        0x69, 0x93, 0xa1, 0x83, 0x7a, 0xc7, 0xfa, 0x1e,
        0x4d, 0x3e, 0x64, 0x1f, 0x05, 0xa1, 0x32, 0x7a } },
    { "/wifi/zero2w/43436/nvram.txt", 1706U,
      { 0x4c, 0xda, 0x90, 0xfa, 0xcd, 0x88, 0x44, 0xcf,
        0xf6, 0x0d, 0x80, 0xb3, 0x4b, 0x24, 0xec, 0xba,
        0xe7, 0x6a, 0xdb, 0x9a, 0x62, 0x50, 0x8a, 0x10,
        0x94, 0x61, 0xb8, 0xbf, 0x42, 0xb4, 0x78, 0xd1 } },
    { "/wifi/zero2w/43436/clm.bin", 11209U,
      { 0xfc, 0xe7, 0xcb, 0xb6, 0x2f, 0xfa, 0x6a, 0x5a,
        0x65, 0xca, 0x97, 0xb1, 0x3f, 0x6f, 0xbf, 0x28,
        0xd0, 0x6c, 0x02, 0xd9, 0x86, 0xc2, 0x07, 0x2d,
        0x65, 0xbf, 0x72, 0x16, 0x47, 0x55, 0xfc, 0x34 } }
};

static const struct wifi_zero2w_firmware_manifest zero2w_43436s = {
    WIFI_ZERO2W_MANIFEST_VERSION, CYW43430_CHIP_ID, 1U, 1U, false, 0U,
    "zero2w-cyw43436s",
    { "/wifi/zero2w/43436s/firmware.bin", 442211U,
      { 0x68, 0xb9, 0xbc, 0xc9, 0x85, 0x5d, 0x91, 0x73,
        0x3c, 0xd4, 0x4c, 0x21, 0xde, 0x4c, 0xb5, 0x07,
        0xc9, 0x1b, 0x0d, 0x32, 0xd8, 0x38, 0xc0, 0x69,
        0x6d, 0xef, 0x5e, 0xb9, 0x6c, 0x99, 0xe2, 0xde } },
    { "/wifi/zero2w/43436s/nvram.txt", 1185U,
      { 0x37, 0xa8, 0xb8, 0x5a, 0x5a, 0x97, 0x42, 0x76,
        0x11, 0x01, 0xb7, 0x64, 0xa0, 0x7b, 0xc4, 0xd0,
        0xc8, 0xb0, 0x9f, 0x2e, 0x18, 0x0e, 0xae, 0xa3,
        0xb5, 0x03, 0xa8, 0x34, 0x27, 0x7a, 0xd5, 0x95 } },
    { 0, 0U, { 0 } }
};

const struct wifi_zero2w_firmware_manifest *
wifi_zero2w_firmware_manifest_for_chip(u32 chipcommon_raw)
{
    u16 chip_id = (u16)(chipcommon_raw & 0xFFFFU);
    u8 revision = (u8)((chipcommon_raw >> 16U) & 0x0FU);

    if (chip_id != CYW43430_CHIP_ID || revision == 0U)
        return 0;
    if (revision == 1U)
        return &zero2w_43436s;
    if (revision <= 15U)
        return &zero2w_43436;
    return 0;
}

u32 wifi_zero2w_firmware_manifest_count(void)
{
    return 2U;
}

const struct wifi_zero2w_firmware_manifest *
wifi_zero2w_firmware_manifest_at(u32 index)
{
    if (index == 0U)
        return &zero2w_43436s;
    if (index == 1U)
        return &zero2w_43436;
    return 0;
}

bool wifi_zero2w_firmware_manifest_valid(
    const struct wifi_zero2w_firmware_manifest *manifest)
{
    bool no_clm;

    if (!manifest || manifest->version != WIFI_ZERO2W_MANIFEST_VERSION ||
        manifest->expected_chip_id != CYW43430_CHIP_ID ||
        !manifest->name || !manifest->firmware.path ||
        manifest->firmware.bytes == 0U || !manifest->nvram.path ||
        manifest->nvram.bytes == 0U || manifest->revision_min == 0U ||
        manifest->revision_min > manifest->revision_max ||
        manifest->revision_max > 15U)
        return false;

    no_clm = !manifest->clm.path && manifest->clm.bytes == 0U;
    if (!manifest->clm_required)
        return manifest->revision_min == 1U && manifest->revision_max == 1U &&
               no_clm;
    return manifest->revision_min == 2U && manifest->revision_max == 15U &&
           manifest->clm.path && manifest->clm.bytes != 0U;
}

bool wifi_firmware_artifact_matches(const struct wifi_firmware_artifact *artifact,
                                    const u8 *data, u32 len)
{
    u8 digest[WIFI_SHA256_BYTES];

    if (!artifact)
        return false;
    if (!artifact->path)
        return artifact->bytes == 0U && len == 0U && !data;
    if (!data || artifact->bytes == 0U || len != artifact->bytes)
        return false;
    sha256(data, len, digest);
    return memcmp(digest, artifact->sha256, sizeof(digest)) == 0;
}

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
    }

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
