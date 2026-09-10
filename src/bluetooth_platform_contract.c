/*
 * ADR-060 immutable Bluetooth topology catalogue.  This unit intentionally
 * has no board discovery, hardware headers, function hooks, or mutable state.
 */
#include "types.h"
#include "bluetooth_platform_contract.h"

#define BT_SOURCE_COMMIT \
    { 0x50U, 0xf8U, 0x87U, 0x24U, 0x51U, 0x8dU, 0x2eU, 0xafU, \
      0xe7U, 0x5bU, 0xfaU, 0xe7U, 0x92U, 0x3eU, 0x90U, 0xa8U, \
      0xfeU, 0x17U, 0x1cU, 0x66U }

#define BT_KNOWN_TRANSPORT_FACTS BLUETOOTH_FACT_TRANSPORT_REQUIRED
#define BT_PI3_AMBIGUOUS_FACTS \
    (BLUETOOTH_FACT_TRANSPORT_REQUIRED & ~BLUETOOTH_FACT_MAX_BAUD)
#define BT_NO_PIOS_AUTHORITY 0U
#define BT_NO_DECLARED_WAKE_OR_RESET 0U

static const struct bluetooth_platform_profile bluetooth_profiles[] = {
    {
        BLUETOOTH_PLATFORM_PROFILE_PI5, "pi5-bcm2712-uarta",
        BT_SOURCE_COMMIT, BLUETOOTH_TRANSPORT_BCM2712_UARTA_H4,
        0x7d50c000ULL, 0x20U, { 25U, 24U, 26U, 27U },
        BLUETOOTH_SIGNAL_ALL, BLUETOOTH_CONTROL_OWNER_SOC_GIO, 29U,
        true, false, 3000000U, false, BT_NO_DECLARED_WAKE_OR_RESET,
        BLUETOOTH_RADIO_BRCM_BCM43438_BT, true, BT_KNOWN_TRANSPORT_FACTS,
        BT_NO_PIOS_AUTHORITY, BT_NO_PIOS_AUTHORITY,
    },
    {
        BLUETOOTH_PLATFORM_PROFILE_PI4_B, "pi4b-pl011-uart0",
        BT_SOURCE_COMMIT, BLUETOOTH_TRANSPORT_PL011_H4,
        0x7e201000ULL, 0x200U, { 30U, 31U, 32U, 33U },
        BLUETOOTH_SIGNAL_ALL, BLUETOOTH_CONTROL_OWNER_FIRMWARE_EXPGIO, 0U,
        true, false, 3000000U, false, BT_NO_DECLARED_WAKE_OR_RESET,
        BLUETOOTH_RADIO_BRCM_BCM43438_BT, true, BT_KNOWN_TRANSPORT_FACTS,
        BT_NO_PIOS_AUTHORITY, BT_NO_PIOS_AUTHORITY,
    },
    {
        BLUETOOTH_PLATFORM_PROFILE_PI3_B, "pi3b-pl011-uart0",
        BT_SOURCE_COMMIT, BLUETOOTH_TRANSPORT_PL011_H4,
        0x7e201000ULL, 0x200U, { 30U, 31U, 32U, 33U },
        BLUETOOTH_SIGNAL_TXD | BLUETOOTH_SIGNAL_RXD,
        BLUETOOTH_CONTROL_OWNER_FIRMWARE_EXPGIO, 0U, true, false, 0U,
        false, BT_NO_DECLARED_WAKE_OR_RESET,
        BLUETOOTH_RADIO_BRCM_BCM43438_BT, true, BT_PI3_AMBIGUOUS_FACTS,
        BT_NO_PIOS_AUTHORITY, BT_NO_PIOS_AUTHORITY,
    },
    {
        BLUETOOTH_PLATFORM_PROFILE_PI3_B_PLUS, "pi3b-plus-pl011-uart0",
        BT_SOURCE_COMMIT, BLUETOOTH_TRANSPORT_PL011_H4,
        0x7e201000ULL, 0x200U, { 30U, 31U, 32U, 33U },
        BLUETOOTH_SIGNAL_TXD | BLUETOOTH_SIGNAL_RXD,
        BLUETOOTH_CONTROL_OWNER_FIRMWARE_EXPGIO, 0U, true, false, 0U,
        false, BT_NO_DECLARED_WAKE_OR_RESET,
        BLUETOOTH_RADIO_BRCM_BCM43438_BT, true, BT_PI3_AMBIGUOUS_FACTS,
        BT_NO_PIOS_AUTHORITY, BT_NO_PIOS_AUTHORITY,
    },
    {
        BLUETOOTH_PLATFORM_PROFILE_ZERO_2_W, "zero2w-pl011-uart0",
        BT_SOURCE_COMMIT, BLUETOOTH_TRANSPORT_PL011_H4,
        0x7e201000ULL, 0x200U, { 30U, 31U, 32U, 33U },
        BLUETOOTH_SIGNAL_ALL, BLUETOOTH_CONTROL_OWNER_SOC_GIO, 42U,
        true, false, 3000000U, false, BT_NO_DECLARED_WAKE_OR_RESET,
        BLUETOOTH_RADIO_BRCM_BCM43438_BT, true, BT_KNOWN_TRANSPORT_FACTS,
        BT_NO_PIOS_AUTHORITY, BT_NO_PIOS_AUTHORITY,
    },
};

_Static_assert(sizeof((const u8[])BT_SOURCE_COMMIT) ==
               BLUETOOTH_PLATFORM_SOURCE_COMMIT_BYTES,
               "Bluetooth source commit must be the complete SHA-1");
_Static_assert(sizeof(bluetooth_profiles) / sizeof(bluetooth_profiles[0]) ==
               BLUETOOTH_PLATFORM_PROFILE_COUNT - 1U,
               "every Bluetooth catalogue ID needs one immutable profile");

static bool bluetooth_profile_is_catalogued(
    const struct bluetooth_platform_profile *profile)
{
    u32 i;

    if (!profile)
        return false;
    for (i = 0U; i < sizeof(bluetooth_profiles) / sizeof(bluetooth_profiles[0]);
         i++) {
        if (profile == &bluetooth_profiles[i])
            return true;
    }
    return false;
}

const struct bluetooth_platform_profile *
bluetooth_platform_profile_get(enum bluetooth_platform_profile_id id)
{
    u32 index;

    if (id <= BLUETOOTH_PLATFORM_PROFILE_INVALID ||
        id >= BLUETOOTH_PLATFORM_PROFILE_COUNT)
        return NULL;
    index = (u32)id - 1U;
    return &bluetooth_profiles[index];
}

bool bluetooth_transport_candidate(
    const struct bluetooth_platform_profile *profile,
    enum bluetooth_transport_kind console_kind, u64 console_dt_bus_base,
    enum bluetooth_platform_overlay_mode overlay_mode, u32 *missing_mask_out)
{
    u32 missing = 0U;

    if (!bluetooth_profile_is_catalogued(profile))
        missing |= BLUETOOTH_TRANSPORT_MISSING_PROFILE;
    else {
        if ((profile->known_fact_mask & BLUETOOTH_FACT_TRANSPORT_REQUIRED) !=
            BLUETOOTH_FACT_TRANSPORT_REQUIRED)
            missing |= BLUETOOTH_TRANSPORT_MISSING_FACTS;
        if (overlay_mode != BLUETOOTH_OVERLAY_BASE)
            missing |= BLUETOOTH_TRANSPORT_MISSING_BASE_OVERLAY;
        if (profile->console_conflict_required &&
            console_kind == profile->transport_kind &&
            console_dt_bus_base == profile->dt_bus_base)
            missing |= BLUETOOTH_TRANSPORT_MISSING_CONSOLE_SEPARATION;
    }
    if (missing_mask_out)
        *missing_mask_out = missing;
    return missing == 0U;
}

bool bluetooth_activation_permitted(
    const struct bluetooth_platform_profile *profile, u32 *missing_mask_out)
{
    u32 missing = BLUETOOTH_ACTIVATION_MISSING_PIOS_OWNERSHIP |
                  BLUETOOTH_ACTIVATION_MISSING_PINMUX_CONTROL |
                  BLUETOOTH_ACTIVATION_MISSING_RESET_POWER |
                  BLUETOOTH_ACTIVATION_MISSING_FIRMWARE_BAUD;

    if (!bluetooth_profile_is_catalogued(profile))
        missing |= BLUETOOTH_ACTIVATION_MISSING_PROFILE;
    else if (profile->id == BLUETOOTH_PLATFORM_PROFILE_PI3_B ||
             profile->id == BLUETOOTH_PLATFORM_PROFILE_PI3_B_PLUS)
        missing |= BLUETOOTH_ACTIVATION_MISSING_FLOW_CONTROL_PINMUX;
    if (missing_mask_out)
        *missing_mask_out = missing;
    return false;
}
