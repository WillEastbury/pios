#pragma once
#include "types.h"

/*
 * Board-specific WiFi facts that must be decided before the CYW firmware is
 * copied into the radio.  Keep this pure so every supported combination is
 * host-testable without an SDIO controller or a mailbox.
 */
#define WIFI_BCM2712_STEPPING_UNKNOWN 0U
#define WIFI_BCM2712_STEPPING_PRE_D0  1U
#define WIFI_BCM2712_STEPPING_D0      2U

struct wifi_firmware_paths {
    const char *name;
    const char *firmware;
    const char *nvram;
    const char *clm;
    u16 expected_chip_id;
};

/* Returns false for a platform/model combination without a verified radio
 * profile.  Callers must not substitute a generic blob set on failure. */
bool wifi_firmware_paths_for(u32 platform, u32 board_model,
                             struct wifi_firmware_paths *out);

/* Decodes the BCM2712 stepping register used by the SDIO2 pinctrl layout.
 * A value not explicitly identifying BCM2712 is unknown, never pre-D0. */
u32 wifi_bcm2712_stepping_from_register(u32 value);

/* Returns the BCM2711 GPIO pull-control register and field for one pin. */
bool wifi_bcm2711_pull_register(u32 pin, u32 *offset, u32 *shift);
