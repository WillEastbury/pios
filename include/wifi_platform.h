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

/*
 * Immutable Zero 2 W firmware manifest.  Both records are read while FAT is
 * still available; the raw ChipCommon word selects one only after SDIO is up.
 */
#define WIFI_ZERO2W_MANIFEST_VERSION 1U
#define WIFI_SHA256_BYTES             32U

struct wifi_firmware_artifact {
    const char *path;
    u32 bytes;
    u8 sha256[WIFI_SHA256_BYTES];
};

struct wifi_zero2w_firmware_manifest {
    u16 version;
    u16 expected_chip_id;
    u8 revision_min;
    u8 revision_max;
    bool clm_required;
    u8 reserved;
    const char *name;
    struct wifi_firmware_artifact firmware;
    struct wifi_firmware_artifact nvram;
    struct wifi_firmware_artifact clm;
};

/* Returns false for a platform/model combination without a verified radio
 * profile.  Callers must not substitute a generic blob set on failure. */
bool wifi_firmware_paths_for(u32 platform, u32 board_model,
                             struct wifi_firmware_paths *out);

/* Selects a Zero 2 W candidate only from its raw ChipCommon id/revision word.
 * Revision 1 selects the no-CLM 43436s record; revisions 2..15 select 43436.
 * Revision zero, an unknown revision, or a mismatching chip fails closed. */
const struct wifi_zero2w_firmware_manifest *
wifi_zero2w_firmware_manifest_for_chip(u32 chipcommon_raw);

/* Enumerates the two immutable records for the FAT preload phase.  This does
 * not select a record; selection is only legal through the raw-chip helper. */
u32 wifi_zero2w_firmware_manifest_count(void);
const struct wifi_zero2w_firmware_manifest *
wifi_zero2w_firmware_manifest_at(u32 index);
bool wifi_zero2w_firmware_manifest_valid(
    const struct wifi_zero2w_firmware_manifest *manifest);

/* Validates the exact length and SHA-256 digest specified by an immutable
 * manifest artifact.  A missing no-CLM artifact is valid only when its
 * manifest entry has both a null path and zero length. */
bool wifi_firmware_artifact_matches(const struct wifi_firmware_artifact *artifact,
                                    const u8 *data, u32 len);

/* Decodes the BCM2712 stepping register used by the SDIO2 pinctrl layout.
 * A value not explicitly identifying BCM2712 is unknown, never pre-D0. */
u32 wifi_bcm2712_stepping_from_register(u32 value);

/* Returns the BCM2711 GPIO pull-control register and field for one pin. */
bool wifi_bcm2711_pull_register(u32 pin, u32 *offset, u32 *shift);
