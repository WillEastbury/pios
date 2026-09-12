/*
 * bluetooth_platform_contract.h - ADR-060 immutable Bluetooth board facts.
 *
 * This catalogue describes upstream DTS topology only.  It does not discover
 * a board, select a live transport, or grant authority to alter a line.
 */
#pragma once
#include "types.h"

#define BLUETOOTH_PLATFORM_SOURCE_COMMIT_BYTES 20U
#define BLUETOOTH_PLATFORM_PIN_NONE            0xFFFFU

enum bluetooth_platform_profile_id {
    BLUETOOTH_PLATFORM_PROFILE_INVALID = 0,
    BLUETOOTH_PLATFORM_PROFILE_PI5,
    BLUETOOTH_PLATFORM_PROFILE_PI4_B,
    BLUETOOTH_PLATFORM_PROFILE_PI3_B,
    BLUETOOTH_PLATFORM_PROFILE_PI3_B_PLUS,
    BLUETOOTH_PLATFORM_PROFILE_ZERO_2_W,
    BLUETOOTH_PLATFORM_PROFILE_COUNT,
};

enum bluetooth_transport_kind {
    BLUETOOTH_TRANSPORT_NONE = 0,
    BLUETOOTH_TRANSPORT_BCM2712_UARTA_H4,
    BLUETOOTH_TRANSPORT_PL011_H4,
};

enum bluetooth_platform_overlay_mode {
    BLUETOOTH_OVERLAY_BASE = 0,
    BLUETOOTH_OVERLAY_MINIUART,
    BLUETOOTH_OVERLAY_DISABLE_BT,
    BLUETOOTH_OVERLAY_UNKNOWN,
};

enum bluetooth_control_owner_kind {
    BLUETOOTH_CONTROL_OWNER_NONE = 0,
    BLUETOOTH_CONTROL_OWNER_SOC_GIO,
    BLUETOOTH_CONTROL_OWNER_FIRMWARE_EXPGIO,
};

enum bluetooth_radio_kind {
    BLUETOOTH_RADIO_NONE = 0,
    BLUETOOTH_RADIO_BRCM_BCM43438_BT,
};

enum bluetooth_platform_signal_mask {
    BLUETOOTH_SIGNAL_CTS = 1U << 0,
    BLUETOOTH_SIGNAL_RTS = 1U << 1,
    BLUETOOTH_SIGNAL_TXD = 1U << 2,
    BLUETOOTH_SIGNAL_RXD = 1U << 3,
    BLUETOOTH_SIGNAL_ALL = BLUETOOTH_SIGNAL_CTS |
                           BLUETOOTH_SIGNAL_RTS |
                           BLUETOOTH_SIGNAL_TXD |
                           BLUETOOTH_SIGNAL_RXD,
};

enum bluetooth_platform_line_mask {
    BLUETOOTH_LINE_RESET = 1U << 0,
    BLUETOOTH_LINE_DEVICE_WAKE = 1U << 1,
    BLUETOOTH_LINE_HOST_WAKE = 1U << 2,
};

enum bluetooth_platform_fact_mask {
    BLUETOOTH_FACT_SOURCE_COMMIT = 1U << 0,
    BLUETOOTH_FACT_TRANSPORT = 1U << 1,
    BLUETOOTH_FACT_DT_BUS = 1U << 2,
    BLUETOOTH_FACT_SIGNALS = 1U << 3,
    BLUETOOTH_FACT_CONTROL = 1U << 4,
    BLUETOOTH_FACT_POLARITY = 1U << 5,
    BLUETOOTH_FACT_MAX_BAUD = 1U << 6,
    BLUETOOTH_FACT_RADIO = 1U << 7,
    BLUETOOTH_FACT_CONSOLE_CONFLICT = 1U << 8,
    BLUETOOTH_FACT_TRANSPORT_REQUIRED =
        BLUETOOTH_FACT_SOURCE_COMMIT | BLUETOOTH_FACT_TRANSPORT |
        BLUETOOTH_FACT_DT_BUS | BLUETOOTH_FACT_SIGNALS |
        BLUETOOTH_FACT_CONTROL | BLUETOOTH_FACT_POLARITY |
        BLUETOOTH_FACT_MAX_BAUD | BLUETOOTH_FACT_RADIO |
        BLUETOOTH_FACT_CONSOLE_CONFLICT,
};

enum bluetooth_transport_missing_mask {
    BLUETOOTH_TRANSPORT_MISSING_PROFILE = 1U << 0,
    BLUETOOTH_TRANSPORT_MISSING_FACTS = 1U << 1,
    BLUETOOTH_TRANSPORT_MISSING_BASE_OVERLAY = 1U << 2,
    BLUETOOTH_TRANSPORT_MISSING_CONSOLE_SEPARATION = 1U << 3,
};

enum bluetooth_activation_missing_mask {
    BLUETOOTH_ACTIVATION_MISSING_PROFILE = 1U << 0,
    BLUETOOTH_ACTIVATION_MISSING_PIOS_OWNERSHIP = 1U << 1,
    BLUETOOTH_ACTIVATION_MISSING_PINMUX_CONTROL = 1U << 2,
    BLUETOOTH_ACTIVATION_MISSING_RESET_POWER = 1U << 3,
    BLUETOOTH_ACTIVATION_MISSING_FIRMWARE_BAUD = 1U << 4,
    BLUETOOTH_ACTIVATION_MISSING_FLOW_CONTROL_PINMUX = 1U << 5,
};

struct bluetooth_platform_signal_pins {
    u16 cts;
    u16 rts;
    u16 txd;
    u16 rxd;
};

/*
 * `dt_bus_*` and control-line fields are source topology facts, not mapped
 * addresses or PIOS write authority.  `control_logical_off` is the inactive
 * logical level; all listed shutdown lines are active high and off at low.
 */
struct bluetooth_platform_profile {
    enum bluetooth_platform_profile_id id;
    const char *name;
    u8 source_commit[BLUETOOTH_PLATFORM_SOURCE_COMMIT_BYTES];
    enum bluetooth_transport_kind transport_kind;
    u64 dt_bus_base;
    u32 dt_bus_size;
    struct bluetooth_platform_signal_pins signals;
    u32 signal_mux_proven_mask;
    enum bluetooth_control_owner_kind control_owner;
    u16 control_line;
    bool control_active_high;
    bool control_logical_off;
    u32 max_baud;
    bool initial_baud_known;
    u32 declared_reset_wake_mask;
    enum bluetooth_radio_kind radio_kind;
    bool console_conflict_required;
    u32 known_fact_mask;
    u32 proven_pios_ownership_mask;
    u32 proven_activation_mask;
};

/*
 * These identifiers are owned by this catalogue, not runtime board or
 * platform IDs. Unsupported IDs return NULL rather than being inferred.
 */
const struct bluetooth_platform_profile *
bluetooth_platform_profile_get(enum bluetooth_platform_profile_id id);

/*
 * A true result identifies only a catalogue transport candidate.  It neither
 * activates hardware nor establishes PIOS control.  Any mini-UART or
 * disable-Bluetooth overlay is unsupported, and an active console with the
 * same kind and source bus prevents selection.
 */
bool bluetooth_transport_candidate(
    const struct bluetooth_platform_profile *profile,
    enum bluetooth_transport_kind console_kind, u64 console_dt_bus_base,
    enum bluetooth_platform_overlay_mode overlay_mode, u32 *missing_mask_out);

/*
 * Deliberately false for every profile.  The returned mask names the proof
 * categories a future, separately approved activation implementation needs.
 */
bool bluetooth_activation_permitted(
    const struct bluetooth_platform_profile *profile, u32 *missing_mask_out);
