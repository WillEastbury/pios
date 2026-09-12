/*
 * legacy_armctrl.h - pure BCM2837 ARMCTRL peripheral-cascade registry.
 *
 * This module owns no MMIO and invokes no interrupt handler. It validates
 * which registered ARMCTRL source a caller may enable or acknowledge.
 */
#pragma once
#include "types.h"

#define LEGACY_ARMCTRL_OWNER_CORE       0U
#define LEGACY_ARMCTRL_BANK1            1U
#define LEGACY_ARMCTRL_BANK2            2U
#define LEGACY_ARMCTRL_SOURCE_CAPACITY  2U

struct legacy_armctrl_route {
    u32 intid;
    u32 bank;
    u32 bit;
    u32 priority;
};

struct legacy_armctrl_source_handle {
    u32 intid;
    u32 generation;
    u32 owner_core;
    u32 _reserved;
};

/*
 * The registry is single-writer core-0 state. Reusable source registrations
 * carry independent generations; a stale handle cannot unregister a new use.
 */
struct legacy_armctrl_registry {
    u32 owner_core;
    u32 initialized;
    u32 registered_bank1;
    u32 registered_bank2;
    u32 generations[LEGACY_ARMCTRL_SOURCE_CAPACITY];
    u32 register_count;
    u32 unregister_count;
    u32 reject_count;
    u8 _reserved[28U];
} ALIGNED(64);

_Static_assert(sizeof(struct legacy_armctrl_registry) == 64U,
               "legacy ARMCTRL registry must own one cache line");

const struct legacy_armctrl_route *legacy_armctrl_route_get(u32 intid);

bool legacy_armctrl_registry_init(struct legacy_armctrl_registry *registry,
                                  u32 owner_core);
bool legacy_armctrl_register(struct legacy_armctrl_registry *registry,
                             u32 caller_core, u32 intid,
                             struct legacy_armctrl_source_handle *handle_out);
bool legacy_armctrl_unregister(
    struct legacy_armctrl_registry *registry, u32 caller_core,
    const struct legacy_armctrl_source_handle *handle);
bool legacy_armctrl_registered(const struct legacy_armctrl_registry *registry,
                               u32 intid);
u32 legacy_armctrl_registered_mask(
    const struct legacy_armctrl_registry *registry, u32 bank);

/*
 * Select one registered pending source. Priority preserves the established
 * SDIO1 route before the dormant DWC2 route. A second pending source remains
 * asserted for the next controller acknowledgement.
 */
bool legacy_armctrl_select_pending(
    const struct legacy_armctrl_registry *registry,
    u32 pending_bank1, u32 pending_bank2, u32 *intid_out);
