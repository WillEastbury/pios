/*
 * legacy_armctrl.c - pure BCM2837 ARMCTRL source registration and demux.
 */
#include "types.h"
#include "legacy_armctrl.h"

#define LEGACY_ARMCTRL_INTID_DWC2   41U
#define LEGACY_ARMCTRL_INTID_SDIO1  62U

static const struct legacy_armctrl_route legacy_routes[
    LEGACY_ARMCTRL_SOURCE_CAPACITY] = {
    { LEGACY_ARMCTRL_INTID_SDIO1, LEGACY_ARMCTRL_BANK2, 30U, 0U },
    { LEGACY_ARMCTRL_INTID_DWC2, LEGACY_ARMCTRL_BANK1, 9U, 1U },
};

static void legacy_armctrl_clear_handle(
    struct legacy_armctrl_source_handle *handle)
{
    if (!handle)
        return;
    handle->intid = 0U;
    handle->generation = 0U;
    handle->owner_core = 0U;
    handle->_reserved = 0U;
}

static bool legacy_armctrl_fresh(const struct legacy_armctrl_registry *registry)
{
    const u8 *bytes;
    u32 i;

    if (!registry)
        return false;
    bytes = (const u8 *)registry;
    for (i = 0U; i < sizeof(*registry); i++) {
        if (bytes[i] != 0U)
            return false;
    }
    return true;
}

static u32 legacy_armctrl_route_index(u32 intid)
{
    u32 i;

    for (i = 0U; i < LEGACY_ARMCTRL_SOURCE_CAPACITY; i++) {
        if (legacy_routes[i].intid == intid)
            return i;
    }
    return LEGACY_ARMCTRL_SOURCE_CAPACITY;
}

static u32 *legacy_armctrl_bank_mask(struct legacy_armctrl_registry *registry,
                                     u32 bank)
{
    if (bank == LEGACY_ARMCTRL_BANK1)
        return &registry->registered_bank1;
    if (bank == LEGACY_ARMCTRL_BANK2)
        return &registry->registered_bank2;
    return NULL;
}

static const u32 *legacy_armctrl_bank_mask_const(
    const struct legacy_armctrl_registry *registry, u32 bank)
{
    if (bank == LEGACY_ARMCTRL_BANK1)
        return &registry->registered_bank1;
    if (bank == LEGACY_ARMCTRL_BANK2)
        return &registry->registered_bank2;
    return NULL;
}

const struct legacy_armctrl_route *legacy_armctrl_route_get(u32 intid)
{
    u32 index = legacy_armctrl_route_index(intid);

    if (index == LEGACY_ARMCTRL_SOURCE_CAPACITY)
        return NULL;
    return &legacy_routes[index];
}

bool legacy_armctrl_registry_init(struct legacy_armctrl_registry *registry,
                                  u32 owner_core)
{
    u32 i;

    if (!registry || owner_core != LEGACY_ARMCTRL_OWNER_CORE ||
        !legacy_armctrl_fresh(registry))
        return false;
    registry->owner_core = owner_core;
    registry->initialized = 1U;
    for (i = 0U; i < LEGACY_ARMCTRL_SOURCE_CAPACITY; i++)
        registry->generations[i] = 1U;
    dmb_ishst();
    return true;
}

bool legacy_armctrl_register(struct legacy_armctrl_registry *registry,
                             u32 caller_core, u32 intid,
                             struct legacy_armctrl_source_handle *handle_out)
{
    const struct legacy_armctrl_route *route;
    u32 *mask;
    u32 index;
    u32 bit;

    legacy_armctrl_clear_handle(handle_out);
    if (!registry || !handle_out || registry->initialized != 1U ||
        caller_core != registry->owner_core) {
        if (registry && registry->initialized == 1U)
            registry->reject_count++;
        return false;
    }
    route = legacy_armctrl_route_get(intid);
    index = legacy_armctrl_route_index(intid);
    if (!route || index == LEGACY_ARMCTRL_SOURCE_CAPACITY ||
        registry->generations[index] == 0U) {
        registry->reject_count++;
        return false;
    }
    mask = legacy_armctrl_bank_mask(registry, route->bank);
    bit = 1U << route->bit;
    if (!mask || (*mask & bit) != 0U) {
        registry->reject_count++;
        return false;
    }
    *mask |= bit;
    registry->register_count++;
    dmb_ishst();
    handle_out->intid = intid;
    handle_out->generation = registry->generations[index];
    handle_out->owner_core = registry->owner_core;
    handle_out->_reserved = 0U;
    return true;
}

bool legacy_armctrl_unregister(
    struct legacy_armctrl_registry *registry, u32 caller_core,
    const struct legacy_armctrl_source_handle *handle)
{
    const struct legacy_armctrl_route *route;
    u32 *mask;
    u32 index;
    u32 bit;

    if (!registry || !handle || registry->initialized != 1U ||
        caller_core != registry->owner_core ||
        handle->owner_core != registry->owner_core ||
        handle->_reserved != 0U || handle->generation == 0U) {
        if (registry && registry->initialized == 1U)
            registry->reject_count++;
        return false;
    }
    route = legacy_armctrl_route_get(handle->intid);
    index = legacy_armctrl_route_index(handle->intid);
    if (!route || index == LEGACY_ARMCTRL_SOURCE_CAPACITY ||
        registry->generations[index] != handle->generation) {
        registry->reject_count++;
        return false;
    }
    mask = legacy_armctrl_bank_mask(registry, route->bank);
    bit = 1U << route->bit;
    if (!mask || (*mask & bit) == 0U) {
        registry->reject_count++;
        return false;
    }
    *mask &= ~bit;
    if (registry->generations[index] == ~0U)
        registry->generations[index] = 0U;
    else
        registry->generations[index]++;
    registry->unregister_count++;
    dmb_ishst();
    return true;
}

bool legacy_armctrl_registered(const struct legacy_armctrl_registry *registry,
                               u32 intid)
{
    const struct legacy_armctrl_route *route;
    const u32 *mask;

    if (!registry || registry->initialized != 1U)
        return false;
    route = legacy_armctrl_route_get(intid);
    if (!route)
        return false;
    mask = legacy_armctrl_bank_mask_const(registry, route->bank);
    dmb_ishld();
    return mask && (*mask & (1U << route->bit)) != 0U;
}

u32 legacy_armctrl_registered_mask(
    const struct legacy_armctrl_registry *registry, u32 bank)
{
    const u32 *mask;

    if (!registry || registry->initialized != 1U)
        return 0U;
    mask = legacy_armctrl_bank_mask_const(registry, bank);
    if (!mask)
        return 0U;
    dmb_ishld();
    return *mask;
}

bool legacy_armctrl_select_pending(
    const struct legacy_armctrl_registry *registry,
    u32 pending_bank1, u32 pending_bank2, u32 *intid_out)
{
    u32 i;

    if (intid_out)
        *intid_out = 0U;
    if (!registry || !intid_out || registry->initialized != 1U)
        return false;
    dmb_ishld();
    for (i = 0U; i < LEGACY_ARMCTRL_SOURCE_CAPACITY; i++) {
        const struct legacy_armctrl_route *route = &legacy_routes[i];
        u32 pending = route->bank == LEGACY_ARMCTRL_BANK1
            ? pending_bank1 : pending_bank2;
        const u32 *registered =
            legacy_armctrl_bank_mask_const(registry, route->bank);
        u32 bit = 1U << route->bit;

        if (registered && (*registered & bit) != 0U &&
            (pending & bit) != 0U) {
            *intid_out = route->intid;
            return true;
        }
    }
    return false;
}
