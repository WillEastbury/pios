/*
 * Pure BCM2837 USB VBUS ownership/evidence contract.  No platform discovery
 * or hardware access belongs in this file.
 */
#include "types.h"
#include "usb_vbus_contract.h"

#define USB_VBUS_HANDLE_MAGIC 0x56425553ULL
#define USB_VBUS_HANDLE_SHIFT 32U

static const struct usb_vbus_profile usb_vbus_profiles[] = {
    {
        USB_VBUS_PROFILE_PI3_B, USB_VBUS_TOPOLOGY_PI3_B_LAN9514,
        0U, 0U, 0U, 1U,
    },
    {
        USB_VBUS_PROFILE_PI3_B_PLUS, USB_VBUS_TOPOLOGY_PI3_B_PLUS_LAN7515,
        0U, 0U, 0U, 1U,
    },
    {
        USB_VBUS_PROFILE_ZERO_2_W, USB_VBUS_TOPOLOGY_ZERO_2_W_OTG_CONNECTOR,
        0U, 0U, 0U, 1U,
    },
};

static void usb_vbus_zero(void *dst, usize bytes)
{
    u8 *p = (u8 *)dst;

    while (bytes != 0U) {
        *p++ = 0U;
        bytes--;
    }
}

static void usb_vbus_copy(void *dst, const void *src, usize bytes)
{
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;

    while (bytes != 0U) {
        *d++ = *s++;
        bytes--;
    }
}

static bool usb_vbus_zeroed(const struct usb_vbus_contract_pool *pool)
{
    const u8 *bytes;
    usize i;

    if (!pool)
        return false;
    bytes = (const u8 *)pool;
    for (i = 0U; i < sizeof(*pool); i++) {
        if (bytes[i] != 0U)
            return false;
    }
    return true;
}

static bool usb_vbus_pool_initialized(const struct usb_vbus_contract_pool *pool)
{
    u32 initialized;

    if (!pool)
        return false;
    initialized = pool->owner.initialized;
    dmb_ishld();
    return initialized == 1U;
}

static void usb_vbus_clear_handle(struct usb_vbus_handle *handle)
{
    if (handle)
        usb_vbus_zero(handle, sizeof(*handle));
}

static u64 usb_vbus_token(u32 controller_id)
{
    return (USB_VBUS_HANDLE_MAGIC << USB_VBUS_HANDLE_SHIFT) |
           (u64)controller_id;
}

static bool usb_vbus_live_record(
    const struct usb_vbus_contract_pool *pool, const struct usb_vbus_handle *handle,
    const struct usb_vbus_record **record_out)
{
    const struct usb_vbus_record *record;

    if (!pool || !handle || !record_out ||
        handle->slot >= USB_VBUS_CONTRACT_CAPACITY ||
        handle->generation == 0U || handle->controller_id == 0U ||
        handle->_reserved != 0U || handle->_pad != 0U ||
        handle->_token != usb_vbus_token(handle->controller_id))
        return false;
    record = &pool->records[handle->slot];
    {
        u32 state = record->state;
        u64 generation;
        u32 controller_id;

        /* Acquire payload/evidence only after observing published live state. */
        dmb_ishld();
        generation = record->generation;
        controller_id = record->controller_id;
        if (generation != handle->generation ||
            controller_id != handle->controller_id ||
            state == USB_VBUS_FREE || state == USB_VBUS_RELEASED ||
            state == USB_VBUS_RETIRED)
            return false;
    }
    *record_out = record;
    return true;
}

static bool usb_vbus_record_still_live(
    const struct usb_vbus_record *record,
    const struct usb_vbus_handle *handle, u32 allowed_state_a,
    u32 allowed_state_b)
{
    u64 generation;
    u32 controller_id;
    u32 state;

    if (!record || !handle)
        return false;
    state = record->state;
    dmb_ishld();
    generation = record->generation;
    controller_id = record->controller_id;
    return generation == handle->generation &&
           controller_id == handle->controller_id &&
           (state == allowed_state_a || state == allowed_state_b);
}

static bool usb_vbus_mutable_record(
    struct usb_vbus_contract_pool *pool, const struct usb_vbus_handle *handle,
    u32 owner_core, struct usb_vbus_record **record_out)
{
    const struct usb_vbus_record *record_const;

    if (!usb_vbus_live_record(pool, handle, &record_const) ||
        owner_core != USB_VBUS_OWNER_CORE ||
        record_const->owner_core != owner_core)
        return false;
    *record_out = &pool->records[handle->slot];
    return true;
}

static bool usb_vbus_mutation_begin(
    struct usb_vbus_contract_pool *pool,
    const struct usb_vbus_handle *handle, u32 owner_core,
    struct usb_vbus_record **record_out)
{
    struct usb_vbus_record *record;

    if (!pool || owner_core != USB_VBUS_OWNER_CORE ||
        __atomic_exchange_n(&pool->owner.mutation_guard, 1U,
                            __ATOMIC_ACQUIRE) != 0U)
        return false;
    if (!usb_vbus_mutable_record(pool, handle, owner_core, &record)) {
        __atomic_store_n(&pool->owner.mutation_guard, 0U, __ATOMIC_RELEASE);
        return false;
    }
    if (__atomic_exchange_n(&record->mutation_guard, 1U,
                            __ATOMIC_ACQUIRE) != 0U) {
        __atomic_store_n(&pool->owner.mutation_guard, 0U, __ATOMIC_RELEASE);
        return false;
    }
    if (record->generation != handle->generation ||
        record->controller_id != handle->controller_id ||
        record->owner_core != owner_core ||
        record->state == USB_VBUS_FREE ||
        record->state == USB_VBUS_RELEASED ||
        record->state == USB_VBUS_RETIRED) {
        __atomic_store_n(&record->mutation_guard, 0U, __ATOMIC_RELEASE);
        __atomic_store_n(&pool->owner.mutation_guard, 0U, __ATOMIC_RELEASE);
        return false;
    }
    *record_out = record;
    return true;
}

static void usb_vbus_mutation_end(struct usb_vbus_contract_pool *pool,
                                  struct usb_vbus_record *record)
{
    __atomic_store_n(&record->mutation_guard, 0U, __ATOMIC_RELEASE);
    __atomic_store_n(&pool->owner.mutation_guard, 0U, __ATOMIC_RELEASE);
}

static void usb_vbus_make_handle(const struct usb_vbus_record *record, u32 slot,
                                 struct usb_vbus_handle *handle_out)
{
    usb_vbus_clear_handle(handle_out);
    if (!handle_out)
        return;
    handle_out->_token = usb_vbus_token(record->controller_id);
    handle_out->generation = record->generation;
    handle_out->controller_id = record->controller_id;
    handle_out->slot = slot;
}

static void usb_vbus_publish_state(struct usb_vbus_record *record, u32 state)
{
    dmb_ishst();
    record->state = state;
    dmb_ishst();
}

static bool usb_vbus_observer_declared(
    const struct usb_vbus_external_attestation *attestation)
{
    return attestation->observer_id != 0U ||
           attestation->observer_generation != 0U;
}

static bool usb_vbus_attestation_valid(
    const struct usb_vbus_record *record,
    const struct usb_vbus_external_attestation *attestation)
{
    const struct usb_vbus_profile *profile;
    bool observer_declared;

    if (!record || !attestation || attestation->_reserved[0] != 0U ||
        attestation->_reserved[1] != 0U ||
        attestation->board_model != record->board_model ||
        attestation->topology_id == 0U ||
        attestation->topology_generation == 0U ||
        attestation->externally_powered != true ||
        attestation->independent_current_protection != true ||
        attestation->port_cable_verified != true ||
        attestation->no_backfeed_verified != true)
        return false;
    profile = usb_vbus_profile_get(attestation->board_model);
    if (!profile)
        return false;
    observer_declared = usb_vbus_observer_declared(attestation);
    if (!observer_declared)
        return attestation->declared_max_milliamps == 0U;
    return attestation->observer_id != 0U &&
           attestation->observer_generation != 0U &&
           attestation->declared_max_milliamps != 0U &&
           attestation->declared_max_milliamps <=
           USB_VBUS_MAX_ATTESTED_MILLIAMPS;
}

static void usb_vbus_quarantine(struct usb_vbus_record *record,
                                enum usb_vbus_fault fault, u32 current)
{
    record->fault = (u32)fault;
    record->last_current_milliamps = current;
    record->fault_count++;
    usb_vbus_publish_state(record, USB_VBUS_QUARANTINED);
}

const struct usb_vbus_profile *usb_vbus_profile_get(u32 board_model)
{
    u32 i;

    for (i = 0U; i < sizeof(usb_vbus_profiles) / sizeof(usb_vbus_profiles[0]);
         i++) {
        if (usb_vbus_profiles[i].board_model == board_model)
            return &usb_vbus_profiles[i];
    }
    return NULL;
}

bool usb_vbus_contract_pool_init(struct usb_vbus_contract_pool *pool)
{
    u32 i;

    if (!usb_vbus_zeroed(pool))
        return false;
    for (i = 0U; i < USB_VBUS_CONTRACT_CAPACITY; i++) {
        pool->records[i].generation = (u64)i + 1U;
        pool->records[i].state = USB_VBUS_FREE;
    }
    pool->owner.initialized = 1U;
    dmb_ishst();
    return true;
}

bool usb_vbus_contract_acquire(struct usb_vbus_contract_pool *pool,
                               u32 controller_id, u32 board_model,
                               u32 owner_core,
                               struct usb_vbus_handle *handle_out)
{
    const struct usb_vbus_profile *profile;
    u32 i;

    usb_vbus_clear_handle(handle_out);
    profile = usb_vbus_profile_get(board_model);
    if (!usb_vbus_pool_initialized(pool) || !handle_out || !profile ||
        controller_id == 0U ||
        owner_core != USB_VBUS_OWNER_CORE)
        return false;
    if (__atomic_exchange_n(&pool->owner.mutation_guard, 1U,
                            __ATOMIC_ACQUIRE) != 0U)
        return false;
    for (i = 0U; i < USB_VBUS_CONTRACT_CAPACITY; i++) {
        if (pool->records[i].controller_id == controller_id &&
            pool->records[i].state != USB_VBUS_FREE &&
            pool->records[i].state != USB_VBUS_RETIRED) {
            __atomic_store_n(&pool->owner.mutation_guard, 0U,
                             __ATOMIC_RELEASE);
            return false;
        }
    }
    for (i = 0U; i < USB_VBUS_CONTRACT_CAPACITY; i++) {
        struct usb_vbus_record *record = &pool->records[i];

        if (record->state != USB_VBUS_FREE)
            continue;
        if (__atomic_exchange_n(&record->mutation_guard, 1U,
                                __ATOMIC_ACQUIRE) != 0U)
            continue;
        if (record->state != USB_VBUS_FREE || record->generation == 0U) {
            __atomic_store_n(&record->mutation_guard, 0U, __ATOMIC_RELEASE);
            continue;
        }
        record->controller_id = controller_id;
        record->board_model = board_model;
        record->owner_core = owner_core;
        usb_vbus_publish_state(record, USB_VBUS_SAFE_UNKNOWN);
        usb_vbus_make_handle(record, i, handle_out);
        usb_vbus_mutation_end(pool, record);
        return true;
    }
    __atomic_store_n(&pool->owner.mutation_guard, 0U, __ATOMIC_RELEASE);
    return false;
}

bool usb_vbus_external_attest(
    struct usb_vbus_contract_pool *pool, const struct usb_vbus_handle *handle,
    u32 owner_core, const struct usb_vbus_external_attestation *attestation)
{
    struct usb_vbus_record *record;

    if (!usb_vbus_mutation_begin(pool, handle, owner_core, &record))
        return false;
    if (record->state != USB_VBUS_SAFE_UNKNOWN ||
        !usb_vbus_attestation_valid(record, attestation)) {
        usb_vbus_mutation_end(pool, record);
        return false;
    }
    usb_vbus_copy(&pool->attestations[handle->slot].value, attestation,
                  sizeof(*attestation));
    record->fault = USB_VBUS_FAULT_NONE;
    record->last_current_milliamps = 0U;
    record->observation_count = 0U;
    usb_vbus_publish_state(record, USB_VBUS_EXTERNAL_ATTESTED);
    usb_vbus_mutation_end(pool, record);
    return true;
}

bool usb_vbus_current_observe(
    struct usb_vbus_contract_pool *pool, const struct usb_vbus_handle *handle,
    u32 owner_core, const struct usb_vbus_current_observation *observation)
{
    struct usb_vbus_record *record;
    const struct usb_vbus_external_attestation *attestation;

    if (!usb_vbus_mutation_begin(pool, handle, owner_core, &record))
        return false;
    if ((record->state != USB_VBUS_EXTERNAL_ATTESTED &&
         record->state != USB_VBUS_AVAILABLE_UNCONTROLLED) ||
        !observation || observation->_reserved != 0U ||
        observation->overcurrent > true) {
        usb_vbus_mutation_end(pool, record);
        return false;
    }
    attestation = &pool->attestations[handle->slot].value;
    if (!usb_vbus_observer_declared(attestation) ||
        observation->observer_id == 0U || observation->observer_generation == 0U ||
        observation->observer_id != attestation->observer_id ||
        observation->observer_generation != attestation->observer_generation) {
        usb_vbus_mutation_end(pool, record);
        return false;
    }
    record->observation_count++;
    if (observation->overcurrent == true) {
        usb_vbus_quarantine(record, USB_VBUS_FAULT_OVERCURRENT,
                            observation->current_milliamps);
        usb_vbus_mutation_end(pool, record);
        return false;
    }
    if (observation->current_milliamps > attestation->declared_max_milliamps) {
        usb_vbus_quarantine(record, USB_VBUS_FAULT_CURRENT_LIMIT,
                            observation->current_milliamps);
        usb_vbus_mutation_end(pool, record);
        return false;
    }
    record->last_current_milliamps = observation->current_milliamps;
    usb_vbus_publish_state(record, USB_VBUS_AVAILABLE_UNCONTROLLED);
    usb_vbus_mutation_end(pool, record);
    return true;
}

bool usb_vbus_contract_release(struct usb_vbus_contract_pool *pool,
                               const struct usb_vbus_handle *handle,
                               u32 owner_core)
{
    struct usb_vbus_record *record;
    u32 slot;

    if (!usb_vbus_mutation_begin(pool, handle, owner_core, &record))
        return false;
    if (record->state != USB_VBUS_SAFE_UNKNOWN &&
        record->state != USB_VBUS_EXTERNAL_ATTESTED &&
        record->state != USB_VBUS_AVAILABLE_UNCONTROLLED &&
        record->state != USB_VBUS_QUARANTINED) {
        usb_vbus_mutation_end(pool, record);
        return false;
    }
    slot = handle->slot;
    usb_vbus_publish_state(record, USB_VBUS_RELEASED);
    if (record->generation == ~0ULL) {
        usb_vbus_zero(&pool->attestations[slot], sizeof(pool->attestations[slot]));
        record->controller_id = 0U;
        record->board_model = 0U;
        record->owner_core = 0U;
        record->fault = USB_VBUS_FAULT_NONE;
        record->fault_count = 0U;
        record->last_current_milliamps = 0U;
        record->observation_count = 0U;
        usb_vbus_publish_state(record, USB_VBUS_RETIRED);
        usb_vbus_mutation_end(pool, record);
        return true;
    }
    record->generation++;
    usb_vbus_zero(&pool->attestations[slot], sizeof(pool->attestations[slot]));
    record->controller_id = 0U;
    record->board_model = 0U;
    record->owner_core = 0U;
    record->fault = USB_VBUS_FAULT_NONE;
    record->fault_count = 0U;
    record->last_current_milliamps = 0U;
    record->observation_count = 0U;
    usb_vbus_publish_state(record, USB_VBUS_FREE);
    usb_vbus_mutation_end(pool, record);
    return true;
}

bool usb_vbus_handle_get(const struct usb_vbus_contract_pool *pool,
                         u32 controller_id, struct usb_vbus_handle *handle_out)
{
    u32 i;

    usb_vbus_clear_handle(handle_out);
    if (!pool || !handle_out || controller_id == 0U)
        return false;
    for (i = 0U; i < USB_VBUS_CONTRACT_CAPACITY; i++) {
        const struct usb_vbus_record *record = &pool->records[i];
        const struct usb_vbus_record *verified;
        u32 state = record->state;
        u32 observed_controller;
        u64 generation;

        dmb_ishld();
        observed_controller = record->controller_id;
        generation = record->generation;
        if (observed_controller != controller_id ||
            state == USB_VBUS_FREE || state == USB_VBUS_RELEASED ||
            state == USB_VBUS_RETIRED)
            continue;
        handle_out->_token = usb_vbus_token(observed_controller);
        handle_out->generation = generation;
        handle_out->controller_id = observed_controller;
        handle_out->slot = i;
        handle_out->_reserved = 0U;
        handle_out->_pad = 0U;
        if (usb_vbus_live_record(pool, handle_out, &verified) &&
            verified == record)
            return true;
        usb_vbus_clear_handle(handle_out);
    }
    return false;
}

bool usb_vbus_state_get(const struct usb_vbus_contract_pool *pool,
                        const struct usb_vbus_handle *handle,
                        enum usb_vbus_state *state_out)
{
    const struct usb_vbus_record *record;
    u32 state;

    if (!state_out || !usb_vbus_live_record(pool, handle, &record))
        return false;
    state = record->state;
    if (!usb_vbus_record_still_live(record, handle, state, state))
        return false;
    *state_out = (enum usb_vbus_state)state;
    return true;
}

bool usb_vbus_attestation_get(
    const struct usb_vbus_contract_pool *pool, const struct usb_vbus_handle *handle,
    struct usb_vbus_external_attestation *attestation_out)
{
    const struct usb_vbus_record *record;

    if (!attestation_out || !usb_vbus_live_record(pool, handle, &record) ||
        (record->state != USB_VBUS_EXTERNAL_ATTESTED &&
         record->state != USB_VBUS_AVAILABLE_UNCONTROLLED &&
         record->state != USB_VBUS_QUARANTINED))
        return false;
    usb_vbus_copy(attestation_out, &pool->attestations[handle->slot].value,
                  sizeof(*attestation_out));
    return usb_vbus_record_still_live(
        record, handle, USB_VBUS_EXTERNAL_ATTESTED,
        record->state == USB_VBUS_QUARANTINED
            ? USB_VBUS_QUARANTINED : USB_VBUS_AVAILABLE_UNCONTROLLED);
}

bool usb_vbus_telemetry_available(const struct usb_vbus_contract_pool *pool,
                                  const struct usb_vbus_handle *handle)
{
    const struct usb_vbus_record *record;
    const struct usb_vbus_external_attestation *attestation;

    if (!usb_vbus_live_record(pool, handle, &record))
        return false;
    attestation = &pool->attestations[handle->slot].value;
    if (!usb_vbus_observer_declared(attestation))
        return false;
    return usb_vbus_record_still_live(
        record, handle, USB_VBUS_EXTERNAL_ATTESTED,
        USB_VBUS_AVAILABLE_UNCONTROLLED);
}

bool usb_vbus_probe_permitted(const struct usb_vbus_contract_pool *pool,
                              const struct usb_vbus_handle *handle)
{
    const struct usb_vbus_record *record;
    const struct usb_vbus_external_attestation *attestation;

    if (!usb_vbus_live_record(pool, handle, &record) ||
        (record->state != USB_VBUS_EXTERNAL_ATTESTED &&
         record->state != USB_VBUS_AVAILABLE_UNCONTROLLED))
        return false;
    attestation = &pool->attestations[handle->slot].value;
    if (!usb_vbus_attestation_valid(record, attestation))
        return false;
    if (usb_vbus_observer_declared(attestation) &&
        record->state != USB_VBUS_AVAILABLE_UNCONTROLLED)
        return false;
    return usb_vbus_record_still_live(
        record, handle, USB_VBUS_EXTERNAL_ATTESTED,
        USB_VBUS_AVAILABLE_UNCONTROLLED);
}

bool usb_vbus_software_enable_permitted(const struct usb_vbus_profile *profile,
                                        u32 *missing_proof_mask_out)
{
    u32 missing = USB_VBUS_PROOF_CONTROL | USB_VBUS_PROOF_CURRENT_LIMIT |
                  USB_VBUS_PROOF_OVERCURRENT;

    /*
     * Profile input deliberately cannot add authority: it merely lets callers
     * report which known board they evaluated.  These three proofs are absent
     * from every BCM2837 profile, and an invented mutable profile is not proof.
     */
    (void)profile;
    if (missing_proof_mask_out)
        *missing_proof_mask_out = missing;
    return false;
}
