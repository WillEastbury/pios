/*
 * Pure ADR-054 framebuffer handoff policy.  No hardware dependency belongs
 * here: callers submit only bounded numeric facts and backend attestations.
 */
#include "types.h"
#include "hvs_handoff_contract.h"

#define HVS_HANDOFF_TOKEN_MAGIC       0x48565348ULL
#define HVS_HANDOFF_TOKEN_MAGIC_SHIFT 32U
static void hvs_handoff_clear_handle(struct hvs_handoff_handle *handle)
{
    if (handle)
        memset(handle, 0, sizeof(*handle));
}

static bool hvs_handoff_zeroed(const struct hvs_handoff_contract *contract)
{
    const u8 *bytes = (const u8 *)contract;
    usize i;

    for (i = 0U; i < sizeof(*contract); i++) {
        if (bytes[i] != 0U)
            return false;
    }
    return true;
}

static bool hvs_handoff_live(const struct hvs_handoff_contract *contract)
{
    return contract && contract->control.controller_id != 0U &&
           contract->control.owner_core == HVS_HANDOFF_OWNER_CORE &&
           contract->control.generation != 0U;
}

static bool hvs_handoff_handle_valid(const struct hvs_handoff_contract *contract,
                                     const struct hvs_handoff_handle *handle)
{
    dmb_ishld();
    return hvs_handoff_live(contract) && handle &&
           handle->_reserved == 0U &&
           handle->_controller_id == contract->control.controller_id &&
           handle->_generation == contract->control.generation &&
           handle->_generation != 0U &&
           handle->_token == ((HVS_HANDOFF_TOKEN_MAGIC <<
                               HVS_HANDOFF_TOKEN_MAGIC_SHIFT) |
                              (u64)handle->_controller_id);
}

static void hvs_handoff_make_handle(const struct hvs_handoff_contract *contract,
                                    struct hvs_handoff_handle *handle_out)
{
    hvs_handoff_clear_handle(handle_out);
    if (!handle_out)
        return;
    handle_out->_token = (HVS_HANDOFF_TOKEN_MAGIC <<
                          HVS_HANDOFF_TOKEN_MAGIC_SHIFT) |
                         (u64)contract->control.controller_id;
    handle_out->_generation = contract->control.generation;
    handle_out->_controller_id = contract->control.controller_id;
}

static void hvs_handoff_publish_state(struct hvs_handoff_contract *contract,
                                      enum hvs_handoff_state state)
{
    dmb_ishst();
    contract->control.state = state;
    dmb_ishst();
}

static void hvs_handoff_quarantine(struct hvs_handoff_contract *contract,
                                   enum hvs_handoff_fault fault, u32 status,
                                   u64 observed)
{
    contract->control.fault = (u32)fault;
    contract->control.last_status = status;
    contract->control.last_observed = observed;
    contract->control.failure_count++;
    hvs_handoff_publish_state(contract, HVS_HANDOFF_QUARANTINED);
}

static bool hvs_handoff_time_valid(struct hvs_handoff_contract *contract,
                                   u64 now_ms)
{
    if (now_ms <= contract->control.deadline_ms)
        return true;
    hvs_handoff_quarantine(contract, HVS_HANDOFF_FAULT_DEADLINE, 0U, now_ms);
    return false;
}

static bool hvs_handoff_mutable(struct hvs_handoff_contract *contract,
                                 const struct hvs_handoff_handle *handle,
                                 u32 owner_core, u64 now_ms)
{
    if (!hvs_handoff_handle_valid(contract, handle))
        return false;
    if (owner_core != HVS_HANDOFF_OWNER_CORE ||
        contract->control.owner_core != owner_core) {
        hvs_handoff_quarantine(contract, HVS_HANDOFF_FAULT_OWNER, owner_core,
                               now_ms);
        return false;
    }
    return hvs_handoff_time_valid(contract, now_ms);
}

static bool hvs_handoff_snapshot_valid(
    const struct hvs_handoff_snapshot_input *snapshot)
{
    u64 row_bytes;
    u64 required_bytes;

    if (!snapshot || snapshot->_reserved != 0U ||
        snapshot->owner != HVS_HANDOFF_OWNER_MAILBOX ||
        snapshot->snapshot_layout_version != HVS_HANDOFF_VERSION_V1 ||
        snapshot->hvs_id == 0U ||
        snapshot->target_channel != HVS_HANDOFF_CHANNEL_V1 ||
        snapshot->scanout_base == 0U || snapshot->scanout_generation == 0U ||
        snapshot->existing_list_generation == 0U ||
        snapshot->existing_list_index > HVS_HANDOFF_MAX_STAGING_INDEX ||
        snapshot->width == 0U || snapshot->height == 0U ||
        snapshot->width > HVS_HANDOFF_MAX_DIMENSION ||
        snapshot->height > HVS_HANDOFF_MAX_DIMENSION ||
        snapshot->pitch == 0U || snapshot->pitch > HVS_HANDOFF_MAX_PITCH ||
        snapshot->framebuffer_bytes == 0U ||
        snapshot->scanout_base >= HVS_HANDOFF_MAX_SCANOUT_BYTES ||
        snapshot->framebuffer_bytes >
        HVS_HANDOFF_MAX_SCANOUT_BYTES - snapshot->scanout_base ||
        snapshot->format != HVS_HANDOFF_FORMAT_RGBA8888 ||
        (snapshot->pixel_order != HVS_HANDOFF_PIXEL_ORDER_RGBA &&
         snapshot->pixel_order != HVS_HANDOFF_PIXEL_ORDER_ARGB))
        return false;
    row_bytes = (u64)snapshot->width * 4U;
    if (row_bytes > snapshot->pitch ||
        snapshot->height > (~0ULL) / snapshot->pitch)
        return false;
    required_bytes = (u64)snapshot->pitch * snapshot->height;
    return required_bytes <= snapshot->framebuffer_bytes;
}

static bool hvs_handoff_list_valid(const struct hvs_handoff_contract *contract,
                                   const struct hvs_handoff_list_input *list)
{
    u32 range_end;

    if (!list || list->_reserved != 0U || list->element_count == 0U ||
        list->element_count > HVS_HANDOFF_MAX_LIST_ELEMENTS ||
        list->list_generation == 0U || list->staging_index >
        HVS_HANDOFF_MAX_STAGING_INDEX || list->staging_range_start >
        HVS_HANDOFF_MAX_STAGING_INDEX || list->staging_range_count == 0U ||
        list->staging_range_count > HVS_HANDOFF_MAX_LIST_ELEMENTS ||
        list->source_scanout_base != contract->snapshot.scanout_base ||
        list->source_scanout_generation != contract->snapshot.scanout_generation ||
        list->width != contract->snapshot.width ||
        list->height != contract->snapshot.height ||
        list->pitch != contract->snapshot.pitch ||
        list->format != HVS_HANDOFF_FORMAT_RGBA8888 ||
        list->format != contract->snapshot.format ||
        list->pixel_order != contract->snapshot.pixel_order)
        return false;
    range_end = list->staging_range_start + list->staging_range_count;
    return range_end >= list->staging_range_start &&
           range_end <= HVS_HANDOFF_MAX_STAGING_INDEX + 1U &&
           list->staging_index >= list->staging_range_start &&
           list->staging_index < range_end;
}

static bool hvs_handoff_observation_matches(
    const struct hvs_handoff_contract *contract,
    const struct hvs_handoff_observation *observation, u32 attestation)
{
    return observation && observation->attestation == attestation &&
           observation->channel == contract->snapshot.target_channel &&
           observation->list_generation == contract->list.list_generation &&
           observation->observed_list_index == contract->list.staging_index;
}

bool hvs_handoff_contract_init(struct hvs_handoff_contract *contract,
                               u32 controller_id, u32 owner_core,
                               u64 now_ms, u32 deadline_ms)
{
    u64 deadline;

    if (!contract || controller_id == 0U ||
        owner_core != HVS_HANDOFF_OWNER_CORE || deadline_ms == 0U ||
        deadline_ms > HVS_HANDOFF_MAX_DEADLINE_MS ||
        !hvs_handoff_zeroed(contract) ||
        (u64)deadline_ms > ~0ULL - now_ms)
        return false;
    deadline = now_ms + deadline_ms;
    contract->control.controller_id = controller_id;
    contract->control.owner_core = owner_core;
    contract->control.generation = 1U;
    contract->control.deadline_ms = deadline;
    hvs_handoff_publish_state(contract, HVS_HANDOFF_MAILBOX_OWNER);
    return true;
}

bool hvs_handoff_snapshot(struct hvs_handoff_contract *contract,
                          const struct hvs_handoff_handle *handle,
                          u32 owner_core, u64 now_ms,
                          const struct hvs_handoff_snapshot_input *snapshot)
{
    if (!hvs_handoff_mutable(contract, handle, owner_core, now_ms))
        return false;
    if (contract->control.state != HVS_HANDOFF_MAILBOX_OWNER)
        return false;
    contract->control.handoff_attempt_count++;
    if (!hvs_handoff_snapshot_valid(snapshot)) {
        hvs_handoff_quarantine(contract, HVS_HANDOFF_FAULT_SNAPSHOT, 0U, 0U);
        return false;
    }
    memcpy(&contract->snapshot, snapshot, sizeof(*snapshot));
    dmb_ishst();
    hvs_handoff_publish_state(contract, HVS_HANDOFF_SNAPSHOT_VALID);
    return true;
}

bool hvs_handoff_stage_list(struct hvs_handoff_contract *contract,
                            const struct hvs_handoff_handle *handle,
                            u32 owner_core, u64 now_ms,
                            const struct hvs_handoff_list_input *list)
{
    if (!hvs_handoff_mutable(contract, handle, owner_core, now_ms))
        return false;
    if (contract->control.state != HVS_HANDOFF_SNAPSHOT_VALID)
        return false;
    if (!hvs_handoff_list_valid(contract, list)) {
        hvs_handoff_quarantine(contract, HVS_HANDOFF_FAULT_LIST, 0U, 0U);
        return false;
    }
    memcpy(&contract->list, list, sizeof(*list));
    dmb_ishst();
    hvs_handoff_publish_state(contract, HVS_HANDOFF_LIST_STAGED);
    return true;
}

bool hvs_handoff_report_arm(struct hvs_handoff_contract *contract,
                            const struct hvs_handoff_handle *handle,
                            u32 owner_core, u64 now_ms,
                            const struct hvs_handoff_observation *observation)
{
    if (!hvs_handoff_mutable(contract, handle, owner_core, now_ms))
        return false;
    if (contract->control.state != HVS_HANDOFF_LIST_STAGED)
        return false;
    if (!hvs_handoff_observation_matches(contract, observation,
                                         HVS_HANDOFF_ATTESTATION_MATCHED)) {
        hvs_handoff_quarantine(contract, HVS_HANDOFF_FAULT_ARM,
                               observation ? observation->status : 0U,
                               observation ? observation->observed_list_index : 0U);
        return false;
    }
    contract->control.last_status = observation->status;
    contract->control.last_observed = observation->observed_list_index;
    hvs_handoff_publish_state(contract, HVS_HANDOFF_ARM_VERIFIED);
    return true;
}

bool hvs_handoff_report_present(struct hvs_handoff_contract *contract,
                                const struct hvs_handoff_handle *handle,
                                u32 owner_core, u64 now_ms,
                                const struct hvs_handoff_observation *observation)
{
    if (!hvs_handoff_mutable(contract, handle, owner_core, now_ms))
        return false;
    if (contract->control.state != HVS_HANDOFF_ARM_VERIFIED)
        return false;
    if (!hvs_handoff_observation_matches(contract, observation,
                                         HVS_HANDOFF_ATTESTATION_PRESENTED)) {
        hvs_handoff_quarantine(contract, HVS_HANDOFF_FAULT_PRESENT,
                               observation ? observation->status : 0U,
                               observation ? observation->observed_list_index : 0U);
        return false;
    }
    contract->control.last_status = observation->status;
    contract->control.last_observed = observation->observed_list_index;
    hvs_handoff_publish_state(contract, HVS_HANDOFF_NATIVE_OWNER);
    return true;
}

bool hvs_handoff_restore_begin(struct hvs_handoff_contract *contract,
                               const struct hvs_handoff_handle *handle,
                               u32 owner_core, u64 now_ms)
{
    if (!hvs_handoff_mutable(contract, handle, owner_core, now_ms))
        return false;
    if (contract->control.state != HVS_HANDOFF_NATIVE_OWNER &&
        contract->control.state != HVS_HANDOFF_QUARANTINED)
        return false;
    if (contract->snapshot.owner != HVS_HANDOFF_OWNER_MAILBOX)
        return false;
    contract->control.restore_attempt_count++;
    hvs_handoff_publish_state(contract, HVS_HANDOFF_RESTORING);
    return true;
}

bool hvs_handoff_report_restore(
    struct hvs_handoff_contract *contract,
    const struct hvs_handoff_handle *handle, u32 owner_core, u64 now_ms,
    const struct hvs_handoff_restore_observation *observation)
{
    if (!hvs_handoff_mutable(contract, handle, owner_core, now_ms))
        return false;
    if (contract->control.state != HVS_HANDOFF_RESTORING)
        return false;
    if (!observation ||
        observation->attestation != HVS_HANDOFF_ATTESTATION_RESTORED ||
        observation->channel != contract->snapshot.target_channel ||
        observation->existing_list_generation !=
        contract->snapshot.existing_list_generation ||
        observation->observed_list_index != contract->snapshot.existing_list_index ||
        observation->observed_owner != HVS_HANDOFF_OWNER_MAILBOX) {
        hvs_handoff_quarantine(contract, HVS_HANDOFF_FAULT_RESTORE,
                               observation ? observation->status : 0U,
                               observation ? observation->observed_list_index : 0U);
        return false;
    }
    contract->control.last_status = observation->status;
    contract->control.last_observed = observation->observed_list_index;
    contract->control.fault = HVS_HANDOFF_FAULT_NONE;
    hvs_handoff_publish_state(contract, HVS_HANDOFF_MAILBOX_OWNER);
    return true;
}

bool hvs_handoff_rearm(struct hvs_handoff_contract *contract,
                       const struct hvs_handoff_handle *handle,
                       u32 owner_core, u64 now_ms,
                       struct hvs_handoff_handle *new_handle_out)
{
    hvs_handoff_clear_handle(new_handle_out);
    if (!hvs_handoff_handle_valid(contract, handle) || !new_handle_out ||
        owner_core != HVS_HANDOFF_OWNER_CORE ||
        contract->control.owner_core != owner_core ||
        contract->control.state != HVS_HANDOFF_QUARANTINED)
        return false;
    if (now_ms > contract->control.deadline_ms) {
        hvs_handoff_quarantine(contract, HVS_HANDOFF_FAULT_DEADLINE, 0U, now_ms);
        return false;
    }
    if (contract->control.generation == ~0ULL)
        return false;
    contract->control.generation++;
    memset(&contract->snapshot, 0, sizeof(contract->snapshot));
    memset(&contract->list, 0, sizeof(contract->list));
    contract->control.fault = HVS_HANDOFF_FAULT_NONE;
    contract->control.last_status = 0U;
    contract->control.last_observed = 0U;
    contract->control.handoff_attempt_count = 0U;
    contract->control.restore_attempt_count = 0U;
    contract->control.failure_count = 0U;
    hvs_handoff_publish_state(contract, HVS_HANDOFF_MAILBOX_OWNER);
    hvs_handoff_make_handle(contract, new_handle_out);
    return true;
}

bool hvs_handoff_handle_get(const struct hvs_handoff_contract *contract,
                            struct hvs_handoff_handle *handle_out)
{
    if (!hvs_handoff_live(contract) || !handle_out)
        return false;
    hvs_handoff_make_handle(contract, handle_out);
    return true;
}

bool hvs_handoff_state_get(const struct hvs_handoff_contract *contract,
                           const struct hvs_handoff_handle *handle,
                           enum hvs_handoff_state *state_out)
{
    if (!state_out || !hvs_handoff_handle_valid(contract, handle))
        return false;
    *state_out = (enum hvs_handoff_state)contract->control.state;
    return true;
}

bool hvs_handoff_snapshot_get(
    const struct hvs_handoff_contract *contract,
    const struct hvs_handoff_handle *handle,
    struct hvs_handoff_snapshot_input *snapshot_out)
{
    if (!snapshot_out || !hvs_handoff_handle_valid(contract, handle))
        return false;
    memcpy(snapshot_out, &contract->snapshot, sizeof(*snapshot_out));
    return true;
}

bool hvs_handoff_list_get(const struct hvs_handoff_contract *contract,
                          const struct hvs_handoff_handle *handle,
                          struct hvs_handoff_list_input *list_out)
{
    if (!list_out || !hvs_handoff_handle_valid(contract, handle))
        return false;
    memcpy(list_out, &contract->list, sizeof(*list_out));
    return true;
}
