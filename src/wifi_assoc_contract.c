/*
 * wifi_assoc_contract.c - ADR-077 bounded offline association evidence model.
 *
 * This unit contains neither a transport adapter nor credentials.  It only
 * accepts copied observations and emits numeric evidence for a future adapter.
 */
#include "types.h"
#include "wifi_assoc_contract.h"

static u64 assoc_irq_save(void)
{
#ifdef PIOS_HOST_TYPES_SHIM
    return 0U;
#else
    u64 daif;

    __asm__ volatile("mrs %0, daif" : "=r"(daif));
    __asm__ volatile("msr daifset, #2" ::: "memory");
    return daif;
#endif
}

static void assoc_irq_restore(u64 daif)
{
#ifdef PIOS_HOST_TYPES_SHIM
    (void)daif;
#else
    __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
#endif
}

static void assoc_zero(void *value, usize bytes)
{
    u8 *p = (u8 *)value;
    usize i;

    for (i = 0U; i < bytes; i++)
        p[i] = 0U;
}

static bool assoc_fresh(const struct wifi_assoc_contract *contract)
{
    const u8 *bytes;
    usize i;

    if (!contract)
        return false;
    bytes = (const u8 *)contract;
    for (i = 0U; i < sizeof(*contract); i++) {
        if (bytes[i] != 0U)
            return false;
    }
    return true;
}

static bool assoc_owner(const struct wifi_assoc_contract *contract,
                        u32 caller_core)
{
    return contract && caller_core == WIFI_ASSOC_OWNER_CORE &&
           core_id() == WIFI_ASSOC_OWNER_CORE &&
           contract->control.owner_core == WIFI_ASSOC_OWNER_CORE &&
           contract->control.instance_epoch != 0U;
}

static bool assoc_active(const struct wifi_assoc_contract *contract)
{
    u32 state = contract->control.state;

    return state == WIFI_ASSOC_PREPARE || state == WIFI_ASSOC_WAIT_CREDIT ||
           state == WIFI_ASSOC_SET_SSID_PUBLISHED ||
           state == WIFI_ASSOC_WAIT_ASSOC;
}

static bool assoc_handle_valid(const struct wifi_assoc_contract *contract,
                               const struct wifi_assoc_handle *handle)
{
    return handle && handle->token == WIFI_ASSOC_HANDLE_MAGIC &&
           handle->_reserved == 0U &&
           handle->instance_epoch == contract->control.instance_epoch &&
           handle->attempt_generation == contract->control.attempt_generation &&
           handle->attempt_generation != 0U;
}

static void assoc_publish(struct wifi_assoc_contract *contract, u32 state)
{
    dmb_ishst();
    contract->control.state = state;
    contract->control.watchdog_evidence++;
    dmb_ishst();
}

static void assoc_fail(struct wifi_assoc_contract *contract,
                       enum wifi_assoc_fault fault, bool quarantine)
{
    contract->control.fault = (u8)fault;
    dmb_ishst();
    contract->control.state = quarantine ? WIFI_ASSOC_QUARANTINED :
                                           WIFI_ASSOC_FAILED;
    dmb_ishst();
}

static void assoc_history_push(struct wifi_assoc_contract *contract,
                               const struct wifi_assoc_event *event)
{
    struct wifi_assoc_control *control = &contract->control;
    struct wifi_assoc_event_record *record =
        &contract->history[control->history_head];

    record->timestamp_ms = event->timestamp_ms;
    record->sequence = event->timestamp_ms;
    record->flags = event->flags;
    record->reason = event->reason;
    record->type = event->type;
    record->status = event->status;
    record->psk_state = event->type == WIFI_ASSOC_EVENT_PSK_SUP ?
                        event->status : 0U;
    dmb_ishst();
    control->history_head =
        (u8)((control->history_head + 1U) % WIFI_ASSOC_EVENT_HISTORY);
    if (control->history_count < WIFI_ASSOC_EVENT_HISTORY)
        control->history_count++;
}

static bool assoc_psk_state_valid(u8 state)
{
    return state >= WIFI_ASSOC_PSK_SUP_WAIT_M1 &&
           state <= WIFI_ASSOC_PSK_SUP_PREP_G2;
}

static bool assoc_eapol_summary_valid(
    const struct wifi_assoc_eapol_summary *summary)
{
    u8 classification;

    if (!summary)
        return false;
    classification = WIFI_ASSOC_EAPOL_OTHER;
    if ((summary->key_info & 0x0188U) == 0x0088U)
        classification = WIFI_ASSOC_EAPOL_M1;
    else if ((summary->key_info & 0x03C8U) == 0x03C8U)
        classification = WIFI_ASSOC_EAPOL_M3;
    else if ((summary->key_info & 0x0388U) == 0x0380U)
        classification = WIFI_ASSOC_EAPOL_G1;
    return summary->valid && summary->replay_counter != 0U &&
           summary->classification <= WIFI_ASSOC_EAPOL_G1 &&
           (summary->key_info & 0x0007U) != 0U &&
           summary->classification == classification &&
           summary->_reserved[0] == 0U && summary->_reserved[1] == 0U &&
           summary->_reserved[2] == 0U && summary->_reserved[3] == 0U;
}

static bool assoc_event_valid_for_state(
    const struct wifi_assoc_contract *contract,
    const struct wifi_assoc_observation *observation)
{
    const struct wifi_assoc_event *event = &observation->event;

    if (event->type == WIFI_ASSOC_EVENT_NONE)
        return event->timestamp_ms == 0U && event->flags == 0U &&
               event->reason == 0U && event->status == 0U;
    if (contract->control.state != WIFI_ASSOC_WAIT_ASSOC ||
        event->timestamp_ms == 0U || event->timestamp_ms > observation->now_ms ||
        event->timestamp_ms <= contract->control.last_event_timestamp)
        return false;
    if (event->type == WIFI_ASSOC_EVENT_PSK_SUP)
        return event->flags == 0U && event->reason == 0U &&
               assoc_psk_state_valid(event->status);
    if (event->type == WIFI_ASSOC_EVENT_DEAUTH)
        return event->flags == 0U && event->status == 0U;
    if (event->type == WIFI_ASSOC_EVENT_LINK)
        return event->reason == 0U && event->status == 0U &&
               event->flags != 0U &&
               (event->flags & ~(WIFI_ASSOC_EVENT_F_LINK_UP |
                                  WIFI_ASSOC_EVENT_F_AUTHORIZED)) == 0U;
    return false;
}

static enum wifi_assoc_step_result assoc_consume_event(
    struct wifi_assoc_contract *contract,
    const struct wifi_assoc_observation *observation)
{
    const struct wifi_assoc_event *event = &observation->event;

    if (event->type == WIFI_ASSOC_EVENT_NONE)
        return WIFI_ASSOC_STEP_NO_PROGRESS;
    if (!assoc_event_valid_for_state(contract, observation)) {
        assoc_fail(contract, WIFI_ASSOC_FAULT_EVENT, true);
        return WIFI_ASSOC_STEP_QUARANTINED;
    }
    contract->control.last_event_timestamp = event->timestamp_ms;
    assoc_history_push(contract, event);
    if (event->type == WIFI_ASSOC_EVENT_DEAUTH ||
        (event->type == WIFI_ASSOC_EVENT_PSK_SUP &&
         event->status == WIFI_ASSOC_PSK_SUP_TIMEOUT)) {
        assoc_fail(contract, WIFI_ASSOC_FAULT_EVENT, false);
        return WIFI_ASSOC_STEP_FAILED;
    }
    if (event->type == WIFI_ASSOC_EVENT_LINK &&
        event->flags == (WIFI_ASSOC_EVENT_F_LINK_UP |
                         WIFI_ASSOC_EVENT_F_AUTHORIZED)) {
        assoc_publish(contract, WIFI_ASSOC_AUTHORIZED);
        return WIFI_ASSOC_STEP_AUTHORIZED;
    }
    return WIFI_ASSOC_STEP_NO_PROGRESS;
}

bool wifi_assoc_hardware_enable_allowed(void)
{
    return false;
}

static bool assoc_init_locked(struct wifi_assoc_contract *contract,
                              u32 caller_core)
{
    if (caller_core != WIFI_ASSOC_OWNER_CORE ||
        core_id() != WIFI_ASSOC_OWNER_CORE || !assoc_fresh(contract))
        return false;
    contract->control.instance_epoch = 1U;
    contract->control.owner_core = WIFI_ASSOC_OWNER_CORE;
    contract->control.state = WIFI_ASSOC_IDLE;
    dmb_ishst();
    return true;
}

bool wifi_assoc_init(struct wifi_assoc_contract *contract, u32 caller_core)
{
    u64 irq_state = assoc_irq_save();
    bool result = assoc_init_locked(contract, caller_core);

    assoc_irq_restore(irq_state);
    return result;
}

static bool assoc_start_locked(struct wifi_assoc_contract *contract,
                               u32 caller_core, u64 now_ms, u32 timeout_ms,
                               struct wifi_assoc_handle *out)
{
    if (out)
        assoc_zero(out, sizeof(*out));
    if (!assoc_owner(contract, caller_core) || !out ||
        contract->control.state != WIFI_ASSOC_IDLE || timeout_ms == 0U ||
        timeout_ms > WIFI_ASSOC_MAX_TIMEOUT_MS ||
        now_ms > ~0ULL - (u64)timeout_ms ||
        contract->control.attempt_generation == ~0ULL)
        return false;
    contract->control.attempt_generation++;
    contract->control.deadline_ms = now_ms + timeout_ms;
    contract->control.last_liveness_sequence = 0U;
    contract->control.last_event_timestamp = 0U;
    contract->control.last_eapol_replay = 0U;
    contract->control.step_count = 0U;
    contract->control.watchdog_evidence = 0U;
    contract->control.history_head = 0U;
    contract->control.history_count = 0U;
    contract->control.fault = WIFI_ASSOC_FAULT_NONE;
    dmb_ishst();
    contract->control.state = WIFI_ASSOC_PREPARE;
    dmb_ishst();
    out->token = WIFI_ASSOC_HANDLE_MAGIC;
    out->instance_epoch = contract->control.instance_epoch;
    out->attempt_generation = contract->control.attempt_generation;
    return true;
}

bool wifi_assoc_start(struct wifi_assoc_contract *contract, u32 caller_core,
                      u64 now_ms, u32 timeout_ms,
                      struct wifi_assoc_handle *out)
{
    u64 irq_state = assoc_irq_save();
    bool result = assoc_start_locked(contract, caller_core, now_ms, timeout_ms,
                                     out);

    assoc_irq_restore(irq_state);
    return result;
}

static enum wifi_assoc_step_result assoc_step_locked(
    struct wifi_assoc_contract *contract, u32 caller_core,
    const struct wifi_assoc_handle *handle,
    const struct wifi_assoc_observation *observation)
{
    enum wifi_assoc_step_result event_result;

    if (!assoc_owner(contract, caller_core) || !assoc_handle_valid(contract, handle) ||
        !observation || !assoc_active(contract))
        return WIFI_ASSOC_STEP_REJECTED;
    if (observation->now_ms >= contract->control.deadline_ms) {
        assoc_fail(contract, WIFI_ASSOC_FAULT_DEADLINE, false);
        return WIFI_ASSOC_STEP_FAILED;
    }
    if (contract->control.step_count >= WIFI_ASSOC_MAX_STEPS) {
        assoc_fail(contract, WIFI_ASSOC_FAULT_STEP_LIMIT, false);
        return WIFI_ASSOC_STEP_FAILED;
    }
    contract->control.step_count++;
    if (!observation->liveness_ran ||
        observation->liveness_sequence <= contract->control.last_liveness_sequence)
        return WIFI_ASSOC_STEP_NO_PROGRESS;
    contract->control.last_liveness_sequence = observation->liveness_sequence;
    if (observation->eapol_present) {
        if (!assoc_eapol_summary_valid(&observation->eapol) ||
            observation->eapol.replay_counter <=
            contract->control.last_eapol_replay) {
            assoc_fail(contract, WIFI_ASSOC_FAULT_EAPOL, true);
            return WIFI_ASSOC_STEP_QUARANTINED;
        }
        contract->control.last_eapol_replay = observation->eapol.replay_counter;
    }
    event_result = assoc_consume_event(contract, observation);
    if (event_result != WIFI_ASSOC_STEP_NO_PROGRESS)
        return event_result;
    if (contract->control.state == WIFI_ASSOC_PREPARE) {
        assoc_publish(contract, WIFI_ASSOC_WAIT_CREDIT);
        return WIFI_ASSOC_STEP_PROGRESS;
    }
    if (contract->control.state == WIFI_ASSOC_WAIT_CREDIT &&
        observation->credit_available) {
        assoc_publish(contract, WIFI_ASSOC_SET_SSID_PUBLISHED);
        return WIFI_ASSOC_STEP_PROGRESS;
    }
    if (contract->control.state == WIFI_ASSOC_SET_SSID_PUBLISHED &&
        observation->control_publish_verified) {
        assoc_publish(contract, WIFI_ASSOC_WAIT_ASSOC);
        return WIFI_ASSOC_STEP_PROGRESS;
    }
    return WIFI_ASSOC_STEP_NO_PROGRESS;
}

enum wifi_assoc_step_result wifi_assoc_step(
    struct wifi_assoc_contract *contract, u32 caller_core,
    const struct wifi_assoc_handle *handle,
    const struct wifi_assoc_observation *observation)
{
    u64 irq_state = assoc_irq_save();
    enum wifi_assoc_step_result result = assoc_step_locked(
        contract, caller_core, handle, observation);

    assoc_irq_restore(irq_state);
    return result;
}

static bool assoc_cancel_locked(struct wifi_assoc_contract *contract,
                                u32 caller_core,
                                const struct wifi_assoc_handle *handle)
{
    if (!assoc_owner(contract, caller_core) || !assoc_handle_valid(contract, handle) ||
        !assoc_active(contract))
        return false;
    assoc_fail(contract, WIFI_ASSOC_FAULT_CANCELLED, false);
    return true;
}

bool wifi_assoc_cancel(struct wifi_assoc_contract *contract, u32 caller_core,
                       const struct wifi_assoc_handle *handle)
{
    u64 irq_state = assoc_irq_save();
    bool result = assoc_cancel_locked(contract, caller_core, handle);

    assoc_irq_restore(irq_state);
    return result;
}

static bool assoc_reset_locked(struct wifi_assoc_contract *contract,
                               u32 caller_core)
{
    if (!assoc_owner(contract, caller_core) ||
        (contract->control.state != WIFI_ASSOC_AUTHORIZED &&
         contract->control.state != WIFI_ASSOC_FAILED &&
         contract->control.state != WIFI_ASSOC_QUARANTINED) ||
        contract->control.instance_epoch == ~0ULL)
        return false;
    contract->control.instance_epoch++;
    contract->control.attempt_generation = 0U;
    contract->control.deadline_ms = 0U;
    contract->control.state = WIFI_ASSOC_IDLE;
    return true;
}

bool wifi_assoc_reset(struct wifi_assoc_contract *contract, u32 caller_core)
{
    u64 irq_state = assoc_irq_save();
    bool result = assoc_reset_locked(contract, caller_core);

    assoc_irq_restore(irq_state);
    return result;
}

static bool assoc_status_copy_locked(const struct wifi_assoc_contract *contract,
                                     u32 caller_core,
                                     struct wifi_assoc_status *out)
{
    if (!assoc_owner(contract, caller_core) || !out)
        return false;
    out->instance_epoch = contract->control.instance_epoch;
    out->attempt_generation = contract->control.attempt_generation;
    out->deadline_ms = contract->control.deadline_ms;
    out->last_liveness_sequence = contract->control.last_liveness_sequence;
    out->last_event_timestamp = contract->control.last_event_timestamp;
    out->last_eapol_replay = contract->control.last_eapol_replay;
    out->state = contract->control.state;
    out->step_count = contract->control.step_count;
    out->watchdog_evidence = contract->control.watchdog_evidence;
    out->history_count = contract->control.history_count;
    out->fault = contract->control.fault;
    dmb_ishld();
    return true;
}

bool wifi_assoc_status_copy(const struct wifi_assoc_contract *contract,
                            u32 caller_core, struct wifi_assoc_status *out)
{
    u64 irq_state = assoc_irq_save();
    bool result = assoc_status_copy_locked(contract, caller_core, out);

    assoc_irq_restore(irq_state);
    return result;
}

static bool assoc_control_publication_copy_locked(
    const struct wifi_assoc_contract *contract, u32 caller_core,
    const struct wifi_assoc_handle *handle,
    struct wifi_assoc_control_publication *out)
{
    if (!assoc_owner(contract, caller_core) || !assoc_handle_valid(contract, handle) ||
        !out || contract->control.state != WIFI_ASSOC_SET_SSID_PUBLISHED)
        return false;
    assoc_zero(out, sizeof(*out));
    out->instance_epoch = contract->control.instance_epoch;
    out->attempt_generation = contract->control.attempt_generation;
    out->publication_sequence = contract->control.watchdog_evidence;
    out->state = WIFI_ASSOC_SET_SSID_PUBLISHED;
    dmb_ishld();
    return true;
}

bool wifi_assoc_control_publication_copy(
    const struct wifi_assoc_contract *contract, u32 caller_core,
    const struct wifi_assoc_handle *handle,
    struct wifi_assoc_control_publication *out)
{
    u64 irq_state = assoc_irq_save();
    bool result = assoc_control_publication_copy_locked(contract, caller_core,
                                                         handle, out);

    assoc_irq_restore(irq_state);
    return result;
}

static bool assoc_history_copy_locked(const struct wifi_assoc_contract *contract,
                                      u32 caller_core, u32 oldest_index,
                                      struct wifi_assoc_event_record *out)
{
    u32 first;
    u32 slot;

    if (!assoc_owner(contract, caller_core) || !out ||
        oldest_index >= contract->control.history_count)
        return false;
    first = (contract->control.history_head + WIFI_ASSOC_EVENT_HISTORY -
             contract->control.history_count) % WIFI_ASSOC_EVENT_HISTORY;
    slot = (first + oldest_index) % WIFI_ASSOC_EVENT_HISTORY;
    *out = contract->history[slot];
    dmb_ishld();
    return true;
}

bool wifi_assoc_history_copy(const struct wifi_assoc_contract *contract,
                             u32 caller_core, u32 oldest_index,
                             struct wifi_assoc_event_record *out)
{
    u64 irq_state = assoc_irq_save();
    bool result = assoc_history_copy_locked(contract, caller_core, oldest_index,
                                            out);

    assoc_irq_restore(irq_state);
    return result;
}

bool wifi_assoc_eapol_decode(const u8 *packet, u32 packet_len,
                             struct wifi_assoc_eapol_summary *out)
{
    u16 declared;
    u16 key_info;
    u16 key_data_len;
    u64 replay;
    u32 i;

    if (!out)
        return false;
    assoc_zero(out, sizeof(*out));
    if (!packet || packet_len < 99U || packet[0] < 1U || packet[0] > 3U ||
        packet[1] != 3U)
        return false;
    declared = ((u16)packet[2] << 8U) | packet[3];
    if ((u32)declared + 4U != packet_len || declared < 95U ||
        packet[4] != 2U)
        return false;
    key_info = ((u16)packet[5] << 8U) | packet[6];
    key_data_len = ((u16)packet[97] << 8U) | packet[98];
    if ((u32)key_data_len + 95U != declared || (key_info & 0x0007U) == 0U)
        return false;
    replay = 0U;
    for (i = 0U; i < 8U; i++)
        replay = (replay << 8U) | packet[9U + i];
    if (replay == 0U)
        return false;
    out->replay_counter = replay;
    out->key_info = key_info;
    out->classification = WIFI_ASSOC_EAPOL_OTHER;
    if ((key_info & 0x0188U) == 0x0088U)
        out->classification = WIFI_ASSOC_EAPOL_M1;
    else if ((key_info & 0x03C8U) == 0x03C8U)
        out->classification = WIFI_ASSOC_EAPOL_M3;
    else if ((key_info & 0x0388U) == 0x0380U)
        out->classification = WIFI_ASSOC_EAPOL_G1;
    out->valid = true;
    return true;
}
