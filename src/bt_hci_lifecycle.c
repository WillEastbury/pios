/*
 * bt_hci_lifecycle.c - bounded offline HCI command/event ownership.
 *
 * The caller supplies explicit byte spans. This unit has no transport,
 * controller, board, allocation, timer, or interrupt-controller dependency.
 */
#include "types.h"
#include "bt_hci_lifecycle.h"

#define BT_HCI_HANDLE_SLOT_MASK (BT_HCI_COMMAND_SLOT_COUNT - 1U)
#define BT_HCI_HANDLE_TOKEN_MASK \
    ((BT_HCI_HANDLE_MAGIC << BT_HCI_HANDLE_SHIFT) | BT_HCI_HANDLE_SLOT_MASK)
#define BT_HCI_NO_SLOT 0xFFFFFFFFU

static inline u64 hci_irq_save(void)
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

static inline void hci_irq_restore(u64 daif)
{
#ifdef PIOS_HOST_TYPES_SHIM
    (void)daif;
#else
    __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
#endif
}

_Static_assert((BT_HCI_COMMAND_SLOT_COUNT &
                (BT_HCI_COMMAND_SLOT_COUNT - 1U)) == 0U,
               "HCI slot count must be a power of two");

static void hci_handle_clear(struct bt_hci_handle *handle)
{
    if (handle)
        *handle = (struct bt_hci_handle){0};
}

static u64 hci_token(u32 slot)
{
    return (BT_HCI_HANDLE_MAGIC << BT_HCI_HANDLE_SHIFT) | (u64)slot;
}

static bool hci_fresh(const struct bt_hci_lifecycle *lifecycle)
{
    const u8 *bytes;
    usize i;

    if (!lifecycle)
        return false;
    bytes = (const u8 *)lifecycle;
    for (i = 0U; i < sizeof(*lifecycle); i++) {
        if (bytes[i] != 0U)
            return false;
    }
    return true;
}

static bool hci_owner(const struct bt_hci_lifecycle *lifecycle, u32 caller_core)
{
    return lifecycle && caller_core == BT_HCI_OWNER_CORE &&
           lifecycle->owner.owner_core == BT_HCI_OWNER_CORE;
}

static bool hci_active(const struct bt_hci_lifecycle *lifecycle, u32 caller_core)
{
    return hci_owner(lifecycle, caller_core) &&
           lifecycle->owner.controller_state == BT_HCI_CONTROLLER_READY;
}

static void hci_copy_bytes(u8 *dst, const u8 *src, u32 count)
{
    u32 i;

    for (i = 0U; i < count; i++)
        dst[i] = src[i];
}

static bool hci_handle_slot(const struct bt_hci_lifecycle *lifecycle,
                            const struct bt_hci_handle *handle, u32 caller_core,
                            u32 *slot_out)
{
    const struct bt_hci_slot_control *slot;
    u32 index;

    if (!hci_owner(lifecycle, caller_core) || !handle || !slot_out ||
        handle->request_id == 0U || handle->generation == 0U ||
        handle->instance_epoch != lifecycle->owner.instance_epoch ||
        (handle->token & ~BT_HCI_HANDLE_TOKEN_MASK) != 0U ||
        (handle->token >> BT_HCI_HANDLE_SHIFT) != BT_HCI_HANDLE_MAGIC)
        return false;
    index = (u32)(handle->token & BT_HCI_HANDLE_SLOT_MASK);
    if (index >= BT_HCI_COMMAND_SLOT_COUNT)
        return false;
    slot = &lifecycle->slots[index];
    if (handle->token != hci_token(index) ||
        slot->generation != handle->generation ||
        slot->request_id != handle->request_id ||
        slot->state == BT_HCI_SLOT_FREE || slot->state == BT_HCI_SLOT_RETIRED)
        return false;
    *slot_out = index;
    return true;
}

static void hci_publish_slot(struct bt_hci_slot_control *slot, u32 state)
{
    dmb_ishst();
    slot->state = state;
    dmb_ishst();
}

static void hci_make_result(struct bt_hci_lifecycle *lifecycle, u32 index,
                            enum bt_hci_result_kind kind, u8 hci_status)
{
    const struct bt_hci_slot_control *slot = &lifecycle->slots[index];
    struct bt_hci_result *result = &lifecycle->results[index];

    result->request_id = slot->request_id;
    result->generation = slot->generation;
    result->instance_epoch = lifecycle->owner.instance_epoch;
    result->attempt_id = slot->attempt_id;
    result->opcode = slot->opcode;
    result->hci_status = hci_status;
    result->kind = (u8)kind;
    result->credits = (u8)lifecycle->owner.credits;
    dmb_ishst();
    hci_publish_slot(&lifecycle->slots[index], BT_HCI_SLOT_RESULT);
}

static void hci_quarantine_internal(struct bt_hci_lifecycle *lifecycle,
                                    enum bt_hci_quarantine_reason reason,
                                    u32 exempt_slot,
                                    enum bt_hci_result_kind exempt_kind)
{
    u32 i;

    if (lifecycle->owner.controller_state != BT_HCI_CONTROLLER_QUARANTINED) {
        lifecycle->owner.controller_state = BT_HCI_CONTROLLER_QUARANTINED;
        lifecycle->owner.quarantine_reason = (u32)reason;
    }
    lifecycle->owner.credits = 0U;
    lifecycle->owner.in_flight_slot = BT_HCI_NO_SLOT;
    for (i = 0U; i < BT_HCI_COMMAND_SLOT_COUNT; i++) {
        u32 state = lifecycle->slots[i].state;

        if (state != BT_HCI_SLOT_QUEUED && state != BT_HCI_SLOT_IN_FLIGHT)
            continue;
        hci_make_result(lifecycle, i,
                        i == exempt_slot ? exempt_kind :
                        BT_HCI_RESULT_QUARANTINED,
                        BT_HCI_STATUS_INTERNAL);
    }
    dmb_ishst();
}

static void hci_quarantine_event(struct bt_hci_lifecycle *lifecycle,
                                 enum bt_hci_quarantine_reason reason)
{
    lifecycle->owner.malformed_events +=
        reason == BT_HCI_QUARANTINE_MALFORMED_EVENT ? 1U : 0U;
    hci_quarantine_internal(lifecycle, reason, BT_HCI_NO_SLOT,
                            BT_HCI_RESULT_QUARANTINED);
}

static bool hci_event_packet_valid(const u8 *packet, u32 packet_len,
                                   u8 *event_code, const u8 **params,
                                   u32 *params_len)
{
    u32 declared;

    if (!packet || !event_code || !params || !params_len || packet_len < 3U)
        return false;
    if (packet[0] != BT_HCI_H4_TYPE_EVENT)
        return false;
    declared = packet[2];
    if (declared != packet_len - 3U)
        return false;
    *event_code = packet[1];
    *params = packet + 3U;
    *params_len = declared;
    return true;
}

static bool hci_terminal_from_event(struct bt_hci_lifecycle *lifecycle,
                                    u16 opcode, u8 status, u8 credits,
                                    enum bt_hci_result_kind kind)
{
    u32 index = lifecycle->owner.in_flight_slot;
    struct bt_hci_slot_control *slot;

    if (index >= BT_HCI_COMMAND_SLOT_COUNT)
        return false;
    slot = &lifecycle->slots[index];
    if (slot->state != BT_HCI_SLOT_IN_FLIGHT || slot->attempt_id == 0U ||
        slot->opcode != opcode)
        return false;
    lifecycle->owner.credits =
        credits > BT_HCI_COMMAND_CREDITS_MAX ? BT_HCI_COMMAND_CREDITS_MAX :
        credits;
    if (credits > BT_HCI_COMMAND_CREDITS_MAX)
        lifecycle->owner.credit_clamps++;
    lifecycle->owner.in_flight_slot = BT_HCI_NO_SLOT;
    lifecycle->owner.last_terminal_attempt = slot->attempt_id;
    hci_make_result(lifecycle, index, kind, status);
    return true;
}

enum bt_hci_capability bt_hci_selected_first_capability(void)
{
    return BT_HCI_CAPABILITY_LE_PASSIVE_SCAN;
}

bool bt_hci_hardware_enable_allowed(void)
{
    return false;
}

bool bt_hci_capability_authorized(enum bt_hci_capability capability)
{
    (void)capability;
    return false;
}

bool bt_hci_implicit_pairing_allowed(void)
{
    return false;
}

static bool hci_lifecycle_init_locked(struct bt_hci_lifecycle *lifecycle, u32 caller_core)
{
    u32 i;

    if (caller_core != BT_HCI_OWNER_CORE || !hci_fresh(lifecycle))
        return false;
    lifecycle->owner.instance_epoch = 1U;
    lifecycle->owner.next_request_id = 1U;
    lifecycle->owner.owner_core = BT_HCI_OWNER_CORE;
    lifecycle->owner.controller_state = BT_HCI_CONTROLLER_READY;
    lifecycle->owner.credits = 1U;
    lifecycle->owner.in_flight_slot = BT_HCI_NO_SLOT;
    for (i = 0U; i < BT_HCI_COMMAND_SLOT_COUNT; i++) {
        lifecycle->slots[i].generation = (u64)i + 1U;
        lifecycle->slots[i].state = BT_HCI_SLOT_FREE;
    }
    dmb_ishst();
    return true;
}

static bool hci_submit_locked(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                   u16 opcode, const u8 *payload, u16 payload_len,
                   u64 now_ms, u64 timeout_ms, struct bt_hci_handle *out)
{
    struct bt_hci_slot_control *slot;
    u32 i;

    hci_handle_clear(out);
    if (!out || !hci_active(lifecycle, caller_core) ||
        (!payload && payload_len != 0U) ||
        payload_len > BT_HCI_COMMAND_PAYLOAD_MAX || timeout_ms == 0U ||
        timeout_ms > BT_HCI_TIMEOUT_MAX_MS || now_ms > ~0ULL - timeout_ms ||
        lifecycle->owner.next_request_id == 0U)
        return false;
    for (i = 0U; i < BT_HCI_COMMAND_SLOT_COUNT; i++) {
        if (lifecycle->slots[i].state == BT_HCI_SLOT_FREE)
            break;
    }
    if (i == BT_HCI_COMMAND_SLOT_COUNT)
        return false;
    slot = &lifecycle->slots[i];
    slot->request_id = lifecycle->owner.next_request_id++;
    slot->attempt_id = 0U;
    slot->deadline_ms = now_ms + timeout_ms;
    slot->opcode = opcode;
    slot->payload_len = payload_len;
    if (payload_len != 0U)
        hci_copy_bytes(lifecycle->data[i].payload, payload, payload_len);
    dmb_ishst();
    hci_publish_slot(slot, BT_HCI_SLOT_QUEUED);
    out->token = hci_token(i);
    out->request_id = slot->request_id;
    out->generation = slot->generation;
    out->instance_epoch = lifecycle->owner.instance_epoch;
    return true;
}

static bool hci_poll_timeouts_locked(struct bt_hci_lifecycle *lifecycle,
                                         u32 caller_core, u64 now_ms);

static bool hci_tx_next_locked(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                    u64 now_ms, struct bt_hci_tx_command *out)
{
    struct bt_hci_slot_control *slot;
    u32 i;

    if (!out || !hci_active(lifecycle, caller_core) ||
        lifecycle->owner.credits == 0U ||
        lifecycle->owner.in_flight_slot != BT_HCI_NO_SLOT)
        return false;
    if (hci_poll_timeouts_locked(lifecycle, caller_core, now_ms))
        return false;
    for (i = 0U; i < BT_HCI_COMMAND_SLOT_COUNT; i++) {
        if (lifecycle->slots[i].state == BT_HCI_SLOT_QUEUED)
            break;
    }
    if (i == BT_HCI_COMMAND_SLOT_COUNT)
        return false;
    slot = &lifecycle->slots[i];
    if (now_ms >= slot->deadline_ms) {
        hci_quarantine_internal(lifecycle, BT_HCI_QUARANTINE_TIMEOUT, i,
                                BT_HCI_RESULT_TIMEOUT);
        return false;
    }
    slot->attempt_id = ++lifecycle->owner.attempt_id;
    if (slot->attempt_id == 0U) {
        hci_quarantine_internal(lifecycle, BT_HCI_QUARANTINE_AER_LIKE_FAULT,
                                i, BT_HCI_RESULT_QUARANTINED);
        return false;
    }
    out->request_id = slot->request_id;
    out->generation = slot->generation;
    out->instance_epoch = lifecycle->owner.instance_epoch;
    out->attempt_id = slot->attempt_id;
    out->deadline_ms = slot->deadline_ms;
    out->opcode = slot->opcode;
    out->byte_count = (u16)(4U + slot->payload_len);
    out->bytes[0] = BT_HCI_H4_TYPE_COMMAND;
    out->bytes[1] = (u8)slot->opcode;
    out->bytes[2] = (u8)(slot->opcode >> 8);
    out->bytes[3] = (u8)slot->payload_len;
    if (slot->payload_len != 0U)
        hci_copy_bytes(out->bytes + 4U, lifecycle->data[i].payload,
                       slot->payload_len);
    dmb_ishst();
    lifecycle->owner.credits--;
    lifecycle->owner.in_flight_slot = i;
    hci_publish_slot(slot, BT_HCI_SLOT_IN_FLIGHT);
    return true;
}

static enum bt_hci_event_result hci_event_from_irq_locked(
    struct bt_hci_lifecycle *lifecycle, u32 caller_core,
    const u8 *packet, u32 packet_len)
{
    const u8 *params;
    u8 code;
    u32 params_len;
    u16 opcode;

    if (!lifecycle || caller_core != BT_HCI_OWNER_CORE ||
        lifecycle->owner.owner_core != BT_HCI_OWNER_CORE)
        return BT_HCI_EVENT_WRONG_CORE;
    if (lifecycle->owner.controller_state != BT_HCI_CONTROLLER_READY)
        return BT_HCI_EVENT_QUARANTINED;
    if (!hci_event_packet_valid(packet, packet_len, &code, &params, &params_len)) {
        hci_quarantine_event(lifecycle, BT_HCI_QUARANTINE_MALFORMED_EVENT);
        return packet ? BT_HCI_EVENT_MALFORMED : BT_HCI_EVENT_ARGUMENT;
    }
    if (code != BT_HCI_EVENT_COMMAND_COMPLETE &&
        code != BT_HCI_EVENT_COMMAND_STATUS) {
        lifecycle->owner.ignored_events++;
        return BT_HCI_EVENT_IGNORED;
    }
    if ((code == BT_HCI_EVENT_COMMAND_COMPLETE && params_len < 4U) ||
        (code == BT_HCI_EVENT_COMMAND_STATUS && params_len != 4U)) {
        hci_quarantine_event(lifecycle, BT_HCI_QUARANTINE_MALFORMED_EVENT);
        return BT_HCI_EVENT_MALFORMED;
    }
    if (code == BT_HCI_EVENT_COMMAND_COMPLETE) {
        opcode = (u16)params[1] | ((u16)params[2] << 8);
        if (!hci_terminal_from_event(lifecycle, opcode, params[3], params[0],
                                     BT_HCI_RESULT_COMMAND_COMPLETE)) {
            bool in_flight =
                lifecycle->owner.in_flight_slot < BT_HCI_COMMAND_SLOT_COUNT;
            hci_quarantine_event(lifecycle,
                                 in_flight ? BT_HCI_QUARANTINE_IDENTITY_MISMATCH :
                                 BT_HCI_QUARANTINE_UNSOLICITED_EVENT);
            if (in_flight)
                return BT_HCI_EVENT_MISMATCH;
            return lifecycle->owner.last_terminal_attempt != 0U ?
                   BT_HCI_EVENT_DUPLICATE : BT_HCI_EVENT_UNSOLICITED;
        }
    } else {
        opcode = (u16)params[2] | ((u16)params[3] << 8);
        if (!hci_terminal_from_event(lifecycle, opcode, params[0], params[1],
                                     BT_HCI_RESULT_COMMAND_STATUS)) {
            bool in_flight =
                lifecycle->owner.in_flight_slot < BT_HCI_COMMAND_SLOT_COUNT;
            hci_quarantine_event(lifecycle,
                                 in_flight ? BT_HCI_QUARANTINE_IDENTITY_MISMATCH :
                                 BT_HCI_QUARANTINE_UNSOLICITED_EVENT);
            if (in_flight)
                return BT_HCI_EVENT_MISMATCH;
            return lifecycle->owner.last_terminal_attempt != 0U ?
                   BT_HCI_EVENT_DUPLICATE : BT_HCI_EVENT_UNSOLICITED;
        }
    }
    return BT_HCI_EVENT_ACCEPTED;
}

static bool hci_result_copy_locked(const struct bt_hci_lifecycle *lifecycle,
                        u32 caller_core, const struct bt_hci_handle *handle,
                        struct bt_hci_result *out)
{
    u32 index;

    if (!out || !hci_handle_slot(lifecycle, handle, caller_core, &index) ||
        lifecycle->slots[index].state != BT_HCI_SLOT_RESULT)
        return false;
    dmb_ishld();
    *out = lifecycle->results[index];
    return true;
}

static bool hci_result_release_locked(struct bt_hci_lifecycle *lifecycle,
                           u32 caller_core,
                           const struct bt_hci_handle *handle)
{
    u32 index;
    struct bt_hci_slot_control *slot;

    if (!hci_handle_slot(lifecycle, handle, caller_core, &index) ||
        lifecycle->slots[index].state != BT_HCI_SLOT_RESULT)
        return false;
    slot = &lifecycle->slots[index];
    slot->request_id = 0U;
    slot->attempt_id = 0U;
    slot->deadline_ms = 0U;
    slot->opcode = 0U;
    slot->payload_len = 0U;
    lifecycle->data[index] = (struct bt_hci_slot_data){0};
    lifecycle->results[index] = (struct bt_hci_result){0};
    if (slot->generation == ~0ULL)
        hci_publish_slot(slot, BT_HCI_SLOT_RETIRED);
    else {
        slot->generation++;
        hci_publish_slot(slot, BT_HCI_SLOT_FREE);
    }
    return true;
}

static bool hci_cancel_locked(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                   const struct bt_hci_handle *handle)
{
    u32 index;
    u32 state;

    if (!hci_active(lifecycle, caller_core) ||
        !hci_handle_slot(lifecycle, handle, caller_core, &index))
        return false;
    state = lifecycle->slots[index].state;
    if (state == BT_HCI_SLOT_QUEUED) {
        hci_make_result(lifecycle, index, BT_HCI_RESULT_CANCELLED,
                        BT_HCI_STATUS_INTERNAL);
        return true;
    }
    if (state == BT_HCI_SLOT_IN_FLIGHT) {
        hci_quarantine_internal(lifecycle,
                                BT_HCI_QUARANTINE_CANCELLED_IN_FLIGHT, index,
                                BT_HCI_RESULT_CANCELLED);
        return true;
    }
    return false;
}

static bool hci_poll_timeouts_locked(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                          u64 now_ms)
{
    u32 i;

    if (!hci_active(lifecycle, caller_core))
        return false;
    for (i = 0U; i < BT_HCI_COMMAND_SLOT_COUNT; i++) {
        u32 state = lifecycle->slots[i].state;

        if ((state == BT_HCI_SLOT_QUEUED || state == BT_HCI_SLOT_IN_FLIGHT) &&
            now_ms >= lifecycle->slots[i].deadline_ms) {
            hci_quarantine_internal(lifecycle, BT_HCI_QUARANTINE_TIMEOUT, i,
                                    BT_HCI_RESULT_TIMEOUT);
            return true;
        }
    }
    return false;
}

static bool hci_quarantine_locked(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                       enum bt_hci_quarantine_reason reason)
{
    if (!hci_owner(lifecycle, caller_core) ||
        reason == BT_HCI_QUARANTINE_NONE)
        return false;
    hci_quarantine_internal(lifecycle, reason, BT_HCI_NO_SLOT,
                            BT_HCI_RESULT_QUARANTINED);
    return true;
}

static bool hci_controller_reset_locked(struct bt_hci_lifecycle *lifecycle,
                             u32 caller_core)
{
    u32 i;

    if (!hci_owner(lifecycle, caller_core))
        return false;
    hci_quarantine_internal(lifecycle, BT_HCI_QUARANTINE_CONTROLLER_RESET,
                            BT_HCI_NO_SLOT, BT_HCI_RESULT_QUARANTINED);
    lifecycle->owner.instance_epoch++;
    if (lifecycle->owner.instance_epoch == 0U)
        lifecycle->owner.instance_epoch = 1U;
    for (i = 0U; i < BT_HCI_COMMAND_SLOT_COUNT; i++) {
        struct bt_hci_slot_control *slot = &lifecycle->slots[i];

        if (slot->generation != ~0ULL)
            slot->generation++;
        slot->request_id = 0U;
        slot->attempt_id = 0U;
        slot->deadline_ms = 0U;
        slot->opcode = 0U;
        slot->payload_len = 0U;
        lifecycle->data[i] = (struct bt_hci_slot_data){0};
        lifecycle->results[i] = (struct bt_hci_result){0};
        if (slot->generation != ~0ULL)
            hci_publish_slot(slot, BT_HCI_SLOT_FREE);
        else
            hci_publish_slot(slot, BT_HCI_SLOT_RETIRED);
    }
    lifecycle->owner.credits = 0U;
    lifecycle->owner.in_flight_slot = BT_HCI_NO_SLOT;
    dmb_ishst();
    return true;
}

static bool hci_status_copy_locked(const struct bt_hci_lifecycle *lifecycle,
                        u32 caller_core, struct bt_hci_status *out)
{
    u32 i;

    if (!out || !hci_owner(lifecycle, caller_core))
        return false;
    *out = (struct bt_hci_status){0};
    dmb_ishld();
    out->instance_epoch = lifecycle->owner.instance_epoch;
    out->next_request_id = lifecycle->owner.next_request_id;
    out->attempt_id = lifecycle->owner.attempt_id;
    out->controller_state = lifecycle->owner.controller_state;
    out->credits = lifecycle->owner.credits;
    out->quarantine_reason = lifecycle->owner.quarantine_reason;
    out->malformed_events = lifecycle->owner.malformed_events;
    out->ignored_events = lifecycle->owner.ignored_events;
    out->credit_clamps = lifecycle->owner.credit_clamps;
    for (i = 0U; i < BT_HCI_COMMAND_SLOT_COUNT; i++) {
        if (lifecycle->slots[i].state == BT_HCI_SLOT_QUEUED)
            out->queued++;
        else if (lifecycle->slots[i].state == BT_HCI_SLOT_IN_FLIGHT)
            out->in_flight++;
        else if (lifecycle->slots[i].state == BT_HCI_SLOT_RESULT)
            out->results++;
    }
    return true;
}


bool bt_hci_lifecycle_init(struct bt_hci_lifecycle *lifecycle, u32 caller_core)
{
    u64 irq_state = hci_irq_save();
    bool result = hci_lifecycle_init_locked(lifecycle, caller_core);

    hci_irq_restore(irq_state);
    return result;
}

bool bt_hci_submit(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                   u16 opcode, const u8 *payload, u16 payload_len,
                   u64 now_ms, u64 timeout_ms, struct bt_hci_handle *out)
{
    u64 irq_state = hci_irq_save();
    bool result = hci_submit_locked(lifecycle, caller_core, opcode, payload,
                                    payload_len, now_ms, timeout_ms, out);

    hci_irq_restore(irq_state);
    return result;
}

bool bt_hci_tx_next(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                    u64 now_ms, struct bt_hci_tx_command *out)
{
    u64 irq_state = hci_irq_save();
    bool result = hci_tx_next_locked(lifecycle, caller_core, now_ms, out);

    hci_irq_restore(irq_state);
    return result;
}

enum bt_hci_event_result bt_hci_event_from_irq(
    struct bt_hci_lifecycle *lifecycle, u32 caller_core,
    const u8 *packet, u32 packet_len)
{
    u64 irq_state = hci_irq_save();
    enum bt_hci_event_result result = hci_event_from_irq_locked(
        lifecycle, caller_core, packet, packet_len);

    hci_irq_restore(irq_state);
    return result;
}

bool bt_hci_result_copy(const struct bt_hci_lifecycle *lifecycle,
                        u32 caller_core, const struct bt_hci_handle *handle,
                        struct bt_hci_result *out)
{
    u64 irq_state = hci_irq_save();
    bool result = hci_result_copy_locked(lifecycle, caller_core, handle, out);

    hci_irq_restore(irq_state);
    return result;
}

bool bt_hci_result_release(struct bt_hci_lifecycle *lifecycle,
                           u32 caller_core,
                           const struct bt_hci_handle *handle)
{
    u64 irq_state = hci_irq_save();
    bool result = hci_result_release_locked(lifecycle, caller_core, handle);

    hci_irq_restore(irq_state);
    return result;
}

bool bt_hci_cancel(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                   const struct bt_hci_handle *handle)
{
    u64 irq_state = hci_irq_save();
    bool result = hci_cancel_locked(lifecycle, caller_core, handle);

    hci_irq_restore(irq_state);
    return result;
}

bool bt_hci_poll_timeouts(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                          u64 now_ms)
{
    u64 irq_state = hci_irq_save();
    bool result = hci_poll_timeouts_locked(lifecycle, caller_core, now_ms);

    hci_irq_restore(irq_state);
    return result;
}

bool bt_hci_quarantine(struct bt_hci_lifecycle *lifecycle, u32 caller_core,
                       enum bt_hci_quarantine_reason reason)
{
    u64 irq_state = hci_irq_save();
    bool result = hci_quarantine_locked(lifecycle, caller_core, reason);

    hci_irq_restore(irq_state);
    return result;
}

bool bt_hci_controller_reset(struct bt_hci_lifecycle *lifecycle,
                             u32 caller_core)
{
    u64 irq_state = hci_irq_save();
    bool result = hci_controller_reset_locked(lifecycle, caller_core);

    hci_irq_restore(irq_state);
    return result;
}

bool bt_hci_status_copy(const struct bt_hci_lifecycle *lifecycle,
                        u32 caller_core, struct bt_hci_status *out)
{
    u64 irq_state = hci_irq_save();
    bool result = hci_status_copy_locked(lifecycle, caller_core, out);

    hci_irq_restore(irq_state);
    return result;
}
