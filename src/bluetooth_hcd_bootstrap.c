/*
 * Offline HCD artifact/baud evidence state machine.  No controller I/O,
 * physical line operation, artifact storage, or interrupt integration lives
 * here.
 */
#include "types.h"
#include "bluetooth_hcd_bootstrap.h"

#define BLUETOOTH_HCD_BOOTSTRAP_TOKEN_MAGIC 0x42484344ULL

static void bootstrap_zero(void *dst, usize bytes)
{
    u8 *p = (u8 *)dst;

    while (bytes != 0U) {
        *p++ = 0U;
        bytes--;
    }
}

static void bootstrap_copy(void *dst, const void *src, usize bytes)
{
    u8 *d = (u8 *)dst;
    const u8 *s = (const u8 *)src;

    while (bytes != 0U) {
        *d++ = *s++;
        bytes--;
    }
}

static bool bootstrap_bytes_zero(const void *src, usize bytes)
{
    const u8 *p = (const u8 *)src;

    while (bytes != 0U) {
        if (*p++ != 0U)
            return false;
        bytes--;
    }
    return true;
}

static bool bootstrap_digest_equal(
    const u8 a[BLUETOOTH_HCD_BOOTSTRAP_SHA256_BYTES],
    const u8 b[BLUETOOTH_HCD_BOOTSTRAP_SHA256_BYTES])
{
    u32 i;
    u8 different = 0U;

    for (i = 0U; i < BLUETOOTH_HCD_BOOTSTRAP_SHA256_BYTES; i++)
        different |= a[i] ^ b[i];
    return different == 0U;
}

static bool bootstrap_digest_present(
    const u8 digest[BLUETOOTH_HCD_BOOTSTRAP_SHA256_BYTES])
{
    return !bootstrap_bytes_zero(digest, BLUETOOTH_HCD_BOOTSTRAP_SHA256_BYTES);
}

static void bootstrap_clear_handle(struct bluetooth_hcd_bootstrap_handle *handle)
{
    if (handle)
        bootstrap_zero(handle, sizeof(*handle));
}

static u64 bootstrap_token(const struct bluetooth_hcd_bootstrap *bootstrap)
{
    return (BLUETOOTH_HCD_BOOTSTRAP_TOKEN_MAGIC << 32U) |
           ((u64)(usize)bootstrap >> 6U);
}

static bool bootstrap_handle_valid(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_handle *handle)
{
    if (!bootstrap || !handle || bootstrap->control.instance_epoch == 0U ||
        handle->instance_epoch == 0U ||
        handle->instance_epoch != bootstrap->control.instance_epoch ||
        handle->attempt_generation != bootstrap->control.attempt_generation ||
        handle->_token != bootstrap_token(bootstrap))
        return false;
    dmb_ishld();
    return true;
}

static bool bootstrap_deadline(u64 now_ms, u32 timeout_ms, u64 *deadline_out)
{
    if (!deadline_out || timeout_ms == 0U ||
        timeout_ms > BLUETOOTH_HCD_BOOTSTRAP_MAX_TIMEOUT_MS ||
        now_ms > ~0ULL - (u64)timeout_ms)
        return false;
    *deadline_out = now_ms + (u64)timeout_ms;
    return true;
}

static void bootstrap_publish(struct bluetooth_hcd_bootstrap *bootstrap,
                              u32 state, u64 deadline_ms)
{
    dmb_ishst();
    bootstrap->control.deadline_ms = deadline_ms;
    bootstrap->control.state = state;
    bootstrap->control.progress_count++;
    dmb_ishst();
}

static void bootstrap_quarantine(struct bluetooth_hcd_bootstrap *bootstrap,
                                  u32 fault)
{
    if (bootstrap->control.state == BLUETOOTH_HCD_BOOTSTRAP_QUARANTINED)
        return;
    bootstrap->control.fault = fault;
    bootstrap->control.fault_count++;
    dmb_ishst();
    bootstrap->control.state = BLUETOOTH_HCD_BOOTSTRAP_QUARANTINED;
    dmb_ishst();
}

static bool bootstrap_state_has_deadline(u32 state)
{
    return state == BLUETOOTH_HCD_BOOTSTRAP_WAIT_ARTIFACT ||
           state == BLUETOOTH_HCD_BOOTSTRAP_WAIT_SAFE_STATE ||
           state == BLUETOOTH_HCD_BOOTSTRAP_WAIT_HCD_ACK ||
           state == BLUETOOTH_HCD_BOOTSTRAP_REQUEST_BAUD ||
           state == BLUETOOTH_HCD_BOOTSTRAP_WAIT_BAUD_ACK;
}

static enum bluetooth_hcd_bootstrap_step_result bootstrap_expired(
    struct bluetooth_hcd_bootstrap *bootstrap, u64 now_ms)
{
    if (bootstrap_state_has_deadline(bootstrap->control.state) &&
        now_ms > bootstrap->control.deadline_ms) {
        bootstrap_quarantine(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_FAULT_DEADLINE);
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
    }
    return BLUETOOTH_HCD_BOOTSTRAP_STEP_NO_PROGRESS;
}

static bool bootstrap_profile_valid(u32 profile_id, u32 target_baud)
{
    const struct bluetooth_platform_profile *profile;

    profile = bluetooth_platform_profile_get(
        (enum bluetooth_platform_profile_id)profile_id);
    return profile && (u32)profile->id == profile_id &&
           profile->max_baud != 0U && target_baud <= profile->max_baud;
}

static bool bootstrap_selection_valid(
    const struct bluetooth_hcd_bootstrap_plan *plan)
{
    const struct bluetooth_hcd_bootstrap_selection *selection;

    if (!plan)
        return false;
    selection = &plan->selection;
    return selection->source_id != 0U && selection->source_generation != 0U &&
           selection->command_id != 0U && selection->controller_id != 0U &&
           selection->source_bytes != 0U &&
           selection->source_bytes == selection->command_bytes &&
           bootstrap_digest_present(selection->source_sha256) &&
           bootstrap_digest_present(selection->command_sha256) &&
           bootstrap_digest_equal(selection->source_sha256,
                                  selection->command_sha256) &&
           bootstrap_bytes_zero(selection->_reserved,
                                sizeof(selection->_reserved)) &&
           plan->initial_baud != 0U && plan->target_baud != 0U &&
           plan->initial_baud != plan->target_baud &&
           plan->activation_authority_attested == false &&
           bootstrap_bytes_zero(plan->_reserved, sizeof(plan->_reserved)) &&
           bootstrap_profile_valid(selection->profile_id, plan->initial_baud) &&
           bootstrap_profile_valid(selection->profile_id, plan->target_baud);
}

static bool bootstrap_artifact_valid(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_source_blob *source,
    const struct bluetooth_hcd_bootstrap_command_descriptor *command,
    enum bluetooth_hcd_bootstrap_fault *fault_out)
{
    const struct bluetooth_hcd_bootstrap_selection *selection =
        &bootstrap->selection;

    if (!source || !command || !source->data || source->_reserved != 0U ||
        command->_reserved != 0U ||
        !bootstrap_bytes_zero(command->_pad, sizeof(command->_pad))) {
        *fault_out = BLUETOOTH_HCD_BOOTSTRAP_FAULT_ARTIFACT;
        return false;
    }
    if (!bootstrap_digest_present(source->sha256) ||
        !bootstrap_digest_present(command->sha256) ||
        !bootstrap_digest_equal(source->sha256, selection->source_sha256) ||
        !bootstrap_digest_equal(command->sha256, selection->command_sha256)) {
        *fault_out = BLUETOOTH_HCD_BOOTSTRAP_FAULT_DIGEST;
        return false;
    }
    if (source->source_id != selection->source_id ||
        source->source_generation != selection->source_generation ||
        source->byte_count != selection->source_bytes ||
        command->command_id != selection->command_id ||
        command->byte_count != selection->command_bytes) {
        *fault_out = BLUETOOTH_HCD_BOOTSTRAP_FAULT_ARTIFACT;
        return false;
    }
    return true;
}

static bool bootstrap_safe_state_valid(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_safe_state_evidence *evidence)
{
    const struct bluetooth_hcd_bootstrap_selection *selection =
        &bootstrap->selection;

    return evidence &&
           evidence->instance_epoch == bootstrap->control.instance_epoch &&
           evidence->attempt_generation ==
               bootstrap->control.attempt_generation &&
           evidence->evidence_id != 0U &&
           evidence->evidence_generation != 0U &&
           evidence->profile_id == selection->profile_id &&
           evidence->controller_id == selection->controller_id &&
           evidence->controller_safe_off == true &&
           evidence->hardware_enable_allowed == false &&
           evidence->activation_authority_attested == false &&
           bootstrap_bytes_zero(evidence->_reserved, sizeof(evidence->_reserved));
}

static bool bootstrap_hcd_ack_valid(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_hcd_ack *ack)
{
    const struct bluetooth_hcd_bootstrap_selection *selection =
        &bootstrap->selection;

    return ack &&
           ack->instance_epoch == bootstrap->control.instance_epoch &&
           ack->attempt_generation == bootstrap->control.attempt_generation &&
           ack->profile_id == selection->profile_id &&
           ack->controller_id == selection->controller_id &&
           ack->source_id == selection->source_id &&
           ack->source_generation == selection->source_generation &&
           ack->command_id == selection->command_id &&
           ack->command_bytes == selection->command_bytes &&
           ack->accepted == true &&
           bootstrap_bytes_zero(ack->_reserved, sizeof(ack->_reserved)) &&
           bootstrap_bytes_zero(ack->_pad, sizeof(ack->_pad)) &&
           bootstrap_digest_equal(ack->command_sha256,
                                  selection->command_sha256);
}

static bool bootstrap_baud_request_valid(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_baud_request *request)
{
    return request &&
           request->instance_epoch == bootstrap->control.instance_epoch &&
           request->attempt_generation ==
               bootstrap->control.attempt_generation &&
           request->controller_id == bootstrap->selection.controller_id &&
           request->target_baud == bootstrap->control.target_baud;
}

static bool bootstrap_baud_ack_valid(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_baud_ack *ack)
{
    return ack &&
           ack->instance_epoch == bootstrap->control.instance_epoch &&
           ack->attempt_generation == bootstrap->control.attempt_generation &&
           ack->controller_id == bootstrap->selection.controller_id &&
           ack->baud == bootstrap->control.target_baud &&
           ack->accepted == true &&
           bootstrap_bytes_zero(ack->_reserved, sizeof(ack->_reserved));
}

bool bluetooth_hcd_bootstrap_init(
    struct bluetooth_hcd_bootstrap *bootstrap,
    u64 instance_epoch,
    struct bluetooth_hcd_bootstrap_handle *handle_out)
{
    bootstrap_clear_handle(handle_out);
    if (!bootstrap || !handle_out || instance_epoch == 0U)
        return false;
    bootstrap_zero(bootstrap, sizeof(*bootstrap));
    bootstrap->control.instance_epoch = instance_epoch;
    bootstrap->control.state = BLUETOOTH_HCD_BOOTSTRAP_SAFE_OFF;
    dmb_ishst();
    handle_out->instance_epoch = instance_epoch;
    handle_out->_token = bootstrap_token(bootstrap);
    return true;
}

enum bluetooth_hcd_bootstrap_step_result bluetooth_hcd_bootstrap_submit(
    struct bluetooth_hcd_bootstrap *bootstrap,
    struct bluetooth_hcd_bootstrap_handle *handle, u64 now_ms,
    const struct bluetooth_hcd_bootstrap_plan *plan)
{
    u64 deadline;

    if (!bootstrap_handle_valid(bootstrap, handle))
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_REJECTED;
    if (bootstrap->control.state == BLUETOOTH_HCD_BOOTSTRAP_QUARANTINED)
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
    if (bootstrap->control.state == BLUETOOTH_HCD_BOOTSTRAP_COMPLETE)
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_COMPLETE;
    if (bootstrap->control.state != BLUETOOTH_HCD_BOOTSTRAP_SAFE_OFF &&
        !bootstrap_state_has_deadline(bootstrap->control.state)) {
        bootstrap_quarantine(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_FAULT_SEQUENCE);
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
    }
    if (bootstrap->control.state != BLUETOOTH_HCD_BOOTSTRAP_SAFE_OFF) {
        bootstrap_quarantine(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_FAULT_SEQUENCE);
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
    }
    if (!bootstrap_selection_valid(plan) ||
        !bootstrap_deadline(now_ms, plan->state_timeout_ms, &deadline)) {
        bootstrap_quarantine(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_FAULT_BAD_PLAN);
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
    }
    if (bootstrap->control.attempt_generation == ~0ULL) {
        bootstrap_quarantine(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_FAULT_SEQUENCE);
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
    }
    bootstrap_copy(&bootstrap->selection, &plan->selection,
                   sizeof(bootstrap->selection));
    bootstrap->control.target_baud = plan->target_baud;
    bootstrap->control.state_timeout_ms = plan->state_timeout_ms;
    bootstrap->control.attempt_generation++;
    handle->attempt_generation = bootstrap->control.attempt_generation;
    bootstrap_publish(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_WAIT_ARTIFACT,
                      deadline);
    return BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS;
}

enum bluetooth_hcd_bootstrap_step_result bluetooth_hcd_bootstrap_step(
    struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_handle *handle, u64 now_ms,
    const struct bluetooth_hcd_bootstrap_observation *observation)
{
    u64 deadline;
    enum bluetooth_hcd_bootstrap_fault fault;

    if (!bootstrap_handle_valid(bootstrap, handle))
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_REJECTED;
    if (bootstrap->control.state == BLUETOOTH_HCD_BOOTSTRAP_QUARANTINED)
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
    if (bootstrap->control.state == BLUETOOTH_HCD_BOOTSTRAP_COMPLETE)
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_COMPLETE;
    if (bootstrap_expired(bootstrap, now_ms) ==
        BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED)
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
    if (!observation || observation->kind == BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_NONE)
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_NO_PROGRESS;
    if (observation->_reserved != 0U ||
        observation->instance_epoch != bootstrap->control.instance_epoch ||
        observation->attempt_generation !=
            bootstrap->control.attempt_generation ||
        !bootstrap_deadline(now_ms, bootstrap->control.state_timeout_ms,
                            &deadline)) {
        bootstrap_quarantine(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_FAULT_SEQUENCE);
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
    }

    if (bootstrap->control.state == BLUETOOTH_HCD_BOOTSTRAP_WAIT_ARTIFACT &&
        observation->kind == BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_ARTIFACT) {
        if (!bootstrap_artifact_valid(bootstrap, observation->value.artifact.source,
                                      observation->value.artifact.command, &fault)) {
            bootstrap_quarantine(bootstrap, fault);
            return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
        }
        bootstrap_publish(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_WAIT_SAFE_STATE,
                          deadline);
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS;
    }
    if (bootstrap->control.state == BLUETOOTH_HCD_BOOTSTRAP_WAIT_SAFE_STATE &&
        observation->kind == BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_SAFE_STATE) {
        if (!bootstrap_safe_state_valid(bootstrap,
                                        &observation->value.safe_state)) {
            bootstrap_quarantine(bootstrap,
                                  BLUETOOTH_HCD_BOOTSTRAP_FAULT_SAFE_STATE);
            return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
        }
        bootstrap_publish(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_WAIT_HCD_ACK,
                          deadline);
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS;
    }
    if (bootstrap->control.state == BLUETOOTH_HCD_BOOTSTRAP_WAIT_HCD_ACK &&
        observation->kind == BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_HCD_ACK) {
        if (!bootstrap_hcd_ack_valid(bootstrap, &observation->value.hcd_ack)) {
            bootstrap_quarantine(bootstrap,
                                  BLUETOOTH_HCD_BOOTSTRAP_FAULT_HCD_ACK);
            return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
        }
        bootstrap_publish(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_REQUEST_BAUD,
                          deadline);
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS;
    }
    if (bootstrap->control.state == BLUETOOTH_HCD_BOOTSTRAP_REQUEST_BAUD &&
        observation->kind == BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_BAUD_REQUEST) {
        if (!bootstrap_baud_request_valid(bootstrap,
                                          &observation->value.baud_request)) {
            bootstrap_quarantine(bootstrap,
                                  BLUETOOTH_HCD_BOOTSTRAP_FAULT_BAUD_REQUEST);
            return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
        }
        bootstrap_publish(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_WAIT_BAUD_ACK,
                          deadline);
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS;
    }
    if (bootstrap->control.state == BLUETOOTH_HCD_BOOTSTRAP_WAIT_BAUD_ACK &&
        observation->kind == BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_BAUD_ACK) {
        if (!bootstrap_baud_ack_valid(bootstrap, &observation->value.baud_ack)) {
            bootstrap_quarantine(bootstrap,
                                  BLUETOOTH_HCD_BOOTSTRAP_FAULT_BAUD_ACK);
            return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
        }
        bootstrap_publish(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_COMPLETE, deadline);
        return BLUETOOTH_HCD_BOOTSTRAP_STEP_COMPLETE;
    }
    bootstrap_quarantine(bootstrap, BLUETOOTH_HCD_BOOTSTRAP_FAULT_SEQUENCE);
    return BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED;
}

bool bluetooth_hcd_bootstrap_baud_request_get(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_handle *handle,
    struct bluetooth_hcd_bootstrap_baud_request *request_out)
{
    if (!request_out || !bootstrap_handle_valid(bootstrap, handle) ||
        bootstrap->control.state != BLUETOOTH_HCD_BOOTSTRAP_REQUEST_BAUD)
        return false;
    request_out->instance_epoch = bootstrap->control.instance_epoch;
    request_out->attempt_generation = bootstrap->control.attempt_generation;
    request_out->controller_id = bootstrap->selection.controller_id;
    request_out->target_baud = bootstrap->control.target_baud;
    return true;
}

bool bluetooth_hcd_bootstrap_state_get(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_handle *handle,
    enum bluetooth_hcd_bootstrap_state *state_out,
    enum bluetooth_hcd_bootstrap_fault *fault_out, u32 *progress_count_out)
{
    if (!state_out || !fault_out || !progress_count_out ||
        !bootstrap_handle_valid(bootstrap, handle))
        return false;
    dmb_ishld();
    *state_out = (enum bluetooth_hcd_bootstrap_state)bootstrap->control.state;
    *fault_out = (enum bluetooth_hcd_bootstrap_fault)bootstrap->control.fault;
    *progress_count_out = bootstrap->control.progress_count;
    return true;
}

bool bluetooth_hcd_bootstrap_progress_evidence_since(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_handle *handle, u32 since_count,
    u32 *current_count_out)
{
    u32 current;

    if (!current_count_out || !bootstrap_handle_valid(bootstrap, handle))
        return false;
    current = bootstrap->control.progress_count;
    dmb_ishld();
    *current_count_out = current;
    return current > since_count;
}

bool bluetooth_hcd_bootstrap_hardware_enable_allowed(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_handle *handle)
{
    (void)bootstrap;
    (void)handle;
    return false;
}
