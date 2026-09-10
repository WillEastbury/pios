/*
 * pisp_be_gate.c - pure core-0-only PiSP-BE brick-test state gate.
 */
#include "types.h"
#include "pisp_be_gate.h"

#define PISP_BE_GATE_TOKEN_MAGIC  0x50424754ULL

static void pisp_be_gate_clear_handle(struct pisp_be_gate_handle *handle)
{
    if (!handle)
        return;
    handle->_token = 0U;
    handle->_generation = 0U;
    handle->_controller_id = 0U;
    handle->_reserved = 0U;
}

static u64 pisp_be_gate_token(const struct pisp_be_gate *gate)
{
    return (PISP_BE_GATE_TOKEN_MAGIC << 32U) |
           (u64)((u32)gate->generation ^ gate->controller_id);
}

static bool pisp_be_gate_deadline(u64 now_ms, u32 timeout_ms,
                                  u64 *deadline_out)
{
    if (!deadline_out || timeout_ms == 0U ||
        timeout_ms > PISP_BE_GATE_MAX_TIMEOUT_MS ||
        now_ms > ~0ULL - (u64)timeout_ms)
        return false;
    *deadline_out = now_ms + (u64)timeout_ms;
    return true;
}

static bool pisp_be_gate_handle_valid(const struct pisp_be_gate *gate,
                                      const struct pisp_be_gate_handle *handle,
                                      u32 caller_core)
{
    if (!gate || !handle || gate->owner_core != PISP_BE_GATE_OWNER_CORE ||
        caller_core != gate->owner_core || gate->controller_id == 0U ||
        handle->_reserved != 0U || handle->_controller_id != gate->controller_id ||
        handle->_generation == 0U || handle->_generation != gate->generation ||
        handle->_token != pisp_be_gate_token(gate))
        return false;
    dmb_ishld();
    return true;
}

static bool pisp_be_gate_storage_fresh(const struct pisp_be_gate *gate)
{
    u32 i;

    if (!gate || gate->generation != 0U || gate->deadline_ms != 0U ||
        gate->controller_id != 0U || gate->owner_core != 0U ||
        gate->state != PISP_BE_GATE_DISABLED ||
        gate->fault != PISP_BE_GATE_FAULT_NONE ||
        gate->transition_count != 0U || gate->failure_count != 0U ||
        gate->last_status != 0U || gate->_reserved0 != 0U)
        return false;
    for (i = 0U; i < sizeof(gate->_reserved); i++) {
        if (gate->_reserved[i] != 0U)
            return false;
    }
    return true;
}

static bool pisp_be_gate_controller_valid(
    const struct pisp_be_gate *gate,
    const struct media_engine_controller *media_controller)
{
    return gate && media_controller && gate->controller_id != 0U &&
           media_controller->owner.controller_id == gate->controller_id;
}

static bool pisp_be_gate_active_lease(
    const struct pisp_be_gate *gate,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease)
{
    return pisp_be_gate_controller_valid(gate, media_controller) &&
           media_engine_lease_active_for(media_controller, engine_lease,
                                         MEDIA_ENGINE_PISP_BE);
}

static void pisp_be_gate_publish(struct pisp_be_gate *gate, u32 state)
{
    dmb_ishst();
    gate->state = state;
    gate->transition_count++;
    dmb_ishst();
}

static void pisp_be_gate_quarantine(struct pisp_be_gate *gate, u32 fault,
                                    u32 status)
{
    if (gate->state == PISP_BE_GATE_QUARANTINED)
        return;
    gate->fault = fault;
    gate->last_status = status;
    gate->failure_count++;
    pisp_be_gate_publish(gate, PISP_BE_GATE_QUARANTINED);
}

static bool pisp_be_gate_expired(struct pisp_be_gate *gate, u64 now_ms)
{
    if (now_ms <= gate->deadline_ms)
        return false;
    pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_DEADLINE, 0U);
    return true;
}

static bool pisp_be_gate_step_ready(struct pisp_be_gate *gate,
                                    const struct pisp_be_gate_handle *handle,
                                    u32 caller_core, u64 now_ms)
{
    return pisp_be_gate_handle_valid(gate, handle, caller_core) &&
           !pisp_be_gate_expired(gate, now_ms);
}

bool pisp_be_gate_init(struct pisp_be_gate *gate, u32 owner_core,
                       u32 controller_id, u64 now_ms, u32 timeout_ms,
                       struct pisp_be_gate_handle *handle_out)
{
    u64 deadline;

    pisp_be_gate_clear_handle(handle_out);
    if (!gate || !handle_out || owner_core != PISP_BE_GATE_OWNER_CORE ||
        controller_id == 0U || !pisp_be_gate_deadline(now_ms, timeout_ms,
                                                       &deadline) ||
        !pisp_be_gate_storage_fresh(gate))
        return false;
    gate->generation = 1U;
    gate->deadline_ms = deadline;
    gate->controller_id = controller_id;
    gate->owner_core = owner_core;
    gate->state = PISP_BE_GATE_DISABLED;
    gate->fault = PISP_BE_GATE_FAULT_NONE;
    gate->transition_count = 0U;
    gate->failure_count = 0U;
    gate->last_status = 0U;
    gate->_reserved0 = 0U;
    memset(gate->_reserved, 0, sizeof(gate->_reserved));
    dmb_ishst();
    handle_out->_token = pisp_be_gate_token(gate);
    handle_out->_generation = gate->generation;
    handle_out->_controller_id = controller_id;
    handle_out->_reserved = 0U;
    return true;
}

bool pisp_be_gate_passive_identify(
    struct pisp_be_gate *gate, const struct pisp_be_gate_handle *handle,
    u32 caller_core, struct media_engine_controller *media_controller,
    u64 now_ms, u32 raw_version)
{
    if (!pisp_be_gate_step_ready(gate, handle, caller_core, now_ms))
        return false;
    if (gate->state != PISP_BE_GATE_DISABLED) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_INTERNAL, 0U);
        return false;
    }
    if (!pisp_be_gate_controller_valid(gate, media_controller) ||
        !media_engine_passive_identify(media_controller, MEDIA_ENGINE_PISP_BE,
                                       raw_version)) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_BAD_ID, raw_version);
        return false;
    }
    pisp_be_gate_publish(gate, PISP_BE_GATE_PASSIVE_ID_OK);
    return true;
}

bool pisp_be_gate_clock_verified(
    struct pisp_be_gate *gate, const struct pisp_be_gate_handle *handle,
    u32 caller_core, const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease, u64 now_ms,
    u32 clock_id, bool clock_ok)
{
    const struct media_engine_descriptor *desc;

    if (!pisp_be_gate_step_ready(gate, handle, caller_core, now_ms))
        return false;
    if (gate->state != PISP_BE_GATE_PASSIVE_ID_OK) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_INTERNAL, 0U);
        return false;
    }
    desc = media_engine_descriptor(MEDIA_ENGINE_PISP_BE);
    if (!pisp_be_gate_active_lease(gate, media_controller, engine_lease) ||
        !desc || desc->clock_count != 1U ||
        (desc->known_mask & MEDIA_ENGINE_KNOWN_CLOCKS) == 0U ||
        clock_id != desc->clock_ids[0] || !clock_ok) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_CLOCK, clock_id);
        return false;
    }
    pisp_be_gate_publish(gate, PISP_BE_GATE_CLOCK_OK);
    return true;
}

bool pisp_be_gate_iommu_verified(
    struct pisp_be_gate *gate, const struct pisp_be_gate_handle *handle,
    u32 caller_core, const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease, u64 now_ms,
    const struct pisp_be_gate_iommu_evidence *evidence, bool iommu_ok)
{
    const struct media_engine_descriptor *desc;

    if (!pisp_be_gate_step_ready(gate, handle, caller_core, now_ms))
        return false;
    if (gate->state != PISP_BE_GATE_CLOCK_OK) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_INTERNAL, 0U);
        return false;
    }
    desc = media_engine_descriptor(MEDIA_ENGINE_PISP_BE);
    if (!pisp_be_gate_active_lease(gate, media_controller, engine_lease) ||
        !evidence || evidence->_reserved != 0U || !desc ||
        (desc->known_mask & MEDIA_ENGINE_KNOWN_IOMMU) == 0U ||
        evidence->iommu_resource != desc->iommu_resource ||
        evidence->lease._token != engine_lease->_token ||
        evidence->lease._controller_id != engine_lease->_controller_id ||
        evidence->lease._reserved != 0U ||
        !media_engine_lease_active_for(media_controller, &evidence->lease,
                                       MEDIA_ENGINE_PISP_BE) ||
        !iommu_ok) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_IOMMU,
                                evidence ? evidence->iommu_resource : 0U);
        return false;
    }
    pisp_be_gate_publish(gate, PISP_BE_GATE_IOMMU_OK);
    return true;
}

bool pisp_be_gate_idle_verified(
    struct pisp_be_gate *gate, const struct pisp_be_gate_handle *handle,
    u32 caller_core, const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease, u64 now_ms,
    const struct pisp_be_gate_idle_evidence *evidence, bool idle_ok)
{
    if (!pisp_be_gate_step_ready(gate, handle, caller_core, now_ms))
        return false;
    if (gate->state != PISP_BE_GATE_IOMMU_OK) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_INTERNAL,
                                evidence ? evidence->raw_status : 0U);
        return false;
    }
    if (!pisp_be_gate_active_lease(gate, media_controller, engine_lease) ||
        !evidence || evidence->_reserved != 0U || !idle_ok ||
        evidence->raw_status != 0U ||
        evidence->batch_started != evidence->batch_done) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_STATUS,
                                evidence ? evidence->raw_status : 0U);
        return false;
    }
    pisp_be_gate_publish(gate, PISP_BE_GATE_IDLE_OK);
    return true;
}

bool pisp_be_gate_admit_prepared_job(
    struct pisp_be_gate *gate, const struct pisp_be_gate_handle *handle,
    u32 caller_core, const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease, u64 now_ms,
    const struct pisp_be_contract *job_contract,
    const struct pisp_be_job_handle *job_handle)
{
    enum pisp_be_job_state job_state;

    if (!pisp_be_gate_step_ready(gate, handle, caller_core, now_ms))
        return false;
    if (gate->state != PISP_BE_GATE_IDLE_OK) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_INTERNAL, 0U);
        return false;
    }
    if (!pisp_be_gate_active_lease(gate, media_controller, engine_lease) ||
        !job_contract ||
        job_contract->owner.controller_id != gate->controller_id ||
        !pisp_be_contract_state_get(job_contract, job_handle, &job_state) ||
        job_state != PISP_BE_JOB_PREPARED) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_JOB, 0U);
        return false;
    }
    pisp_be_gate_publish(gate, PISP_BE_GATE_JOB_ADMITTED);
    return true;
}

bool pisp_be_gate_report_completion(
    struct pisp_be_gate *gate, const struct pisp_be_gate_handle *handle,
    u32 caller_core, u64 now_ms, bool job_ok, bool canaries_ok)
{
    if (!pisp_be_gate_step_ready(gate, handle, caller_core, now_ms))
        return false;
    if (gate->state != PISP_BE_GATE_JOB_ADMITTED) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_INTERNAL, 0U);
        return false;
    }
    if (!job_ok) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_JOB, 0U);
        return false;
    }
    if (!canaries_ok) {
        pisp_be_gate_quarantine(gate, PISP_BE_GATE_FAULT_CANARY, 0U);
        return false;
    }
    pisp_be_gate_publish(gate, PISP_BE_GATE_JOB_COMPLETED);
    return true;
}

bool pisp_be_gate_rearm(struct pisp_be_gate *gate,
                        const struct pisp_be_gate_handle *handle,
                        u32 caller_core, u64 now_ms, u32 timeout_ms,
                        struct pisp_be_gate_handle *handle_out)
{
    u64 deadline;

    pisp_be_gate_clear_handle(handle_out);
    if (!handle_out || !pisp_be_gate_handle_valid(gate, handle, caller_core) ||
        gate->state != PISP_BE_GATE_QUARANTINED ||
        !pisp_be_gate_deadline(now_ms, timeout_ms, &deadline))
        return false;
    if (gate->generation == ~0ULL)
        return false;
    gate->generation++;
    gate->deadline_ms = deadline;
    gate->state = PISP_BE_GATE_DISABLED;
    gate->fault = PISP_BE_GATE_FAULT_NONE;
    gate->transition_count = 0U;
    gate->failure_count = 0U;
    gate->last_status = 0U;
    dmb_ishst();
    handle_out->_token = pisp_be_gate_token(gate);
    handle_out->_generation = gate->generation;
    handle_out->_controller_id = gate->controller_id;
    handle_out->_reserved = 0U;
    return true;
}

bool pisp_be_gate_state_get(const struct pisp_be_gate *gate,
                            const struct pisp_be_gate_handle *handle,
                            u32 caller_core,
                            enum pisp_be_gate_state *state_out,
                            enum pisp_be_gate_fault *fault_out)
{
    if (!state_out || !fault_out ||
        !pisp_be_gate_handle_valid(gate, handle, caller_core))
        return false;
    dmb_ishld();
    *state_out = (enum pisp_be_gate_state)gate->state;
    *fault_out = (enum pisp_be_gate_fault)gate->fault;
    return true;
}
