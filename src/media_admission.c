/*
 * media_admission.c - ADR-072 offline cross-engine media admission.
 *
 * No hardware resource is enabled here.  Caller-supplied leases attest
 * engine ownership, while immutable numeric descriptions make replay useful.
 */
#include "types.h"
#include "media_admission.h"

static u64 media_admission_irq_save(void)
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

static void media_admission_irq_restore(u64 daif)
{
#ifndef PIOS_HOST_TYPES_SHIM
    __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
#else
    (void)daif;
#endif
}

static void media_admission_zero(void *value, usize bytes)
{
    u8 *p = (u8 *)value;
    usize i;

    for (i = 0U; i < bytes; i++)
        p[i] = 0U;
}

static bool media_admission_fresh(const struct media_admission *admission)
{
    const u8 *p;
    usize i;

    if (!admission)
        return false;
    p = (const u8 *)admission;
    for (i = 0U; i < sizeof(*admission); i++) {
        if (p[i] != 0U)
            return false;
    }
    return true;
}

static void media_admission_clear_plan_handle(
    struct media_admission_plan_handle *out)
{
    if (out)
        media_admission_zero(out, sizeof(*out));
}

static void media_admission_clear_job_handle(
    struct media_admission_job_handle *out)
{
    if (out)
        media_admission_zero(out, sizeof(*out));
}

static bool media_admission_ready(const struct media_admission *admission,
                                  u32 caller_core)
{
    return admission && caller_core == MEDIA_ADMISSION_OWNER_CORE &&
           core_id() == MEDIA_ADMISSION_OWNER_CORE &&
           admission->owner.initialized == 1U &&
           admission->owner.controller_id != 0U &&
           admission->owner.hardware_enable == 0U &&
           admission->media_controller &&
           admission->media_controller->owner.controller_id ==
               admission->owner.controller_id;
}

static bool media_admission_begin(struct media_admission *admission,
                                  u32 caller_core, u64 *daif_out)
{
    u64 daif;

    if (!daif_out || !media_admission_ready(admission, caller_core))
        return false;
    daif = media_admission_irq_save();
    if (admission->owner.transition_active != 0U) {
        media_admission_irq_restore(daif);
        return false;
    }
    admission->owner.transition_active = 1U;
    dmb_ishst();
    *daif_out = daif;
    return true;
}

static void media_admission_end(struct media_admission *admission, u64 daif)
{
    dmb_ishst();
    admission->owner.transition_active = 0U;
    media_admission_irq_restore(daif);
}

static bool media_admission_generation_bump(u32 *generation)
{
    if (!generation || *generation == ~0U)
        return false;
    (*generation)++;
    return true;
}

static u32 media_admission_plan_slot(const struct media_admission *admission,
                                     u64 plan_id)
{
    u32 i;

    for (i = 0U; i < MEDIA_ADMISSION_MAX_PLANS; i++) {
        if (admission->plans[i].state != MEDIA_ADMISSION_PLAN_EMPTY &&
            admission->plans[i].state != MEDIA_ADMISSION_PLAN_RETIRED &&
            admission->plans[i].plan_id == plan_id)
            return i;
    }
    return MEDIA_ADMISSION_MAX_PLANS;
}

static bool media_admission_plan_handle_valid(
    const struct media_admission *admission,
    const struct media_admission_plan_handle *handle, u32 *slot_out)
{
    u32 slot;

    if (!admission || !handle ||
        handle->controller_id != admission->owner.controller_id ||
        handle->plan_id == 0U || handle->slot >= MEDIA_ADMISSION_MAX_PLANS ||
        handle->generation == 0U || handle->_reserved != 0U)
        return false;
    slot = handle->slot;
    if (admission->plans[slot].plan_id != handle->plan_id ||
        admission->plans[slot].generation != handle->generation ||
        admission->plans[slot].state == MEDIA_ADMISSION_PLAN_EMPTY ||
        admission->plans[slot].state == MEDIA_ADMISSION_PLAN_RETIRED)
        return false;
    if (slot_out)
        *slot_out = slot;
    return true;
}

static bool media_admission_job_handle_valid(
    const struct media_admission *admission,
    const struct media_admission_job_handle *handle, u32 *plan_slot_out,
    u32 *job_slot_out)
{
    const struct media_admission_plan_control *plan;
    const struct media_admission_job_control *control;

    if (!admission || !handle ||
        handle->controller_id != admission->owner.controller_id ||
        handle->plan_id == 0U || handle->job_id == 0U ||
        handle->plan_slot >= MEDIA_ADMISSION_MAX_PLANS ||
        handle->job_slot >= MEDIA_ADMISSION_MAX_JOBS ||
        handle->plan_generation == 0U || handle->job_generation == 0U ||
        handle->_reserved != 0U)
        return false;
    plan = &admission->plans[handle->plan_slot];
    control = &admission->controls[handle->plan_slot][handle->job_slot];
    if (plan->state == MEDIA_ADMISSION_PLAN_EMPTY ||
        plan->state == MEDIA_ADMISSION_PLAN_RETIRED ||
        plan->plan_id != handle->plan_id ||
        plan->generation != handle->plan_generation ||
        control->state == MEDIA_ADMISSION_JOB_EMPTY ||
        control->state == MEDIA_ADMISSION_JOB_RETIRED ||
        control->generation != handle->job_generation ||
        admission->descriptors[handle->plan_slot][handle->job_slot].job_id !=
            handle->job_id)
        return false;
    if (plan_slot_out)
        *plan_slot_out = handle->plan_slot;
    if (job_slot_out)
        *job_slot_out = handle->job_slot;
    return true;
}

static void media_admission_replay(struct media_admission *admission,
                                   u32 type, u32 plan_slot, u32 job_slot,
                                   u32 fault, u64 now_ms)
{
    struct media_admission_replay_owner *owner = &admission->replay_owner;
    struct media_admission_replay_event *event;
    u32 slot;

    if (owner->next_sequence == ~0ULL)
        return;
    owner->next_sequence++;
    slot = (u32)((owner->oldest_slot + owner->count) %
                 MEDIA_ADMISSION_MAX_REPLAY);
    if (owner->count == MEDIA_ADMISSION_MAX_REPLAY) {
        slot = owner->oldest_slot;
        owner->oldest_slot = (owner->oldest_slot + 1U) %
                             MEDIA_ADMISSION_MAX_REPLAY;
    } else {
        owner->count++;
    }
    event = &admission->replay[slot];
    media_admission_zero(event, sizeof(*event));
    event->sequence = owner->next_sequence;
    event->plan_id = admission->plans[plan_slot].plan_id;
    event->plan_generation = admission->plans[plan_slot].generation;
    if (job_slot < MEDIA_ADMISSION_MAX_JOBS) {
        event->job_id = admission->descriptors[plan_slot][job_slot].job_id;
        event->job_generation =
            admission->controls[plan_slot][job_slot].generation;
        event->engine =
            admission->descriptors[plan_slot][job_slot].engine;
    }
    event->time_ms = now_ms;
    event->type = type;
    event->fault = fault;
    dmb_ishst();
}

static bool media_admission_dma_valid(
    const struct media_admission_dma_span *dma)
{
    return dma && dma->span_id != 0U && dma->span_bytes != 0U &&
           dma->span_capacity >= dma->span_bytes && dma->generation != 0U &&
           dma->_reserved == 0U &&
           dma->span_start <= ~0ULL - dma->span_bytes;
}

static bool media_admission_resource_matches(
    const struct media_admission_job_request *request,
    const struct media_engine_descriptor *engine)
{
    u32 i;

    if (!request || !engine || request->_reserved != 0U ||
        request->job_id == 0U || request->resources._reserved != 0U ||
        request->input._reserved != 0U ||
        (request->input.producer_job_id == 0U &&
         request->input.producer_generation != 0U) ||
        (request->input.producer_job_id != 0U &&
         request->input.producer_generation == 0U) ||
        !media_admission_dma_valid(&request->resources.dma))
        return false;
    if ((engine->capabilities & (MEDIA_ENGINE_CAP_ACTIVE_LEASE |
                                 MEDIA_ENGINE_CAP_DMA_EXCLUSIVE)) !=
        (MEDIA_ENGINE_CAP_ACTIVE_LEASE | MEDIA_ENGINE_CAP_DMA_EXCLUSIVE) ||
        (engine->known_mask & (MEDIA_ENGINE_KNOWN_CLOCKS |
                                MEDIA_ENGINE_KNOWN_GIC_SPI |
                                MEDIA_ENGINE_KNOWN_IOMMU)) !=
        (MEDIA_ENGINE_KNOWN_CLOCKS | MEDIA_ENGINE_KNOWN_GIC_SPI |
         MEDIA_ENGINE_KNOWN_IOMMU) ||
        engine->clock_count == 0U ||
        engine->clock_count > MEDIA_ENGINE_MAX_CLOCKS ||
        request->resources.clock_count != engine->clock_count ||
        request->resources.iommu_resource != engine->iommu_resource ||
        request->resources.gic_spi != engine->gic_spi)
        return false;
    for (i = 0U; i < engine->clock_count; i++) {
        if (request->resources.clock_ids[i] != engine->clock_ids[i])
            return false;
    }
    for (; i < MEDIA_ENGINE_MAX_CLOCKS; i++) {
        if (request->resources.clock_ids[i] != 0U)
            return false;
    }
    return true;
}

static bool media_admission_spans_overlap(u64 a_start, u64 a_bytes,
                                          u64 b_start, u64 b_bytes)
{
    return a_start < b_start + b_bytes && b_start < a_start + a_bytes;
}

static bool media_admission_resources_unique(
    const struct media_admission *admission, u32 plan_slot,
    const struct media_admission_job_request *request)
{
    u32 i;
    const struct media_admission_dma_span *dma = &request->resources.dma;

    for (i = 0U; i < MEDIA_ADMISSION_MAX_JOBS; i++) {
        const struct media_admission_job_control *control =
            &admission->controls[plan_slot][i];
        const struct media_admission_job_descriptor *other =
            &admission->descriptors[plan_slot][i];

        if (control->state == MEDIA_ADMISSION_JOB_EMPTY ||
            control->state == MEDIA_ADMISSION_JOB_RETIRED ||
            control->state == MEDIA_ADMISSION_JOB_FAULTED)
            continue;
        /* DMA authority remains unique for the plan's full replay lifetime. */
        if (other->dma_span_id == dma->span_id ||
            media_admission_spans_overlap(other->dma_span_start,
                                          other->dma_span_bytes,
                                          dma->span_start, dma->span_bytes))
            return false;
        /* Completed jobs no longer hold engine-owned execution resources. */
        if (control->state != MEDIA_ADMISSION_JOB_COMPLETED &&
            (other->engine == (u32)request->engine ||
            other->iommu_resource == request->resources.iommu_resource ||
            other->gic_spi == request->resources.gic_spi))
            return false;
    }
    return true;
}

static u32 media_admission_job_slot_by_id(
    const struct media_admission *admission, u32 plan_slot, u64 job_id,
    u32 generation)
{
    u32 i;

    for (i = 0U; i < MEDIA_ADMISSION_MAX_JOBS; i++) {
        if (admission->controls[plan_slot][i].state !=
                MEDIA_ADMISSION_JOB_EMPTY &&
            admission->descriptors[plan_slot][i].job_id == job_id &&
            admission->controls[plan_slot][i].generation == generation)
            return i;
    }
    return MEDIA_ADMISSION_MAX_JOBS;
}

static void media_admission_fault_dependents(struct media_admission *admission,
                                              u32 plan_slot, u32 failed_slot,
                                              u32 fault, u64 now_ms)
{
    bool changed;
    u32 i;

    if (failed_slot < MEDIA_ADMISSION_MAX_JOBS) {
        struct media_admission_job_control *failed =
            &admission->controls[plan_slot][failed_slot];

        failed->state = MEDIA_ADMISSION_JOB_FAULTED;
        failed->fault = fault;
        failed->canary_valid = 0U;
        media_admission_replay(admission, MEDIA_ADMISSION_REPLAY_PLAN_FAULTED,
                               plan_slot, failed_slot, fault, now_ms);
    }
    do {
        changed = false;
        for (i = 0U; i < MEDIA_ADMISSION_MAX_JOBS; i++) {
            struct media_admission_job_control *control =
                &admission->controls[plan_slot][i];
            const struct media_admission_job_descriptor *desc =
                &admission->descriptors[plan_slot][i];
            u32 input_slot;

            if (control->state == MEDIA_ADMISSION_JOB_EMPTY ||
                control->state == MEDIA_ADMISSION_JOB_FAULTED ||
                desc->input_job_id == 0U)
                continue;
            input_slot = media_admission_job_slot_by_id(
                admission, plan_slot, desc->input_job_id,
                desc->input_generation);
            if (input_slot < MEDIA_ADMISSION_MAX_JOBS &&
                admission->controls[plan_slot][input_slot].state ==
                    MEDIA_ADMISSION_JOB_FAULTED) {
                control->state = MEDIA_ADMISSION_JOB_FAULTED;
                control->fault = fault;
                control->canary_valid = 0U;
                media_admission_replay(admission,
                    MEDIA_ADMISSION_REPLAY_PLAN_FAULTED, plan_slot, i, fault,
                    now_ms);
                changed = true;
            }
        }
    } while (changed);
}

static void media_admission_fault_plan(struct media_admission *admission,
                                       u32 plan_slot, u32 job_slot,
                                       u32 fault, u64 now_ms)
{
    struct media_admission_plan_control *plan =
        &admission->plans[plan_slot];

    if (plan->state != MEDIA_ADMISSION_PLAN_ACTIVE)
        return;
    plan->state = MEDIA_ADMISSION_PLAN_FAULTED;
    plan->fault = fault;
    media_admission_fault_dependents(admission, plan_slot, job_slot, fault,
                                     now_ms);
    dmb_ishst();
}

bool media_admission_init(struct media_admission *admission, u32 controller_id,
                          const struct media_engine_controller *media_controller,
                          u32 caller_core)
{
    if (!admission || !media_controller || controller_id == 0U ||
        caller_core != MEDIA_ADMISSION_OWNER_CORE ||
        core_id() != MEDIA_ADMISSION_OWNER_CORE ||
        media_controller->owner.controller_id != controller_id ||
        !media_admission_fresh(admission))
        return false;
    admission->owner.controller_id = controller_id;
    admission->owner.initialized = 1U;
    admission->owner.hardware_enable = 0U;
    admission->owner.transition_active = 0U;
    admission->media_controller = media_controller;
    admission->replay_owner.next_sequence = 0U;
    admission->replay_owner.oldest_slot = 0U;
    admission->replay_owner.count = 0U;
    dmb_ishst();
    return true;
}

bool media_admission_plan_create(struct media_admission *admission, u64 plan_id,
                                 u32 caller_core,
                                 struct media_admission_plan_handle *out)
{
    u64 daif;
    u32 i, generation;

    media_admission_clear_plan_handle(out);
    if (!out || plan_id == 0U ||
        !media_admission_begin(admission, caller_core, &daif))
        return false;
    if (media_admission_plan_slot(admission, plan_id) <
        MEDIA_ADMISSION_MAX_PLANS) {
        media_admission_end(admission, daif);
        return false;
    }
    for (i = 0U; i < MEDIA_ADMISSION_MAX_PLANS; i++) {
        if (admission->plans[i].state == MEDIA_ADMISSION_PLAN_EMPTY ||
            admission->plans[i].state == MEDIA_ADMISSION_PLAN_RETIRED)
            break;
    }
    if (i == MEDIA_ADMISSION_MAX_PLANS) {
        media_admission_end(admission, daif);
        return false;
    }
    generation = admission->plans[i].generation;
    if (generation == ~0U) {
        media_admission_end(admission, daif);
        return false;
    }
    media_admission_zero(&admission->plans[i], sizeof(admission->plans[i]));
    admission->plans[i].plan_id = plan_id;
    admission->plans[i].generation = generation + 1U;
    admission->plans[i].state = MEDIA_ADMISSION_PLAN_ACTIVE;
    media_admission_zero(admission->controls[i],
                         sizeof(admission->controls[i]));
    media_admission_zero(admission->descriptors[i],
                         sizeof(admission->descriptors[i]));
    media_admission_replay(admission, MEDIA_ADMISSION_REPLAY_PLAN_CREATED, i,
                           MEDIA_ADMISSION_MAX_JOBS,
                           MEDIA_ADMISSION_FAULT_NONE, 0U);
    out->plan_id = plan_id;
    out->controller_id = admission->owner.controller_id;
    out->slot = i;
    out->generation = admission->plans[i].generation;
    out->_reserved = 0U;
    media_admission_end(admission, daif);
    return true;
}

bool media_admission_job_prepare(
    struct media_admission *admission,
    const struct media_admission_plan_handle *plan_handle,
    const struct media_admission_job_request *request,
    const struct media_engine_lease *lease, u32 caller_core,
    struct media_admission_job_handle *out)
{
    const struct media_engine_descriptor *engine;
    struct media_admission_plan_control *plan;
    struct media_admission_job_descriptor *desc;
    struct media_admission_job_control *control;
    u64 daif;
    u32 plan_slot, job_slot, i;

    media_admission_clear_job_handle(out);
    if (!out || !request || !lease ||
        !media_admission_begin(admission, caller_core, &daif))
        return false;
    if (!media_admission_plan_handle_valid(admission, plan_handle, &plan_slot)) {
        media_admission_end(admission, daif);
        return false;
    }
    plan = &admission->plans[plan_slot];
    engine = media_engine_descriptor(request->engine);
    if (plan->state != MEDIA_ADMISSION_PLAN_ACTIVE || !engine ||
        !media_admission_resource_matches(request, engine) ||
        !media_engine_lease_active_for(admission->media_controller, lease,
                                       request->engine) ||
        !media_admission_resources_unique(admission, plan_slot, request)) {
        media_admission_end(admission, daif);
        return false;
    }
    if (request->input.producer_job_id != 0U &&
        media_admission_job_slot_by_id(admission, plan_slot,
                                       request->input.producer_job_id,
                                       request->input.producer_generation) ==
            MEDIA_ADMISSION_MAX_JOBS) {
        media_admission_end(admission, daif);
        return false;
    }
    for (i = 0U; i < MEDIA_ADMISSION_MAX_JOBS; i++) {
        if (admission->controls[plan_slot][i].state !=
                MEDIA_ADMISSION_JOB_EMPTY &&
            admission->descriptors[plan_slot][i].job_id == request->job_id) {
            media_admission_end(admission, daif);
            return false;
        }
    }
    for (job_slot = 0U; job_slot < MEDIA_ADMISSION_MAX_JOBS; job_slot++) {
        if (admission->controls[plan_slot][job_slot].state ==
            MEDIA_ADMISSION_JOB_EMPTY)
            break;
    }
    if (job_slot == MEDIA_ADMISSION_MAX_JOBS ||
        admission->controls[plan_slot][job_slot].generation == ~0U) {
        media_admission_end(admission, daif);
        return false;
    }
    control = &admission->controls[plan_slot][job_slot];
    desc = &admission->descriptors[plan_slot][job_slot];
    control->generation++;
    media_admission_zero(desc, sizeof(*desc));
    desc->job_id = request->job_id;
    desc->input_job_id = request->input.producer_job_id;
    desc->input_generation = request->input.producer_generation;
    desc->dma_span_id = request->resources.dma.span_id;
    desc->dma_span_start = request->resources.dma.span_start;
    desc->dma_span_bytes = request->resources.dma.span_bytes;
    desc->dma_span_capacity = request->resources.dma.span_capacity;
    desc->dma_generation = request->resources.dma.generation;
    desc->plan_generation = plan->generation;
    desc->job_generation = control->generation;
    desc->engine = (u32)request->engine;
    desc->clock_count = request->resources.clock_count;
    desc->clock_ids[0] = request->resources.clock_ids[0];
    desc->clock_ids[1] = request->resources.clock_ids[1];
    desc->iommu_resource = request->resources.iommu_resource;
    desc->gic_spi = request->resources.gic_spi;
    desc->lease_token = lease->_token;
    desc->lease_controller_id = lease->_controller_id;
    desc->lease_reserved = lease->_reserved;
    control->deadline_ms = 0U;
    control->completion_sequence = 0U;
    control->fault = MEDIA_ADMISSION_FAULT_NONE;
    control->canary_valid = 0U;
    dmb_ishst();
    control->state = MEDIA_ADMISSION_JOB_PREPARED;
    plan->job_count++;
    media_admission_replay(admission, MEDIA_ADMISSION_REPLAY_JOB_PREPARED,
                           plan_slot, job_slot, MEDIA_ADMISSION_FAULT_NONE, 0U);
    out->plan_id = plan->plan_id;
    out->job_id = request->job_id;
    out->controller_id = admission->owner.controller_id;
    out->plan_slot = plan_slot;
    out->plan_generation = plan->generation;
    out->job_slot = job_slot;
    out->job_generation = control->generation;
    out->_reserved = 0U;
    media_admission_end(admission, daif);
    return true;
}

bool media_admission_job_admit(struct media_admission *admission,
                               const struct media_admission_job_handle *job,
                               u64 now_ms, u32 timeout_ms, u32 caller_core)
{
    struct media_admission_plan_control *plan;
    struct media_admission_job_control *control;
    const struct media_admission_job_descriptor *desc;
    struct media_engine_lease lease;
    u64 daif, deadline;
    u32 plan_slot, job_slot, input_slot;

    if (!media_admission_begin(admission, caller_core, &daif))
        return false;
    if (!media_admission_job_handle_valid(admission, job, &plan_slot,
                                          &job_slot)) {
        media_admission_end(admission, daif);
        return false;
    }
    plan = &admission->plans[plan_slot];
    control = &admission->controls[plan_slot][job_slot];
    desc = &admission->descriptors[plan_slot][job_slot];
    if (timeout_ms == 0U || timeout_ms > MEDIA_ADMISSION_MAX_TIMEOUT_MS ||
        now_ms > ~0ULL - (u64)timeout_ms) {
        media_admission_fault_plan(admission, plan_slot, job_slot,
                                   MEDIA_ADMISSION_FAULT_ARGUMENT, now_ms);
        media_admission_end(admission, daif);
        return false;
    }
    if (plan->state != MEDIA_ADMISSION_PLAN_ACTIVE ||
        control->state != MEDIA_ADMISSION_JOB_PREPARED) {
        media_admission_end(admission, daif);
        return false;
    }
    lease._token = desc->lease_token;
    lease._controller_id = desc->lease_controller_id;
    lease._reserved = desc->lease_reserved;
    if (!media_engine_lease_active_for(admission->media_controller, &lease,
                                       (enum media_engine_kind)desc->engine)) {
        media_admission_fault_plan(admission, plan_slot, job_slot,
                                   MEDIA_ADMISSION_FAULT_ENGINE_LOST, now_ms);
        media_admission_end(admission, daif);
        return false;
    }
    if (desc->input_job_id != 0U) {
        input_slot = media_admission_job_slot_by_id(
            admission, plan_slot, desc->input_job_id, desc->input_generation);
        if (input_slot == MEDIA_ADMISSION_MAX_JOBS ||
            admission->controls[plan_slot][input_slot].state ==
                MEDIA_ADMISSION_JOB_FAULTED ||
            (admission->controls[plan_slot][input_slot].state ==
                 MEDIA_ADMISSION_JOB_COMPLETED &&
             admission->controls[plan_slot][input_slot].canary_valid == 0U)) {
            media_admission_fault_plan(admission, plan_slot, job_slot,
                                       MEDIA_ADMISSION_FAULT_DEPENDENCY,
                                       now_ms);
            media_admission_end(admission, daif);
            return false;
        }
        if (admission->controls[plan_slot][input_slot].state !=
                MEDIA_ADMISSION_JOB_COMPLETED ||
            admission->controls[plan_slot][input_slot].canary_valid != 1U) {
            media_admission_end(admission, daif);
            return false;
        }
    }
    deadline = now_ms + (u64)timeout_ms;
    control->deadline_ms = deadline;
    control->state = MEDIA_ADMISSION_JOB_ADMITTED;
    media_admission_replay(admission, MEDIA_ADMISSION_REPLAY_JOB_ADMITTED,
                           plan_slot, job_slot, MEDIA_ADMISSION_FAULT_NONE,
                           now_ms);
    media_admission_end(admission, daif);
    return true;
}

bool media_admission_job_complete(struct media_admission *admission,
                                  const struct media_admission_job_handle *job,
                                  u64 now_ms, bool engine_ok, bool canary_ok,
                                  u32 caller_core)
{
    struct media_admission_plan_control *plan;
    struct media_admission_job_control *control;
    u64 daif;
    u32 plan_slot, job_slot, fault;

    if (!media_admission_begin(admission, caller_core, &daif))
        return false;
    if (!media_admission_job_handle_valid(admission, job, &plan_slot,
                                          &job_slot)) {
        media_admission_end(admission, daif);
        return false;
    }
    plan = &admission->plans[plan_slot];
    control = &admission->controls[plan_slot][job_slot];
    if (plan->state != MEDIA_ADMISSION_PLAN_ACTIVE ||
        control->state != MEDIA_ADMISSION_JOB_ADMITTED) {
        media_admission_end(admission, daif);
        return false;
    }
    if (now_ms > control->deadline_ms) {
        media_admission_fault_plan(admission, plan_slot, job_slot,
                                   MEDIA_ADMISSION_FAULT_TIMEOUT, now_ms);
        media_admission_end(admission, daif);
        return false;
    }
    if (!engine_ok || !canary_ok) {
        fault = !engine_ok ? MEDIA_ADMISSION_FAULT_ENGINE_FAILURE :
                             MEDIA_ADMISSION_FAULT_CANARY;
        media_admission_fault_plan(admission, plan_slot, job_slot, fault,
                                   now_ms);
        media_admission_end(admission, daif);
        return false;
    }
    control->state = MEDIA_ADMISSION_JOB_COMPLETED;
    control->canary_valid = 1U;
    control->completion_sequence = ++plan->completion_sequence;
    plan->completed_count++;
    media_admission_replay(admission, MEDIA_ADMISSION_REPLAY_JOB_COMPLETED,
                           plan_slot, job_slot, MEDIA_ADMISSION_FAULT_NONE,
                           now_ms);
    dmb_ishst();
    media_admission_end(admission, daif);
    return true;
}

bool media_admission_tick(struct media_admission *admission, u64 now_ms,
                          u32 caller_core)
{
    u64 daif;
    u32 p, j;

    if (!media_admission_begin(admission, caller_core, &daif))
        return false;
    for (p = 0U; p < MEDIA_ADMISSION_MAX_PLANS; p++) {
        if (admission->plans[p].state != MEDIA_ADMISSION_PLAN_ACTIVE)
            continue;
        for (j = 0U; j < MEDIA_ADMISSION_MAX_JOBS; j++) {
            if (admission->controls[p][j].state ==
                    MEDIA_ADMISSION_JOB_ADMITTED &&
                now_ms > admission->controls[p][j].deadline_ms) {
                media_admission_fault_plan(admission, p, j,
                                           MEDIA_ADMISSION_FAULT_TIMEOUT,
                                           now_ms);
                break;
            }
        }
    }
    media_admission_end(admission, daif);
    return true;
}

bool media_admission_engine_lost(struct media_admission *admission,
                                 enum media_engine_kind engine, u64 now_ms,
                                 u32 caller_core)
{
    u64 daif;
    u32 p, j;
    bool found = false;

    if (!media_engine_descriptor(engine) ||
        !media_admission_begin(admission, caller_core, &daif))
        return false;
    for (p = 0U; p < MEDIA_ADMISSION_MAX_PLANS; p++) {
        if (admission->plans[p].state != MEDIA_ADMISSION_PLAN_ACTIVE)
            continue;
        for (j = 0U; j < MEDIA_ADMISSION_MAX_JOBS; j++) {
            if (admission->controls[p][j].state != MEDIA_ADMISSION_JOB_EMPTY &&
                admission->descriptors[p][j].engine == (u32)engine) {
                media_admission_fault_plan(admission, p, j,
                                           MEDIA_ADMISSION_FAULT_ENGINE_LOST,
                                           now_ms);
                media_admission_replay(admission,
                    MEDIA_ADMISSION_REPLAY_ENGINE_LOST, p, j,
                    MEDIA_ADMISSION_FAULT_ENGINE_LOST, now_ms);
                found = true;
                break;
            }
        }
    }
    media_admission_end(admission, daif);
    return found;
}

bool media_admission_plan_complete(
    struct media_admission *admission,
    const struct media_admission_plan_handle *plan_handle, u64 now_ms,
    u32 caller_core)
{
    u64 daif;
    u32 slot;

    if (!media_admission_begin(admission, caller_core, &daif))
        return false;
    if (!media_admission_plan_handle_valid(admission, plan_handle, &slot) ||
        admission->plans[slot].state != MEDIA_ADMISSION_PLAN_ACTIVE ||
        admission->plans[slot].job_count == 0U ||
        admission->plans[slot].completed_count !=
            admission->plans[slot].job_count) {
        media_admission_end(admission, daif);
        return false;
    }
    admission->plans[slot].state = MEDIA_ADMISSION_PLAN_COMPLETED;
    media_admission_replay(admission, MEDIA_ADMISSION_REPLAY_JOB_COMPLETED,
                           slot, MEDIA_ADMISSION_MAX_JOBS,
                           MEDIA_ADMISSION_FAULT_NONE, now_ms);
    media_admission_end(admission, daif);
    return true;
}

bool media_admission_plan_retire(struct media_admission *admission,
                                 const struct media_admission_plan_handle *plan,
                                 u64 now_ms, u32 caller_core)
{
    u64 daif;
    u32 slot, j;

    if (!media_admission_begin(admission, caller_core, &daif))
        return false;
    if (!media_admission_plan_handle_valid(admission, plan, &slot) ||
        (admission->plans[slot].state != MEDIA_ADMISSION_PLAN_COMPLETED &&
         admission->plans[slot].state != MEDIA_ADMISSION_PLAN_FAULTED) ||
        !media_admission_generation_bump(&admission->plans[slot].generation)) {
        media_admission_end(admission, daif);
        return false;
    }
    for (j = 0U; j < MEDIA_ADMISSION_MAX_JOBS; j++) {
        if (admission->controls[slot][j].state != MEDIA_ADMISSION_JOB_EMPTY)
            admission->controls[slot][j].state = MEDIA_ADMISSION_JOB_RETIRED;
    }
    admission->plans[slot].state = MEDIA_ADMISSION_PLAN_RETIRED;
    media_admission_replay(admission, MEDIA_ADMISSION_REPLAY_PLAN_RETIRED,
                           slot, MEDIA_ADMISSION_MAX_JOBS,
                           MEDIA_ADMISSION_FAULT_NONE, now_ms);
    media_admission_end(admission, daif);
    return true;
}

bool media_admission_plan_state_get(
    const struct media_admission *admission,
    const struct media_admission_plan_handle *plan,
    enum media_admission_plan_state *state_out,
    enum media_admission_fault *fault_out)
{
    u32 slot;

    if (!state_out || !fault_out ||
        !media_admission_plan_handle_valid(admission, plan, &slot))
        return false;
    dmb_ishld();
    *state_out = (enum media_admission_plan_state)admission->plans[slot].state;
    *fault_out = (enum media_admission_fault)admission->plans[slot].fault;
    return true;
}

bool media_admission_job_state_get(
    const struct media_admission *admission,
    const struct media_admission_job_handle *job,
    enum media_admission_job_state *state_out,
    enum media_admission_fault *fault_out)
{
    u32 plan_slot, job_slot;

    if (!state_out || !fault_out ||
        !media_admission_job_handle_valid(admission, job, &plan_slot,
                                          &job_slot))
        return false;
    dmb_ishld();
    *state_out = (enum media_admission_job_state)
        admission->controls[plan_slot][job_slot].state;
    *fault_out = (enum media_admission_fault)
        admission->controls[plan_slot][job_slot].fault;
    return true;
}

bool media_admission_snapshot_get(
    const struct media_admission *admission,
    const struct media_admission_plan_handle *plan_handle,
    struct media_admission_snapshot *out)
{
    u32 plan_slot, i;

    if (!out || !media_admission_plan_handle_valid(admission, plan_handle,
                                                   &plan_slot))
        return false;
    dmb_ishld();
    media_admission_zero(out, sizeof(*out));
    out->plan_id = admission->plans[plan_slot].plan_id;
    out->completion_sequence = admission->plans[plan_slot].completion_sequence;
    out->plan_generation = admission->plans[plan_slot].generation;
    out->state = admission->plans[plan_slot].state;
    out->fault = admission->plans[plan_slot].fault;
    out->job_count = admission->plans[plan_slot].job_count;
    out->completed_count = admission->plans[plan_slot].completed_count;
    out->replay_count = admission->replay_owner.count;
    for (i = 0U; i < MEDIA_ADMISSION_MAX_JOBS; i++) {
        out->jobs[i].job_id = admission->descriptors[plan_slot][i].job_id;
        out->jobs[i].deadline_ms =
            admission->controls[plan_slot][i].deadline_ms;
        out->jobs[i].completion_sequence =
            admission->controls[plan_slot][i].completion_sequence;
        out->jobs[i].generation =
            admission->controls[plan_slot][i].generation;
        out->jobs[i].state = admission->controls[plan_slot][i].state;
        out->jobs[i].fault = admission->controls[plan_slot][i].fault;
        out->jobs[i].canary_valid =
            admission->controls[plan_slot][i].canary_valid;
        out->jobs[i].engine = (enum media_engine_kind)
            admission->descriptors[plan_slot][i].engine;
    }
    return true;
}

bool media_admission_replay_get(
    const struct media_admission *admission, u64 after_sequence,
    struct media_admission_replay_event *out, u32 capacity, u32 *count_out)
{
    u32 i, slot, count = 0U;

    if (!admission || !count_out || (capacity != 0U && !out))
        return false;
    dmb_ishld();
    for (i = 0U; i < admission->replay_owner.count && count < capacity; i++) {
        slot = (admission->replay_owner.oldest_slot + i) %
               MEDIA_ADMISSION_MAX_REPLAY;
        if (admission->replay[slot].sequence > after_sequence)
            out[count++] = admission->replay[slot];
    }
    *count_out = count;
    return true;
}

bool media_admission_hardware_enable_allowed(
    const struct media_admission *admission, u32 caller_core)
{
    (void)admission;
    (void)caller_core;
    return false;
}
