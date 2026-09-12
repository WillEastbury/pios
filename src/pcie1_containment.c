/*
 * pcie1_containment.c - pure PCIe1 endpoint/MSI/DMA ownership contract.
 *
 * The 2 MiB inbound window is Normal-NC from first MMU enable. CPU-to-device
 * and device-to-CPU ownership transitions therefore use system-scoped
 * barriers, not cache maintenance. This module never enables hardware.
 */
#include "types.h"
#include "pcie1_containment.h"

#define ENDPOINT_TOKEN_LOW_MASK 0xFFFFULL
#define ENDPOINT_TOKEN_MASK \
    ((PCIE1_CONTAINMENT_ENDPOINT_MAGIC << \
      PCIE1_CONTAINMENT_ENDPOINT_SHIFT) | ENDPOINT_TOKEN_LOW_MASK)
#define DMA_TOKEN_SLOT_MASK (PCIE1_CONTAINMENT_DMA_SLOT_COUNT - 1U)
#define DMA_TOKEN_MASK \
    ((PCIE1_CONTAINMENT_DMA_MAGIC << PCIE1_CONTAINMENT_DMA_SHIFT) | \
     DMA_TOKEN_SLOT_MASK)
#define MSI_TOKEN_MASK \
    (PCIE1_CONTAINMENT_MSI_MAGIC << PCIE1_CONTAINMENT_MSI_SHIFT)

static inline u64 containment_irq_save(void)
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

static inline void containment_irq_restore(u64 daif)
{
#ifdef PIOS_HOST_TYPES_SHIM
    (void)daif;
#else
    __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
#endif
}

static bool contract_fresh(const struct pcie1_containment *contract)
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

static bool layout_valid(void)
{
    return PIOS_HAS_PCIE1 && PIOS_DMA_PCIE1_BASE != 0U &&
           PIOS_DMA_PCIE1_SIZE ==
               PCIE1_CONTAINMENT_DMA_SLOT_COUNT *
               PCIE1_CONTAINMENT_DMA_SLOT_BYTES &&
           PIOS_DMA_PCIE1_BASE <=
               ~0ULL - PIOS_DMA_PCIE1_SIZE &&
           PCIE1_CONTAINMENT_IOVA_BASE <=
               ~0ULL - PIOS_DMA_PCIE1_SIZE;
}

bool pcie1_containment_bdf_valid(u32 bdf)
{
    u32 bus = (bdf >> 8) & 0xFFU;
    u32 device = (bdf >> 3) & 0x1FU;
    u32 function = bdf & 0x7U;

    return (bdf & ~0xFFFFU) == 0U && bus != 0U &&
           device <= 31U && function <= 7U;
}

static u64 endpoint_token(u32 bdf)
{
    return (PCIE1_CONTAINMENT_ENDPOINT_MAGIC <<
            PCIE1_CONTAINMENT_ENDPOINT_SHIFT) | (u64)bdf;
}

static u64 dma_token(u32 slot)
{
    return (PCIE1_CONTAINMENT_DMA_MAGIC << PCIE1_CONTAINMENT_DMA_SHIFT) |
           (u64)slot;
}

static u64 msi_token(void)
{
    return PCIE1_CONTAINMENT_MSI_MAGIC << PCIE1_CONTAINMENT_MSI_SHIFT;
}

static void endpoint_handle_clear(
    struct pcie1_containment_endpoint_handle *handle)
{
    if (handle)
        *handle = (struct pcie1_containment_endpoint_handle){0};
}

static void dma_handle_clear(struct pcie1_containment_dma_handle *handle)
{
    if (handle)
        *handle = (struct pcie1_containment_dma_handle){0};
}

static void msi_ticket_clear(struct pcie1_containment_msi_ticket *ticket)
{
    if (ticket)
        *ticket = (struct pcie1_containment_msi_ticket){0};
}

static bool endpoint_handle_valid(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core)
{
    return contract && endpoint &&
           caller_core == PCIE1_CONTAINMENT_OWNER_CORE &&
           contract->owner.owner_core == PCIE1_CONTAINMENT_OWNER_CORE &&
           contract->owner.contract_id != 0U &&
           endpoint->contract_id == contract->owner.contract_id &&
           endpoint->generation == contract->owner.endpoint_generation &&
           endpoint->bdf == contract->owner.bdf &&
           endpoint->_reserved[0] == 0U &&
           endpoint->_reserved[1] == 0U &&
           (endpoint->token & ~ENDPOINT_TOKEN_MASK) == 0U &&
           endpoint->token == endpoint_token(endpoint->bdf);
}

static bool endpoint_active(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core)
{
    return endpoint_handle_valid(contract, endpoint, caller_core) &&
           contract->owner.state == PCIE1_CONTAINMENT_ENDPOINT_ATTACHED &&
           contract->owner.fault == PCIE1_CONTAINMENT_FAULT_NONE;
}

static void span_clear(struct pcie1_containment_dma_span *span)
{
    if (span)
        *span = (struct pcie1_containment_dma_span){0};
}

static void publish_dma_state(struct pcie1_containment_dma_control *control,
                              u32 state)
{
    dmb_ishst();
    control->state = state;
    dmb_ishst();
}

static void publish_msi_state(struct pcie1_containment_msi_control *msi,
                              u32 state)
{
    dmb_ishst();
    msi->state = state;
    dmb_ishst();
}

static bool dma_handle_slot(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core,
    u32 *slot_out)
{
    const struct pcie1_containment_dma_control *control;
    u32 slot;

    if (!contract || !handle || !slot_out ||
        caller_core != PCIE1_CONTAINMENT_OWNER_CORE ||
        contract->owner.owner_core != PCIE1_CONTAINMENT_OWNER_CORE ||
        handle->contract_id != contract->owner.contract_id ||
        handle->endpoint_generation != contract->owner.endpoint_generation ||
        handle->_reserved != 0U || handle->generation == 0U ||
        handle->request_id == 0U ||
        (handle->token & ~DMA_TOKEN_MASK) != 0U ||
        (handle->token >> PCIE1_CONTAINMENT_DMA_SHIFT) !=
            PCIE1_CONTAINMENT_DMA_MAGIC)
        return false;
    slot = (u32)(handle->token & DMA_TOKEN_SLOT_MASK);
    if (slot >= PCIE1_CONTAINMENT_DMA_SLOT_COUNT)
        return false;
    control = &contract->dma[slot];
    if (control->generation != handle->generation ||
        control->request_id != handle->request_id ||
        control->state == PCIE1_CONTAINMENT_DMA_FREE ||
        control->state == PCIE1_CONTAINMENT_DMA_RELEASED ||
        control->state == PCIE1_CONTAINMENT_DMA_RETIRED)
        return false;
    dmb_ishld();
    *slot_out = slot;
    return true;
}

static bool dma_handle_control(
    struct pcie1_containment *contract,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core,
    struct pcie1_containment_dma_control **control_out, u32 *slot_out)
{
    u32 slot;

    if (!control_out ||
        !dma_handle_slot(contract, handle, caller_core, &slot))
        return false;
    *control_out = &contract->dma[slot];
    if (slot_out)
        *slot_out = slot;
    return true;
}

static bool dma_handle_control_const(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core,
    const struct pcie1_containment_dma_control **control_out, u32 *slot_out)
{
    u32 slot;

    if (!control_out ||
        !dma_handle_slot(contract, handle, caller_core, &slot))
        return false;
    *control_out = &contract->dma[slot];
    if (slot_out)
        *slot_out = slot;
    return true;
}

static bool msi_ticket_valid(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_msi_ticket *ticket, u32 caller_core)
{
    return contract && ticket &&
           caller_core == PCIE1_CONTAINMENT_OWNER_CORE &&
           contract->owner.owner_core == PCIE1_CONTAINMENT_OWNER_CORE &&
           ticket->contract_id == contract->owner.contract_id &&
           ticket->endpoint_generation ==
               contract->owner.endpoint_generation &&
           ticket->generation == contract->msi.generation &&
           ticket->event_seq != 0U &&
           ticket->event_seq == contract->msi.event_seq &&
           ticket->intid == contract->msi.intid &&
           (ticket->token & ~MSI_TOKEN_MASK) == 0U &&
           ticket->token == msi_token();
}

static void quarantine_internal(struct pcie1_containment *contract,
                                enum pcie1_containment_fault fault)
{
    u32 i;

    if (!contract || fault == PCIE1_CONTAINMENT_FAULT_NONE)
        return;
    if (contract->owner.fault == PCIE1_CONTAINMENT_FAULT_NONE)
        contract->owner.fault = (u32)fault;
    contract->owner.fault_count++;
    contract->owner.quarantine_count++;
    contract->owner.state =
        fault == PCIE1_CONTAINMENT_FAULT_REMOVED ?
        PCIE1_CONTAINMENT_ENDPOINT_REMOVED :
        PCIE1_CONTAINMENT_ENDPOINT_QUARANTINED;
    publish_msi_state(&contract->msi, PCIE1_CONTAINMENT_MSI_QUARANTINED);
    for (i = 0U; i < PCIE1_CONTAINMENT_DMA_SLOT_COUNT; i++) {
        u32 state = contract->dma[i].state;
        if (state != PCIE1_CONTAINMENT_DMA_FREE &&
            state != PCIE1_CONTAINMENT_DMA_RELEASED &&
            state != PCIE1_CONTAINMENT_DMA_RETIRED)
            publish_dma_state(&contract->dma[i],
                              PCIE1_CONTAINMENT_DMA_FAILED);
    }
    dmb();
}

static bool containment_init_locked(
    struct pcie1_containment *contract, u32 contract_id, u32 bdf,
    u64 endpoint_generation, u32 caller_core,
    struct pcie1_containment_endpoint_handle *endpoint_out)
{
    u32 i;

    endpoint_handle_clear(endpoint_out);
    if (!contract || !endpoint_out || contract_id == 0U ||
        endpoint_generation == 0U ||
        caller_core != PCIE1_CONTAINMENT_OWNER_CORE ||
        !pcie1_containment_bdf_valid(bdf) || !layout_valid() ||
        !contract_fresh(contract))
        return false;
    contract->owner.endpoint_generation = endpoint_generation;
    contract->owner.contract_id = contract_id;
    contract->owner.owner_core = caller_core;
    contract->owner.state = PCIE1_CONTAINMENT_ENDPOINT_ATTACHED;
    contract->owner.bdf = bdf;
    contract->msi.generation = 1U;
    contract->msi.state = PCIE1_CONTAINMENT_MSI_UNBOUND;
    for (i = 0U; i < PCIE1_CONTAINMENT_DMA_SLOT_COUNT; i++) {
        contract->dma[i].generation = (u64)i + 1U;
        contract->dma[i].state = PCIE1_CONTAINMENT_DMA_FREE;
    }
    dmb_ishst();
    endpoint_out->token = endpoint_token(bdf);
    endpoint_out->generation = endpoint_generation;
    endpoint_out->contract_id = contract_id;
    endpoint_out->bdf = bdf;
    return true;
}

static bool dma_direction_valid(
    enum pcie1_containment_dma_direction direction)
{
    return direction == PCIE1_CONTAINMENT_TO_DEVICE ||
           direction == PCIE1_CONTAINMENT_FROM_DEVICE;
}

static bool dma_acquire_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, u64 request_id, u32 requested,
    enum pcie1_containment_dma_direction direction,
    u64 now_ms, u64 timeout_ms,
    struct pcie1_containment_dma_handle *handle_out)
{
    u32 i;

    dma_handle_clear(handle_out);
    if (!handle_out || !endpoint_active(contract, endpoint, caller_core) ||
        request_id == 0U ||
        request_id <= contract->owner.last_request_id || requested == 0U ||
        requested > PCIE1_CONTAINMENT_DMA_PAYLOAD_BYTES ||
        (requested & (PCIE1_CONTAINMENT_DMA_ALIGNMENT - 1U)) != 0U ||
        !dma_direction_valid(direction) || timeout_ms == 0U ||
        timeout_ms > PCIE1_CONTAINMENT_TIMEOUT_MAX_MS ||
        now_ms > ~0ULL - timeout_ms)
        return false;
    for (i = 0U; i < PCIE1_CONTAINMENT_DMA_SLOT_COUNT; i++) {
        struct pcie1_containment_dma_control *control = &contract->dma[i];
        struct pcie1_containment_dma_span *span = &contract->spans[i];
        u64 allocation_cpu;
        u64 allocation_iova;

        if (control->state != PCIE1_CONTAINMENT_DMA_FREE ||
            control->exhausted != 0U)
            continue;
        allocation_cpu = PIOS_DMA_PCIE1_BASE +
                         (u64)i * PCIE1_CONTAINMENT_DMA_SLOT_BYTES;
        allocation_iova = PCIE1_CONTAINMENT_IOVA_BASE +
                          (u64)i * PCIE1_CONTAINMENT_DMA_SLOT_BYTES;
        span->payload_cpu_phys =
            allocation_cpu + PCIE1_CONTAINMENT_DMA_GUARD_BYTES;
        span->payload_iova =
            allocation_iova + PCIE1_CONTAINMENT_DMA_GUARD_BYTES;
        span->endpoint_generation = endpoint->generation;
        span->slot_generation = control->generation;
        span->request_id = request_id;
        span->used = direction == PCIE1_CONTAINMENT_TO_DEVICE ?
                     requested : 0U;
        span->requested = requested;
        span->capacity = PCIE1_CONTAINMENT_DMA_PAYLOAD_BYTES;
        span->slot = i;
        span->direction = (u8)direction;
        span->cache_policy = PCIE1_CONTAINMENT_CACHE_NORMAL_NC;
        span->guard_lines = 1U;
        control->deadline_ms = now_ms + timeout_ms;
        control->request_id = request_id;
        control->actual = 0U;
        publish_dma_state(control, PCIE1_CONTAINMENT_DMA_CPU_OWNED);
        contract->owner.last_request_id = request_id;
        handle_out->token = dma_token(i);
        handle_out->generation = control->generation;
        handle_out->endpoint_generation = endpoint->generation;
        handle_out->request_id = request_id;
        handle_out->contract_id = contract->owner.contract_id;
        return true;
    }
    return false;
}

static bool dma_publish_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core)
{
    struct pcie1_containment_dma_control *control;

    if (!endpoint_active(contract, endpoint, caller_core) ||
        !dma_handle_control(contract, handle, caller_core, &control, NULL) ||
        control->state != PCIE1_CONTAINMENT_DMA_CPU_OWNED ||
        contract->msi.state != PCIE1_CONTAINMENT_MSI_ARMED)
        return false;
    dmb();
    publish_dma_state(control, PCIE1_CONTAINMENT_DMA_DEVICE_OWNED);
    return true;
}

static bool dma_complete_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle,
    const struct pcie1_containment_msi_ticket *ticket,
    const struct pcie1_containment_completion *completion, u32 caller_core,
    bool guards_intact)
{
    struct pcie1_containment_dma_control *control;
    u32 slot;

    if (!endpoint_active(contract, endpoint, caller_core) ||
        !msi_ticket_valid(contract, ticket, caller_core) ||
        contract->msi.state != PCIE1_CONTAINMENT_MSI_DISPATCHED ||
        !completion ||
        !dma_handle_control(contract, handle, caller_core, &control, &slot) ||
        control->state != PCIE1_CONTAINMENT_DMA_DEVICE_OWNED)
        return false;
    if (completion->endpoint_generation !=
            contract->owner.endpoint_generation ||
        completion->slot_generation != handle->generation ||
        completion->slot_generation !=
            contract->spans[slot].slot_generation ||
        completion->request_id == 0U ||
        completion->request_id != handle->request_id ||
        completion->request_id != control->request_id ||
        completion->request_id != contract->spans[slot].request_id ||
        completion->actual > contract->spans[slot].requested ||
        completion->success > 1U) {
        quarantine_internal(contract, PCIE1_CONTAINMENT_FAULT_COMPLETION);
        return false;
    }
    if (!guards_intact) {
        quarantine_internal(contract, PCIE1_CONTAINMENT_FAULT_RED_ZONE);
        return false;
    }
    dmb();
    control->actual = completion->actual;
    publish_dma_state(control, completion->success ?
                      PCIE1_CONTAINMENT_DMA_CPU_COMPLETE :
                      PCIE1_CONTAINMENT_DMA_FAILED);
    return true;
}

static bool completion_submit_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_msi_ticket *ticket,
    const struct pcie1_containment_completion *completion, u32 caller_core,
    bool guards_intact)
{
    struct pcie1_containment_dma_handle handle = {0};
    u32 match = PCIE1_CONTAINMENT_DMA_SLOT_COUNT;
    u32 matches = 0U;
    u32 i;

    if (!endpoint_active(contract, endpoint, caller_core) ||
        !msi_ticket_valid(contract, ticket, caller_core) ||
        contract->msi.state != PCIE1_CONTAINMENT_MSI_DISPATCHED ||
        !completion)
        return false;
    for (i = 0U; i < PCIE1_CONTAINMENT_DMA_SLOT_COUNT; i++) {
        const struct pcie1_containment_dma_control *control =
            &contract->dma[i];
        const struct pcie1_containment_dma_span *span = &contract->spans[i];

        if (control->state == PCIE1_CONTAINMENT_DMA_DEVICE_OWNED &&
            span->endpoint_generation == completion->endpoint_generation &&
            span->slot_generation == completion->slot_generation &&
            span->request_id == completion->request_id) {
            match = i;
            matches++;
        }
    }
    if (matches != 1U) {
        quarantine_internal(contract, PCIE1_CONTAINMENT_FAULT_COMPLETION);
        return false;
    }
    handle.token = dma_token(match);
    handle.generation = contract->dma[match].generation;
    handle.endpoint_generation = contract->owner.endpoint_generation;
    handle.request_id = contract->dma[match].request_id;
    handle.contract_id = contract->owner.contract_id;
    return dma_complete_locked(contract, endpoint, &handle, ticket,
                               completion, caller_core, guards_intact);
}

static bool dma_cancel_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core)
{
    struct pcie1_containment_dma_control *control;

    if (!endpoint_active(contract, endpoint, caller_core) ||
        !dma_handle_control(contract, handle, caller_core, &control, NULL) ||
        control->state != PCIE1_CONTAINMENT_DMA_CPU_OWNED)
        return false;
    publish_dma_state(control, PCIE1_CONTAINMENT_DMA_FAILED);
    return true;
}

static bool dma_expire_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle,
    u32 caller_core, u64 now_ms)
{
    struct pcie1_containment_dma_control *control;

    if (!endpoint_active(contract, endpoint, caller_core) ||
        !dma_handle_control(contract, handle, caller_core, &control, NULL) ||
        control->state != PCIE1_CONTAINMENT_DMA_DEVICE_OWNED ||
        now_ms < control->deadline_ms)
        return false;
    quarantine_internal(contract, PCIE1_CONTAINMENT_FAULT_TIMEOUT);
    return true;
}

static bool dma_release_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core)
{
    struct pcie1_containment_dma_control *control;
    u32 slot;

    if (!endpoint_handle_valid(contract, endpoint, caller_core) ||
        !dma_handle_control(contract, handle, caller_core, &control, &slot) ||
        (control->state != PCIE1_CONTAINMENT_DMA_CPU_COMPLETE &&
         control->state != PCIE1_CONTAINMENT_DMA_FAILED))
        return false;
    publish_dma_state(control, PCIE1_CONTAINMENT_DMA_RELEASED);
    if (control->generation == ~0ULL)
        control->exhausted = 1U;
    else
        control->generation++;
    dmb_ishst();
    span_clear(&contract->spans[slot]);
    control->deadline_ms = 0U;
    control->request_id = 0U;
    control->actual = 0U;
    if (control->exhausted != 0U)
        publish_dma_state(control, PCIE1_CONTAINMENT_DMA_RETIRED);
    else
        publish_dma_state(control, PCIE1_CONTAINMENT_DMA_FREE);
    return true;
}

static bool msi_intid_valid(u32 intid)
{
    return intid != 0U &&
           (intid == PIOS_PCIE1_IRQ || intid == PIOS_PCIE1_MSI_IRQ);
}

static bool msi_bind_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, u32 intid, u32 airq_source, u32 target_core,
    bool line_masked, bool airq_handler_registered)
{
    if (!endpoint_active(contract, endpoint, caller_core) ||
        contract->msi.state != PCIE1_CONTAINMENT_MSI_UNBOUND ||
        !msi_intid_valid(intid) ||
        airq_source != PCIE1_CONTAINMENT_AIRQ_SOURCE ||
        target_core != PCIE1_CONTAINMENT_OWNER_CORE ||
        !line_masked || !airq_handler_registered)
        return false;
    contract->msi.intid = intid;
    contract->msi.airq_source = airq_source;
    contract->msi.target_core = target_core;
    publish_msi_state(&contract->msi, PCIE1_CONTAINMENT_MSI_MASKED);
    return true;
}

static bool msi_arm_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core)
{
    if (!endpoint_active(contract, endpoint, caller_core) ||
        contract->msi.state != PCIE1_CONTAINMENT_MSI_MASKED ||
        contract->msi.exhausted != 0U)
        return false;
    publish_msi_state(&contract->msi, PCIE1_CONTAINMENT_MSI_ARMED);
    return true;
}

static bool msi_top_half_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, u32 intid, u32 cause, bool acknowledged,
    bool line_masked, u64 now_ms, u64 timeout_ms,
    struct pcie1_containment_msi_ticket *ticket_out)
{
    msi_ticket_clear(ticket_out);
    if (!ticket_out ||
        !endpoint_active(contract, endpoint, caller_core))
        return false;
    if (intid != contract->msi.intid || !acknowledged || !line_masked) {
        quarantine_internal(contract, PCIE1_CONTAINMENT_FAULT_MSI_PROTOCOL);
        return false;
    }
    if (contract->msi.state != PCIE1_CONTAINMENT_MSI_ARMED) {
        contract->msi.storm_count++;
        quarantine_internal(contract, PCIE1_CONTAINMENT_FAULT_MSI_STORM);
        return false;
    }
    if (cause == 0U || timeout_ms == 0U ||
        timeout_ms > PCIE1_CONTAINMENT_TIMEOUT_MAX_MS ||
        now_ms > ~0ULL - timeout_ms) {
        quarantine_internal(contract, PCIE1_CONTAINMENT_FAULT_MSI_PROTOCOL);
        return false;
    }
    if (contract->msi.event_seq == ~0ULL) {
        contract->msi.exhausted = 1U;
        quarantine_internal(contract, PCIE1_CONTAINMENT_FAULT_GENERATION);
        return false;
    }
    contract->msi.event_seq++;
    contract->msi.deadline_ms = now_ms + timeout_ms;
    contract->msi.cause = cause;
    publish_msi_state(&contract->msi,
                      PCIE1_CONTAINMENT_MSI_EVENT_PENDING);
    ticket_out->token = msi_token();
    ticket_out->generation = contract->msi.generation;
    ticket_out->endpoint_generation = endpoint->generation;
    ticket_out->event_seq = contract->msi.event_seq;
    ticket_out->contract_id = contract->owner.contract_id;
    ticket_out->intid = intid;
    return true;
}

static bool msi_publish_result_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_msi_ticket *ticket,
    u32 caller_core, bool airq_posted)
{
    if (!msi_ticket_valid(contract, ticket, caller_core) ||
        contract->msi.state != PCIE1_CONTAINMENT_MSI_EVENT_PENDING)
        return false;
    if (!airq_posted) {
        contract->msi.deferred_count++;
        return true;
    }
    contract->msi.posted_count++;
    publish_msi_state(&contract->msi,
                      PCIE1_CONTAINMENT_MSI_QUEUED);
    return true;
}

static bool msi_dispatch_begin_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_msi_ticket *ticket, u32 caller_core)
{
    if (!msi_ticket_valid(contract, ticket, caller_core) ||
        contract->msi.state != PCIE1_CONTAINMENT_MSI_QUEUED)
        return false;
    /* Acquire endpoint DMA writes before the scheduled handler inspects
     * completion records, payload, or red zones. */
    dmb();
    publish_msi_state(&contract->msi,
                      PCIE1_CONTAINMENT_MSI_DISPATCHED);
    return true;
}

static bool msi_pending_ticket_locked(
    const struct pcie1_containment *contract, u32 caller_core,
    struct pcie1_containment_msi_ticket *ticket_out)
{
    u32 state;

    msi_ticket_clear(ticket_out);
    if (!contract || !ticket_out ||
        caller_core != PCIE1_CONTAINMENT_OWNER_CORE ||
        contract->owner.owner_core != PCIE1_CONTAINMENT_OWNER_CORE ||
        contract->owner.state != PCIE1_CONTAINMENT_ENDPOINT_ATTACHED)
        return false;
    state = contract->msi.state;
    if (state != PCIE1_CONTAINMENT_MSI_EVENT_PENDING)
        return false;
    dmb_ishld();
    ticket_out->token = msi_token();
    ticket_out->generation = contract->msi.generation;
    ticket_out->endpoint_generation = contract->owner.endpoint_generation;
    ticket_out->event_seq = contract->msi.event_seq;
    ticket_out->contract_id = contract->owner.contract_id;
    ticket_out->intid = contract->msi.intid;
    return true;
}

static bool msi_dispatch_complete_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_msi_ticket *ticket, u32 caller_core)
{
    if (!msi_ticket_valid(contract, ticket, caller_core) ||
        contract->msi.state != PCIE1_CONTAINMENT_MSI_DISPATCHED)
        return false;
    contract->msi.deadline_ms = 0U;
    contract->msi.cause = 0U;
    publish_msi_state(&contract->msi, PCIE1_CONTAINMENT_MSI_MASKED);
    return true;
}

static bool msi_expire_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, u64 now_ms)
{
    if (!endpoint_active(contract, endpoint, caller_core) ||
        (contract->msi.state != PCIE1_CONTAINMENT_MSI_EVENT_PENDING &&
         contract->msi.state != PCIE1_CONTAINMENT_MSI_QUEUED &&
         contract->msi.state != PCIE1_CONTAINMENT_MSI_DISPATCHED) ||
        now_ms < contract->msi.deadline_ms)
        return false;
    quarantine_internal(contract, PCIE1_CONTAINMENT_FAULT_TIMEOUT);
    return true;
}

static bool quarantine_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, enum pcie1_containment_fault fault)
{
    if (!endpoint_active(contract, endpoint, caller_core) ||
        (fault != PCIE1_CONTAINMENT_FAULT_AER &&
         fault != PCIE1_CONTAINMENT_FAULT_MSI_PROTOCOL &&
         fault != PCIE1_CONTAINMENT_FAULT_MSI_STORM))
        return false;
    quarantine_internal(contract, fault);
    return true;
}

static bool remove_locked(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core)
{
    if (!endpoint_active(contract, endpoint, caller_core))
        return false;
    quarantine_internal(contract, PCIE1_CONTAINMENT_FAULT_REMOVED);
    return true;
}

static bool dma_state_get_locked(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core,
    enum pcie1_containment_dma_state *state_out)
{
    const struct pcie1_containment_dma_control *control;

    if (!state_out ||
        !dma_handle_control_const(contract, handle, caller_core,
                                  &control, NULL))
        return false;
    dmb_ishld();
    *state_out = (enum pcie1_containment_dma_state)control->state;
    return true;
}

static bool dma_span_get_locked(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core,
    struct pcie1_containment_dma_span *span_out)
{
    const struct pcie1_containment_dma_control *control;
    u32 slot;

    if (!span_out ||
        !dma_handle_control_const(contract, handle, caller_core,
                                  &control, &slot))
        return false;
    dmb_ishld();
    *span_out = contract->spans[slot];
    return true;
}

static bool dma_completion_get_locked(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core,
    u32 *actual_out, bool *success_out)
{
    const struct pcie1_containment_dma_control *control;

    if (actual_out)
        *actual_out = 0U;
    if (success_out)
        *success_out = false;
    if (!actual_out || !success_out ||
        !dma_handle_control_const(contract, handle, caller_core,
                                  &control, NULL) ||
        (control->state != PCIE1_CONTAINMENT_DMA_CPU_COMPLETE &&
         control->state != PCIE1_CONTAINMENT_DMA_FAILED))
        return false;
    dmb_ishld();
    *actual_out = control->actual;
    *success_out =
        control->state == PCIE1_CONTAINMENT_DMA_CPU_COMPLETE;
    return true;
}

static bool status_get_locked(
    const struct pcie1_containment *contract, u32 caller_core,
    struct pcie1_containment_status *status_out)
{
    u32 i;

    if (status_out)
        *status_out = (struct pcie1_containment_status){0};
    if (!contract || !status_out ||
        caller_core != PCIE1_CONTAINMENT_OWNER_CORE ||
        contract->owner.owner_core != PCIE1_CONTAINMENT_OWNER_CORE ||
        contract->owner.contract_id == 0U)
        return false;
    dmb_ishld();
    status_out->endpoint_generation = contract->owner.endpoint_generation;
    status_out->msi_event_seq = contract->msi.event_seq;
    status_out->msi_deadline_ms = contract->msi.deadline_ms;
    status_out->endpoint_state = contract->owner.state;
    status_out->bdf = contract->owner.bdf;
    status_out->fault = contract->owner.fault;
    status_out->fault_count = contract->owner.fault_count;
    status_out->quarantine_count = contract->owner.quarantine_count;
    status_out->msi_state = contract->msi.state;
    status_out->msi_posted = contract->msi.posted_count;
    status_out->msi_deferred = contract->msi.deferred_count;
    status_out->msi_storms = contract->msi.storm_count;
    for (i = 0U; i < PCIE1_CONTAINMENT_DMA_SLOT_COUNT; i++) {
        if (contract->dma[i].state != PCIE1_CONTAINMENT_DMA_FREE &&
            contract->dma[i].state != PCIE1_CONTAINMENT_DMA_RELEASED &&
            contract->dma[i].state != PCIE1_CONTAINMENT_DMA_RETIRED)
            status_out->active_dma++;
        if (contract->dma[i].state ==
            PCIE1_CONTAINMENT_DMA_DEVICE_OWNED)
            status_out->device_owned_dma++;
    }
    return true;
}

bool pcie1_containment_init(
    struct pcie1_containment *contract, u32 contract_id, u32 bdf,
    u64 endpoint_generation, u32 caller_core,
    struct pcie1_containment_endpoint_handle *endpoint_out)
{
    u64 irq_state = containment_irq_save();
    bool result = containment_init_locked(
        contract, contract_id, bdf, endpoint_generation, caller_core,
        endpoint_out);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_dma_acquire(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, u64 request_id, u32 requested,
    enum pcie1_containment_dma_direction direction,
    u64 now_ms, u64 timeout_ms,
    struct pcie1_containment_dma_handle *handle_out)
{
    u64 irq_state = containment_irq_save();
    bool result = dma_acquire_locked(
        contract, endpoint, caller_core, request_id, requested, direction,
        now_ms, timeout_ms, handle_out);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_dma_publish(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core)
{
    u64 irq_state = containment_irq_save();
    bool result = dma_publish_locked(
        contract, endpoint, handle, caller_core);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_completion_submit(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_msi_ticket *ticket,
    const struct pcie1_containment_completion *completion, u32 caller_core,
    bool guards_intact)
{
    u64 irq_state = containment_irq_save();
    bool result = completion_submit_locked(
        contract, endpoint, ticket, completion, caller_core, guards_intact);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_dma_cancel(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core)
{
    u64 irq_state = containment_irq_save();
    bool result = dma_cancel_locked(
        contract, endpoint, handle, caller_core);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_dma_expire(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle,
    u32 caller_core, u64 now_ms)
{
    u64 irq_state = containment_irq_save();
    bool result = dma_expire_locked(
        contract, endpoint, handle, caller_core, now_ms);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_dma_release(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core)
{
    u64 irq_state = containment_irq_save();
    bool result = dma_release_locked(
        contract, endpoint, handle, caller_core);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_msi_bind(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, u32 intid, u32 airq_source, u32 target_core,
    bool line_masked, bool airq_handler_registered)
{
    u64 irq_state = containment_irq_save();
    bool result = msi_bind_locked(
        contract, endpoint, caller_core, intid, airq_source, target_core,
        line_masked, airq_handler_registered);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_msi_arm(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core)
{
    u64 irq_state = containment_irq_save();
    bool result = msi_arm_locked(contract, endpoint, caller_core);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_msi_top_half(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, u32 intid, u32 cause, bool acknowledged,
    bool line_masked, u64 now_ms, u64 timeout_ms,
    struct pcie1_containment_msi_ticket *ticket_out)
{
    u64 irq_state = containment_irq_save();
    bool result = msi_top_half_locked(
        contract, endpoint, caller_core, intid, cause, acknowledged,
        line_masked, now_ms, timeout_ms, ticket_out);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_msi_publish_result(
    struct pcie1_containment *contract,
    const struct pcie1_containment_msi_ticket *ticket,
    u32 caller_core, bool airq_posted)
{
    u64 irq_state = containment_irq_save();
    bool result = msi_publish_result_locked(
        contract, ticket, caller_core, airq_posted);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_msi_dispatch_begin(
    struct pcie1_containment *contract,
    const struct pcie1_containment_msi_ticket *ticket, u32 caller_core)
{
    u64 irq_state = containment_irq_save();
    bool result = msi_dispatch_begin_locked(
        contract, ticket, caller_core);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_msi_pending_ticket(
    const struct pcie1_containment *contract, u32 caller_core,
    struct pcie1_containment_msi_ticket *ticket_out)
{
    u64 irq_state = containment_irq_save();
    bool result = msi_pending_ticket_locked(
        contract, caller_core, ticket_out);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_msi_dispatch_complete(
    struct pcie1_containment *contract,
    const struct pcie1_containment_msi_ticket *ticket, u32 caller_core)
{
    u64 irq_state = containment_irq_save();
    bool result = msi_dispatch_complete_locked(
        contract, ticket, caller_core);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_msi_expire(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, u64 now_ms)
{
    u64 irq_state = containment_irq_save();
    bool result = msi_expire_locked(
        contract, endpoint, caller_core, now_ms);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_quarantine(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core, enum pcie1_containment_fault fault)
{
    u64 irq_state = containment_irq_save();
    bool result = quarantine_locked(
        contract, endpoint, caller_core, fault);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_remove(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core)
{
    u64 irq_state = containment_irq_save();
    bool result = remove_locked(contract, endpoint, caller_core);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_dma_state_get(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core,
    enum pcie1_containment_dma_state *state_out)
{
    u64 irq_state = containment_irq_save();
    bool result = dma_state_get_locked(
        contract, handle, caller_core, state_out);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_dma_span_get(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core,
    struct pcie1_containment_dma_span *span_out)
{
    u64 irq_state = containment_irq_save();
    bool result = dma_span_get_locked(
        contract, handle, caller_core, span_out);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_dma_completion_get(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_dma_handle *handle, u32 caller_core,
    u32 *actual_out, bool *success_out)
{
    u64 irq_state = containment_irq_save();
    bool result = dma_completion_get_locked(
        contract, handle, caller_core, actual_out, success_out);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_status_get(
    const struct pcie1_containment *contract, u32 caller_core,
    struct pcie1_containment_status *status_out)
{
    u64 irq_state = containment_irq_save();
    bool result = status_get_locked(contract, caller_core, status_out);
    containment_irq_restore(irq_state);
    return result;
}

bool pcie1_containment_hardware_enable_allowed(
    const struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint,
    u32 caller_core)
{
    (void)contract;
    (void)endpoint;
    (void)caller_core;
    return false;
}
