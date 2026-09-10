/*
 * pisp_be_contract.c - pure ADR-055 PiSP-BE request/lifetime contract.
 */
#include "types.h"
#include "pisp_be_contract.h"

#define PISP_BE_TOKEN_SLOT_MASK         0xFFULL
#define PISP_BE_TOKEN_MAGIC             0xB55EULL
#define PISP_BE_TOKEN_MAGIC_SHIFT       48U
#define PISP_BE_TOKEN_ALLOWED_MASK      \
    ((PISP_BE_TOKEN_MAGIC << PISP_BE_TOKEN_MAGIC_SHIFT) | \
     PISP_BE_TOKEN_SLOT_MASK)

static u32 pisp_be_load_le32(const u8 *bytes)
{
    return (u32)bytes[0] | ((u32)bytes[1] << 8) |
           ((u32)bytes[2] << 16) | ((u32)bytes[3] << 24);
}

static void pisp_be_clear_handle(struct pisp_be_job_handle *handle)
{
    if (!handle)
        return;
    handle->_token = 0U;
    handle->_generation = 0U;
    handle->_controller_id = 0U;
    handle->_reserved = 0U;
}

static u64 pisp_be_make_token(u32 slot)
{
    return (PISP_BE_TOKEN_MAGIC << PISP_BE_TOKEN_MAGIC_SHIFT) |
           (u64)slot;
}

static bool pisp_be_active_lease(
    const struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease)
{
    return contract && media_controller && engine_lease &&
           contract->owner.controller_id != 0U &&
           contract->owner.controller_id ==
           media_controller->owner.controller_id &&
           media_engine_lease_active_for(media_controller, engine_lease,
                                         MEDIA_ENGINE_PISP_BE);
}

static bool pisp_be_handle_control(
    struct pisp_be_contract *contract,
    const struct pisp_be_job_handle *handle,
    struct pisp_be_job_control **control_out, u32 *slot_out)
{
    u32 slot;
    u64 generation;

    if (!contract || !handle || !control_out || !slot_out ||
        handle->_controller_id == 0U ||
        handle->_controller_id != contract->owner.controller_id ||
        handle->_reserved != 0U ||
        (handle->_token & ~PISP_BE_TOKEN_ALLOWED_MASK) != 0U ||
        (handle->_token >> PISP_BE_TOKEN_MAGIC_SHIFT) != PISP_BE_TOKEN_MAGIC)
        return false;
    slot = (u32)(handle->_token & PISP_BE_TOKEN_SLOT_MASK);
    generation = handle->_generation;
    if (slot >= PISP_BE_JOB_CAPACITY || generation == 0U ||
        contract->jobs[slot].generation != generation ||
        contract->jobs[slot].state == PISP_BE_JOB_FREE)
        return false;
    dmb_ishld();
    *control_out = &contract->jobs[slot];
    *slot_out = slot;
    return true;
}

static bool pisp_be_handle_control_const(
    const struct pisp_be_contract *contract,
    const struct pisp_be_job_handle *handle,
    const struct pisp_be_job_control **control_out, u32 *slot_out)
{
    u32 slot;
    u64 generation;

    if (!contract || !handle || !control_out || !slot_out ||
        handle->_controller_id == 0U ||
        handle->_controller_id != contract->owner.controller_id ||
        handle->_reserved != 0U ||
        (handle->_token & ~PISP_BE_TOKEN_ALLOWED_MASK) != 0U ||
        (handle->_token >> PISP_BE_TOKEN_MAGIC_SHIFT) != PISP_BE_TOKEN_MAGIC)
        return false;
    slot = (u32)(handle->_token & PISP_BE_TOKEN_SLOT_MASK);
    generation = handle->_generation;
    if (slot >= PISP_BE_JOB_CAPACITY || generation == 0U ||
        contract->jobs[slot].generation != generation ||
        contract->jobs[slot].state == PISP_BE_JOB_FREE)
        return false;
    dmb_ishld();
    *control_out = &contract->jobs[slot];
    *slot_out = slot;
    return true;
}

static void pisp_be_publish_state(struct pisp_be_job_control *control, u32 state)
{
    dmb_ishst();
    control->state = state;
    dmb_ishst();
}

static bool pisp_be_span_role_valid(u32 role)
{
    return role == PISP_BE_SPAN_CONFIG ||
           role == PISP_BE_SPAN_MAIN_INPUT ||
           role == PISP_BE_SPAN_AUX_INPUT ||
           role == PISP_BE_SPAN_OUTPUT;
}

static bool pisp_be_span_range(const struct pisp_be_dma_span *span,
                               u64 *actual_start_out, u64 *actual_end_out,
                               u64 *window_end_out)
{
    u64 actual_start;
    u64 actual_end;
    u64 window_end;

    if (!span || !actual_start_out || !actual_end_out || !window_end_out ||
        span->allocation_id == 0U || span->allocation_generation == 0U ||
        span->dma_allocation_base == 0U || span->allocation_capacity == 0U ||
        span->window_capacity == 0U || span->used == 0U ||
        span->used > span->window_capacity ||
        span->allocation_offset >= span->allocation_capacity ||
        span->window_capacity >
        span->allocation_capacity - span->allocation_offset ||
        span->dma_allocation_base >= PISP_BE_DMA_ADDRESS_LIMIT ||
        span->allocation_capacity >
        PISP_BE_DMA_ADDRESS_LIMIT - span->dma_allocation_base)
        return false;
    actual_start = span->dma_allocation_base + span->allocation_offset;
    if (actual_start >= PISP_BE_DMA_ADDRESS_LIMIT ||
        span->used > PISP_BE_DMA_ADDRESS_LIMIT - actual_start)
        return false;
    actual_end = actual_start + span->used;
    window_end = span->allocation_offset + span->window_capacity;
    *actual_start_out = actual_start;
    *actual_end_out = actual_end;
    *window_end_out = window_end;
    return true;
}

static bool pisp_be_span_valid(const struct pisp_be_dma_span *span,
                               u64 *actual_start_out, u64 *actual_end_out,
                               u64 *window_end_out)
{
    u64 actual_start;

    if (!span || span->_reserved != 0U || !pisp_be_span_role_valid(
        span->logical_role) || span->access == 0U ||
        (span->access & ~(PISP_BE_DEVICE_READ | PISP_BE_DEVICE_WRITE)) != 0U ||
        !pisp_be_span_range(span, &actual_start, actual_end_out, window_end_out))
        return false;

    if (span->logical_role == PISP_BE_SPAN_CONFIG) {
        if ((span->access & PISP_BE_DEVICE_READ) == 0U ||
            span->used != PISP_BE_CONFIG_BYTES || span->output_canary != 0U ||
            (actual_start & 3U) != 0U)
            return false;
    } else if (span->logical_role == PISP_BE_SPAN_MAIN_INPUT ||
               span->logical_role == PISP_BE_SPAN_AUX_INPUT) {
        if ((span->access & PISP_BE_DEVICE_READ) == 0U ||
            span->output_canary != 0U || (actual_start & 3U) != 0U)
            return false;
    } else {
        if ((span->access & PISP_BE_DEVICE_WRITE) == 0U ||
            span->output_canary == 0U || (actual_start & 15U) != 0U)
            return false;
    }
    *actual_start_out = actual_start;
    return true;
}

static bool pisp_be_spans_valid(const struct pisp_be_dma_span *spans,
                                u32 span_count)
{
    u32 config_count = 0U;
    u32 main_input_count = 0U;
    u32 output_count = 0U;
    u32 i;
    u32 j;

    if (!spans || span_count == 0U || span_count > PISP_BE_MAX_SPANS)
        return false;
    for (i = 0U; i < span_count; i++) {
        u64 start_i;
        u64 end_i;
        u64 window_end_i;

        if (!pisp_be_span_valid(&spans[i], &start_i, &end_i, &window_end_i))
            return false;
        if (spans[i].logical_role == PISP_BE_SPAN_CONFIG)
            config_count++;
        else if (spans[i].logical_role == PISP_BE_SPAN_MAIN_INPUT)
            main_input_count++;
        else if (spans[i].logical_role == PISP_BE_SPAN_OUTPUT)
            output_count++;
        for (j = 0U; j < i; j++) {
            u64 start_j;
            u64 end_j;
            u64 window_end_j;

            if (!pisp_be_span_valid(&spans[j], &start_j, &end_j, &window_end_j))
                return false;
            if (start_i < end_j && start_j < end_i)
                return false;
            if (spans[i].allocation_id == spans[j].allocation_id &&
                spans[i].allocation_generation == spans[j].allocation_generation) {
                if (spans[i].dma_allocation_base !=
                    spans[j].dma_allocation_base ||
                    spans[i].allocation_capacity !=
                    spans[j].allocation_capacity ||
                    (spans[i].allocation_offset < window_end_j &&
                     spans[j].allocation_offset < window_end_i))
                    return false;
            }
        }
    }
    return config_count == 1U && main_input_count != 0U && output_count != 0U;
}

static bool pisp_be_config_valid(const struct pisp_be_request *request)
{
    const u8 *config;
    u32 num_tiles;
    u32 bayer;
    u32 rgb;

    if (!request || request->layout_version != PISP_BE_REQUEST_LAYOUT_V1 ||
        !request->config_bytes || request->config_bytes_len != PISP_BE_CONFIG_BYTES ||
        request->_reserved != 0U)
        return false;
    config = (const u8 *)request->config_bytes;
    num_tiles = pisp_be_load_le32(config + PISP_BE_CONFIG_NUM_TILES_OFFSET);
    bayer = pisp_be_load_le32(config + PISP_BE_CONFIG_GLOBAL_BAYER_OFFSET);
    rgb = pisp_be_load_le32(config + PISP_BE_CONFIG_GLOBAL_RGB_OFFSET);
    return num_tiles >= 1U && num_tiles <= PISP_BE_CONFIG_INLINE_TILE_COUNT &&
           ((bayer & PISP_BE_CONFIG_ROOT_BIT) != 0U) !=
           ((rgb & PISP_BE_CONFIG_ROOT_BIT) != 0U);
}

bool pisp_be_contract_init(struct pisp_be_contract *contract, u32 controller_id)
{
    u32 i;

    if (!contract || controller_id == 0U)
        return false;
    contract->owner.controller_id = controller_id;
    for (i = 0U; i < PISP_BE_JOB_CAPACITY; i++) {
        contract->jobs[i].generation = (u64)i + 1U;
        contract->jobs[i].state = PISP_BE_JOB_FREE;
        contract->jobs[i].canaries_intact = 0U;
    }
    return true;
}

bool pisp_be_contract_prepare(
    struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct pisp_be_request *request,
    const struct pisp_be_dma_span *spans, u32 span_count,
    struct pisp_be_job_handle *handle_out)
{
    u32 i;
    struct pisp_be_job_control *control;
    struct pisp_be_job_payload *payload;

    pisp_be_clear_handle(handle_out);
    if (!contract || !handle_out || !pisp_be_active_lease(contract,
        media_controller, engine_lease) || !pisp_be_config_valid(request) ||
        !pisp_be_spans_valid(spans, span_count))
        return false;
    for (i = 0U; i < PISP_BE_JOB_CAPACITY; i++) {
        if (contract->jobs[i].state == PISP_BE_JOB_FREE)
            break;
    }
    if (i == PISP_BE_JOB_CAPACITY)
        return false;
    control = &contract->jobs[i];
    payload = &contract->payloads[i];
    memcpy(payload->config, request->config_bytes, PISP_BE_CONFIG_BYTES);
    memcpy(payload->spans, spans, (usize)span_count * sizeof(spans[0]));
    payload->span_count = span_count;
    payload->_reserved = 0U;
    control->canaries_intact = 0U;
    pisp_be_publish_state(control, PISP_BE_JOB_PREPARED);
    handle_out->_token = pisp_be_make_token(i);
    handle_out->_generation = control->generation;
    handle_out->_controller_id = contract->owner.controller_id;
    handle_out->_reserved = 0U;
    return true;
}

bool pisp_be_contract_mark_in_flight(
    struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct pisp_be_job_handle *handle)
{
    struct pisp_be_job_control *control;
    u32 slot;

    if (!pisp_be_active_lease(contract, media_controller, engine_lease) ||
        !pisp_be_handle_control(contract, handle, &control, &slot) ||
        control->state != PISP_BE_JOB_PREPARED)
        return false;
    pisp_be_publish_state(control, PISP_BE_JOB_IN_FLIGHT);
    return true;
}

bool pisp_be_contract_complete(
    struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct pisp_be_job_handle *handle)
{
    struct pisp_be_job_control *control;
    u32 slot;

    if (!pisp_be_active_lease(contract, media_controller, engine_lease) ||
        !pisp_be_handle_control(contract, handle, &control, &slot) ||
        control->state != PISP_BE_JOB_IN_FLIGHT)
        return false;
    pisp_be_publish_state(control, PISP_BE_JOB_COMPLETE);
    return true;
}

bool pisp_be_contract_report_output_canaries(
    struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct pisp_be_job_handle *handle,
    const struct pisp_be_output_canary *reports, u32 report_count)
{
    struct pisp_be_job_control *control;
    struct pisp_be_job_payload *payload;
    u32 output_count = 0U;
    u32 seen = 0U;
    u32 i;
    u32 slot;

    if (!pisp_be_active_lease(contract, media_controller, engine_lease) ||
        !pisp_be_handle_control(contract, handle, &control, &slot) ||
        control->state != PISP_BE_JOB_COMPLETE ||
        control->canaries_intact != 0U || !reports)
        return false;
    dmb_ishld();
    payload = &contract->payloads[slot];
    for (i = 0U; i < payload->span_count; i++) {
        if (payload->spans[i].logical_role == PISP_BE_SPAN_OUTPUT)
            output_count++;
    }
    if (report_count != output_count)
        return false;
    for (i = 0U; i < report_count; i++) {
        const struct pisp_be_output_canary *report = &reports[i];
        u32 bit;

        if (report->_reserved != 0U || report->span_index >= payload->span_count)
            return false;
        bit = 1U << report->span_index;
        if ((seen & bit) != 0U ||
            payload->spans[report->span_index].logical_role !=
            PISP_BE_SPAN_OUTPUT ||
            report->observed_canary !=
            payload->spans[report->span_index].output_canary)
            return false;
        seen |= bit;
    }
    control->canaries_intact = 1U;
    dmb_ishst();
    return true;
}

bool pisp_be_contract_abort(
    struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct pisp_be_job_handle *handle)
{
    struct pisp_be_job_control *control;
    u32 slot;

    if (!pisp_be_active_lease(contract, media_controller, engine_lease) ||
        !pisp_be_handle_control(contract, handle, &control, &slot) ||
        (control->state != PISP_BE_JOB_PREPARED &&
         control->state != PISP_BE_JOB_IN_FLIGHT))
        return false;
    pisp_be_publish_state(control, PISP_BE_JOB_FAILED);
    return true;
}

bool pisp_be_contract_release(
    struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct pisp_be_job_handle *handle)
{
    struct pisp_be_job_control *control;
    u32 slot;

    if (!pisp_be_active_lease(contract, media_controller, engine_lease) ||
        !pisp_be_handle_control(contract, handle, &control, &slot) ||
        (control->state != PISP_BE_JOB_COMPLETE &&
         control->state != PISP_BE_JOB_FAILED) ||
        (control->state == PISP_BE_JOB_COMPLETE &&
         control->canaries_intact == 0U))
        return false;
    pisp_be_publish_state(control, PISP_BE_JOB_RELEASED);
    control->generation++;
    if (control->generation == 0U)
        control->generation = 1U;
    dmb_ishst();
    memset(&contract->payloads[slot], 0, sizeof(contract->payloads[slot]));
    control->canaries_intact = 0U;
    pisp_be_publish_state(control, PISP_BE_JOB_FREE);
    return true;
}

bool pisp_be_contract_state_get(
    const struct pisp_be_contract *contract,
    const struct pisp_be_job_handle *handle,
    enum pisp_be_job_state *state_out)
{
    const struct pisp_be_job_control *control;
    u32 slot;

    if (!state_out || !pisp_be_handle_control_const(contract, handle, &control,
                                                     &slot))
        return false;
    dmb_ishld();
    *state_out = (enum pisp_be_job_state)control->state;
    return true;
}

bool pisp_be_contract_config_byte(
    const struct pisp_be_contract *contract,
    const struct pisp_be_job_handle *handle, u32 offset, u8 *byte_out)
{
    const struct pisp_be_job_control *control;
    u32 slot;

    if (!byte_out || offset >= PISP_BE_CONFIG_BYTES ||
        !pisp_be_handle_control_const(contract, handle, &control, &slot))
        return false;
    dmb_ishld();
    *byte_out = contract->payloads[slot].config[offset];
    return true;
}
