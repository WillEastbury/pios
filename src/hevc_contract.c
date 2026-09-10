/*
 * hevc_contract.c - pure ADR-056 HEVC metadata and ownership contract.
 */
#include "types.h"
#include "hevc_contract.h"

#define HEVC_JOB_TOKEN_MAGIC          0x48564A4FULL
#define HEVC_FRAME_TOKEN_MAGIC        0x48564652ULL
#define HEVC_TOKEN_MAGIC_SHIFT        32U
#define HEVC_TOKEN_SLOT_MASK          0xFFULL
#define HEVC_TOKEN_ALLOWED_MASK       \
    ((0xFFFFFFFFULL << HEVC_TOKEN_MAGIC_SHIFT) | HEVC_TOKEN_SLOT_MASK)

static void hevc_clear_job_handle(struct hevc_job_handle *handle)
{
    if (!handle)
        return;
    handle->_token = 0U;
    handle->_generation = 0U;
    handle->_controller_id = 0U;
    handle->_reserved = 0U;
}

static void hevc_clear_frame_handle(struct hevc_frame_handle *handle)
{
    if (!handle)
        return;
    handle->_token = 0U;
    handle->_generation = 0U;
    handle->_controller_id = 0U;
    handle->_reserved = 0U;
}

static u64 hevc_token(u64 magic, u32 slot)
{
    return (magic << HEVC_TOKEN_MAGIC_SHIFT) | (u64)slot;
}

static void hevc_bump_generation(u64 *generation)
{
    (*generation)++;
    if (*generation == 0U)
        *generation = 1U;
}

static bool hevc_active_lease(
    const struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease)
{
    return contract && media_controller && engine_lease &&
           contract->owner.controller_id != 0U &&
           contract->owner.controller_id ==
           media_controller->owner.controller_id &&
           media_engine_lease_active_for(media_controller, engine_lease,
                                         MEDIA_ENGINE_HEVC);
}

static bool hevc_span_range(const struct hevc_dma_span *span,
                            u64 *actual_start_out, u64 *actual_end_out,
                            u64 *window_end_out)
{
    u64 actual_start;

    if (!span || !actual_start_out || !actual_end_out || !window_end_out ||
        span->allocation_id == 0U || span->allocation_generation == 0U ||
        span->dma_allocation_base == 0U || span->allocation_capacity == 0U ||
        span->window_capacity == 0U || span->used == 0U ||
        span->used > span->window_capacity ||
        span->allocation_offset >= span->allocation_capacity ||
        span->window_capacity >
        span->allocation_capacity - span->allocation_offset ||
        span->dma_allocation_base >= HEVC_DMA_ADDRESS_LIMIT ||
        span->allocation_capacity >
        HEVC_DMA_ADDRESS_LIMIT - span->dma_allocation_base)
        return false;
    actual_start = span->dma_allocation_base + span->allocation_offset;
    if (actual_start >= HEVC_DMA_ADDRESS_LIMIT ||
        span->used > HEVC_DMA_ADDRESS_LIMIT - actual_start)
        return false;
    *actual_start_out = actual_start;
    *actual_end_out = actual_start + span->used;
    *window_end_out = span->allocation_offset + span->window_capacity;
    return true;
}

static bool hevc_span_valid(const struct hevc_dma_span *span, bool capture,
                            u64 *actual_start_out, u64 *actual_end_out,
                            u64 *window_end_out)
{
    u64 actual_start;
    u32 expected_access = capture ? HEVC_DEVICE_WRITE : HEVC_DEVICE_READ;

    if (!span || span->_reserved0 != 0U || span->_reserved1 != 0U ||
        span->access != expected_access ||
        !hevc_span_range(span, &actual_start, actual_end_out, window_end_out))
        return false;
    if ((!capture && (actual_start & 3U) != 0U) ||
        (capture && ((actual_start & 15U) != 0U ||
                     span->output_canary == 0U)) ||
        (!capture && span->output_canary != 0U))
        return false;
    *actual_start_out = actual_start;
    return true;
}

static bool hevc_spans_valid(const struct hevc_dma_span *source,
                             const struct hevc_dma_span *capture)
{
    u64 source_start;
    u64 source_end;
    u64 source_window_end;
    u64 capture_start;
    u64 capture_end;
    u64 capture_window_end;

    if (!hevc_span_valid(source, false, &source_start, &source_end,
                         &source_window_end) ||
        !hevc_span_valid(capture, true, &capture_start, &capture_end,
                         &capture_window_end) ||
        (source_start < capture_end && capture_start < source_end))
        return false;
    if (source->allocation_id == capture->allocation_id &&
        source->allocation_generation == capture->allocation_generation) {
        if (source->dma_allocation_base != capture->dma_allocation_base ||
            source->allocation_capacity != capture->allocation_capacity ||
            (source->allocation_offset < capture_window_end &&
             capture->allocation_offset < source_window_end))
            return false;
    }
    return true;
}

static bool hevc_span_equal(const struct hevc_dma_span *left,
                            const struct hevc_dma_span *right)
{
    return left->allocation_id == right->allocation_id &&
           left->dma_allocation_base == right->dma_allocation_base &&
           left->allocation_capacity == right->allocation_capacity &&
           left->allocation_offset == right->allocation_offset &&
           left->window_capacity == right->window_capacity &&
           left->used == right->used &&
           left->output_canary == right->output_canary &&
           left->allocation_generation == right->allocation_generation &&
           left->access == right->access &&
           left->_reserved0 == right->_reserved0 &&
           left->_reserved1 == right->_reserved1;
}

static bool hevc_picture_valid(const struct hevc_picture_info *picture)
{
    if (!picture || picture->width < 16U || picture->width > 4096U ||
        picture->height < 16U || picture->height > 4096U ||
        picture->ctb_count == 0U || picture->ctb_count > HEVC_MAX_CTB_COUNT ||
        picture->chroma_format != HEVC_CHROMA_420 ||
        (picture->luma_bit_depth != 8U && picture->luma_bit_depth != 10U) ||
        picture->chroma_bit_depth != picture->luma_bit_depth ||
        picture->scaling_mode != HEVC_SCALING_DEFAULT ||
        picture->syntax_flags != 0U || picture->_reserved != 0U)
        return false;
    return true;
}

static bool hevc_slices_valid(const struct hevc_request *request)
{
    u32 i;

    if (!request || !request->slices || request->slice_count == 0U ||
        request->slice_count > HEVC_MAX_SLICES ||
        request->entry_offsets != NULL || request->entry_offset_count != 0U)
        return false;
    for (i = 0U; i < request->slice_count; i++) {
        const struct hevc_slice *slice = &request->slices[i];
        u64 source_bytes;

        if (slice->_reserved != 0U || slice->slice_type != HEVC_SLICE_I ||
            slice->reference_count != 0U ||
            slice->bit_length == 0U || slice->ctb_address >=
            request->picture.ctb_count ||
            slice->source_byte_offset >= request->source.used)
            return false;
        source_bytes = slice->bit_length / 8U;
        if ((slice->bit_length & 7U) != 0U)
            source_bytes++;
        if (source_bytes == 0U ||
            source_bytes > request->source.used - slice->source_byte_offset)
            return false;
    }
    return true;
}

static bool hevc_job_control(
    struct hevc_contract *contract, const struct hevc_job_handle *handle,
    struct hevc_job_control **control_out, u32 *slot_out)
{
    u32 slot;

    if (!contract || !handle || !control_out || !slot_out ||
        handle->_controller_id == 0U ||
        handle->_controller_id != contract->owner.controller_id ||
        handle->_reserved != 0U ||
        (handle->_token & ~HEVC_TOKEN_ALLOWED_MASK) != 0U ||
        (handle->_token >> HEVC_TOKEN_MAGIC_SHIFT) != HEVC_JOB_TOKEN_MAGIC)
        return false;
    slot = (u32)(handle->_token & HEVC_TOKEN_SLOT_MASK);
    if (slot >= HEVC_JOB_CAPACITY || handle->_generation == 0U ||
        contract->jobs[slot].generation != handle->_generation ||
        contract->jobs[slot].state == HEVC_JOB_FREE)
        return false;
    dmb_ishld();
    *control_out = &contract->jobs[slot];
    *slot_out = slot;
    return true;
}

static bool hevc_job_control_const(
    const struct hevc_contract *contract, const struct hevc_job_handle *handle,
    const struct hevc_job_control **control_out, u32 *slot_out)
{
    struct hevc_job_control *control;

    if (!hevc_job_control((struct hevc_contract *)contract, handle, &control,
                          slot_out))
        return false;
    *control_out = control;
    return true;
}

static bool hevc_frame_control(
    struct hevc_contract *contract, const struct hevc_frame_handle *handle,
    struct hevc_frame_control **control_out, u32 *slot_out)
{
    u32 slot;

    if (!contract || !handle || !control_out || !slot_out ||
        handle->_controller_id == 0U ||
        handle->_controller_id != contract->owner.controller_id ||
        handle->_reserved != 0U ||
        (handle->_token & ~HEVC_TOKEN_ALLOWED_MASK) != 0U ||
        (handle->_token >> HEVC_TOKEN_MAGIC_SHIFT) != HEVC_FRAME_TOKEN_MAGIC)
        return false;
    slot = (u32)(handle->_token & HEVC_TOKEN_SLOT_MASK);
    if (slot >= HEVC_FRAME_CAPACITY || handle->_generation == 0U ||
        contract->frames[slot].generation != handle->_generation ||
        contract->frames[slot].state == HEVC_FRAME_FREE)
        return false;
    dmb_ishld();
    *control_out = &contract->frames[slot];
    *slot_out = slot;
    return true;
}

static bool hevc_frame_control_const(
    const struct hevc_contract *contract,
    const struct hevc_frame_handle *handle,
    const struct hevc_frame_control **control_out, u32 *slot_out)
{
    struct hevc_frame_control *control;

    if (!hevc_frame_control((struct hevc_contract *)contract, handle, &control,
                            slot_out))
        return false;
    *control_out = control;
    return true;
}

static void hevc_publish_job_state(struct hevc_job_control *control, u32 state)
{
    dmb_ishst();
    control->state = state;
    dmb_ishst();
}

static void hevc_publish_frame_state(struct hevc_frame_control *control,
                                     u32 state)
{
    dmb_ishst();
    control->state = state;
    dmb_ishst();
}

static bool hevc_job_frame(struct hevc_contract *contract, u32 job_slot,
                           struct hevc_frame_control **frame_out)
{
    const struct hevc_job_control *job = &contract->jobs[job_slot];
    u32 slot = job->output_slot;

    if (slot >= HEVC_FRAME_CAPACITY ||
        contract->frames[slot].generation != job->output_generation ||
        contract->frames[slot].state == HEVC_FRAME_FREE)
        return false;
    *frame_out = &contract->frames[slot];
    return true;
}

bool hevc_contract_init(struct hevc_contract *contract, u32 controller_id)
{
    u32 i;

    if (!contract || controller_id == 0U || contract->owner.controller_id != 0U)
        return false;
    memset(contract, 0, sizeof(*contract));
    contract->owner.controller_id = controller_id;
    for (i = 0U; i < HEVC_JOB_CAPACITY; i++)
        contract->jobs[i].generation = (u64)i + 1U;
    for (i = 0U; i < HEVC_FRAME_CAPACITY; i++)
        contract->frames[i].generation = (u64)i + 1U;
    return true;
}

bool hevc_contract_register_capture(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_dma_span *capture,
    struct hevc_frame_handle *frame_out)
{
    u32 i;
    u64 start;
    u64 end;
    u64 window_end;

    hevc_clear_frame_handle(frame_out);
    if (!contract || !frame_out || !hevc_active_lease(contract, media_controller,
        engine_lease) || !hevc_span_valid(capture, true, &start, &end,
                                          &window_end))
        return false;
    for (i = 0U; i < HEVC_FRAME_CAPACITY; i++) {
        if (contract->frames[i].state == HEVC_FRAME_FREE)
            break;
    }
    if (i == HEVC_FRAME_CAPACITY)
        return false;
    memcpy(&contract->frame_payloads[i].capture, capture, sizeof(*capture));
    contract->frames[i].job_users = 0U;
    contract->frames[i].producing_jobs = 0U;
    contract->frames[i].reference_users = 0U;
    hevc_publish_frame_state(&contract->frames[i], HEVC_FRAME_CAPTURE_READY);
    frame_out->_token = hevc_token(HEVC_FRAME_TOKEN_MAGIC, i);
    frame_out->_generation = contract->frames[i].generation;
    frame_out->_controller_id = contract->owner.controller_id;
    frame_out->_reserved = 0U;
    return true;
}

bool hevc_contract_prepare(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_request *request,
    struct hevc_job_handle *job_out)
{
    struct hevc_frame_control *frame;
    struct hevc_job_payload *payload;
    u32 frame_slot;
    u32 job_slot;
    u32 i;

    hevc_clear_job_handle(job_out);
    if (!contract || !job_out || !request || request->layout_version !=
        HEVC_REQUEST_LAYOUT_V1 || request->_reserved != 0U ||
        !hevc_active_lease(contract, media_controller, engine_lease) ||
        !hevc_picture_valid(&request->picture) ||
        !hevc_spans_valid(&request->source, &request->capture) ||
        !hevc_slices_valid(request) ||
        !hevc_frame_control(contract, &request->capture_frame, &frame,
                            &frame_slot) ||
        frame->state != HEVC_FRAME_CAPTURE_READY ||
        frame->job_users != 0U || frame->producing_jobs != 0U ||
        !hevc_span_equal(&request->capture,
                         &contract->frame_payloads[frame_slot].capture))
        return false;
    for (job_slot = 0U; job_slot < HEVC_JOB_CAPACITY; job_slot++) {
        if (contract->jobs[job_slot].state == HEVC_JOB_FREE)
            break;
    }
    if (job_slot == HEVC_JOB_CAPACITY)
        return false;
    payload = &contract->job_payloads[job_slot];
    memset(payload, 0, sizeof(*payload));
    payload->picture = request->picture;
    memcpy(&payload->source, &request->source, sizeof(payload->source));
    memcpy(&payload->capture, &request->capture, sizeof(payload->capture));
    payload->output_canary = request->capture.output_canary;
    payload->slice_count = request->slice_count;
    for (i = 0U; i < request->slice_count; i++) {
        payload->slices[i].source_byte_offset =
            request->slices[i].source_byte_offset;
        payload->slices[i].bit_length = request->slices[i].bit_length;
        payload->slices[i].ctb_address = request->slices[i].ctb_address;
        payload->slices[i].slice_type = request->slices[i].slice_type;
    }
    contract->jobs[job_slot].canary_intact = 0U;
    contract->jobs[job_slot].output_slot = frame_slot;
    contract->jobs[job_slot].output_generation = frame->generation;
    frame->job_users++;
    frame->producing_jobs++;
    hevc_publish_frame_state(frame, HEVC_FRAME_PRODUCING);
    hevc_publish_job_state(&contract->jobs[job_slot], HEVC_JOB_PREPARED);
    job_out->_token = hevc_token(HEVC_JOB_TOKEN_MAGIC, job_slot);
    job_out->_generation = contract->jobs[job_slot].generation;
    job_out->_controller_id = contract->owner.controller_id;
    job_out->_reserved = 0U;
    return true;
}

bool hevc_contract_mark_in_flight(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_job_handle *job)
{
    struct hevc_job_control *control;
    u32 slot;

    if (!hevc_active_lease(contract, media_controller, engine_lease) ||
        !hevc_job_control(contract, job, &control, &slot) ||
        control->state != HEVC_JOB_PREPARED)
        return false;
    hevc_publish_job_state(control, HEVC_JOB_IN_FLIGHT);
    return true;
}

bool hevc_contract_complete(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_job_handle *job, bool success, bool retain)
{
    struct hevc_job_control *control;
    struct hevc_frame_control *frame;
    u32 slot;

    if ((success != false && success != true) ||
        (retain != false && retain != true) ||
        (!success && retain) ||
        !hevc_active_lease(contract, media_controller, engine_lease) ||
        !hevc_job_control(contract, job, &control, &slot) ||
        control->state != HEVC_JOB_IN_FLIGHT ||
        !hevc_job_frame(contract, slot, &frame) ||
        frame->producing_jobs != 1U)
        return false;
    frame->producing_jobs = 0U;
    hevc_publish_job_state(control, success ? HEVC_JOB_COMPLETE :
                           HEVC_JOB_FAILED);
    hevc_publish_frame_state(frame, success && retain ?
                             HEVC_FRAME_REFERENCEABLE :
                             HEVC_FRAME_CAPTURE_READY);
    return true;
}

bool hevc_contract_report_output_canary(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_job_handle *job, u64 observed_canary)
{
    struct hevc_job_control *control;
    u32 slot;

    if (!hevc_active_lease(contract, media_controller, engine_lease) ||
        !hevc_job_control(contract, job, &control, &slot) ||
        control->state != HEVC_JOB_COMPLETE || control->canary_intact != 0U ||
        observed_canary == 0U ||
        observed_canary != contract->job_payloads[slot].output_canary)
        return false;
    control->canary_intact = 1U;
    dmb_ishst();
    return true;
}

bool hevc_contract_abort(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_job_handle *job)
{
    struct hevc_job_control *control;
    struct hevc_frame_control *frame;
    u32 slot;

    if (!hevc_active_lease(contract, media_controller, engine_lease) ||
        !hevc_job_control(contract, job, &control, &slot) ||
        (control->state != HEVC_JOB_PREPARED &&
         control->state != HEVC_JOB_IN_FLIGHT) ||
        !hevc_job_frame(contract, slot, &frame) ||
        frame->job_users != 1U || frame->producing_jobs != 1U)
        return false;
    frame->producing_jobs = 0U;
    hevc_publish_frame_state(frame, HEVC_FRAME_CAPTURE_READY);
    hevc_publish_job_state(control, HEVC_JOB_FAILED);
    return true;
}

bool hevc_contract_release_job(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_job_handle *job)
{
    struct hevc_job_control *control;
    struct hevc_frame_control *frame;
    u32 slot;

    if (!hevc_active_lease(contract, media_controller, engine_lease) ||
        !hevc_job_control(contract, job, &control, &slot) ||
        (control->state != HEVC_JOB_COMPLETE &&
         control->state != HEVC_JOB_FAILED) ||
        (control->state == HEVC_JOB_COMPLETE &&
         control->canary_intact == 0U) ||
        !hevc_job_frame(contract, slot, &frame) || frame->job_users != 1U ||
        frame->producing_jobs != 0U)
        return false;
    frame->job_users = 0U;
    hevc_publish_job_state(control, HEVC_JOB_RELEASED);
    hevc_bump_generation(&control->generation);
    dmb_ishst();
    memset(&contract->job_payloads[slot], 0,
           sizeof(contract->job_payloads[slot]));
    control->canary_intact = 0U;
    control->output_slot = 0U;
    control->output_generation = 0U;
    hevc_publish_job_state(control, HEVC_JOB_FREE);
    return true;
}

bool hevc_contract_release_frame(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_frame_handle *frame)
{
    struct hevc_frame_control *control;
    u32 slot;

    if (!hevc_active_lease(contract, media_controller, engine_lease) ||
        !hevc_frame_control(contract, frame, &control, &slot) ||
        (control->state != HEVC_FRAME_CAPTURE_READY &&
         control->state != HEVC_FRAME_REFERENCEABLE) ||
        control->job_users != 0U || control->producing_jobs != 0U ||
        control->reference_users != 0U)
        return false;
    hevc_publish_frame_state(control, HEVC_FRAME_RELEASED);
    hevc_bump_generation(&control->generation);
    dmb_ishst();
    memset(&contract->frame_payloads[slot], 0,
           sizeof(contract->frame_payloads[slot]));
    hevc_publish_frame_state(control, HEVC_FRAME_FREE);
    return true;
}

bool hevc_contract_job_state_get(const struct hevc_contract *contract,
                                 const struct hevc_job_handle *job,
                                 enum hevc_job_state *state_out)
{
    const struct hevc_job_control *control;
    u32 slot;

    if (!state_out || !hevc_job_control_const(contract, job, &control, &slot))
        return false;
    dmb_ishld();
    *state_out = (enum hevc_job_state)control->state;
    return true;
}

bool hevc_contract_frame_state_get(const struct hevc_contract *contract,
                                   const struct hevc_frame_handle *frame,
                                   enum hevc_frame_state *state_out)
{
    const struct hevc_frame_control *control;
    u32 slot;

    if (!state_out || !hevc_frame_control_const(contract, frame, &control,
                                                 &slot))
        return false;
    dmb_ishld();
    *state_out = (enum hevc_frame_state)control->state;
    return true;
}

bool hevc_contract_picture_get(const struct hevc_contract *contract,
                               const struct hevc_job_handle *job,
                               struct hevc_picture_info *picture_out)
{
    const struct hevc_job_control *control;
    u32 slot;

    if (!picture_out || !hevc_job_control_const(contract, job, &control,
                                                 &slot))
        return false;
    dmb_ishld();
    *picture_out = contract->job_payloads[slot].picture;
    return true;
}

bool hevc_contract_slice_get(const struct hevc_contract *contract,
                             const struct hevc_job_handle *job, u32 index,
                             struct hevc_slice_payload *slice_out)
{
    const struct hevc_job_control *control;
    u32 slot;

    if (!slice_out || !hevc_job_control_const(contract, job, &control,
                                               &slot) ||
        index >= contract->job_payloads[slot].slice_count)
        return false;
    dmb_ishld();
    *slice_out = contract->job_payloads[slot].slices[index];
    return true;
}
