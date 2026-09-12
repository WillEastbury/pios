#include <stdio.h>
#include <string.h>

#include "types.h"
#include "media_engine_contract.h"
#include "hevc_contract.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static void make_span(struct hevc_dma_span *span, u64 id, u64 base,
                      u64 used, u32 access, u64 canary)
{
    memset(span, 0, sizeof(*span));
    span->allocation_id = id;
    span->dma_allocation_base = base;
    span->allocation_capacity = 0x1000U;
    span->allocation_offset = 0U;
    span->window_capacity = 0x400U;
    span->used = used;
    span->output_canary = canary;
    span->allocation_generation = (u32)id;
    span->access = access;
}

static void make_request(struct hevc_request *request,
                         struct hevc_slice slices[HEVC_MAX_SLICES],
                         const struct hevc_frame_handle *frame)
{
    memset(request, 0, sizeof(*request));
    memset(slices, 0, sizeof(struct hevc_slice) * HEVC_MAX_SLICES);
    request->layout_version = HEVC_REQUEST_LAYOUT_V1;
    request->picture.width = 1920U;
    request->picture.height = 1080U;
    request->picture.ctb_count = 510U;
    request->picture.chroma_format = HEVC_CHROMA_420;
    request->picture.luma_bit_depth = 8U;
    request->picture.chroma_bit_depth = 8U;
    request->picture.scaling_mode = HEVC_SCALING_DEFAULT;
    request->slices = slices;
    request->slice_count = 1U;
    slices[0].source_byte_offset = 0U;
    slices[0].bit_length = 8U;
    slices[0].ctb_address = 0U;
    slices[0].slice_type = HEVC_SLICE_I;
    make_span(&request->source, 1U, 0x10000U, 64U, HEVC_DEVICE_READ, 0U);
    make_span(&request->capture, 2U, 0x20000U, 128U, HEVC_DEVICE_WRITE,
              0xCAFE1001U);
    request->capture_frame = *frame;
}

static void init_engine(struct media_engine_controller *controller,
                        struct media_engine_lease *lease, u32 id)
{
    memset(controller, 0, sizeof(*controller));
    memset(lease, 0, sizeof(*lease));
    CHECK(media_engine_controller_init(controller, id));
    CHECK(media_engine_passive_identify(controller, MEDIA_ENGINE_HEVC,
                                        0x00000202U));
    CHECK(media_engine_lease_acquire(controller, MEDIA_ENGINE_HEVC, lease));
    CHECK(media_engine_lease_active_for(controller, lease, MEDIA_ENGINE_HEVC));
}

static void init_contract(struct hevc_contract *contract,
                          struct media_engine_controller *controller,
                          struct media_engine_lease *lease, u32 id)
{
    memset(contract, 0, sizeof(*contract));
    CHECK(hevc_contract_init(contract, id));
    init_engine(controller, lease, id);
}

static bool register_capture(struct hevc_contract *contract,
                             struct media_engine_controller *controller,
                             struct media_engine_lease *lease,
                             struct hevc_frame_handle *frame)
{
    struct hevc_dma_span capture;

    make_span(&capture, 2U, 0x20000U, 128U, HEVC_DEVICE_WRITE, 0xCAFE1001U);
    return hevc_contract_register_capture(contract, controller, lease, &capture,
                                          frame);
}

static bool prepare(struct hevc_contract *contract,
                    struct media_engine_controller *controller,
                    struct media_engine_lease *lease,
                    struct hevc_request *request,
                    struct hevc_job_handle *job)
{
    return hevc_contract_prepare(contract, controller, lease, request, job);
}

static void test_abi_and_init(void)
{
    struct hevc_contract contract;

    CHECK(HEVC_REQUEST_LAYOUT_V1 == 1U);
    CHECK(HEVC_JOB_CAPACITY == 4U);
    CHECK(HEVC_FRAME_CAPACITY == 16U);
    CHECK(HEVC_MAX_SLICES == 16U);
    CHECK(HEVC_MAX_ENTRY_OFFSETS == 64U);
    CHECK(HEVC_DMA_ADDRESS_BITS == 36U);
    CHECK(HEVC_DMA_ADDRESS_LIMIT == 0x1000000000ULL);
    CHECK(HEVC_MAX_CTB_COUNT == 65536U);
    CHECK(sizeof(struct hevc_job_control) == 64U);
    CHECK(sizeof(struct hevc_frame_control) == 64U);
    CHECK(((usize)&((struct hevc_contract *)0)->frames -
           (usize)&((struct hevc_contract *)0)->jobs) ==
          HEVC_JOB_CAPACITY * 64U);
    CHECK(((usize)&((struct hevc_contract *)0)->job_payloads -
           (usize)&((struct hevc_contract *)0)->frames) ==
          HEVC_FRAME_CAPACITY * 64U);
    CHECK(!hevc_contract_init(NULL, 1U));
    memset(&contract, 0, sizeof(contract));
    CHECK(!hevc_contract_init(&contract, 0U));
    CHECK(hevc_contract_init(&contract, 99U));
    CHECK(!hevc_contract_init(&contract, 99U));
    CHECK(contract.owner.controller_id == 99U);
    CHECK(contract.jobs[0].generation == 1U);
    CHECK(contract.jobs[3].generation == 4U);
    CHECK(contract.frames[0].generation == 1U);
    CHECK(contract.frames[15].generation == 16U);
}

static void test_metadata_validation(void)
{
    struct hevc_contract contract;
    struct media_engine_controller controller;
    struct media_engine_lease lease;
    struct hevc_frame_handle frame;
    struct hevc_job_handle job;
    struct hevc_request request;
    struct hevc_slice slices[HEVC_MAX_SLICES];
    u32 i;

    init_contract(&contract, &controller, &lease, 10U);
    CHECK(register_capture(&contract, &controller, &lease, &frame));
    make_request(&request, slices, &frame);
    CHECK(prepare(&contract, &controller, &lease, &request, &job));
    CHECK(hevc_contract_abort(&contract, &controller, &lease, &job));
    CHECK(hevc_contract_release_job(&contract, &controller, &lease, &job));

    request.layout_version = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.layout_version = 2U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.layout_version = HEVC_REQUEST_LAYOUT_V1;
    request._reserved = 1U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request._reserved = 0U;

    request.picture.width = 15U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.width = 4097U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.width = 16U;
    CHECK(prepare(&contract, &controller, &lease, &request, &job));
    CHECK(hevc_contract_abort(&contract, &controller, &lease, &job));
    CHECK(hevc_contract_release_job(&contract, &controller, &lease, &job));
    request.picture.width = 4096U;
    CHECK(prepare(&contract, &controller, &lease, &request, &job));
    CHECK(hevc_contract_abort(&contract, &controller, &lease, &job));
    CHECK(hevc_contract_release_job(&contract, &controller, &lease, &job));
    request.picture.width = 1920U;
    request.picture.height = 15U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.height = 4097U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.height = 1080U;
    request.picture.ctb_count = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.ctb_count = HEVC_MAX_CTB_COUNT + 1U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.ctb_count = HEVC_MAX_CTB_COUNT;
    CHECK(prepare(&contract, &controller, &lease, &request, &job));
    CHECK(hevc_contract_abort(&contract, &controller, &lease, &job));
    CHECK(hevc_contract_release_job(&contract, &controller, &lease, &job));
    request.picture.ctb_count = 510U;

    request.picture.chroma_format = HEVC_CHROMA_INVALID;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.chroma_format = 2U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.chroma_format = HEVC_CHROMA_420;
    request.picture.luma_bit_depth = 7U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.luma_bit_depth = 9U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.luma_bit_depth = 10U;
    request.picture.chroma_bit_depth = 10U;
    CHECK(prepare(&contract, &controller, &lease, &request, &job));
    CHECK(hevc_contract_abort(&contract, &controller, &lease, &job));
    CHECK(hevc_contract_release_job(&contract, &controller, &lease, &job));
    request.picture.chroma_bit_depth = 8U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.luma_bit_depth = 8U;
    request.picture.chroma_bit_depth = 8U;
    request.picture.scaling_mode = HEVC_SCALING_INVALID;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.scaling_mode = 2U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.scaling_mode = HEVC_SCALING_DEFAULT;
    request.picture.syntax_flags = 1U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture.syntax_flags = 0U;
    request.picture._reserved = 1U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.picture._reserved = 0U;

    request.slices = NULL;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.slices = slices;
    request.slice_count = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.slice_count = HEVC_MAX_SLICES + 1U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.slice_count = 1U;
    request.entry_offsets = (const u32 *)slices;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.entry_offsets = NULL;
    request.entry_offset_count = 1U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.entry_offset_count = 0U;

    for (i = 0U; i < 18U; i++) {
        slices[0].slice_type = (i & 1U) ? HEVC_SLICE_P : HEVC_SLICE_B;
        CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    }
    slices[0].slice_type = HEVC_SLICE_I;
    slices[0]._reserved = 1U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    slices[0]._reserved = 0U;
    slices[0].bit_length = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    slices[0].bit_length = 8U;
    slices[0].ctb_address = request.picture.ctb_count;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    slices[0].ctb_address = 0U;
    slices[0].source_byte_offset = request.source.used;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
}

static void test_numeric_spans_and_ranges(void)
{
    struct hevc_contract contract;
    struct media_engine_controller controller;
    struct media_engine_lease lease;
    struct hevc_frame_handle frame;
    struct hevc_job_handle job;
    struct hevc_request request;
    struct hevc_slice slices[HEVC_MAX_SLICES];
    u32 i;

    init_contract(&contract, &controller, &lease, 11U);
    CHECK(register_capture(&contract, &controller, &lease, &frame));
    make_request(&request, slices, &frame);
    slices[0].source_byte_offset = 63U;
    slices[0].bit_length = 1U;
    CHECK(prepare(&contract, &controller, &lease, &request, &job));
    CHECK(hevc_contract_abort(&contract, &controller, &lease, &job));
    CHECK(hevc_contract_release_job(&contract, &controller, &lease, &job));
    slices[0].bit_length = 9U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    slices[0].source_byte_offset = 62U;
    CHECK(prepare(&contract, &controller, &lease, &request, &job));
    CHECK(hevc_contract_abort(&contract, &controller, &lease, &job));
    CHECK(hevc_contract_release_job(&contract, &controller, &lease, &job));
    slices[0].source_byte_offset = 0U;
    slices[0].bit_length = 8U;

    request.source.allocation_id = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.allocation_id = 1U;
    request.source.allocation_generation = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.allocation_generation = 1U;
    request.source.dma_allocation_base = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.dma_allocation_base = 0x10001U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.dma_allocation_base = HEVC_DMA_ADDRESS_LIMIT;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.dma_allocation_base = HEVC_DMA_ADDRESS_LIMIT - 16U;
    request.source.allocation_capacity = 32U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.dma_allocation_base = 0x10000U;
    request.source.allocation_capacity = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.allocation_capacity = HEVC_DMA_ADDRESS_LIMIT;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.allocation_capacity = 0x1000U;
    request.source.allocation_offset = 0x1000U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.allocation_offset = 0U;
    request.source.window_capacity = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.window_capacity = 0x1001U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.window_capacity = 0x400U;
    request.source.used = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.used = 0x401U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.used = 64U;
    request.source.output_canary = 1U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.output_canary = 0U;
    request.source.access = HEVC_DEVICE_WRITE;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.access = HEVC_DEVICE_READ | HEVC_DEVICE_WRITE;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source.access = HEVC_DEVICE_READ;
    request.source._reserved0 = 1U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source._reserved0 = 0U;
    request.source._reserved1 = 1U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.source._reserved1 = 0U;

    request.capture.dma_allocation_base = 0x20004U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.capture.dma_allocation_base = 0x20000U;
    request.capture.output_canary = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.capture.output_canary = 0xCAFE1001U;
    request.capture.access = HEVC_DEVICE_READ;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.capture.access = HEVC_DEVICE_READ | HEVC_DEVICE_WRITE;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.capture.access = HEVC_DEVICE_WRITE;
    request.capture.used = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.capture.used = 128U;
    request.capture.dma_allocation_base = request.source.dma_allocation_base;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.capture.dma_allocation_base = 0x20000U;

    request.capture.dma_allocation_base = request.source.dma_allocation_base;
    request.capture.allocation_id = request.source.allocation_id;
    request.capture.allocation_generation = request.source.allocation_generation;
    request.capture.allocation_capacity = request.source.allocation_capacity;
    request.capture.allocation_offset = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.capture.allocation_offset = 0x400U;
    request.capture.window_capacity = 0x400U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    make_span(&request.capture, 2U, 0x20000U, 128U, HEVC_DEVICE_WRITE,
              0xCAFE1001U);

    for (i = 1U; i <= HEVC_MAX_SLICES; i++) {
        request.slice_count = i;
        slices[i - 1U] = slices[0];
        slices[i - 1U].ctb_address = i - 1U;
        CHECK(prepare(&contract, &controller, &lease, &request, &job));
        CHECK(hevc_contract_abort(&contract, &controller, &lease, &job));
        CHECK(hevc_contract_release_job(&contract, &controller, &lease, &job));
    }
}

static void test_lease_frame_and_lifecycle(void)
{
    struct hevc_contract contract;
    struct hevc_contract other_contract;
    struct media_engine_controller controller;
    struct media_engine_controller other_controller;
    struct media_engine_lease lease;
    struct media_engine_lease other_lease;
    struct hevc_frame_handle frame;
    struct hevc_frame_handle other_frame;
    struct hevc_job_handle job;
    struct hevc_request request;
    struct hevc_slice slices[HEVC_MAX_SLICES];
    enum hevc_job_state job_state;
    enum hevc_frame_state frame_state;

    init_contract(&contract, &controller, &lease, 12U);
    CHECK(register_capture(&contract, &controller, &lease, &frame));
    make_request(&request, slices, &frame);
    memset(&job, 0, sizeof(job));
    CHECK(!hevc_contract_prepare(&contract, &controller, &lease, &request, NULL));
    CHECK(!hevc_contract_mark_in_flight(&contract, &controller, &lease, &job));
    CHECK(prepare(&contract, &controller, &lease, &request, &job));
    CHECK(hevc_contract_job_state_get(&contract, &job, &job_state));
    CHECK(job_state == HEVC_JOB_PREPARED);
    CHECK(hevc_contract_frame_state_get(&contract, &frame, &frame_state));
    CHECK(frame_state == HEVC_FRAME_PRODUCING);
    CHECK(!hevc_contract_complete(&contract, &controller, &lease, &job, true,
                                  false));
    CHECK(!hevc_contract_release_job(&contract, &controller, &lease, &job));
    CHECK(!hevc_contract_release_frame(&contract, &controller, &lease, &frame));
    CHECK(hevc_contract_mark_in_flight(&contract, &controller, &lease, &job));
    CHECK(hevc_contract_abort(&contract, &controller, &lease, &job));
    CHECK(hevc_contract_release_job(&contract, &controller, &lease, &job));
    CHECK(prepare(&contract, &controller, &lease, &request, &job));
    CHECK(hevc_contract_mark_in_flight(&contract, &controller, &lease, &job));
    CHECK(!hevc_contract_complete(&contract, &controller, &lease, &job, false,
                                  true));
    CHECK(hevc_contract_complete(&contract, &controller, &lease, &job, true,
                                 true));
    CHECK(hevc_contract_frame_state_get(&contract, &frame, &frame_state));
    CHECK(frame_state == HEVC_FRAME_REFERENCEABLE);
    CHECK(!hevc_contract_report_output_canary(&contract, &controller, &lease,
                                              &job, 0U));
    CHECK(!hevc_contract_report_output_canary(&contract, &controller, &lease,
                                              &job, 0xCAFE1002U));
    CHECK(hevc_contract_report_output_canary(&contract, &controller, &lease,
                                             &job, 0xCAFE1001U));
    CHECK(!hevc_contract_report_output_canary(&contract, &controller, &lease,
                                              &job, 0xCAFE1001U));
    CHECK(!hevc_contract_release_frame(&contract, &controller, &lease, &frame));
    CHECK(hevc_contract_release_job(&contract, &controller, &lease, &job));
    CHECK(!hevc_contract_job_state_get(&contract, &job, &job_state));
    CHECK(hevc_contract_release_frame(&contract, &controller, &lease, &frame));
    CHECK(!hevc_contract_frame_state_get(&contract, &frame, &frame_state));

    init_contract(&other_contract, &other_controller, &other_lease, 13U);
    CHECK(register_capture(&other_contract, &other_controller, &other_lease,
                           &other_frame));
    make_request(&request, slices, &other_frame);
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    CHECK(!hevc_contract_register_capture(&contract, &other_controller,
                                          &other_lease, &request.capture,
                                          &frame));

    init_contract(&other_contract, &other_controller, &other_lease, 14U);
    CHECK(register_capture(&other_contract, &other_controller, &other_lease,
                           &other_frame));
    make_request(&request, slices, &other_frame);
    slices[0].reference_count = 1U;
    CHECK(!prepare(&other_contract, &other_controller, &other_lease, &request,
                   &job));
    CHECK(hevc_contract_release_frame(&other_contract, &other_controller,
                                      &other_lease, &other_frame));
    CHECK(!prepare(&other_contract, &other_controller, &other_lease, &request,
                   &job));
}

static void test_frame_registry_and_failed_completion(void)
{
    struct hevc_contract contract;
    struct media_engine_controller controller;
    struct media_engine_lease lease;
    struct hevc_frame_handle frames[HEVC_FRAME_CAPACITY + 1U];
    struct hevc_job_handle job;
    struct hevc_request request;
    struct hevc_slice slices[HEVC_MAX_SLICES];
    struct hevc_dma_span capture;
    u32 i;

    init_contract(&contract, &controller, &lease, 17U);
    for (i = 0U; i < HEVC_FRAME_CAPACITY; i++) {
        make_span(&capture, (u64)i + 1U, 0x30000U + (u64)i * 0x2000U, 128U,
                  HEVC_DEVICE_WRITE, 0xAA000000U + i + 1U);
        CHECK(hevc_contract_register_capture(&contract, &controller, &lease,
                                             &capture, &frames[i]));
        CHECK(frames[i]._generation == (u64)i + 1U);
    }
    make_span(&capture, 99U, 0x60000U, 128U, HEVC_DEVICE_WRITE, 0xAA0099U);
    memset(&frames[HEVC_FRAME_CAPACITY], 0xA5,
           sizeof(frames[HEVC_FRAME_CAPACITY]));
    CHECK(!hevc_contract_register_capture(&contract, &controller, &lease,
                                          &capture,
                                          &frames[HEVC_FRAME_CAPACITY]));
    CHECK(frames[HEVC_FRAME_CAPACITY]._token == 0U);
    for (i = 0U; i < HEVC_FRAME_CAPACITY; i++)
        CHECK(hevc_contract_release_frame(&contract, &controller, &lease,
                                          &frames[i]));
    CHECK(!hevc_contract_release_frame(&contract, &controller, &lease,
                                       &frames[0]));

    CHECK(register_capture(&contract, &controller, &lease, &frames[0]));
    make_request(&request, slices, &frames[0]);
    request.capture_frame._reserved = 1U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.capture_frame._reserved = 0U;
    request.capture_frame._token = 0U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    request.capture_frame = frames[0];
    slices[0].reference_count = 1U;
    CHECK(!prepare(&contract, &controller, &lease, &request, &job));
    slices[0].reference_count = 0U;
    CHECK(prepare(&contract, &controller, &lease, &request, &job));
    CHECK(hevc_contract_mark_in_flight(&contract, &controller, &lease, &job));
    CHECK(hevc_contract_complete(&contract, &controller, &lease, &job, false,
                                 false));
    CHECK(hevc_contract_release_job(&contract, &controller, &lease, &job));
    CHECK(hevc_contract_release_frame(&contract, &controller, &lease,
                                      &frames[0]));

    CHECK(register_capture(&contract, &controller, &lease, &frames[0]));
    contract.frames[0].reference_users = 1U;
    CHECK(!hevc_contract_release_frame(&contract, &controller, &lease,
                                       &frames[0]));
    contract.frames[0].reference_users = 0U;
    CHECK(hevc_contract_release_frame(&contract, &controller, &lease,
                                      &frames[0]));
}

static void test_copy_pool_generations_and_lease_failures(void)
{
    struct hevc_contract contract;
    struct hevc_contract bad_contract;
    struct media_engine_controller controller;
    struct media_engine_controller pisp_controller;
    struct media_engine_lease lease;
    struct media_engine_lease pisp_lease;
    struct hevc_frame_handle frames[HEVC_FRAME_CAPACITY + 1U];
    struct hevc_job_handle jobs[HEVC_JOB_CAPACITY + 1U];
    struct hevc_request request;
    struct hevc_slice slices[HEVC_MAX_SLICES];
    struct hevc_picture_info picture;
    struct hevc_slice_payload slice;
    struct hevc_dma_span capture;
    u32 i;

    init_contract(&contract, &controller, &lease, 15U);
    CHECK(register_capture(&contract, &controller, &lease, &frames[0]));
    make_request(&request, slices, &frames[0]);
    CHECK(prepare(&contract, &controller, &lease, &request, &jobs[0]));
    request.picture.width = 16U;
    request.source.used = 1U;
    slices[0].source_byte_offset = 99U;
    slices[0].bit_length = 999U;
    CHECK(hevc_contract_picture_get(&contract, &jobs[0], &picture));
    CHECK(picture.width == 1920U);
    CHECK(hevc_contract_slice_get(&contract, &jobs[0], 0U, &slice));
    CHECK(slice.source_byte_offset == 0U && slice.bit_length == 8U);
    CHECK(!hevc_contract_slice_get(&contract, &jobs[0], 1U, &slice));
    CHECK(hevc_contract_abort(&contract, &controller, &lease, &jobs[0]));
    CHECK(hevc_contract_release_job(&contract, &controller, &lease, &jobs[0]));
    CHECK(hevc_contract_release_frame(&contract, &controller, &lease, &frames[0]));
    CHECK(!hevc_contract_picture_get(&contract, &jobs[0], &picture));

    for (i = 0U; i < HEVC_JOB_CAPACITY; i++) {
        make_span(&capture, (u64)i + 2U, 0x20000U + (u64)i * 0x2000U, 128U,
                  HEVC_DEVICE_WRITE, 0xD00D0000U + i);
        CHECK(hevc_contract_register_capture(&contract, &controller, &lease,
                                             &capture, &frames[i]));
        make_request(&request, slices, &frames[i]);
        request.capture = capture;
        CHECK(prepare(&contract, &controller, &lease, &request, &jobs[i]));
    }
    make_span(&capture, 99U, 0x40000U, 128U, HEVC_DEVICE_WRITE, 0xD00D0099U);
    CHECK(hevc_contract_register_capture(&contract, &controller, &lease,
                                         &capture, &frames[4]));
    make_request(&request, slices, &frames[4]);
    request.capture = capture;
    CHECK(!prepare(&contract, &controller, &lease, &request, &jobs[4]));
    for (i = 0U; i < HEVC_JOB_CAPACITY; i++) {
        CHECK(hevc_contract_abort(&contract, &controller, &lease, &jobs[i]));
        CHECK(hevc_contract_release_job(&contract, &controller, &lease, &jobs[i]));
        CHECK(hevc_contract_release_frame(&contract, &controller, &lease,
                                          &frames[i]));
    }
    CHECK(hevc_contract_release_frame(&contract, &controller, &lease, &frames[4]));

    contract.jobs[0].generation = ~0ULL;
    CHECK(register_capture(&contract, &controller, &lease, &frames[0]));
    make_request(&request, slices, &frames[0]);
    CHECK(prepare(&contract, &controller, &lease, &request, &jobs[0]));
    CHECK(jobs[0]._generation == ~0ULL);
    CHECK(hevc_contract_abort(&contract, &controller, &lease, &jobs[0]));
    CHECK(hevc_contract_release_job(&contract, &controller, &lease, &jobs[0]));
    CHECK(contract.jobs[0].generation == 1U);
    CHECK(!hevc_contract_mark_in_flight(&contract, &controller, &lease, &jobs[0]));
    CHECK(hevc_contract_release_frame(&contract, &controller, &lease, &frames[0]));

    contract.frames[0].generation = ~0ULL;
    CHECK(register_capture(&contract, &controller, &lease, &frames[0]));
    CHECK(frames[0]._generation == ~0ULL);
    CHECK(hevc_contract_release_frame(&contract, &controller, &lease, &frames[0]));
    CHECK(contract.frames[0].generation == 1U);
    CHECK(!hevc_contract_release_frame(&contract, &controller, &lease, &frames[0]));
    CHECK(!hevc_contract_init(&contract, 15U));

    memset(&bad_contract, 0, sizeof(bad_contract));
    CHECK(hevc_contract_init(&bad_contract, 16U));
    make_span(&capture, 8U, 0x60000U, 128U, HEVC_DEVICE_WRITE, 0x8888U);
    CHECK(!hevc_contract_register_capture(&bad_contract, &controller, &lease,
                                          &capture, &frames[0]));

    memset(&pisp_controller, 0, sizeof(pisp_controller));
    memset(&pisp_lease, 0, sizeof(pisp_lease));
    CHECK(media_engine_controller_init(&pisp_controller, 15U));
    CHECK(media_engine_passive_identify(&pisp_controller, MEDIA_ENGINE_PISP_BE,
                                        0x02252700U));
    CHECK(media_engine_lease_acquire(&pisp_controller, MEDIA_ENGINE_PISP_BE,
                                     &pisp_lease));
    CHECK(!hevc_contract_register_capture(&contract, &pisp_controller,
                                          &pisp_lease, &capture, &frames[0]));
    CHECK(media_engine_lease_complete(&controller, &lease));
    CHECK(!hevc_contract_register_capture(&contract, &controller, &lease,
                                          &capture, &frames[0]));
}

int main(void)
{
    test_abi_and_init();
    test_metadata_validation();
    test_numeric_spans_and_ranges();
    test_lease_frame_and_lifecycle();
    test_frame_registry_and_failed_completion();
    test_copy_pool_generations_and_lease_failures();
    printf("hevc contract: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
