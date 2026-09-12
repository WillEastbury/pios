#include <stdio.h>
#include <string.h>

#include "types.h"
#include "media_engine_contract.h"
#include "pisp_be_contract.h"
#include "pisp_be_gate.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

struct fixture {
    struct pisp_be_gate gate;
    struct pisp_be_gate_handle gate_handle;
    struct media_engine_controller controller;
    struct media_engine_lease lease;
    struct pisp_be_contract jobs;
    struct pisp_be_job_handle job;
    u8 config[PISP_BE_CONFIG_BYTES];
    struct pisp_be_dma_span spans[PISP_BE_MAX_SPANS];
};

static void put_le32(u8 *out, u32 value)
{
    out[0] = (u8)value;
    out[1] = (u8)(value >> 8);
    out[2] = (u8)(value >> 16);
    out[3] = (u8)(value >> 24);
}

static void make_request(struct fixture *f)
{
    memset(f->config, 0, sizeof(f->config));
    put_le32(f->config + PISP_BE_CONFIG_NUM_TILES_OFFSET, 1U);
    put_le32(f->config + PISP_BE_CONFIG_GLOBAL_BAYER_OFFSET,
             PISP_BE_CONFIG_ROOT_BIT);
    memset(f->spans, 0, sizeof(f->spans));
    f->spans[0].allocation_id = 1U;
    f->spans[0].dma_allocation_base = 0x10000U;
    f->spans[0].allocation_capacity = 0x10000U;
    f->spans[0].window_capacity = 0x5000U;
    f->spans[0].used = PISP_BE_CONFIG_BYTES;
    f->spans[0].allocation_generation = 1U;
    f->spans[0].access = PISP_BE_DEVICE_READ;
    f->spans[0].logical_role = PISP_BE_SPAN_CONFIG;
    f->spans[1].allocation_id = 2U;
    f->spans[1].dma_allocation_base = 0x30000U;
    f->spans[1].allocation_capacity = 0x1000U;
    f->spans[1].window_capacity = 0x400U;
    f->spans[1].used = 0x200U;
    f->spans[1].allocation_generation = 2U;
    f->spans[1].access = PISP_BE_DEVICE_READ;
    f->spans[1].logical_role = PISP_BE_SPAN_MAIN_INPUT;
    f->spans[2].allocation_id = 3U;
    f->spans[2].dma_allocation_base = 0x40000U;
    f->spans[2].allocation_capacity = 0x1000U;
    f->spans[2].window_capacity = 0x400U;
    f->spans[2].used = 0x200U;
    f->spans[2].output_canary = 0xCAFE1234U;
    f->spans[2].allocation_generation = 3U;
    f->spans[2].access = PISP_BE_DEVICE_WRITE;
    f->spans[2].logical_role = PISP_BE_SPAN_OUTPUT;
}

static void fixture_init_for(struct fixture *f, u32 controller_id,
                             u64 now_ms, u32 timeout_ms)
{
    memset(f, 0, sizeof(*f));
    CHECK(media_engine_controller_init(&f->controller, controller_id));
    CHECK(pisp_be_contract_init(&f->jobs, controller_id));
    CHECK(pisp_be_gate_init(&f->gate, PISP_BE_GATE_OWNER_CORE, controller_id,
                            now_ms, timeout_ms, &f->gate_handle));
    make_request(f);
}

static void fixture_init(struct fixture *f, u64 now_ms, u32 timeout_ms)
{
    fixture_init_for(f, 71U, now_ms, timeout_ms);
}

static void expect_gate(const struct fixture *f, enum pisp_be_gate_state state,
                        enum pisp_be_gate_fault fault)
{
    enum pisp_be_gate_state actual_state;
    enum pisp_be_gate_fault actual_fault;

    CHECK(pisp_be_gate_state_get(&f->gate, &f->gate_handle, 0U, &actual_state,
                                 &actual_fault));
    CHECK(actual_state == state);
    CHECK(actual_fault == fault);
}

static void identify_and_lease(struct fixture *f, u64 now_ms)
{
    CHECK(pisp_be_gate_passive_identify(&f->gate, &f->gate_handle, 0U,
                                        &f->controller, now_ms, 0x0225270FU));
    CHECK(media_engine_lease_acquire(&f->controller, MEDIA_ENGINE_PISP_BE,
                                     &f->lease));
}

static void drive_iommu(struct fixture *f, u64 now_ms)
{
    struct pisp_be_gate_iommu_evidence evidence;

    identify_and_lease(f, now_ms);
    CHECK(pisp_be_gate_clock_verified(&f->gate, &f->gate_handle, 0U,
                                      &f->controller, &f->lease, now_ms,
                                      7U, true));
    evidence.lease = f->lease;
    evidence.iommu_resource = 2U;
    evidence._reserved = 0U;
    CHECK(pisp_be_gate_iommu_verified(&f->gate, &f->gate_handle, 0U,
                                      &f->controller, &f->lease, now_ms,
                                      &evidence, true));
}

static struct pisp_be_gate_idle_evidence idle_evidence(u32 raw_status,
                                                        u32 batch_started,
                                                        u32 batch_done)
{
    struct pisp_be_gate_idle_evidence evidence = {
        .raw_status = raw_status,
        .batch_started = batch_started,
        .batch_done = batch_done,
        ._reserved = 0U,
    };

    return evidence;
}

static void drive_idle(struct fixture *f, u64 now_ms)
{
    struct pisp_be_gate_idle_evidence evidence = idle_evidence(0U, 1U, 1U);

    drive_iommu(f, now_ms);
    CHECK(pisp_be_gate_idle_verified(&f->gate, &f->gate_handle, 0U,
                                     &f->controller, &f->lease, now_ms,
                                     &evidence, true));
}

static void prepare_job(struct fixture *f)
{
    struct pisp_be_request request = {
        .layout_version = PISP_BE_REQUEST_LAYOUT_V1,
        .config_bytes = f->config,
        .config_bytes_len = PISP_BE_CONFIG_BYTES,
        ._reserved = 0U,
    };

    CHECK(pisp_be_contract_prepare(&f->jobs, &f->controller, &f->lease,
                                   &request, f->spans, 3U, &f->job));
}

static void admit_job(struct fixture *f, u64 now_ms)
{
    prepare_job(f);
    CHECK(pisp_be_gate_admit_prepared_job(&f->gate, &f->gate_handle, 0U,
                                          &f->controller, &f->lease, now_ms,
                                          &f->jobs, &f->job));
}

static void test_abi_and_init(void)
{
    struct pisp_be_gate gate;
    struct pisp_be_gate_handle handle;
    struct pisp_be_gate_handle second_handle;
    enum pisp_be_gate_state state;
    enum pisp_be_gate_fault fault;

    CHECK(PISP_BE_GATE_OWNER_CORE == 0U);
    CHECK(PISP_BE_GATE_MAX_TIMEOUT_MS == 1000U);
    CHECK(sizeof(struct pisp_be_gate) == 64U);
    CHECK(_Alignof(struct pisp_be_gate) == 64U);
    CHECK(sizeof(struct pisp_be_gate_idle_evidence) == 16U);
    CHECK(PISP_BE_GATE_DISABLED == 0);
    CHECK(PISP_BE_GATE_QUARANTINED > PISP_BE_GATE_JOB_COMPLETED);
    CHECK(PISP_BE_GATE_FAULT_NONE == 0);
    CHECK(PISP_BE_GATE_FAULT_INTERNAL > PISP_BE_GATE_FAULT_CANARY);
    memset(&gate, 0, sizeof(gate));
    CHECK(!pisp_be_gate_init(NULL, 0U, 1U, 0U, 1U, &handle));
    CHECK(!pisp_be_gate_init(&gate, 1U, 1U, 0U, 1U, &handle));
    CHECK(!pisp_be_gate_init(&gate, 0U, 0U, 0U, 1U, &handle));
    CHECK(!pisp_be_gate_init(&gate, 0U, 1U, 0U, 0U, &handle));
    CHECK(!pisp_be_gate_init(&gate, 0U, 1U, 0U,
                             PISP_BE_GATE_MAX_TIMEOUT_MS + 1U, &handle));
    CHECK(!pisp_be_gate_init(&gate, 0U, 1U, ~0ULL, 1U, &handle));
    CHECK(!pisp_be_gate_init(&gate, 0U, 1U, 0U, 1U, NULL));
    CHECK(pisp_be_gate_init(&gate, 0U, 1U, 100U, 10U, &handle));
    CHECK(gate.generation == 1U);
    CHECK(gate.deadline_ms == 110U);
    CHECK(gate.controller_id == 1U && gate.owner_core == 0U);
    CHECK(handle._token != 0U && handle._generation == 1U);
    CHECK(handle._controller_id == 1U && handle._reserved == 0U);
    CHECK(pisp_be_gate_state_get(&gate, &handle, 0U, &state, &fault));
    CHECK(state == PISP_BE_GATE_DISABLED);
    CHECK(fault == PISP_BE_GATE_FAULT_NONE);
    CHECK(gate.transition_count == 0U && gate.failure_count == 0U);
    CHECK(!pisp_be_gate_init(&gate, 0U, 1U, 100U, 10U, &second_handle));
    CHECK(second_handle._token == 0U && second_handle._generation == 0U);
    CHECK(gate.generation == 1U);
    CHECK(pisp_be_gate_state_get(&gate, &handle, 0U, &state, &fault));
    CHECK(!pisp_be_gate_state_get(&gate, &handle, 1U, &state, &fault));
    CHECK(!pisp_be_gate_state_get(&gate, &handle, 0U, NULL, &fault));
    CHECK(!pisp_be_gate_state_get(&gate, &handle, 0U, &state, NULL));
}

static void test_identity_and_sequence(void)
{
    struct fixture f;
    struct media_engine_controller other;
    enum media_engine_state engine_state;

    fixture_init(&f, 100U, 10U);
    CHECK(!pisp_be_gate_passive_identify(&f.gate, &f.gate_handle, 1U,
                                         &f.controller, 100U, 0x02252700U));
    expect_gate(&f, PISP_BE_GATE_DISABLED, PISP_BE_GATE_FAULT_NONE);
    CHECK(pisp_be_gate_passive_identify(&f.gate, &f.gate_handle, 0U,
                                        &f.controller, 100U, 0x02252700U));
    expect_gate(&f, PISP_BE_GATE_PASSIVE_ID_OK, PISP_BE_GATE_FAULT_NONE);

    fixture_init(&f, 100U, 10U);
    CHECK(!pisp_be_gate_clock_verified(&f.gate, &f.gate_handle, 0U,
                                       &f.controller, &f.lease, 100U,
                                       7U, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_INTERNAL);

    fixture_init(&f, 100U, 10U);
    CHECK(!pisp_be_gate_passive_identify(&f.gate, &f.gate_handle, 0U,
                                         &f.controller, 100U, 0x02252710U));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_BAD_ID);
    CHECK(media_engine_state_get(&f.controller, MEDIA_ENGINE_PISP_BE,
                                 &engine_state));
    CHECK(engine_state == MEDIA_ENGINE_QUARANTINED);

    fixture_init(&f, 100U, 10U);
    memset(&other, 0, sizeof(other));
    CHECK(media_engine_controller_init(&other, 72U));
    CHECK(!pisp_be_gate_passive_identify(&f.gate, &f.gate_handle, 0U, &other,
                                         100U, 0x02252700U));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_BAD_ID);

    fixture_init(&f, 100U, 10U);
    identify_and_lease(&f, 100U);
    CHECK(!pisp_be_gate_passive_identify(&f.gate, &f.gate_handle, 0U,
                                         &f.controller, 101U, 0x02252700U));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_INTERNAL);
    CHECK(media_engine_lease_active_for(&f.controller, &f.lease,
                                        MEDIA_ENGINE_PISP_BE));
}

static void test_observation_failures(void)
{
    struct fixture f;
    struct pisp_be_gate_iommu_evidence evidence;
    struct pisp_be_gate_idle_evidence idle;
    struct media_engine_lease hevc_lease;

    fixture_init(&f, 100U, 10U);
    identify_and_lease(&f, 100U);
    CHECK(!pisp_be_gate_clock_verified(&f.gate, &f.gate_handle, 0U,
                                       &f.controller, &f.lease, 101U,
                                       7U, false));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_CLOCK);
    CHECK(f.gate.last_status == 7U);

    fixture_init(&f, 100U, 10U);
    identify_and_lease(&f, 100U);
    CHECK(!pisp_be_gate_clock_verified(&f.gate, &f.gate_handle, 0U,
                                       &f.controller, &f.lease, 101U,
                                       8U, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_CLOCK);

    fixture_init(&f, 100U, 10U);
    CHECK(pisp_be_gate_passive_identify(&f.gate, &f.gate_handle, 0U,
                                        &f.controller, 100U, 0x02252700U));
    CHECK(media_engine_passive_identify(&f.controller, MEDIA_ENGINE_HEVC,
                                        0x00000202U));
    CHECK(media_engine_lease_acquire(&f.controller, MEDIA_ENGINE_HEVC,
                                     &hevc_lease));
    CHECK(!pisp_be_gate_clock_verified(&f.gate, &f.gate_handle, 0U,
                                       &f.controller, &hevc_lease, 101U,
                                       7U, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_CLOCK);
    CHECK(media_engine_lease_active_for(&f.controller, &hevc_lease,
                                        MEDIA_ENGINE_HEVC));

    fixture_init(&f, 100U, 10U);
    identify_and_lease(&f, 100U);
    CHECK(media_engine_lease_abort(&f.controller, &f.lease));
    CHECK(!pisp_be_gate_clock_verified(&f.gate, &f.gate_handle, 0U,
                                       &f.controller, &f.lease, 101U,
                                       7U, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_CLOCK);

    fixture_init(&f, 100U, 10U);
    identify_and_lease(&f, 100U);
    CHECK(pisp_be_gate_clock_verified(&f.gate, &f.gate_handle, 0U,
                                      &f.controller, &f.lease, 101U, 7U, true));
    evidence.lease = f.lease;
    evidence.iommu_resource = 2U;
    evidence._reserved = 0U;
    CHECK(!pisp_be_gate_iommu_verified(&f.gate, &f.gate_handle, 0U,
                                       &f.controller, &f.lease, 102U,
                                       &evidence, false));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_IOMMU);

    fixture_init(&f, 100U, 10U);
    identify_and_lease(&f, 100U);
    CHECK(pisp_be_gate_clock_verified(&f.gate, &f.gate_handle, 0U,
                                      &f.controller, &f.lease, 101U, 7U, true));
    evidence.lease = f.lease;
    evidence.iommu_resource = 3U;
    evidence._reserved = 0U;
    CHECK(!pisp_be_gate_iommu_verified(&f.gate, &f.gate_handle, 0U,
                                       &f.controller, &f.lease, 102U,
                                       &evidence, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_IOMMU);

    fixture_init(&f, 100U, 10U);
    identify_and_lease(&f, 100U);
    CHECK(pisp_be_gate_clock_verified(&f.gate, &f.gate_handle, 0U,
                                      &f.controller, &f.lease, 101U, 7U, true));
    evidence.lease = f.lease;
    evidence.lease._token ^= 0x100U;
    evidence.iommu_resource = 2U;
    evidence._reserved = 0U;
    CHECK(!pisp_be_gate_iommu_verified(&f.gate, &f.gate_handle, 0U,
                                       &f.controller, &f.lease, 102U,
                                       &evidence, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_IOMMU);

    fixture_init(&f, 100U, 10U);
    drive_iommu(&f, 100U);
    idle = idle_evidence(0U, 3U, 3U);
    CHECK(!pisp_be_gate_idle_verified(&f.gate, &f.gate_handle, 0U,
                                      &f.controller, &f.lease, 101U,
                                      &idle, false));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_STATUS);
    CHECK(f.gate.last_status == 0U);

    fixture_init(&f, 100U, 10U);
    drive_iommu(&f, 100U);
    idle = idle_evidence(7U, 3U, 3U);
    CHECK(!pisp_be_gate_idle_verified(&f.gate, &f.gate_handle, 0U,
                                      &f.controller, &f.lease, 101U,
                                      &idle, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_STATUS);
    CHECK(f.gate.last_status == 7U);

    fixture_init(&f, 100U, 10U);
    drive_iommu(&f, 100U);
    idle = idle_evidence(0U, 4U, 3U);
    CHECK(!pisp_be_gate_idle_verified(&f.gate, &f.gate_handle, 0U,
                                      &f.controller, &f.lease, 101U,
                                      &idle, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_STATUS);
    CHECK(f.gate.last_status == 0U);

    fixture_init(&f, 100U, 10U);
    drive_iommu(&f, 100U);
    idle = idle_evidence(0U, 4U, 4U);
    idle._reserved = 1U;
    CHECK(!pisp_be_gate_idle_verified(&f.gate, &f.gate_handle, 0U,
                                      &f.controller, &f.lease, 101U,
                                      &idle, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_STATUS);

    fixture_init(&f, 100U, 10U);
    drive_iommu(&f, 100U);
    CHECK(!pisp_be_gate_idle_verified(&f.gate, &f.gate_handle, 0U,
                                      &f.controller, &f.lease, 101U,
                                      NULL, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_STATUS);

    fixture_init(&f, 100U, 10U);
    drive_iommu(&f, 100U);
    idle = idle_evidence(0U, 9U, 9U);
    CHECK(pisp_be_gate_idle_verified(&f.gate, &f.gate_handle, 0U,
                                     &f.controller, &f.lease, 101U,
                                     &idle, true));
    expect_gate(&f, PISP_BE_GATE_IDLE_OK, PISP_BE_GATE_FAULT_NONE);
}

static void test_admission_and_completion(void)
{
    struct fixture f;
    enum pisp_be_job_state job_state;
    enum media_engine_state engine_state;

    fixture_init(&f, 100U, 20U);
    drive_idle(&f, 101U);
    CHECK(!pisp_be_gate_admit_prepared_job(&f.gate, &f.gate_handle, 0U,
                                           &f.controller, &f.lease, 102U,
                                           &f.jobs, &f.job));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_JOB);

    fixture_init(&f, 100U, 20U);
    drive_idle(&f, 101U);
    prepare_job(&f);
    CHECK(pisp_be_contract_mark_in_flight(&f.jobs, &f.controller, &f.lease,
                                          &f.job));
    CHECK(!pisp_be_gate_admit_prepared_job(&f.gate, &f.gate_handle, 0U,
                                           &f.controller, &f.lease, 102U,
                                           &f.jobs, &f.job));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_JOB);

    fixture_init(&f, 100U, 20U);
    drive_idle(&f, 101U);
    admit_job(&f, 102U);
    CHECK(pisp_be_contract_state_get(&f.jobs, &f.job, &job_state));
    CHECK(job_state == PISP_BE_JOB_PREPARED);
    CHECK(media_engine_state_get(&f.controller, MEDIA_ENGINE_PISP_BE,
                                 &engine_state));
    CHECK(engine_state == MEDIA_ENGINE_LEASED);
    CHECK(pisp_be_gate_report_completion(&f.gate, &f.gate_handle, 0U, 103U,
                                         true, true));
    expect_gate(&f, PISP_BE_GATE_JOB_COMPLETED, PISP_BE_GATE_FAULT_NONE);
    CHECK(pisp_be_contract_state_get(&f.jobs, &f.job, &job_state));
    CHECK(job_state == PISP_BE_JOB_PREPARED);
    CHECK(media_engine_lease_active_for(&f.controller, &f.lease,
                                        MEDIA_ENGINE_PISP_BE));
    CHECK(!pisp_be_gate_report_completion(&f.gate, &f.gate_handle, 0U, 104U,
                                          true, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_INTERNAL);

    fixture_init(&f, 100U, 20U);
    drive_idle(&f, 101U);
    admit_job(&f, 102U);
    CHECK(!pisp_be_gate_report_completion(&f.gate, &f.gate_handle, 0U, 103U,
                                          false, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_JOB);

    fixture_init(&f, 100U, 20U);
    drive_idle(&f, 101U);
    admit_job(&f, 102U);
    CHECK(!pisp_be_gate_report_completion(&f.gate, &f.gate_handle, 0U, 103U,
                                          true, false));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_CANARY);
}

static void test_deadlines(void)
{
    struct fixture f;
    struct pisp_be_gate_iommu_evidence evidence;
    struct pisp_be_gate_idle_evidence idle;

    fixture_init(&f, 100U, 10U);
    CHECK(!pisp_be_gate_passive_identify(&f.gate, &f.gate_handle, 0U,
                                         &f.controller, 111U, 0x02252700U));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_DEADLINE);

    fixture_init(&f, 100U, 10U);
    identify_and_lease(&f, 100U);
    CHECK(!pisp_be_gate_clock_verified(&f.gate, &f.gate_handle, 0U,
                                       &f.controller, &f.lease, 111U,
                                       7U, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_DEADLINE);

    fixture_init(&f, 100U, 10U);
    identify_and_lease(&f, 100U);
    CHECK(pisp_be_gate_clock_verified(&f.gate, &f.gate_handle, 0U,
                                      &f.controller, &f.lease, 100U, 7U, true));
    evidence.lease = f.lease;
    evidence.iommu_resource = 2U;
    evidence._reserved = 0U;
    CHECK(!pisp_be_gate_iommu_verified(&f.gate, &f.gate_handle, 0U,
                                       &f.controller, &f.lease, 111U,
                                       &evidence, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_DEADLINE);

    fixture_init(&f, 100U, 10U);
    identify_and_lease(&f, 100U);
    CHECK(pisp_be_gate_clock_verified(&f.gate, &f.gate_handle, 0U,
                                      &f.controller, &f.lease, 100U, 7U, true));
    evidence.lease = f.lease;
    evidence.iommu_resource = 2U;
    evidence._reserved = 0U;
    CHECK(pisp_be_gate_iommu_verified(&f.gate, &f.gate_handle, 0U,
                                      &f.controller, &f.lease, 100U,
                                      &evidence, true));
    idle = idle_evidence(0U, 1U, 1U);
    CHECK(!pisp_be_gate_idle_verified(&f.gate, &f.gate_handle, 0U,
                                      &f.controller, &f.lease, 111U,
                                      &idle, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_DEADLINE);

    fixture_init(&f, 100U, 10U);
    drive_idle(&f, 100U);
    prepare_job(&f);
    CHECK(!pisp_be_gate_admit_prepared_job(&f.gate, &f.gate_handle, 0U,
                                           &f.controller, &f.lease, 111U,
                                           &f.jobs, &f.job));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_DEADLINE);

    fixture_init(&f, 100U, 10U);
    drive_idle(&f, 100U);
    admit_job(&f, 100U);
    CHECK(!pisp_be_gate_report_completion(&f.gate, &f.gate_handle, 0U, 111U,
                                          true, true));
    expect_gate(&f, PISP_BE_GATE_QUARANTINED, PISP_BE_GATE_FAULT_DEADLINE);

    fixture_init(&f, 100U, 10U);
    drive_idle(&f, 110U);
    admit_job(&f, 110U);
    CHECK(pisp_be_gate_report_completion(&f.gate, &f.gate_handle, 0U, 110U,
                                         true, true));
    expect_gate(&f, PISP_BE_GATE_JOB_COMPLETED, PISP_BE_GATE_FAULT_NONE);
}

static void test_handles_and_rearm(void)
{
    struct fixture f;
    struct fixture other;
    struct pisp_be_gate_handle forged;
    struct pisp_be_gate_handle stale;
    struct pisp_be_gate_handle rearmed;
    enum pisp_be_gate_state state;
    enum pisp_be_gate_fault fault;

    fixture_init(&f, 100U, 10U);
    forged = f.gate_handle;
    forged._controller_id++;
    CHECK(!pisp_be_gate_state_get(&f.gate, &forged, 0U, &state, &fault));
    forged = f.gate_handle;
    forged._generation++;
    CHECK(!pisp_be_gate_state_get(&f.gate, &forged, 0U, &state, &fault));
    forged = f.gate_handle;
    forged._token ^= 1U;
    CHECK(!pisp_be_gate_state_get(&f.gate, &forged, 0U, &state, &fault));
    forged = f.gate_handle;
    forged._reserved = 1U;
    CHECK(!pisp_be_gate_state_get(&f.gate, &forged, 0U, &state, &fault));
    CHECK(!pisp_be_gate_state_get(&f.gate, &f.gate_handle, 1U, &state, &fault));
    expect_gate(&f, PISP_BE_GATE_DISABLED, PISP_BE_GATE_FAULT_NONE);

    fixture_init_for(&other, 72U, 100U, 10U);
    CHECK(!pisp_be_gate_state_get(&f.gate, &other.gate_handle, 0U, &state,
                                  &fault));
    CHECK(!pisp_be_gate_rearm(&f.gate, &f.gate_handle, 0U, 101U, 10U,
                              &rearmed));

    CHECK(!pisp_be_gate_passive_identify(&f.gate, &f.gate_handle, 0U,
                                         &f.controller, 100U, 0U));
    stale = f.gate_handle;
    CHECK(!pisp_be_gate_rearm(&f.gate, &stale, 1U, 101U, 10U, &rearmed));
    CHECK(!pisp_be_gate_rearm(&f.gate, &stale, 0U, 101U, 0U, &rearmed));
    CHECK(pisp_be_gate_rearm(&f.gate, &stale, 0U, 120U, 10U, &rearmed));
    CHECK(rearmed._generation != stale._generation);
    CHECK(f.gate.deadline_ms == 130U);
    CHECK(f.gate.transition_count == 0U && f.gate.failure_count == 0U);
    CHECK(!pisp_be_gate_state_get(&f.gate, &stale, 0U, &state, &fault));
    CHECK(pisp_be_gate_state_get(&f.gate, &rearmed, 0U, &state, &fault));
    CHECK(state == PISP_BE_GATE_DISABLED && fault == PISP_BE_GATE_FAULT_NONE);

    CHECK(!pisp_be_gate_passive_identify(&f.gate, &rearmed, 0U, &f.controller,
                                         131U, 0x02252700U));
    stale = rearmed;
    f.gate.generation = ~0ULL;
    stale._generation = ~0ULL;
    stale._token = (0x50424754ULL << 32U) |
                   (u64)((u32)stale._generation ^ stale._controller_id);
    CHECK(!pisp_be_gate_rearm(&f.gate, &stale, 0U, 140U, 1U, &rearmed));
    CHECK(f.gate.generation == ~0ULL);
    CHECK(f.gate.state == PISP_BE_GATE_QUARANTINED);
}

static void test_media_lifecycle_remains_caller_owned(void)
{
    struct fixture f;
    struct pisp_be_gate_handle rearmed;
    struct pisp_be_gate_handle rearmed_again;
    enum media_engine_state engine_state;
    enum pisp_be_job_state job_state;
    struct media_engine_lease lease_before_rearm;

    fixture_init(&f, 100U, 30U);
    CHECK(!pisp_be_gate_passive_identify(&f.gate, &f.gate_handle, 0U,
                                         &f.controller, 101U, 0U));
    CHECK(media_engine_state_get(&f.controller, MEDIA_ENGINE_PISP_BE,
                                 &engine_state));
    CHECK(engine_state == MEDIA_ENGINE_QUARANTINED);
    CHECK(!pisp_be_gate_init(&f.gate, PISP_BE_GATE_OWNER_CORE, 71U, 102U,
                             30U, &rearmed));
    CHECK(pisp_be_gate_rearm(&f.gate, &f.gate_handle, 0U, 102U, 30U,
                             &rearmed));
    CHECK(media_engine_state_get(&f.controller, MEDIA_ENGINE_PISP_BE,
                                 &engine_state));
    CHECK(engine_state == MEDIA_ENGINE_QUARANTINED);
    CHECK(!pisp_be_gate_passive_identify(&f.gate, &rearmed, 0U,
                                         &f.controller, 103U, 0x02252700U));
    CHECK(pisp_be_gate_rearm(&f.gate, &rearmed, 0U, 104U, 30U,
                             &rearmed_again));
    CHECK(media_engine_rearm(&f.controller, MEDIA_ENGINE_PISP_BE));
    CHECK(pisp_be_gate_passive_identify(&f.gate, &rearmed_again, 0U,
                                        &f.controller, 105U, 0x02252700U));

    fixture_init(&f, 100U, 30U);
    drive_idle(&f, 101U);
    admit_job(&f, 102U);
    CHECK(!pisp_be_gate_report_completion(&f.gate, &f.gate_handle, 0U, 103U,
                                          false, true));
    CHECK(pisp_be_contract_state_get(&f.jobs, &f.job, &job_state));
    CHECK(job_state == PISP_BE_JOB_PREPARED);
    CHECK(media_engine_state_get(&f.controller, MEDIA_ENGINE_PISP_BE,
                                 &engine_state));
    CHECK(engine_state == MEDIA_ENGINE_LEASED);
    lease_before_rearm = f.lease;
    CHECK(pisp_be_gate_rearm(&f.gate, &f.gate_handle, 0U, 104U, 30U,
                             &rearmed));
    CHECK(memcmp(&lease_before_rearm, &f.lease, sizeof(f.lease)) == 0);
    CHECK(!pisp_be_gate_passive_identify(&f.gate, &rearmed, 0U,
                                         &f.controller, 105U, 0x02252700U));
    CHECK(pisp_be_contract_abort(&f.jobs, &f.controller, &f.lease, &f.job));
    CHECK(pisp_be_contract_release(&f.jobs, &f.controller, &f.lease, &f.job));
    CHECK(media_engine_lease_complete(&f.controller, &f.lease));
    CHECK(media_engine_lease_release(&f.controller, &f.lease));
    CHECK(media_engine_state_get(&f.controller, MEDIA_ENGINE_PISP_BE,
                                 &engine_state));
    CHECK(engine_state == MEDIA_ENGINE_RELEASED);
    CHECK(pisp_be_gate_rearm(&f.gate, &rearmed, 0U, 106U, 30U,
                             &rearmed_again));
    CHECK(pisp_be_gate_passive_identify(&f.gate, &rearmed_again, 0U,
                                        &f.controller, 107U, 0x02252700U));
}

int main(void)
{
    test_abi_and_init();
    test_identity_and_sequence();
    test_observation_failures();
    test_admission_and_completion();
    test_deadlines();
    test_handles_and_rearm();
    test_media_lifecycle_remains_caller_owned();

    if (failures) {
        printf("pisp be gate: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    if (checks <= 100) {
        printf("pisp be gate: only %d checks\n", checks);
        return 1;
    }
    printf("pisp be gate: %d checks passed\n", checks);
    return 0;
}
