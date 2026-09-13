#include <stdio.h>
#include <string.h>

#include "types.h"
#include "media_admission.h"

static int failures;
static int checks;
static u32 test_core;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

u32 pios_host_core_id(void)
{
    return test_core;
}

struct fixture {
    struct media_engine_controller engines;
    struct media_admission admission;
};

static u32 raw_version(enum media_engine_kind kind)
{
    if (kind == MEDIA_ENGINE_HEVC)
        return 0x202U;
    if (kind == MEDIA_ENGINE_PISP_BE)
        return 0x0225270FU;
    if (kind == MEDIA_ENGINE_HVS)
        return 0x53U;
    return 0U;
}

static void fixture_init(struct fixture *fixture, u32 id)
{
    memset(fixture, 0, sizeof(*fixture));
    test_core = 0U;
    CHECK(media_engine_controller_init(&fixture->engines, id));
    CHECK(media_admission_init(&fixture->admission, id, &fixture->engines, 0U));
}

static bool acquire(struct fixture *fixture, enum media_engine_kind engine,
                    struct media_engine_lease *lease)
{
    return media_engine_passive_identify(&fixture->engines, engine,
                                         raw_version(engine)) &&
           media_engine_lease_acquire(&fixture->engines, engine, lease);
}

static void release_complete(struct fixture *fixture,
                             struct media_engine_lease *lease)
{
    CHECK(media_engine_lease_complete(&fixture->engines, lease));
    CHECK(media_engine_lease_release(&fixture->engines, lease));
}

static struct media_admission_job_request request_for(
    enum media_engine_kind engine, u64 job_id, u64 span_start)
{
    const struct media_engine_descriptor *desc = media_engine_descriptor(engine);
    struct media_admission_job_request request;
    u32 i;

    memset(&request, 0, sizeof(request));
    request.job_id = job_id;
    request.engine = engine;
    request.resources.clock_count = desc ? desc->clock_count : 0U;
    request.resources.iommu_resource = desc ? desc->iommu_resource : 0U;
    request.resources.gic_spi = desc ? desc->gic_spi : 0U;
    for (i = 0U; desc && i < desc->clock_count; i++)
        request.resources.clock_ids[i] = desc->clock_ids[i];
    request.resources.dma.span_id = job_id + 1000U;
    request.resources.dma.span_start = span_start;
    request.resources.dma.span_bytes = 64U;
    request.resources.dma.span_capacity = 64U;
    request.resources.dma.generation = 1U;
    return request;
}

static void test_layout_and_owner_guards(void)
{
    struct fixture fixture;
    struct media_admission bad;
    struct media_engine_controller engines;
    struct media_admission_plan_handle plan;

    CHECK(MEDIA_ADMISSION_MAX_JOBS == 8U);
    CHECK(sizeof(struct media_admission_job_descriptor) == 128U);
    CHECK(sizeof(struct media_admission_job_control) == 64U);
    CHECK(sizeof(struct media_admission_plan_control) == 64U);
    CHECK(sizeof(struct media_admission_replay_event) == 64U);
    CHECK(!media_admission_hardware_enable_allowed(NULL, 0U));
    memset(&bad, 0, sizeof(bad));
    memset(&engines, 0, sizeof(engines));
    CHECK(!media_admission_init(NULL, 1U, &engines, 0U));
    CHECK(!media_admission_init(&bad, 0U, &engines, 0U));
    CHECK(!media_admission_init(&bad, 1U, NULL, 0U));
    CHECK(media_engine_controller_init(&engines, 11U));
    CHECK(!media_admission_init(&bad, 12U, &engines, 0U));
    test_core = 1U;
    CHECK(!media_admission_init(&bad, 11U, &engines, 0U));
    test_core = 0U;
    CHECK(media_admission_init(&bad, 11U, &engines, 0U));
    CHECK(!media_admission_init(&bad, 11U, &engines, 0U));
    CHECK(!media_admission_hardware_enable_allowed(&bad, 0U));
    CHECK(!media_admission_plan_create(&bad, 1U, 1U, &plan));
    test_core = 1U;
    CHECK(!media_admission_plan_create(&bad, 1U, 0U, &plan));
    test_core = 0U;

    fixture_init(&fixture, 1U);
    fixture.admission.owner.transition_active = 1U;
    CHECK(!media_admission_plan_create(&fixture.admission, 1U, 0U, &plan));
    fixture.admission.owner.transition_active = 0U;
}

static void test_plan_and_resource_validation(void)
{
    struct fixture fixture;
    struct media_engine_lease lease;
    struct media_admission_plan_handle plan, duplicate;
    struct media_admission_job_handle job, duplicate_job;
    struct media_admission_job_request request;
    u32 which;

    fixture_init(&fixture, 2U);
    CHECK(media_admission_plan_create(&fixture.admission, 7U, 0U, &plan));
    CHECK(plan.plan_id == 7U && plan.generation == 1U && plan.slot == 0U);
    CHECK(!media_admission_plan_create(&fixture.admission, 7U, 0U, &duplicate));
    CHECK(duplicate.plan_id == 0U && duplicate.generation == 0U);
    CHECK(!media_admission_plan_create(&fixture.admission, 0U, 0U, &duplicate));
    CHECK(acquire(&fixture, MEDIA_ENGINE_HEVC, &lease));

    for (which = 0U; which < 15U; which++) {
        request = request_for(MEDIA_ENGINE_HEVC, 100U + which,
                              0x1000U + (u64)which * 0x100U);
        if (which == 0U) request.job_id = 0U;
        if (which == 1U) request._reserved = 1U;
        if (which == 2U) request.resources._reserved = 1U;
        if (which == 3U) request.input.producer_generation = 1U;
        if (which == 4U) request.input.producer_job_id = 99U;
        if (which == 5U) request.resources.clock_count = 0U;
        if (which == 6U) request.resources.clock_ids[0]++;
        if (which == 7U) request.resources.iommu_resource++;
        if (which == 8U) request.resources.gic_spi++;
        if (which == 9U) request.resources.dma.span_id = 0U;
        if (which == 10U) request.resources.dma.span_bytes = 0U;
        if (which == 11U) request.resources.dma.span_capacity = 63U;
        if (which == 12U) request.resources.dma.generation = 0U;
        if (which == 13U) request.resources.dma._reserved = 1U;
        if (which == 14U) {
            request.resources.dma.span_start = ~0ULL - 63U;
            request.resources.dma.span_bytes = 64U;
        }
        CHECK(!media_admission_job_prepare(&fixture.admission, &plan, &request,
                                           &lease, 0U, &job));
        CHECK(job.job_id == 0U && job.job_generation == 0U);
    }

    request = request_for(MEDIA_ENGINE_PISP_FE, 250U, 0x3000U);
    CHECK(!media_admission_job_prepare(&fixture.admission, &plan, &request,
                                       &lease, 0U, &job));
    request = request_for(MEDIA_ENGINE_HVS, 251U, 0x4000U);
    CHECK(!media_admission_job_prepare(&fixture.admission, &plan, &request,
                                       &lease, 0U, &job));
    request = request_for(MEDIA_ENGINE_INVALID, 252U, 0x5000U);
    CHECK(!media_admission_job_prepare(&fixture.admission, &plan, &request,
                                       &lease, 0U, &job));

    request = request_for(MEDIA_ENGINE_HEVC, 300U, 0x6000U);
    CHECK(media_admission_job_prepare(&fixture.admission, &plan, &request,
                                      &lease, 0U, &job));
    CHECK(job.job_id == 300U && job.job_generation == 1U);
    CHECK(!media_admission_job_prepare(&fixture.admission, &plan, &request,
                                       &lease, 0U, &duplicate_job));
    request = request_for(MEDIA_ENGINE_HEVC, 301U, 0x6100U);
    CHECK(!media_admission_job_prepare(&fixture.admission, &plan, &request,
                                       &lease, 0U, &duplicate_job));
    request = request_for(MEDIA_ENGINE_HEVC, 302U, 0x6000U + 32U);
    CHECK(!media_admission_job_prepare(&fixture.admission, &plan, &request,
                                       &lease, 0U, &duplicate_job));
    request = request_for(MEDIA_ENGINE_HEVC, 303U, 0x7000U);
    request.resources.dma.span_id = 1300U;
    CHECK(!media_admission_job_prepare(&fixture.admission, &plan, &request,
                                       &lease, 0U, &duplicate_job));
}

static void test_dependency_serial_cross_engine_flow(void)
{
    struct fixture fixture;
    struct media_engine_lease hevc, pisp;
    struct media_admission_plan_handle plan;
    struct media_admission_job_handle producer, consumer;
    struct media_admission_job_request request;
    enum media_admission_plan_state plan_state;
    enum media_admission_job_state job_state;
    enum media_admission_fault fault;
    struct media_admission_snapshot snapshot;
    struct media_admission_replay_event replay[MEDIA_ADMISSION_MAX_REPLAY];
    u32 count;

    fixture_init(&fixture, 3U);
    CHECK(acquire(&fixture, MEDIA_ENGINE_HEVC, &hevc));
    CHECK(media_admission_plan_create(&fixture.admission, 3000U, 0U, &plan));
    request = request_for(MEDIA_ENGINE_HEVC, 1U, 0x10000U);
    CHECK(media_admission_job_prepare(&fixture.admission, &plan, &request,
                                      &hevc, 0U, &producer));
    CHECK(!media_admission_job_admit(&fixture.admission, &producer, 10U, 0U, 0U));
    CHECK(!media_admission_job_admit(&fixture.admission, &producer, ~0ULL, 1U, 0U));
    CHECK(media_admission_plan_state_get(&fixture.admission, &plan, &plan_state,
                                         &fault));
    CHECK(plan_state == MEDIA_ADMISSION_PLAN_FAULTED);
    CHECK(fault == MEDIA_ADMISSION_FAULT_ARGUMENT);
    CHECK(media_admission_job_state_get(&fixture.admission, &producer, &job_state,
                                        &fault));
    CHECK(job_state == MEDIA_ADMISSION_JOB_FAULTED);
    CHECK(!media_admission_plan_retire(&fixture.admission, &plan, 11U, 1U));
    CHECK(media_admission_plan_retire(&fixture.admission, &plan, 11U, 0U));
    CHECK(!media_admission_plan_state_get(&fixture.admission, &plan, &plan_state,
                                          &fault));

    fixture_init(&fixture, 4U);
    CHECK(acquire(&fixture, MEDIA_ENGINE_HEVC, &hevc));
    CHECK(media_admission_plan_create(&fixture.admission, 4000U, 0U, &plan));
    request = request_for(MEDIA_ENGINE_HEVC, 10U, 0x20000U);
    CHECK(media_admission_job_prepare(&fixture.admission, &plan, &request,
                                      &hevc, 0U, &producer));
    CHECK(media_admission_job_admit(&fixture.admission, &producer, 100U, 20U, 0U));
    CHECK(media_admission_job_complete(&fixture.admission, &producer, 110U, true,
                                       true, 0U));
    release_complete(&fixture, &hevc);
    CHECK(acquire(&fixture, MEDIA_ENGINE_PISP_BE, &pisp));
    request = request_for(MEDIA_ENGINE_PISP_BE, 11U, 0x21000U);
    request.input.producer_job_id = producer.job_id;
    request.input.producer_generation = producer.job_generation;
    CHECK(media_admission_job_prepare(&fixture.admission, &plan, &request,
                                      &pisp, 0U, &consumer));
    CHECK(media_admission_job_admit(&fixture.admission, &consumer, 120U, 20U, 0U));
    CHECK(media_admission_job_complete(&fixture.admission, &consumer, 130U, true,
                                       true, 0U));
    CHECK(media_admission_plan_complete(&fixture.admission, &plan, 131U, 0U));
    CHECK(media_admission_plan_state_get(&fixture.admission, &plan, &plan_state,
                                         &fault));
    CHECK(plan_state == MEDIA_ADMISSION_PLAN_COMPLETED);
    CHECK(fault == MEDIA_ADMISSION_FAULT_NONE);
    CHECK(media_admission_snapshot_get(&fixture.admission, &plan, &snapshot));
    CHECK(snapshot.plan_id == 4000U && snapshot.job_count == 2U);
    CHECK(snapshot.completed_count == 2U && snapshot.jobs[0].canary_valid == 1U);
    CHECK(snapshot.jobs[1].engine == MEDIA_ENGINE_PISP_BE);
    CHECK(media_admission_replay_get(&fixture.admission, 0U, replay,
                                     MEDIA_ADMISSION_MAX_REPLAY, &count));
    CHECK(count >= 7U && replay[0].sequence == 1U);
    CHECK(replay[count - 1U].sequence >= replay[0].sequence);
    CHECK(media_admission_plan_retire(&fixture.admission, &plan, 132U, 0U));
    CHECK(!media_admission_job_admit(&fixture.admission, &consumer, 133U, 1U, 0U));
    release_complete(&fixture, &pisp);
}

static void test_dependency_wait_fault_timeout_and_removal(void)
{
    struct fixture fixture;
    struct media_engine_lease lease;
    struct media_admission_plan_handle plan;
    struct media_admission_job_handle first, second;
    struct media_admission_job_request request;
    enum media_admission_plan_state plan_state;
    enum media_admission_job_state job_state;
    enum media_admission_fault fault;
    u32 i;

    /* A dependency waits; it cannot be admitted before a canary-valid output. */
    fixture_init(&fixture, 5U);
    CHECK(acquire(&fixture, MEDIA_ENGINE_HEVC, &lease));
    CHECK(media_admission_plan_create(&fixture.admission, 5000U, 0U, &plan));
    request = request_for(MEDIA_ENGINE_HEVC, 1U, 0x30000U);
    CHECK(media_admission_job_prepare(&fixture.admission, &plan, &request,
                                      &lease, 0U, &first));
    request = request_for(MEDIA_ENGINE_HEVC, 2U, 0x31000U);
    request.input.producer_job_id = first.job_id;
    request.input.producer_generation = first.job_generation;
    CHECK(!media_admission_job_prepare(&fixture.admission, &plan, &request,
                                       &lease, 0U, &second));
    CHECK(media_admission_job_admit(&fixture.admission, &first, 0U, 10U, 0U));
    CHECK(!media_admission_job_complete(&fixture.admission, &first, 2U, true,
                                        false, 0U));
    CHECK(media_admission_plan_state_get(&fixture.admission, &plan, &plan_state,
                                         &fault));
    CHECK(plan_state == MEDIA_ADMISSION_PLAN_FAULTED);
    CHECK(fault == MEDIA_ADMISSION_FAULT_CANARY);
    CHECK(media_admission_job_state_get(&fixture.admission, &first, &job_state,
                                        &fault));
    CHECK(job_state == MEDIA_ADMISSION_JOB_FAULTED);
    CHECK(fault == MEDIA_ADMISSION_FAULT_CANARY);

    /* Every timeout faults the plan and prevents automatic rescheduling. */
    for (i = 0U; i < 8U; i++) {
        fixture_init(&fixture, 100U + i);
        CHECK(acquire(&fixture, MEDIA_ENGINE_HEVC, &lease));
        CHECK(media_admission_plan_create(&fixture.admission, 6000U + i, 0U,
                                          &plan));
        request = request_for(MEDIA_ENGINE_HEVC, 10U + i,
                              0x40000U + (u64)i * 0x1000U);
        CHECK(media_admission_job_prepare(&fixture.admission, &plan, &request,
                                          &lease, 0U, &first));
        CHECK(media_admission_job_admit(&fixture.admission, &first, 50U, 1U, 0U));
        CHECK(media_admission_tick(&fixture.admission, 52U, 0U));
        CHECK(media_admission_plan_state_get(&fixture.admission, &plan,
                                             &plan_state, &fault));
        CHECK(plan_state == MEDIA_ADMISSION_PLAN_FAULTED);
        CHECK(fault == MEDIA_ADMISSION_FAULT_TIMEOUT);
        CHECK(!media_admission_job_complete(&fixture.admission, &first, 52U,
                                            true, true, 0U));
    }

    fixture_init(&fixture, 6U);
    CHECK(acquire(&fixture, MEDIA_ENGINE_HEVC, &lease));
    CHECK(media_admission_plan_create(&fixture.admission, 7000U, 0U, &plan));
    request = request_for(MEDIA_ENGINE_HEVC, 70U, 0x70000U);
    CHECK(media_admission_job_prepare(&fixture.admission, &plan, &request,
                                      &lease, 0U, &first));
    CHECK(media_admission_engine_lost(&fixture.admission, MEDIA_ENGINE_HEVC,
                                      10U, 0U));
    CHECK(media_admission_plan_state_get(&fixture.admission, &plan, &plan_state,
                                         &fault));
    CHECK(plan_state == MEDIA_ADMISSION_PLAN_FAULTED);
    CHECK(fault == MEDIA_ADMISSION_FAULT_ENGINE_LOST);
    CHECK(!media_admission_engine_lost(&fixture.admission, MEDIA_ENGINE_HEVC,
                                       11U, 0U));
    CHECK(!media_admission_engine_lost(&fixture.admission, MEDIA_ENGINE_INVALID,
                                       11U, 0U));
}

static void test_capacity_generations_and_snapshot_matrix(void)
{
    struct fixture fixture;
    struct media_engine_lease lease;
    struct media_admission_plan_handle plans[MEDIA_ADMISSION_MAX_PLANS], stale;
    struct media_admission_job_handle jobs[MEDIA_ADMISSION_MAX_JOBS];
    struct media_admission_job_request request;
    struct media_admission_snapshot snapshot;
    struct media_admission_replay_event events[2];
    enum media_admission_plan_state state;
    enum media_admission_fault fault;
    u32 i, count;

    fixture_init(&fixture, 9U);
    for (i = 0U; i < MEDIA_ADMISSION_MAX_PLANS; i++) {
        CHECK(media_admission_plan_create(&fixture.admission, 9000U + i, 0U,
                                          &plans[i]));
        CHECK(plans[i].slot == i);
    }
    CHECK(!media_admission_plan_create(&fixture.admission, 9999U, 0U, &stale));

    /* A fixed plan rejects a ninth immutable descriptor. */
    fixture_init(&fixture, 10U);
    CHECK(acquire(&fixture, MEDIA_ENGINE_HEVC, &lease));
    CHECK(media_admission_plan_create(&fixture.admission, 10000U, 0U, &plans[0]));
    for (i = 0U; i < MEDIA_ADMISSION_MAX_JOBS; i++) {
        request = request_for(MEDIA_ENGINE_HEVC, 100U + i,
                              0x80000U + (u64)i * 0x100U);
        if (i != 0U) {
            fixture.admission.controls[0][i - 1U].state =
                MEDIA_ADMISSION_JOB_COMPLETED;
        }
        CHECK(media_admission_job_prepare(&fixture.admission, &plans[0], &request,
                                          &lease, 0U, &jobs[i]));
        CHECK(jobs[i].job_slot == i && jobs[i].job_generation == 1U);
    }
    request = request_for(MEDIA_ENGINE_HEVC, 200U, 0x90000U);
    CHECK(!media_admission_job_prepare(&fixture.admission, &plans[0], &request,
                                       &lease, 0U, &jobs[0]));
    CHECK(media_admission_snapshot_get(&fixture.admission, &plans[0], &snapshot));
    CHECK(snapshot.job_count == MEDIA_ADMISSION_MAX_JOBS);
    for (i = 0U; i < MEDIA_ADMISSION_MAX_JOBS; i++) {
        CHECK(snapshot.jobs[i].job_id == 100U + i);
        CHECK(snapshot.jobs[i].generation == 1U);
        CHECK(snapshot.jobs[i].state != MEDIA_ADMISSION_JOB_EMPTY);
    }
    CHECK(media_admission_replay_get(&fixture.admission, 0U, events, 0U, &count));
    CHECK(count == 0U);
    CHECK(media_admission_replay_get(&fixture.admission, 0U, events, 2U, &count));
    CHECK(count == 2U && events[0].sequence == 1U);

    /* Retired plan generations make the old capability permanently stale. */
    fixture_init(&fixture, 11U);
    CHECK(acquire(&fixture, MEDIA_ENGINE_HEVC, &lease));
    CHECK(media_admission_plan_create(&fixture.admission, 11000U, 0U, &plans[0]));
    request = request_for(MEDIA_ENGINE_HEVC, 1U, 0xA0000U);
    CHECK(media_admission_job_prepare(&fixture.admission, &plans[0], &request,
                                      &lease, 0U, &jobs[0]));
    CHECK(media_admission_job_admit(&fixture.admission, &jobs[0], 0U, 1U, 0U));
    CHECK(media_admission_job_complete(&fixture.admission, &jobs[0], 1U, true,
                                       true, 0U));
    CHECK(media_admission_plan_complete(&fixture.admission, &plans[0], 1U, 0U));
    stale = plans[0];
    CHECK(media_admission_plan_retire(&fixture.admission, &plans[0], 2U, 0U));
    CHECK(media_admission_plan_create(&fixture.admission, 11000U, 0U, &plans[1]));
    CHECK(plans[1].generation != stale.generation);
    CHECK(!media_admission_plan_state_get(&fixture.admission, &stale,
                                          &state, &fault));
    CHECK(!media_admission_job_admit(&fixture.admission, &jobs[0], 2U, 1U, 0U));
}

int main(void)
{
    test_layout_and_owner_guards();
    test_plan_and_resource_validation();
    test_dependency_serial_cross_engine_flow();
    test_dependency_wait_fault_timeout_and_removal();
    test_capacity_generations_and_snapshot_matrix();

    if (failures) {
        printf("media admission: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("media admission: %d checks passed\n", checks);
    return 0;
}
