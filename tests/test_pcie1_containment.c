#include <stdio.h>
#include <string.h>

#include "types.h"
#include "pcie1_containment.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

#define CONTRACT_ID 189U
#define ENDPOINT_GENERATION 23ULL
#define ENDPOINT_BDF pcie1_containment_bdf(1U, 0U, 0U)

static void init_contract(
    struct pcie1_containment *contract,
    struct pcie1_containment_endpoint_handle *endpoint)
{
    memset(contract, 0, sizeof(*contract));
    CHECK(pcie1_containment_init(contract, CONTRACT_ID, ENDPOINT_BDF,
                                 ENDPOINT_GENERATION, 0U, endpoint));
}

static void bind_and_arm(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint)
{
    CHECK(pcie1_containment_msi_bind(
        contract, endpoint, 0U, PIOS_PCIE1_MSI_IRQ,
        AIRQ_SRC_PCIE1_MSI, 0U, true, true));
    CHECK(pcie1_containment_msi_arm(contract, endpoint, 0U));
}

static struct pcie1_containment_msi_ticket post_msi(
    struct pcie1_containment *contract,
    const struct pcie1_containment_endpoint_handle *endpoint, u64 now_ms)
{
    struct pcie1_containment_msi_ticket ticket;

    CHECK(pcie1_containment_msi_top_half(
        contract, endpoint, 0U, PIOS_PCIE1_MSI_IRQ, 0x55U, true, true,
        now_ms, 10U, &ticket));
    CHECK(pcie1_containment_msi_publish_result(contract, &ticket, 0U, true));
    CHECK(pcie1_containment_msi_dispatch_begin(
        contract, &ticket, 0U));
    return ticket;
}

static struct pcie1_containment_completion make_completion(
    const struct pcie1_containment_dma_handle *handle,
    u32 actual, bool success)
{
    struct pcie1_containment_completion completion = {
        .endpoint_generation = handle->endpoint_generation,
        .slot_generation = handle->generation,
        .request_id = handle->request_id,
        .actual = actual,
        .success = success ? 1U : 0U,
    };
    return completion;
}

static void test_layout_and_initialization(void)
{
    struct pcie1_containment contract;
    struct pcie1_containment_endpoint_handle endpoint;

    CHECK(PIOS_HAS_PCIE1 == 1);
    CHECK(PIOS_DMA_PCIE1_SIZE == 0x00200000UL);
    CHECK(PCIE1_CONTAINMENT_DMA_SLOT_COUNT == 8U);
    CHECK(PCIE1_CONTAINMENT_DMA_SLOT_BYTES == 0x00040000U);
    CHECK(PCIE1_CONTAINMENT_DMA_PAYLOAD_BYTES == 0x0003FF80U);
    CHECK(PCIE1_CONTAINMENT_DMA_SLOT_COUNT *
          PCIE1_CONTAINMENT_DMA_SLOT_BYTES == PIOS_DMA_PCIE1_SIZE);
    CHECK(sizeof(struct pcie1_containment_owner) == 64U);
    CHECK(sizeof(struct pcie1_containment_msi_control) == 64U);
    CHECK(sizeof(struct pcie1_containment_dma_control) == 64U);
    CHECK(sizeof(struct pcie1_containment_dma_span) == 64U);
    CHECK(sizeof(struct pcie1_containment_endpoint_handle) == 32U);
    CHECK(sizeof(struct pcie1_containment_dma_handle) == 40U);
    CHECK(sizeof(struct pcie1_containment_msi_ticket) == 40U);
    CHECK(sizeof(struct pcie1_containment_completion) == 32U);
    CHECK((usize)&((struct pcie1_containment *)0)->msi == 64U);
    CHECK((usize)&((struct pcie1_containment *)0)->dma == 128U);
    CHECK((usize)&((struct pcie1_containment *)0)->spans ==
          128U + PCIE1_CONTAINMENT_DMA_SLOT_COUNT * 64U);
    CHECK(AIRQ_SRC_PCIE1_MSI == 14U);

    CHECK(pcie1_containment_bdf_valid(ENDPOINT_BDF));
    CHECK(pcie1_containment_bdf_valid(
        pcie1_containment_bdf(255U, 31U, 7U)));
    CHECK(!pcie1_containment_bdf_valid(
        pcie1_containment_bdf(0U, 0U, 0U)));
    CHECK(pcie1_containment_bdf(1U, 32U, 0U) == 0U);
    CHECK(pcie1_containment_bdf(1U, 0U, 8U) == 0U);
    CHECK(pcie1_containment_bdf(256U, 0U, 0U) == 0U);
    CHECK(!pcie1_containment_bdf_valid(0x10000U));

    memset(&contract, 0, sizeof(contract));
    CHECK(!pcie1_containment_init(NULL, CONTRACT_ID, ENDPOINT_BDF,
                                  ENDPOINT_GENERATION, 0U, &endpoint));
    CHECK(!pcie1_containment_init(&contract, 0U, ENDPOINT_BDF,
                                  ENDPOINT_GENERATION, 0U, &endpoint));
    CHECK(!pcie1_containment_init(&contract, CONTRACT_ID, ENDPOINT_BDF,
                                  0U, 0U, &endpoint));
    CHECK(!pcie1_containment_init(&contract, CONTRACT_ID, ENDPOINT_BDF,
                                  ENDPOINT_GENERATION, 1U, &endpoint));
    CHECK(!pcie1_containment_init(&contract, CONTRACT_ID, 0U,
                                  ENDPOINT_GENERATION, 0U, &endpoint));
    contract.owner._pad[0] = 1U;
    CHECK(!pcie1_containment_init(&contract, CONTRACT_ID, ENDPOINT_BDF,
                                  ENDPOINT_GENERATION, 0U, &endpoint));

    init_contract(&contract, &endpoint);
    CHECK(endpoint.contract_id == CONTRACT_ID);
    CHECK(endpoint.generation == ENDPOINT_GENERATION);
    CHECK(endpoint.bdf == ENDPOINT_BDF);
    CHECK(contract.owner.state == PCIE1_CONTAINMENT_ENDPOINT_ATTACHED);
    CHECK(contract.owner.fault == PCIE1_CONTAINMENT_FAULT_NONE);
    CHECK(contract.msi.state == PCIE1_CONTAINMENT_MSI_UNBOUND);
    CHECK(!pcie1_containment_init(&contract, CONTRACT_ID, ENDPOINT_BDF,
                                  ENDPOINT_GENERATION, 0U, &endpoint));
    CHECK(!pcie1_containment_hardware_enable_allowed(
        &contract, &endpoint, 0U));
}

static void test_dma_slots_and_saturation(void)
{
    struct pcie1_containment contract;
    struct pcie1_containment_endpoint_handle endpoint;
    struct pcie1_containment_dma_handle
        handles[PCIE1_CONTAINMENT_DMA_SLOT_COUNT];
    struct pcie1_containment_dma_handle extra;
    struct pcie1_containment_dma_span span;
    u32 i;
    u32 j;

    init_contract(&contract, &endpoint);
    for (i = 0U; i < PCIE1_CONTAINMENT_DMA_SLOT_COUNT; i++) {
        u32 requested = 64U * (i + 1U);
        u64 allocation_cpu = PIOS_DMA_PCIE1_BASE +
            (u64)i * PCIE1_CONTAINMENT_DMA_SLOT_BYTES;
        u64 allocation_iova = PCIE1_CONTAINMENT_IOVA_BASE +
            (u64)i * PCIE1_CONTAINMENT_DMA_SLOT_BYTES;
        enum pcie1_containment_dma_direction direction =
            (i & 1U) ? PCIE1_CONTAINMENT_FROM_DEVICE :
                       PCIE1_CONTAINMENT_TO_DEVICE;

        CHECK(pcie1_containment_dma_acquire(
            &contract, &endpoint, 0U, 100U + i, requested, direction,
            1000U, 50U, &handles[i]));
        CHECK(pcie1_containment_dma_span_get(
            &contract, &handles[i], 0U, &span));
        CHECK(span.payload_cpu_phys == allocation_cpu +
              PCIE1_CONTAINMENT_DMA_GUARD_BYTES);
        CHECK(span.payload_iova == allocation_iova +
              PCIE1_CONTAINMENT_DMA_GUARD_BYTES);
        CHECK(span.endpoint_generation == ENDPOINT_GENERATION);
        CHECK(span.slot_generation == handles[i].generation);
        CHECK(span.request_id == 100U + i);
        CHECK(span.used == (direction == PCIE1_CONTAINMENT_TO_DEVICE ?
                            requested : 0U));
        CHECK(span.requested == requested);
        CHECK(span.capacity == PCIE1_CONTAINMENT_DMA_PAYLOAD_BYTES);
        CHECK(span.slot == i);
        CHECK(span.direction == (u32)direction);
        CHECK(span.cache_policy == PCIE1_CONTAINMENT_CACHE_NORMAL_NC);
        CHECK(span.guard_lines == 1U);
        CHECK((span.payload_cpu_phys &
               (PCIE1_CONTAINMENT_DMA_ALIGNMENT - 1U)) == 0U);
        CHECK((span.payload_iova &
               (PCIE1_CONTAINMENT_DMA_ALIGNMENT - 1U)) == 0U);
        for (j = 0U; j < i; j++)
            CHECK(handles[i].token != handles[j].token);
    }
    memset(&extra, 0xA5, sizeof(extra));
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 999U, 64U,
        PCIE1_CONTAINMENT_TO_DEVICE, 0U, 1U, &extra));
    CHECK(extra.token == 0U && extra.generation == 0U &&
          extra.endpoint_generation == 0U && extra.request_id == 0U &&
          extra.contract_id == 0U && extra._reserved == 0U);

    for (i = 0U; i < PCIE1_CONTAINMENT_DMA_SLOT_COUNT; i++) {
        CHECK(pcie1_containment_dma_cancel(
            &contract, &endpoint, &handles[i], 0U));
        CHECK(pcie1_containment_dma_release(
            &contract, &endpoint, &handles[i], 0U));
    }
}

static void test_validation_and_msi_continuation(void)
{
    struct pcie1_containment contract;
    struct pcie1_containment_endpoint_handle endpoint;
    struct pcie1_containment_endpoint_handle forged_endpoint;
    struct pcie1_containment_dma_handle dma;
    struct pcie1_containment_dma_handle stale;
    struct pcie1_containment_msi_ticket ticket;
    struct pcie1_containment_msi_ticket pending;
    struct pcie1_containment_completion completion;
    struct pcie1_containment_status status;
    enum pcie1_containment_dma_state state;
    u32 actual;
    bool success;

    init_contract(&contract, &endpoint);
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 1U, 0U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 0U, 1U, &dma));
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 1U, 63U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 0U, 1U, &dma));
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 1U,
        PCIE1_CONTAINMENT_DMA_PAYLOAD_BYTES + 64U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 0U, 1U, &dma));
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 0U, 64U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 0U, 1U, &dma));
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 1U, 64U,
        (enum pcie1_containment_dma_direction)3U, 0U, 1U, &dma));
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 1U, 64U,
        PCIE1_CONTAINMENT_FROM_DEVICE, ~0ULL, 1U, &dma));
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 1U, 64U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 0U,
        PCIE1_CONTAINMENT_TIMEOUT_MAX_MS + 1U, &dma));

    forged_endpoint = endpoint;
    forged_endpoint.generation++;
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &forged_endpoint, 0U, 1U, 64U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 0U, 1U, &dma));
    CHECK(contract.owner.state == PCIE1_CONTAINMENT_ENDPOINT_ATTACHED);

    CHECK(!pcie1_containment_msi_bind(
        &contract, &endpoint, 1U, PIOS_PCIE1_MSI_IRQ,
        AIRQ_SRC_PCIE1_MSI, 0U, true, true));
    CHECK(!pcie1_containment_msi_bind(
        &contract, &endpoint, 0U, 99U,
        AIRQ_SRC_PCIE1_MSI, 0U, true, true));
    CHECK(!pcie1_containment_msi_bind(
        &contract, &endpoint, 0U, PIOS_PCIE1_MSI_IRQ,
        AIRQ_SRC_PCIE1_MSI, 0U, false, true));
    CHECK(!pcie1_containment_msi_bind(
        &contract, &endpoint, 0U, PIOS_PCIE1_MSI_IRQ,
        AIRQ_SRC_PCIE1_MSI, 0U, true, false));
    bind_and_arm(&contract, &endpoint);

    CHECK(pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 0x1234U, 4096U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 100U, 40U, &dma));
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 0x1234U, 64U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 100U, 40U, &stale));
    stale = dma;
    CHECK(pcie1_containment_dma_publish(
        &contract, &endpoint, &dma, 0U));
    CHECK(!pcie1_containment_dma_cancel(
        &contract, &endpoint, &dma, 0U));
    CHECK(!pcie1_containment_dma_release(
        &contract, &endpoint, &dma, 0U));

    CHECK(pcie1_containment_msi_top_half(
        &contract, &endpoint, 0U, PIOS_PCIE1_MSI_IRQ, 1U, true, true,
        110U, 10U, &ticket));
    CHECK(pcie1_containment_msi_publish_result(
        &contract, &ticket, 0U, false));
    CHECK(contract.msi.state == PCIE1_CONTAINMENT_MSI_EVENT_PENDING);
    CHECK(pcie1_containment_msi_pending_ticket(
        &contract, 0U, &pending));
    CHECK(memcmp(&pending, &ticket, sizeof(ticket)) == 0);
    CHECK(pcie1_containment_msi_publish_result(
        &contract, &pending, 0U, true));
    CHECK(contract.msi.state == PCIE1_CONTAINMENT_MSI_QUEUED);
    CHECK(!pcie1_containment_msi_publish_result(
        &contract, &pending, 0U, true));
    completion = make_completion(&dma, 2048U, true);
    CHECK(!pcie1_containment_completion_submit(
        &contract, &endpoint, &ticket, &completion, 0U, true));
    CHECK(pcie1_containment_msi_dispatch_begin(
        &contract, &ticket, 0U));
    CHECK(pcie1_containment_completion_submit(
        &contract, &endpoint, &ticket, &completion, 0U, true));
    CHECK(pcie1_containment_dma_state_get(
        &contract, &dma, 0U, &state));
    CHECK(state == PCIE1_CONTAINMENT_DMA_CPU_COMPLETE);
    actual = 0U;
    success = false;
    CHECK(pcie1_containment_dma_completion_get(
        &contract, &dma, 0U, &actual, &success));
    CHECK(actual == 2048U && success);
    {
        struct pcie1_containment_dma_span span;
        CHECK(pcie1_containment_dma_span_get(
            &contract, &dma, 0U, &span));
        CHECK(span.used == 0U && span.requested == 4096U);
    }
    CHECK(pcie1_containment_msi_dispatch_complete(
        &contract, &ticket, 0U));
    CHECK(contract.msi.state == PCIE1_CONTAINMENT_MSI_MASKED);
    CHECK(pcie1_containment_dma_release(
        &contract, &endpoint, &dma, 0U));
    CHECK(!pcie1_containment_dma_state_get(
        &contract, &stale, 0U, &state));
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 0x1234U, 64U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 200U, 10U, &dma));
    CHECK(pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 0x1235U, 64U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 200U, 10U, &dma));
    CHECK(pcie1_containment_dma_cancel(
        &contract, &endpoint, &dma, 0U));
    CHECK(pcie1_containment_dma_release(
        &contract, &endpoint, &dma, 0U));
    CHECK(pcie1_containment_msi_arm(&contract, &endpoint, 0U));

    CHECK(pcie1_containment_status_get(&contract, 0U, &status));
    CHECK(status.endpoint_state == PCIE1_CONTAINMENT_ENDPOINT_ATTACHED);
    CHECK(status.msi_posted == 1U && status.msi_deferred == 1U);
    CHECK(status.msi_event_seq == 1U);
    CHECK(status.active_dma == 0U && status.device_owned_dma == 0U);
    CHECK(!pcie1_containment_hardware_enable_allowed(
        &contract, &endpoint, 0U));
}

static void prepare_in_flight(
    struct pcie1_containment *contract,
    struct pcie1_containment_endpoint_handle *endpoint,
    struct pcie1_containment_dma_handle *dma,
    u64 request_id, u64 now_ms)
{
    init_contract(contract, endpoint);
    bind_and_arm(contract, endpoint);
    CHECK(pcie1_containment_dma_acquire(
        contract, endpoint, 0U, request_id, 256U,
        PCIE1_CONTAINMENT_FROM_DEVICE, now_ms, 10U, dma));
    CHECK(pcie1_containment_dma_publish(
        contract, endpoint, dma, 0U));
}

static void test_fault_containment(void)
{
    struct pcie1_containment contract;
    struct pcie1_containment_endpoint_handle endpoint;
    struct pcie1_containment_dma_handle dma;
    struct pcie1_containment_dma_handle second;
    struct pcie1_containment_msi_ticket ticket;
    struct pcie1_containment_completion completion;
    struct pcie1_containment_status status;
    enum pcie1_containment_dma_state state;

    prepare_in_flight(&contract, &endpoint, &dma, 90U, 100U);
    ticket = post_msi(&contract, &endpoint, 101U);
    completion = make_completion(&dma, 256U, true);
    completion.request_id = 91U;
    CHECK(!pcie1_containment_completion_submit(
        &contract, &endpoint, &ticket, &completion, 0U, true));
    CHECK(pcie1_containment_status_get(&contract, 0U, &status));
    CHECK(status.endpoint_state ==
          PCIE1_CONTAINMENT_ENDPOINT_QUARANTINED);
    CHECK(status.fault == PCIE1_CONTAINMENT_FAULT_COMPLETION);
    CHECK(status.msi_state == PCIE1_CONTAINMENT_MSI_QUARANTINED);
    CHECK(pcie1_containment_dma_state_get(
        &contract, &dma, 0U, &state));
    CHECK(state == PCIE1_CONTAINMENT_DMA_FAILED);
    CHECK(pcie1_containment_dma_release(
        &contract, &endpoint, &dma, 0U));

    prepare_in_flight(&contract, &endpoint, &dma, 100U, 200U);
    ticket = post_msi(&contract, &endpoint, 201U);
    completion = make_completion(&dma, 128U, true);
    CHECK(!pcie1_containment_completion_submit(
        &contract, &endpoint, &ticket, &completion, 0U, false));
    CHECK(contract.owner.fault == PCIE1_CONTAINMENT_FAULT_RED_ZONE);

    prepare_in_flight(&contract, &endpoint, &dma, 105U, 250U);
    ticket = post_msi(&contract, &endpoint, 251U);
    completion = make_completion(&dma, 128U, true);
    completion.slot_generation++;
    CHECK(!pcie1_containment_completion_submit(
        &contract, &endpoint, &ticket, &completion, 0U, true));
    CHECK(contract.owner.fault == PCIE1_CONTAINMENT_FAULT_COMPLETION);

    prepare_in_flight(&contract, &endpoint, &dma, 110U, 300U);
    CHECK(!pcie1_containment_dma_expire(
        &contract, &endpoint, &dma, 0U, 309U));
    CHECK(pcie1_containment_dma_expire(
        &contract, &endpoint, &dma, 0U, 310U));
    CHECK(contract.owner.fault == PCIE1_CONTAINMENT_FAULT_TIMEOUT);

    prepare_in_flight(&contract, &endpoint, &dma, 120U, 400U);
    CHECK(pcie1_containment_msi_top_half(
        &contract, &endpoint, 0U, PIOS_PCIE1_MSI_IRQ, 1U, true, true,
        401U, 10U, &ticket));
    CHECK(!pcie1_containment_msi_expire(
        &contract, &endpoint, 0U, 410U - 1U));
    CHECK(pcie1_containment_msi_expire(
        &contract, &endpoint, 0U, 411U));
    CHECK(contract.owner.fault == PCIE1_CONTAINMENT_FAULT_TIMEOUT);

    init_contract(&contract, &endpoint);
    CHECK(pcie1_containment_quarantine(
        &contract, &endpoint, 0U, PCIE1_CONTAINMENT_FAULT_AER));
    CHECK(contract.owner.fault == PCIE1_CONTAINMENT_FAULT_AER);
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 1U, 64U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 0U, 1U, &dma));

    init_contract(&contract, &endpoint);
    CHECK(pcie1_containment_remove(&contract, &endpoint, 0U));
    CHECK(contract.owner.state == PCIE1_CONTAINMENT_ENDPOINT_REMOVED);
    CHECK(contract.owner.fault == PCIE1_CONTAINMENT_FAULT_REMOVED);

    init_contract(&contract, &endpoint);
    CHECK(pcie1_containment_msi_bind(
        &contract, &endpoint, 0U, PIOS_PCIE1_MSI_IRQ,
        AIRQ_SRC_PCIE1_MSI, 0U, true, true));
    CHECK(!pcie1_containment_msi_top_half(
        &contract, &endpoint, 0U, PIOS_PCIE1_MSI_IRQ, 1U, true, true,
        0U, 1U, &ticket));
    CHECK(contract.owner.fault == PCIE1_CONTAINMENT_FAULT_MSI_STORM);
    CHECK(contract.msi.storm_count == 1U);

    init_contract(&contract, &endpoint);
    CHECK(pcie1_containment_msi_bind(
        &contract, &endpoint, 0U, PIOS_PCIE1_MSI_IRQ,
        AIRQ_SRC_PCIE1_MSI, 0U, true, true));
    CHECK(!pcie1_containment_msi_top_half(
        &contract, &endpoint, 0U, PIOS_PCIE1_MSI_IRQ, 1U, false, false,
        0U, 1U, &ticket));
    CHECK(contract.owner.fault ==
          PCIE1_CONTAINMENT_FAULT_MSI_PROTOCOL);
    CHECK(ticket.token == 0U);

    prepare_in_flight(&contract, &endpoint, &dma, 130U, 500U);
    CHECK(pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 131U, 256U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 500U, 10U, &second));
    ticket = post_msi(&contract, &endpoint, 501U);
    completion = make_completion(&dma, 257U, true);
    CHECK(!pcie1_containment_completion_submit(
        &contract, &endpoint, &ticket, &completion, 0U, true));
    CHECK(pcie1_containment_dma_state_get(
        &contract, &second, 0U, &state));
    CHECK(state == PCIE1_CONTAINMENT_DMA_FAILED);
}

static void test_generation_exhaustion(void)
{
    struct pcie1_containment contract;
    struct pcie1_containment_endpoint_handle endpoint;
    struct pcie1_containment_dma_handle dma;
    struct pcie1_containment_dma_handle next;
    struct pcie1_containment_msi_ticket ticket;
    struct pcie1_containment_completion completion;
    u32 i;

    init_contract(&contract, &endpoint);
    bind_and_arm(&contract, &endpoint);
    contract.dma[0].generation = ~0ULL;
    for (i = 1U; i < PCIE1_CONTAINMENT_DMA_SLOT_COUNT; i++)
        contract.dma[i].exhausted = 1U;
    CHECK(pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 500U, 64U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 0U, 10U, &dma));
    CHECK(dma.generation == ~0ULL);
    CHECK(pcie1_containment_dma_publish(
        &contract, &endpoint, &dma, 0U));
    ticket = post_msi(&contract, &endpoint, 1U);
    completion = make_completion(&dma, 64U, true);
    CHECK(pcie1_containment_completion_submit(
        &contract, &endpoint, &ticket, &completion, 0U, true));
    CHECK(pcie1_containment_dma_release(
        &contract, &endpoint, &dma, 0U));
    CHECK(contract.dma[0].state == PCIE1_CONTAINMENT_DMA_RETIRED);
    CHECK(!pcie1_containment_dma_acquire(
        &contract, &endpoint, 0U, 501U, 64U,
        PCIE1_CONTAINMENT_FROM_DEVICE, 2U, 10U, &next));

    init_contract(&contract, &endpoint);
    bind_and_arm(&contract, &endpoint);
    contract.msi.event_seq = ~0ULL;
    CHECK(!pcie1_containment_msi_top_half(
        &contract, &endpoint, 0U, PIOS_PCIE1_MSI_IRQ, 1U, true, true,
        0U, 1U, &ticket));
    CHECK(contract.owner.fault == PCIE1_CONTAINMENT_FAULT_GENERATION);
    CHECK(contract.msi.exhausted == 1U);
}

int main(void)
{
    test_layout_and_initialization();
    test_dma_slots_and_saturation();
    test_validation_and_msi_continuation();
    test_fault_containment();
    test_generation_exhaustion();

    CHECK(checks > 250);
    if (failures) {
        printf("pcie1 containment: %d/%d checks failed\n",
               failures, checks);
        return 1;
    }
    printf("pcie1 containment: %d checks passed\n", checks);
    return 0;
}
