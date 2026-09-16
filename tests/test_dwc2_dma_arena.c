#include <stdio.h>
#include <string.h>

#include "types.h"
#include "platform.h"
#include "dwc2_dma_arena.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

#define CONTROLLER_A 176U
#define CONTROLLER_B 177U

static void init_arena(struct dwc2_dma_arena *arena)
{
    memset(arena, 0, sizeof(*arena));
    CHECK(dwc2_dma_arena_init(arena, CONTROLLER_A, 0U));
}

static void expect_state(const struct dwc2_dma_arena *arena,
                         const struct dwc2_dma_arena_handle *handle,
                         enum dwc2_dma_arena_state expected)
{
    enum dwc2_dma_arena_state state = DWC2_DMA_ARENA_RELEASED;

    CHECK(dwc2_dma_arena_state_get(arena, 0U, handle, &state));
    CHECK(state == expected);
}

static void test_layout_and_one_shot_init(void)
{
    struct dwc2_dma_arena arena;

    CHECK(PIOS_PLATFORM == PIOS_PLATFORM_PI3);
    CHECK(PIOS_DWC2_DMA_BASE == 0x06400000UL);
    CHECK(PIOS_DWC2_DMA_SIZE == 0x00200000UL);
    CHECK(PIOS_DWC2_DMA_BASE + PIOS_DWC2_DMA_SIZE == 0x06600000UL);
    CHECK(PIOS_DWC2_DMA_BASE ==
          PIOS_SHARED_ASSET_BASE + PIOS_SHARED_ASSET_SIZE);
    CHECK(PIOS_DWC2_DMA_BASE + PIOS_DWC2_DMA_SIZE <= 0x08000000UL);
    CHECK(PIOS_DWC2_DMA_BASE + PIOS_DWC2_DMA_SIZE <= DWC2_DMA_PHYS_LIMIT);
    CHECK(DWC2_DMA_ARENA_SLOT_COUNT == 8U);
    CHECK(DWC2_DMA_ARENA_SLOT_SIZE == 0x40000U);
    CHECK(DWC2_DMA_ARENA_SLOT_COUNT * DWC2_DMA_ARENA_SLOT_SIZE ==
          PIOS_DWC2_DMA_SIZE);
    CHECK(sizeof(struct dwc2_dma_arena_owner) == 64U);
    CHECK(sizeof(struct dwc2_dma_arena_slot_control) == 64U);
    CHECK(sizeof(struct dwc2_dma_arena_span) == 64U);
    CHECK(sizeof(struct dwc2_dma_arena_handle) == 24U);
    CHECK((usize)&((struct dwc2_dma_arena *)0)->controls ==
          (usize)&((struct dwc2_dma_arena *)0)->owner + 64U);
    CHECK((usize)&((struct dwc2_dma_arena *)0)->spans ==
          (usize)&((struct dwc2_dma_arena *)0)->controls +
          DWC2_DMA_ARENA_SLOT_COUNT * 64U);
    CHECK(DWC2_DMA_TO_DEVICE == 0U);
    CHECK(DWC2_DMA_FROM_DEVICE == 1U);
    CHECK(DWC2_DMA_CACHE_NORMAL_NC == 1U);

    memset(&arena, 0, sizeof(arena));
    CHECK(!dwc2_dma_arena_init(NULL, CONTROLLER_A, 0U));
    CHECK(!dwc2_dma_arena_init(&arena, 0U, 0U));
    CHECK(!dwc2_dma_arena_init(&arena, CONTROLLER_A, 1U));
    arena.owner._pad[0] = 1U;
    CHECK(!dwc2_dma_arena_init(&arena, CONTROLLER_A, 0U));
    memset(&arena, 0, sizeof(arena));
    CHECK(dwc2_dma_arena_init(&arena, CONTROLLER_A, 0U));
    CHECK(arena.owner.controller_id == CONTROLLER_A);
    CHECK(arena.owner.owner_core == 0U && arena.owner.initialized == 1U);
    CHECK(!dwc2_dma_arena_init(&arena, CONTROLLER_A, 0U));
}

static void test_all_slots_and_saturation(void)
{
    struct dwc2_dma_arena arena;
    struct dwc2_dma_arena_handle handles[DWC2_DMA_ARENA_SLOT_COUNT];
    struct dwc2_dma_arena_handle extra;
    struct dwc2_dma_arena_span span;
    u32 i;
    u32 j;

    init_arena(&arena);
    for (i = 0U; i < DWC2_DMA_ARENA_SLOT_COUNT; i++) {
        enum dwc2_dma_direction direction = (i & 1U) ?
            DWC2_DMA_FROM_DEVICE : DWC2_DMA_TO_DEVICE;
        u32 requested = 4U * (i + 1U);
        u64 expected_cpu = PIOS_DWC2_DMA_BASE +
                           (u64)i * DWC2_DMA_ARENA_SLOT_SIZE;

        CHECK(dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, requested,
                                     direction, &handles[i]));
        CHECK(handles[i].token == ((DWC2_DMA_ARENA_TOKEN_MAGIC <<
                                    DWC2_DMA_ARENA_TOKEN_SHIFT) | i));
        CHECK(handles[i].generation == (u64)i + 1U);
        CHECK(handles[i].controller_id == CONTROLLER_A);
        CHECK(dwc2_dma_arena_span_get(&arena, 0U, &handles[i], &span));
        CHECK(span.cpu_phys == expected_cpu);
        CHECK(span.bcm_bus_addr == (DWC2_DMA_BUS_ALIAS | expected_cpu));
        CHECK(span.used == requested && span.requested == requested);
        CHECK(span.capacity == DWC2_DMA_ARENA_SLOT_SIZE);
        CHECK(span.slot == i && span.generation == handles[i].generation);
        CHECK(span.direction == (u32)direction);
        CHECK(span.cache_policy == DWC2_DMA_CACHE_NORMAL_NC);
        CHECK((span.cpu_phys & (DWC2_DMA_ALIGNMENT - 1U)) == 0U);
        CHECK((span.bcm_bus_addr & (DWC2_DMA_ALIGNMENT - 1U)) == 0U);
        for (j = 0U; j < i; j++)
            CHECK(handles[i].token != handles[j].token);
    }
    memset(&extra, 0xA5, sizeof(extra));
    CHECK(!dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, 4U,
                                  DWC2_DMA_TO_DEVICE, &extra));
    CHECK(extra.token == 0U && extra.generation == 0U &&
          extra.controller_id == 0U && extra._reserved == 0U);
    for (i = 0U; i < DWC2_DMA_ARENA_SLOT_COUNT; i++)
        CHECK(dwc2_dma_arena_release(&arena, 0U, &handles[i]));
}

static void test_validation_and_transitions(void)
{
    struct dwc2_dma_arena arena;
    struct dwc2_dma_arena_handle handle;
    struct dwc2_dma_arena_span before;
    struct dwc2_dma_arena_span after;
    enum dwc2_dma_cache_policy policy;
    enum dwc2_dma_barrier_scope scope;
    bool publish_only;
    bool consume_only;
    bool success;
    u32 actual;

    init_arena(&arena);
    CHECK(!dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, 0U,
                                  DWC2_DMA_TO_DEVICE, &handle));
    CHECK(!dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, 2U,
                                  DWC2_DMA_TO_DEVICE, &handle));
    CHECK(!dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U,
                                  DWC2_DMA_ARENA_SLOT_SIZE + 4U,
                                  DWC2_DMA_TO_DEVICE, &handle));
    CHECK(!dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, 4U,
                                  (enum dwc2_dma_direction)2U, &handle));
    CHECK(!dwc2_dma_arena_acquire(&arena, CONTROLLER_B, 0U, 4U,
                                  DWC2_DMA_TO_DEVICE, &handle));
    CHECK(!dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 1U, 4U,
                                  DWC2_DMA_TO_DEVICE, &handle));
    CHECK(!dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, 4U,
                                  DWC2_DMA_TO_DEVICE, NULL));
    CHECK(dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, 0x40U,
                                 DWC2_DMA_FROM_DEVICE, &handle));
    expect_state(&arena, &handle, DWC2_DMA_ARENA_CPU_OWNED);
    CHECK(dwc2_dma_arena_span_get(&arena, 0U, &handle, &before));
    CHECK(!dwc2_dma_arena_complete_from_device(&arena, 0U, &handle, 0U, true));
    CHECK(!dwc2_dma_arena_publish_to_device(&arena, 1U, &handle));
    CHECK(dwc2_dma_arena_publish_to_device(&arena, 0U, &handle));
    expect_state(&arena, &handle, DWC2_DMA_ARENA_DEVICE_OWNED);
    CHECK(!dwc2_dma_arena_completion_get(&arena, 0U, &handle, &actual,
                                         &success));
    CHECK(!dwc2_dma_arena_publish_to_device(&arena, 0U, &handle));
    CHECK(!dwc2_dma_arena_cancel(&arena, 0U, &handle));
    CHECK(!dwc2_dma_arena_release(&arena, 0U, &handle));
    CHECK(!dwc2_dma_arena_complete_from_device(&arena, 0U, &handle, 0x44U, true));
    CHECK(dwc2_dma_arena_complete_from_device(&arena, 0U, &handle, 0x20U, true));
    expect_state(&arena, &handle, DWC2_DMA_ARENA_CPU_COMPLETE);
    CHECK(dwc2_dma_arena_completion_get(&arena, 0U, &handle, &actual,
                                        &success));
    CHECK(actual == 0x20U && success);
    CHECK(dwc2_dma_arena_span_get(&arena, 0U, &handle, &after));
    CHECK(memcmp(&before, &after, sizeof(before)) == 0);
    policy = 0U;
    publish_only = false;
    consume_only = false;
    scope = 0U;
    CHECK(dwc2_dma_arena_cache_contract_get(&arena, 0U, &handle, &policy,
                                            &publish_only, &consume_only,
                                            &scope));
    CHECK(policy == DWC2_DMA_CACHE_NORMAL_NC);
    CHECK(publish_only && consume_only);
    CHECK(scope == DWC2_DMA_BARRIER_SYSTEM);
    CHECK(dwc2_dma_arena_release(&arena, 0U, &handle));
    CHECK(!dwc2_dma_arena_state_get(&arena, 0U, &handle,
                                    (enum dwc2_dma_arena_state *)&policy));
    CHECK(!dwc2_dma_arena_span_get(&arena, 0U, &handle, &after));

    CHECK(dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, 4U,
                                 DWC2_DMA_TO_DEVICE, &handle));
    CHECK(dwc2_dma_arena_cancel(&arena, 0U, &handle));
    expect_state(&arena, &handle, DWC2_DMA_ARENA_FAILED);
    CHECK(dwc2_dma_arena_release(&arena, 0U, &handle));
    CHECK(dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, 4U,
                                 DWC2_DMA_TO_DEVICE, &handle));
    CHECK(dwc2_dma_arena_release(&arena, 0U, &handle));

    CHECK(dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, 8U,
                                 DWC2_DMA_TO_DEVICE, &handle));
    CHECK(dwc2_dma_arena_publish_to_device(&arena, 0U, &handle));
    CHECK(dwc2_dma_arena_complete_from_device(&arena, 0U, &handle, 0U, false));
    expect_state(&arena, &handle, DWC2_DMA_ARENA_FAILED);
    CHECK(dwc2_dma_arena_completion_get(&arena, 0U, &handle, &actual,
                                        &success));
    CHECK(actual == 0U && !success);
    CHECK(dwc2_dma_arena_release(&arena, 0U, &handle));
}

static void test_forged_cross_controller_and_exhaustion(void)
{
    struct dwc2_dma_arena arena;
    struct dwc2_dma_arena other;
    struct dwc2_dma_arena_handle handle;
    struct dwc2_dma_arena_handle forged;
    struct dwc2_dma_arena_handle next;
    struct dwc2_dma_arena_span span;
    enum dwc2_dma_arena_state state;

    init_arena(&arena);
    memset(&other, 0, sizeof(other));
    CHECK(dwc2_dma_arena_init(&other, CONTROLLER_B, 0U));
    CHECK(dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, 4U,
                                 DWC2_DMA_TO_DEVICE, &handle));
    forged = handle;
    forged.generation++;
    CHECK(!dwc2_dma_arena_publish_to_device(&arena, 0U, &forged));
    forged = handle;
    forged.token ^= 1ULL << 8;
    CHECK(!dwc2_dma_arena_publish_to_device(&arena, 0U, &forged));
    forged = handle;
    forged.token = 0U;
    CHECK(!dwc2_dma_arena_publish_to_device(&arena, 0U, &forged));
    forged = handle;
    forged.controller_id = CONTROLLER_B;
    CHECK(!dwc2_dma_arena_publish_to_device(&arena, 0U, &forged));
    forged = handle;
    forged._reserved = 1U;
    CHECK(!dwc2_dma_arena_publish_to_device(&arena, 0U, &forged));
    CHECK(!dwc2_dma_arena_state_get(&other, 0U, &handle, &state));
    CHECK(!dwc2_dma_arena_span_get(&arena, 2U, &handle, &span));
    CHECK(dwc2_dma_arena_release(&arena, 0U, &handle));

    arena.controls[0].generation = ~0ULL;
    CHECK(dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, 4U,
                                 DWC2_DMA_TO_DEVICE, &handle));
    CHECK(handle.token == (DWC2_DMA_ARENA_TOKEN_MAGIC <<
                           DWC2_DMA_ARENA_TOKEN_SHIFT));
    CHECK(handle.generation == ~0ULL);
    CHECK(dwc2_dma_arena_publish_to_device(&arena, 0U, &handle));
    CHECK(dwc2_dma_arena_complete_from_device(&arena, 0U, &handle, 4U, true));
    CHECK(dwc2_dma_arena_release(&arena, 0U, &handle));
    CHECK(arena.controls[0].state == DWC2_DMA_ARENA_RELEASED);
    CHECK(arena.controls[0].exhausted == 1U);
    CHECK(!dwc2_dma_arena_state_get(&arena, 0U, &handle, &state));
    CHECK(dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, 4U,
                                 DWC2_DMA_TO_DEVICE, &next));
    CHECK(next.token != handle.token);
    CHECK(dwc2_dma_arena_release(&arena, 0U, &next));
}

static void test_repeated_generation_matrix(void)
{
    struct dwc2_dma_arena arena;
    struct dwc2_dma_arena_handle handle;
    struct dwc2_dma_arena_handle stale;
    u32 i;

    init_arena(&arena);
    for (i = 0U; i < 48U; i++) {
        u32 requested = 4U * ((i % 32U) + 1U);
        bool success = (i & 1U) == 0U;

        CHECK(dwc2_dma_arena_acquire(&arena, CONTROLLER_A, 0U, requested,
                                     success ? DWC2_DMA_TO_DEVICE :
                                     DWC2_DMA_FROM_DEVICE, &handle));
        stale = handle;
        CHECK(dwc2_dma_arena_publish_to_device(&arena, 0U, &handle));
        CHECK(dwc2_dma_arena_complete_from_device(&arena, 0U, &handle,
                                                   requested - 4U, success));
        expect_state(&arena, &handle, success ? DWC2_DMA_ARENA_CPU_COMPLETE :
                     DWC2_DMA_ARENA_FAILED);
        CHECK(dwc2_dma_arena_release(&arena, 0U, &handle));
        CHECK(!dwc2_dma_arena_publish_to_device(&arena, 0U, &stale));
    }
}

int main(void)
{
    test_layout_and_one_shot_init();
    test_all_slots_and_saturation();
    test_validation_and_transitions();
    test_forged_cross_controller_and_exhaustion();
    test_repeated_generation_matrix();

    CHECK(checks > 250);
    if (failures) {
        printf("dwc2 dma arena: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("dwc2 dma arena: %d checks passed\n", checks);
    return 0;
}
