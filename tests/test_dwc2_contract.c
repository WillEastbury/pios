#include <stdio.h>
#include <string.h>

#include "types.h"
#include "dwc2_contract.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static void init_pool(struct dwc2_transfer_pool *pool)
{
    memset(pool, 0, sizeof(*pool));
    CHECK(dwc2_transfer_pool_init(pool));
}

static void test_constants(void)
{
    struct dwc2_transfer_pool pool;

    CHECK(DWC2_BCM2837_BASE == 0x3F980000ULL);
    CHECK(DWC2_ARMCTRL_BANK1_IRQ_BIT == 9U);
    CHECK(DWC2_GPU_IRQ == 41U);
    CHECK(DWC2_GPU_IRQ == 32U + DWC2_ARMCTRL_BANK1_IRQ_BIT);
    CHECK(DWC2_DMA_BUS_ALIAS == 0xC0000000ULL);
    CHECK(DWC2_DMA_PHYS_LIMIT == 0x3F000000ULL);
    CHECK(DWC2_DMA_ALIGNMENT == 4U);
    memset(&pool, 0, sizeof(pool));
    CHECK(!dwc2_transfer_pool_init(NULL));
    CHECK(dwc2_transfer_pool_init(&pool));
    CHECK(!dwc2_transfer_pool_init(&pool));
}

static void test_dma_conversion_rejections(void)
{
    u64 bus = ~0ULL;

    CHECK(dwc2_dma_bus_addr(0x1000U, 0x80U, 0x100U, &bus));
    CHECK(bus == 0xC0001000ULL);
    CHECK(dwc2_dma_bus_addr(DWC2_DMA_PHYS_LIMIT - 4U, 4U, 4U, &bus));
    CHECK(bus == 0xFEFFFFFCULL);
    CHECK(!dwc2_dma_bus_addr(0U, 4U, 4U, &bus));
    CHECK(bus == 0U);
    CHECK(!dwc2_dma_bus_addr(0x1000U, 0U, 4U, &bus));
    CHECK(!dwc2_dma_bus_addr(0x1000U, 4U, 0U, &bus));
    CHECK(!dwc2_dma_bus_addr(0x1000U, 8U, 4U, &bus));
    CHECK(!dwc2_dma_bus_addr(0x1002U, 4U, 4U, &bus));
    CHECK(!dwc2_dma_bus_addr(0x1000U, 2U, 4U, &bus));
    CHECK(!dwc2_dma_bus_addr(0x1000U, 4U, 6U, &bus));
    CHECK(!dwc2_dma_bus_addr(DWC2_DMA_PHYS_LIMIT, 4U, 4U, &bus));
    CHECK(!dwc2_dma_bus_addr(DWC2_DMA_PHYS_LIMIT - 4U, 4U, 8U, &bus));
    CHECK(!dwc2_dma_bus_addr(0xFFFFFFFFFFFFFFFCULL, 4U, 4U, &bus));
    CHECK(!dwc2_dma_bus_addr(0x1000U, 4U, 4U, NULL));
}

static void test_lifecycle_and_generations(void)
{
    struct dwc2_transfer_pool pool;
    struct dwc2_transfer_handle handle, stale, forged;
    enum dwc2_transfer_state state;
    u64 bus;

    init_pool(&pool);
    CHECK(dwc2_transfer_prepare(&pool, 0x2000U, 0x40U, 0x80U, &handle));
    stale = handle;
    CHECK(dwc2_transfer_get_state(&pool, &handle, &state));
    CHECK(state == DWC2_TRANSFER_PREPARED);
    CHECK(dwc2_transfer_get_dma_bus(&pool, &handle, &bus));
    CHECK(bus == 0xC0002000ULL);
    CHECK(!dwc2_transfer_complete(&pool, &handle, 0U));
    CHECK(!dwc2_transfer_fail(&pool, &handle));
    CHECK(!dwc2_transfer_release(&pool, &handle));
    CHECK(dwc2_transfer_submit(&pool, &handle));
    CHECK(!dwc2_transfer_submit(&pool, &handle));
    CHECK(!dwc2_transfer_release(&pool, &handle));
    CHECK(!dwc2_transfer_complete(&pool, &handle, 0x41U));
    CHECK(dwc2_transfer_complete(&pool, &handle, 0x40U));
    CHECK(!dwc2_transfer_get_dma_bus(&pool, &handle, &bus));
    CHECK(!dwc2_transfer_submit(&pool, &handle));
    CHECK(dwc2_transfer_release(&pool, &handle));
    CHECK(!dwc2_transfer_get_state(&pool, &stale, &state));
    CHECK(!dwc2_transfer_release(&pool, &stale));
    CHECK(dwc2_transfer_prepare(&pool, 0x3000U, 0x40U, 0x40U, &handle));
    CHECK(handle.slot == stale.slot);
    CHECK(handle.generation != stale.generation);
    forged = handle;
    forged.generation++;
    CHECK(!dwc2_transfer_submit(&pool, &forged));
    CHECK(dwc2_transfer_submit(&pool, &handle));
    CHECK(dwc2_transfer_fail(&pool, &handle));
    CHECK(dwc2_transfer_release(&pool, &handle));
}

static void test_saturation_and_unique_ownership(void)
{
    struct dwc2_transfer_pool pool;
    struct dwc2_transfer_handle handles[DWC2_TRANSFER_CAPACITY];
    struct dwc2_transfer_handle extra;
    u32 i, j;

    init_pool(&pool);
    for (i = 0U; i < DWC2_TRANSFER_CAPACITY; i++) {
        CHECK(dwc2_transfer_prepare(&pool, 0x4000U + (u64)i * 0x100U,
                                    0x40U, 0x80U, &handles[i]));
        for (j = 0U; j < i; j++)
            CHECK(handles[i].slot != handles[j].slot);
    }
    extra.generation = ~0ULL;
    extra.slot = ~0U;
    extra._reserved = ~0U;
    CHECK(!dwc2_transfer_prepare(&pool, 0x5000U, 0x40U, 0x40U, &extra));
    CHECK(extra.generation == 0U && extra.slot == 0U && extra._reserved == 0U);
    for (i = 0U; i < DWC2_TRANSFER_CAPACITY; i++) {
        CHECK(dwc2_transfer_submit(&pool, &handles[i]));
        CHECK(dwc2_transfer_complete(&pool, &handles[i], 0x40U));
        CHECK(dwc2_transfer_release(&pool, &handles[i]));
    }
}

static void test_generation_exhaustion_retires_slot(void)
{
    struct dwc2_transfer_pool pool;
    struct dwc2_transfer_handle handle;
    enum dwc2_transfer_state state;

    init_pool(&pool);
    pool.records[0].generation = ~0ULL;
    CHECK(dwc2_transfer_prepare(&pool, 0x6000U, 0x40U, 0x40U, &handle));
    CHECK(handle.slot == 0U && handle.generation == ~0ULL);
    CHECK(dwc2_transfer_submit(&pool, &handle));
    CHECK(dwc2_transfer_complete(&pool, &handle, 0x40U));
    CHECK(dwc2_transfer_release(&pool, &handle));
    CHECK(!dwc2_transfer_get_state(&pool, &handle, &state));
    CHECK(pool.records[0].state == DWC2_TRANSFER_RETIRED);
    CHECK(!dwc2_transfer_prepare(&pool, 0x7000U, 0x40U, 0x40U, &handle) ||
          handle.slot != 0U);
}

int main(void)
{
    test_constants();
    test_dma_conversion_rejections();
    test_lifecycle_and_generations();
    test_saturation_and_unique_ownership();
    test_generation_exhaustion_retires_slot();

    if (failures) {
        printf("dwc2 contract: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("dwc2 contract: %d checks passed\n", checks);
    return 0;
}
