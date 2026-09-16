#include <stdio.h>
#include <string.h>

#include "types.h"
#include "media_engine_contract.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static void init(struct media_engine_controller *controller, u32 controller_id)
{
    memset(controller, 0, sizeof(*controller));
    CHECK(media_engine_controller_init(controller, controller_id));
}

static void test_resource_facts(void)
{
    const struct media_engine_descriptor *hevc;
    const struct media_engine_descriptor *be;
    const struct media_engine_descriptor *fe;
    const struct media_engine_descriptor *hvs;

    hevc = media_engine_descriptor(MEDIA_ENGINE_HEVC);
    be = media_engine_descriptor(MEDIA_ENGINE_PISP_BE);
    fe = media_engine_descriptor(MEDIA_ENGINE_PISP_FE);
    hvs = media_engine_descriptor(MEDIA_ENGINE_HVS);
    CHECK(hevc != NULL);
    CHECK(be != NULL);
    CHECK(fe != NULL);
    CHECK(hvs != NULL);
    CHECK(media_engine_descriptor(MEDIA_ENGINE_INVALID) == NULL);
    CHECK(media_engine_descriptor((enum media_engine_kind)99) == NULL);

    CHECK(hevc->kind == MEDIA_ENGINE_HEVC);
    CHECK(hevc->known_mask == (MEDIA_ENGINE_KNOWN_MMIO0 |
                               MEDIA_ENGINE_KNOWN_MMIO1 |
                               MEDIA_ENGINE_KNOWN_VERSION |
                               MEDIA_ENGINE_KNOWN_CLOCKS |
                               MEDIA_ENGINE_KNOWN_GIC_SPI |
                               MEDIA_ENGINE_KNOWN_IOMMU));
    CHECK(hevc->mmio[0].cpu_phys_base == 0x1000800000ULL);
    CHECK(hevc->mmio[0].bytes == 0x10000U);
    CHECK(hevc->mmio[1].cpu_phys_base == 0x1000840000ULL);
    CHECK(hevc->mmio[1].bytes == 0x1000U);
    CHECK(hevc->gic_spi == 98U);
    CHECK(hevc->clock_count == 1U && hevc->clock_ids[0] == 11U);
    CHECK(hevc->iommu_resource == 2U);
    CHECK(hevc->version_offset == 0x3CU);
    CHECK(hevc->version_match == MEDIA_ENGINE_VERSION_EXACT);
    CHECK(hevc->version_value == 0x00000202U);
    CHECK((hevc->capabilities & (MEDIA_ENGINE_CAP_PASSIVE_IDENTIFY |
                                 MEDIA_ENGINE_CAP_ACTIVE_LEASE |
                                 MEDIA_ENGINE_CAP_DMA_EXCLUSIVE)) != 0U);

    CHECK(be->kind == MEDIA_ENGINE_PISP_BE);
    CHECK((be->known_mask & MEDIA_ENGINE_KNOWN_MMIO0) != 0U);
    CHECK((be->known_mask & MEDIA_ENGINE_KNOWN_MMIO1) == 0U);
    CHECK(be->mmio[0].cpu_phys_base == 0x1000880000ULL);
    CHECK(be->mmio[0].bytes == 0x4000U);
    CHECK(be->gic_spi == 72U);
    CHECK(be->clock_count == 1U && be->clock_ids[0] == 7U);
    CHECK(be->iommu_resource == 2U);
    CHECK(be->version_offset == 0U);
    CHECK(be->version_mask == 0xFFFFFFF0U);
    CHECK(be->version_value == 0x02252700U);
    CHECK(be->version_match == MEDIA_ENGINE_VERSION_MASKED);

    CHECK(fe->kind == MEDIA_ENGINE_PISP_FE);
    CHECK(fe->known_mask == 0U);
    CHECK(fe->capabilities == MEDIA_ENGINE_CAP_INCOMPLETE);
    CHECK(fe->mmio[0].cpu_phys_base == 0U && fe->mmio[0].bytes == 0U);
    CHECK(fe->clock_count == 0U && fe->gic_spi == 0U && fe->iommu_resource == 0U);
    CHECK(fe->version_match == MEDIA_ENGINE_VERSION_NONE);

    CHECK(hvs->kind == MEDIA_ENGINE_HVS);
    CHECK((hvs->known_mask & MEDIA_ENGINE_KNOWN_GIC_SPI) == 0U);
    CHECK(hvs->mmio[0].cpu_phys_base == 0x107C580000ULL);
    CHECK(hvs->mmio[0].bytes == 0x1A000U);
    CHECK(hvs->clock_count == 2U);
    CHECK(hvs->clock_ids[0] == 4U && hvs->clock_ids[1] == 16U);
    CHECK(hvs->iommu_resource == 4U);
    CHECK(hvs->version_offset == 0U);
    CHECK(hvs->version_match == MEDIA_ENGINE_VERSION_LOW_BYTE_53_OR_54);
    CHECK((hvs->capabilities & MEDIA_ENGINE_CAP_ACTIVE_LEASE) == 0U);
    CHECK((hvs->capabilities & MEDIA_ENGINE_CAP_PASSIVE_ONLY) != 0U);
}

static void test_version_matching(void)
{
    CHECK(media_engine_version_matches(MEDIA_ENGINE_HEVC, 0x00000202U));
    CHECK(!media_engine_version_matches(MEDIA_ENGINE_HEVC, 0x00000203U));
    CHECK(!media_engine_version_matches(MEDIA_ENGINE_HEVC, 0x00020202U));
    CHECK(media_engine_version_matches(MEDIA_ENGINE_PISP_BE, 0x02252700U));
    CHECK(media_engine_version_matches(MEDIA_ENGINE_PISP_BE, 0x0225270FU));
    CHECK(!media_engine_version_matches(MEDIA_ENGINE_PISP_BE, 0x02252710U));
    CHECK(!media_engine_version_matches(MEDIA_ENGINE_PISP_BE, 0x12252700U));
    CHECK(media_engine_version_matches(MEDIA_ENGINE_HVS, 0x00000053U));
    CHECK(media_engine_version_matches(MEDIA_ENGINE_HVS, 0xA5A50054U));
    CHECK(!media_engine_version_matches(MEDIA_ENGINE_HVS, 0x00000052U));
    CHECK(!media_engine_version_matches(MEDIA_ENGINE_HVS, 0x00000055U));
    CHECK(!media_engine_version_matches(MEDIA_ENGINE_PISP_FE, 0U));
    CHECK(!media_engine_version_matches(MEDIA_ENGINE_INVALID, 0U));
}

static void test_identification_and_quarantine(void)
{
    struct media_engine_controller controller;
    struct media_engine_lease lease;
    enum media_engine_state state;

    init(&controller, 1U);
    CHECK(media_engine_state_get(&controller, MEDIA_ENGINE_HEVC, &state));
    CHECK(state == MEDIA_ENGINE_DISABLED);
    lease._token = ~0ULL;
    CHECK(!media_engine_passive_identify(&controller, MEDIA_ENGINE_INVALID, 0U));
    CHECK(!media_engine_lease_acquire(&controller, MEDIA_ENGINE_INVALID, &lease));
    CHECK(lease._token == 0U);
    CHECK(!media_engine_rearm(&controller, MEDIA_ENGINE_INVALID));
    CHECK(!media_engine_passive_identify(&controller, MEDIA_ENGINE_PISP_FE, 0U));
    CHECK(!media_engine_lease_acquire(&controller, MEDIA_ENGINE_PISP_FE, NULL));
    CHECK(!media_engine_passive_identify(&controller, MEDIA_ENGINE_HEVC, 0U));
    CHECK(media_engine_state_get(&controller, MEDIA_ENGINE_HEVC, &state));
    CHECK(state == MEDIA_ENGINE_QUARANTINED);
    CHECK(!media_engine_passive_identify(&controller, MEDIA_ENGINE_HEVC, 0x202U));
    CHECK(!media_engine_lease_acquire(&controller, MEDIA_ENGINE_HEVC, NULL));
    CHECK(!media_engine_rearm(&controller, MEDIA_ENGINE_PISP_BE));
    CHECK(media_engine_rearm(&controller, MEDIA_ENGINE_HEVC));
    CHECK(media_engine_state_get(&controller, MEDIA_ENGINE_HEVC, &state));
    CHECK(state == MEDIA_ENGINE_DISABLED);
    CHECK(media_engine_passive_identify(&controller, MEDIA_ENGINE_HEVC, 0x202U));
    CHECK(!media_engine_passive_identify(&controller, MEDIA_ENGINE_HEVC, 0x202U));
    CHECK(!media_engine_state_get(NULL, MEDIA_ENGINE_HEVC, &state));
    CHECK(!media_engine_state_get(&controller, MEDIA_ENGINE_INVALID, &state));
    CHECK(!media_engine_state_get(&controller, MEDIA_ENGINE_HEVC, NULL));
}

static void test_hevc_lifecycle_and_handles(void)
{
    struct media_engine_controller controller;
    struct media_engine_lease lease, stale, forged;
    enum media_engine_state state;

    init(&controller, 1U);
    CHECK(media_engine_passive_identify(&controller, MEDIA_ENGINE_HEVC, 0x202U));
    CHECK(media_engine_lease_acquire(&controller, MEDIA_ENGINE_HEVC, &lease));
    stale = lease;
    CHECK(lease._token != 0U);
    CHECK(media_engine_state_get(&controller, MEDIA_ENGINE_HEVC, &state));
    CHECK(state == MEDIA_ENGINE_LEASED);
    CHECK(!media_engine_lease_acquire(&controller, MEDIA_ENGINE_HEVC, &forged));
    CHECK(forged._token == 0U);
    CHECK(!media_engine_lease_release(&controller, &lease));
    forged = lease;
    forged._token ^= 0x100ULL;
    CHECK(!media_engine_lease_complete(&controller, &forged));
    forged = lease;
    forged._token = (forged._token & ~0xFFULL) | 0x7FULL;
    CHECK(!media_engine_lease_complete(&controller, &forged));
    forged = lease;
    forged._token ^= 0x1000000000000000ULL;
    CHECK(!media_engine_lease_complete(&controller, &forged));
    CHECK(media_engine_lease_complete(&controller, &lease));
    CHECK(!media_engine_lease_complete(&controller, &lease));
    CHECK(!media_engine_lease_abort(&controller, &lease));
    CHECK(media_engine_lease_release(&controller, &lease));
    CHECK(media_engine_state_get(&controller, MEDIA_ENGINE_HEVC, &state));
    CHECK(state == MEDIA_ENGINE_RELEASED);
    CHECK(!media_engine_lease_release(&controller, &lease));
    CHECK(!media_engine_lease_complete(&controller, &stale));
    CHECK(media_engine_passive_identify(&controller, MEDIA_ENGINE_HEVC, 0x202U));
    CHECK(media_engine_lease_acquire(&controller, MEDIA_ENGINE_HEVC, &lease));
    CHECK(lease._token != stale._token);
    CHECK(media_engine_lease_abort(&controller, &lease));
    CHECK(!media_engine_lease_abort(&controller, &lease));
    CHECK(media_engine_state_get(&controller, MEDIA_ENGINE_HEVC, &state));
    CHECK(state == MEDIA_ENGINE_QUARANTINED);
    CHECK(!media_engine_passive_identify(&controller, MEDIA_ENGINE_HEVC, 0x202U));
    CHECK(media_engine_rearm(&controller, MEDIA_ENGINE_HEVC));
    CHECK(media_engine_passive_identify(&controller, MEDIA_ENGINE_HEVC, 0x202U));
}

static void test_hvs_and_iommu2_exclusion(void)
{
    struct media_engine_controller controller;
    struct media_engine_lease hevc, be;
    enum media_engine_state state;

    init(&controller, 1U);
    CHECK(media_engine_passive_identify(&controller, MEDIA_ENGINE_HVS, 0x53U));
    CHECK(!media_engine_lease_acquire(&controller, MEDIA_ENGINE_HVS, &hevc));
    CHECK(hevc._token == 0U);
    CHECK(media_engine_state_get(&controller, MEDIA_ENGINE_HVS, &state));
    CHECK(state == MEDIA_ENGINE_PASSIVELY_IDENTIFIED);

    CHECK(media_engine_passive_identify(&controller, MEDIA_ENGINE_HEVC, 0x202U));
    CHECK(media_engine_passive_identify(&controller, MEDIA_ENGINE_PISP_BE, 0x0225270FU));
    CHECK(media_engine_lease_acquire(&controller, MEDIA_ENGINE_HEVC, &hevc));
    CHECK(!media_engine_lease_acquire(&controller, MEDIA_ENGINE_PISP_BE, &be));
    CHECK(be._token == 0U);
    CHECK(controller.iommu2.kind == MEDIA_ENGINE_HEVC);
    CHECK(media_engine_lease_complete(&controller, &hevc));
    CHECK(media_engine_lease_release(&controller, &hevc));
    CHECK(controller.iommu2.kind == 0U);
    CHECK(media_engine_lease_acquire(&controller, MEDIA_ENGINE_PISP_BE, &be));
    CHECK(controller.iommu2.kind == MEDIA_ENGINE_PISP_BE);
    CHECK(!media_engine_lease_acquire(&controller, MEDIA_ENGINE_HEVC, &hevc));
    CHECK(media_engine_lease_abort(&controller, &be));
    CHECK(controller.iommu2.kind == 0U);
    CHECK(media_engine_rearm(&controller, MEDIA_ENGINE_PISP_BE));
    CHECK(media_engine_passive_identify(&controller, MEDIA_ENGINE_PISP_BE, 0x02252700U));
    CHECK(media_engine_lease_acquire(&controller, MEDIA_ENGINE_PISP_BE, &be));
    CHECK(media_engine_lease_complete(&controller, &be));
    CHECK(media_engine_lease_release(&controller, &be));
}

static void test_controller_identity(void)
{
    struct media_engine_controller first, second;
    struct media_engine_lease first_lease;

    memset(&first, 0, sizeof(first));
    memset(&second, 0, sizeof(second));
    CHECK(!media_engine_controller_init(NULL, 1U));
    CHECK(!media_engine_controller_init(&first, 0U));
    CHECK(media_engine_controller_init(&first, 1U));
    CHECK(!media_engine_controller_init(&first, 1U));
    CHECK(media_engine_controller_init(&second, 2U));
    CHECK(media_engine_passive_identify(&first, MEDIA_ENGINE_HEVC, 0x202U));
    CHECK(media_engine_passive_identify(&second, MEDIA_ENGINE_HEVC, 0x202U));
    CHECK(media_engine_lease_acquire(&first, MEDIA_ENGINE_HEVC, &first_lease));
    CHECK(first_lease._controller_id == 1U);
    CHECK(!media_engine_lease_complete(&second, &first_lease));
    CHECK(media_engine_lease_complete(&first, &first_lease));
    CHECK(media_engine_lease_release(&first, &first_lease));
}

static void test_generation_exhaustion(void)
{
    struct media_engine_controller controller;
    struct media_engine_lease lease;
    enum media_engine_state state;
    u32 index = (u32)MEDIA_ENGINE_HEVC - 1U;

    init(&controller, 3U);
    controller.engines[index].generation = ~0U;
    CHECK(media_engine_passive_identify(&controller, MEDIA_ENGINE_HEVC,
                                        0x202U));
    CHECK(media_engine_lease_acquire(&controller, MEDIA_ENGINE_HEVC, &lease));
    CHECK(media_engine_lease_complete(&controller, &lease));
    CHECK(media_engine_lease_release(&controller, &lease));
    CHECK(media_engine_state_get(&controller, MEDIA_ENGINE_HEVC, &state));
    CHECK(state == MEDIA_ENGINE_RETIRED);
    CHECK(controller.engines[index].generation == 0U);
    CHECK(!media_engine_passive_identify(&controller, MEDIA_ENGINE_HEVC,
                                         0x202U));
}

int main(void)
{
    test_resource_facts();
    test_version_matching();
    test_identification_and_quarantine();
    test_hevc_lifecycle_and_handles();
    test_hvs_and_iommu2_exclusion();
    test_controller_identity();
    test_generation_exhaustion();

    if (failures) {
        printf("media engine contract: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("media engine contract: %d checks passed\n", checks);
    return 0;
}
