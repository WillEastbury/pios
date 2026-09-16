#include <stdio.h>
#include <string.h>

#include "types.h"
#include "legacy_armctrl.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static void test_routes(void)
{
    const struct legacy_armctrl_route *sdio =
        legacy_armctrl_route_get(62U);
    const struct legacy_armctrl_route *dwc2 =
        legacy_armctrl_route_get(41U);

    CHECK(sdio != NULL);
    CHECK(sdio->intid == 62U);
    CHECK(sdio->bank == LEGACY_ARMCTRL_BANK2);
    CHECK(sdio->bit == 30U);
    CHECK(sdio->priority == 0U);
    CHECK(dwc2 != NULL);
    CHECK(dwc2->intid == 41U);
    CHECK(dwc2->bank == LEGACY_ARMCTRL_BANK1);
    CHECK(dwc2->bit == 9U);
    CHECK(dwc2->priority == 1U);
    CHECK(legacy_armctrl_route_get(0U) == NULL);
    CHECK(legacy_armctrl_route_get(63U) == NULL);
}

static void test_registration_and_pending(void)
{
    struct legacy_armctrl_registry registry;
    struct legacy_armctrl_source_handle sdio, dwc2, duplicate, stale;
    u32 intid;

    memset(&registry, 0, sizeof(registry));
    CHECK(sizeof(registry) == 64U);
    CHECK(!legacy_armctrl_registry_init(NULL, 0U));
    CHECK(!legacy_armctrl_registry_init(&registry, 1U));
    CHECK(legacy_armctrl_registry_init(&registry, 0U));
    CHECK(!legacy_armctrl_registry_init(&registry, 0U));
    CHECK(registry.generations[0] == 1U);
    CHECK(registry.generations[1] == 1U);
    CHECK(!legacy_armctrl_registered(&registry, 62U));
    CHECK(!legacy_armctrl_registered(&registry, 41U));
    CHECK(legacy_armctrl_registered_mask(&registry,
                                         LEGACY_ARMCTRL_BANK1) == 0U);
    CHECK(legacy_armctrl_registered_mask(&registry,
                                         LEGACY_ARMCTRL_BANK2) == 0U);
    CHECK(legacy_armctrl_registered_mask(&registry, 0U) == 0U);

    memset(&sdio, 0xA5, sizeof(sdio));
    CHECK(!legacy_armctrl_register(&registry, 1U, 62U, &sdio));
    CHECK(sdio.intid == 0U && sdio.generation == 0U);
    CHECK(!legacy_armctrl_register(&registry, 0U, 99U, &sdio));
    CHECK(!legacy_armctrl_register(&registry, 0U, 62U, NULL));
    CHECK(legacy_armctrl_register(&registry, 0U, 62U, &sdio));
    CHECK(sdio.intid == 62U && sdio.generation == 1U);
    CHECK(sdio.owner_core == 0U && sdio._reserved == 0U);
    CHECK(legacy_armctrl_registered(&registry, 62U));
    CHECK(legacy_armctrl_registered_mask(&registry,
                                         LEGACY_ARMCTRL_BANK2) == (1U << 30));
    CHECK(!legacy_armctrl_register(&registry, 0U, 62U, &duplicate));
    CHECK(duplicate.intid == 0U && duplicate.generation == 0U);

    intid = ~0U;
    CHECK(!legacy_armctrl_select_pending(&registry, 1U << 9, 0U, &intid));
    CHECK(intid == 0U);
    CHECK(!legacy_armctrl_select_pending(&registry, 0U, 1U << 29, &intid));
    CHECK(legacy_armctrl_select_pending(&registry, 0U, 1U << 30, &intid));
    CHECK(intid == 62U);

    CHECK(legacy_armctrl_register(&registry, 0U, 41U, &dwc2));
    CHECK(legacy_armctrl_registered(&registry, 41U));
    CHECK(legacy_armctrl_registered_mask(&registry,
                                         LEGACY_ARMCTRL_BANK1) == (1U << 9));
    CHECK(legacy_armctrl_select_pending(&registry, 1U << 9, 0U, &intid));
    CHECK(intid == 41U);
    CHECK(legacy_armctrl_select_pending(&registry, 1U << 9,
                                        1U << 30, &intid));
    CHECK(intid == 62U);

    stale = sdio;
    CHECK(!legacy_armctrl_unregister(&registry, 1U, &sdio));
    duplicate = sdio;
    duplicate.generation++;
    CHECK(!legacy_armctrl_unregister(&registry, 0U, &duplicate));
    duplicate = sdio;
    duplicate._reserved = 1U;
    CHECK(!legacy_armctrl_unregister(&registry, 0U, &duplicate));
    CHECK(legacy_armctrl_unregister(&registry, 0U, &sdio));
    CHECK(!legacy_armctrl_registered(&registry, 62U));
    CHECK(!legacy_armctrl_unregister(&registry, 0U, &stale));
    CHECK(legacy_armctrl_select_pending(&registry, 1U << 9,
                                        1U << 30, &intid));
    CHECK(intid == 41U);
    CHECK(legacy_armctrl_register(&registry, 0U, 62U, &sdio));
    CHECK(sdio.generation == 2U);
    CHECK(legacy_armctrl_unregister(&registry, 0U, &dwc2));
    CHECK(!legacy_armctrl_select_pending(&registry, 1U << 9, 0U, &intid));
    CHECK(registry.register_count == 3U);
    CHECK(registry.unregister_count == 2U);
    CHECK(registry.reject_count >= 6U);
}

static void test_generation_exhaustion_and_matrix(void)
{
    struct legacy_armctrl_registry registry;
    struct legacy_armctrl_source_handle handle, stale;
    u32 i;
    u32 intid;

    memset(&registry, 0, sizeof(registry));
    CHECK(legacy_armctrl_registry_init(&registry, 0U));
    registry.generations[1] = ~0U;
    CHECK(legacy_armctrl_register(&registry, 0U, 41U, &handle));
    stale = handle;
    CHECK(legacy_armctrl_unregister(&registry, 0U, &handle));
    CHECK(registry.generations[1] == 0U);
    CHECK(!legacy_armctrl_register(&registry, 0U, 41U, &handle));
    CHECK(!legacy_armctrl_unregister(&registry, 0U, &stale));

    for (i = 0U; i < 64U; i++) {
        u32 p1 = (i & 1U) ? (1U << 9) : 0U;
        u32 p2 = (i & 2U) ? (1U << 30) : 0U;

        intid = ~0U;
        CHECK(!legacy_armctrl_select_pending(&registry, p1, p2, &intid));
        CHECK(intid == 0U);
    }
}

int main(void)
{
    test_routes();
    test_registration_and_pending();
    test_generation_exhaustion_and_matrix();

    if (failures) {
        printf("legacy ARMCTRL: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("legacy ARMCTRL: %d checks passed\n", checks);
    return checks > 100 ? 0 : 1;
}
