#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <string.h>

#include "types.h"
#include "bluetooth_platform_contract.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

struct expected_profile {
    enum bluetooth_platform_profile_id id;
    enum bluetooth_transport_kind transport;
    u64 base;
    u32 size;
    u16 cts, rts, txd, rxd;
    u32 mux_mask;
    enum bluetooth_control_owner_kind control_owner;
    u16 control_line;
    u32 max_baud;
};

static const u8 source_commit[BLUETOOTH_PLATFORM_SOURCE_COMMIT_BYTES] = {
    0x50U, 0xf8U, 0x87U, 0x24U, 0x51U, 0x8dU, 0x2eU, 0xafU,
    0xe7U, 0x5bU, 0xfaU, 0xe7U, 0x92U, 0x3eU, 0x90U, 0xa8U,
    0xfeU, 0x17U, 0x1cU, 0x66U,
};

static void test_source_gate_file(const char *path, bool source)
{
    static const char *const forbidden[] = {
        "#include <", "platform.h", "board_detect", "mmio_", "mailbox_",
        "gpio_", "uart_", "pinmux_", "kernel_", "malloc", "calloc",
        "realloc", "free(", "volatile", "__asm", "callback",
    };
    char line[256];
    FILE *file;
    u32 i;
    u32 includes = 0U;

    file = fopen(path, "rb");
    CHECK(file != NULL);
    if (!file)
        return;
    while (fgets(line, sizeof(line), file) != NULL) {
        for (i = 0U; i < sizeof(forbidden) / sizeof(forbidden[0]); i++)
            CHECK(strstr(line, forbidden[i]) == NULL);
        if (strstr(line, "#include ") != NULL) {
            includes++;
            CHECK(strstr(line, "#include \"types.h\"") != NULL ||
                  (source &&
                   strstr(line,
                          "#include \"bluetooth_platform_contract.h\"") !=
                   NULL));
        }
    }
    CHECK(includes == (source ? 2U : 1U));
    CHECK(fclose(file) == 0);
}

static void test_source_gate(void)
{
    test_source_gate_file("include\\bluetooth_platform_contract.h", false);
    test_source_gate_file("src\\bluetooth_platform_contract.c", true);
}

static void test_exact_profiles(void)
{
    static const struct expected_profile expected[] = {
        { BLUETOOTH_PLATFORM_PROFILE_PI5,
          BLUETOOTH_TRANSPORT_BCM2712_UARTA_H4, 0x7d50c000ULL, 0x20U,
          25U, 24U, 26U, 27U, BLUETOOTH_SIGNAL_ALL,
          BLUETOOTH_CONTROL_OWNER_SOC_GIO, 29U, 3000000U },
        { BLUETOOTH_PLATFORM_PROFILE_PI4_B, BLUETOOTH_TRANSPORT_PL011_H4,
          0x7e201000ULL, 0x200U, 30U, 31U, 32U, 33U,
          BLUETOOTH_SIGNAL_ALL, BLUETOOTH_CONTROL_OWNER_FIRMWARE_EXPGIO,
          0U, 3000000U },
        { BLUETOOTH_PLATFORM_PROFILE_PI3_B, BLUETOOTH_TRANSPORT_PL011_H4,
          0x7e201000ULL, 0x200U, 30U, 31U, 32U, 33U,
          BLUETOOTH_SIGNAL_TXD | BLUETOOTH_SIGNAL_RXD,
          BLUETOOTH_CONTROL_OWNER_FIRMWARE_EXPGIO, 0U, 0U },
        { BLUETOOTH_PLATFORM_PROFILE_PI3_B_PLUS,
          BLUETOOTH_TRANSPORT_PL011_H4, 0x7e201000ULL, 0x200U,
          30U, 31U, 32U, 33U, BLUETOOTH_SIGNAL_TXD | BLUETOOTH_SIGNAL_RXD,
          BLUETOOTH_CONTROL_OWNER_FIRMWARE_EXPGIO, 0U, 0U },
        { BLUETOOTH_PLATFORM_PROFILE_ZERO_2_W,
          BLUETOOTH_TRANSPORT_PL011_H4, 0x7e201000ULL, 0x200U,
          30U, 31U, 32U, 33U, BLUETOOTH_SIGNAL_ALL,
          BLUETOOTH_CONTROL_OWNER_SOC_GIO, 42U, 3000000U },
    };
    const struct bluetooth_platform_profile *profile;
    u32 i, byte;

    CHECK(BLUETOOTH_PLATFORM_SOURCE_COMMIT_BYTES == 20U);
    CHECK(BLUETOOTH_PLATFORM_PIN_NONE == 0xFFFFU);
    CHECK(BLUETOOTH_PLATFORM_PROFILE_COUNT == 6);
    CHECK(BLUETOOTH_FACT_TRANSPORT_REQUIRED == 0x1FFU);
    CHECK((BLUETOOTH_LINE_RESET | BLUETOOTH_LINE_DEVICE_WAKE |
           BLUETOOTH_LINE_HOST_WAKE) == 7U);
    for (i = 0U; i < sizeof(expected) / sizeof(expected[0]); i++) {
        profile = bluetooth_platform_profile_get(expected[i].id);
        CHECK(profile != NULL);
        CHECK(profile == bluetooth_platform_profile_get(expected[i].id));
        CHECK(profile->id == expected[i].id);
        CHECK(profile->name != NULL);
        for (byte = 0U; byte < BLUETOOTH_PLATFORM_SOURCE_COMMIT_BYTES;
             byte++)
            CHECK(profile->source_commit[byte] == source_commit[byte]);
        CHECK(profile->transport_kind == expected[i].transport);
        CHECK(profile->dt_bus_base == expected[i].base);
        CHECK(profile->dt_bus_size == expected[i].size);
        CHECK(profile->signals.cts == expected[i].cts);
        CHECK(profile->signals.rts == expected[i].rts);
        CHECK(profile->signals.txd == expected[i].txd);
        CHECK(profile->signals.rxd == expected[i].rxd);
        CHECK(profile->signal_mux_proven_mask == expected[i].mux_mask);
        CHECK(profile->control_owner == expected[i].control_owner);
        CHECK(profile->control_line == expected[i].control_line);
        CHECK(profile->control_active_high == true);
        CHECK(profile->control_logical_off == false);
        CHECK(profile->max_baud == expected[i].max_baud);
        CHECK(profile->initial_baud_known == false);
        CHECK(profile->declared_reset_wake_mask == 0U);
        CHECK(profile->radio_kind == BLUETOOTH_RADIO_BRCM_BCM43438_BT);
        CHECK(profile->console_conflict_required == true);
        if (profile->id == BLUETOOTH_PLATFORM_PROFILE_PI3_B ||
            profile->id == BLUETOOTH_PLATFORM_PROFILE_PI3_B_PLUS)
            CHECK(profile->known_fact_mask ==
                  (BLUETOOTH_FACT_TRANSPORT_REQUIRED &
                   ~BLUETOOTH_FACT_MAX_BAUD));
        else
            CHECK(profile->known_fact_mask ==
                  BLUETOOTH_FACT_TRANSPORT_REQUIRED);
        CHECK(profile->proven_pios_ownership_mask == 0U);
        CHECK(profile->proven_activation_mask == 0U);
    }
    CHECK(bluetooth_platform_profile_get(BLUETOOTH_PLATFORM_PROFILE_INVALID) ==
          NULL);
    CHECK(bluetooth_platform_profile_get(BLUETOOTH_PLATFORM_PROFILE_COUNT) ==
          NULL);
    CHECK(bluetooth_platform_profile_get(
              (enum bluetooth_platform_profile_id)0x7fffffff) == NULL);
    CHECK(bluetooth_platform_profile_get(
              (enum bluetooth_platform_profile_id)-1) == NULL);
    profile = bluetooth_platform_profile_get(BLUETOOTH_PLATFORM_PROFILE_PI5);
    CHECK(profile->dt_bus_base != 0x1f00000000ULL);
    CHECK(profile->dt_bus_size == 0x20U);
}

static void test_candidate_selection(void)
{
    const struct bluetooth_platform_profile *profile;
    struct bluetooth_platform_profile forged;
    u32 missing;
    u32 i;

    for (i = BLUETOOTH_PLATFORM_PROFILE_PI5;
         i < BLUETOOTH_PLATFORM_PROFILE_COUNT; i++) {
        u32 fact_missing;

        profile = bluetooth_platform_profile_get(
            (enum bluetooth_platform_profile_id)i);
        fact_missing =
            (profile->id == BLUETOOTH_PLATFORM_PROFILE_PI3_B ||
             profile->id == BLUETOOTH_PLATFORM_PROFILE_PI3_B_PLUS)
                ? BLUETOOTH_TRANSPORT_MISSING_FACTS : 0U;
        missing = ~0U;
        if (fact_missing != 0U) {
            CHECK(!bluetooth_transport_candidate(
                profile, BLUETOOTH_TRANSPORT_NONE, 0U,
                BLUETOOTH_OVERLAY_BASE, &missing));
            CHECK(missing == BLUETOOTH_TRANSPORT_MISSING_FACTS);
        } else {
            CHECK(bluetooth_transport_candidate(
                profile, BLUETOOTH_TRANSPORT_NONE, 0U,
                BLUETOOTH_OVERLAY_BASE, &missing));
            CHECK(missing == 0U);
        }
        missing = 0U;
        CHECK(!bluetooth_transport_candidate(
            profile, profile->transport_kind, profile->dt_bus_base,
            BLUETOOTH_OVERLAY_BASE, &missing));
        CHECK(missing == (fact_missing |
                          BLUETOOTH_TRANSPORT_MISSING_CONSOLE_SEPARATION));
        missing = 0U;
        CHECK(!bluetooth_transport_candidate(
            profile, BLUETOOTH_TRANSPORT_NONE, 0U,
            BLUETOOTH_OVERLAY_MINIUART, &missing));
        CHECK(missing == (fact_missing |
                          BLUETOOTH_TRANSPORT_MISSING_BASE_OVERLAY));
        missing = 0U;
        CHECK(!bluetooth_transport_candidate(
            profile, BLUETOOTH_TRANSPORT_NONE, 0U,
            BLUETOOTH_OVERLAY_DISABLE_BT, &missing));
        CHECK(missing == (fact_missing |
                          BLUETOOTH_TRANSPORT_MISSING_BASE_OVERLAY));
        missing = 0U;
        CHECK(!bluetooth_transport_candidate(
            profile, BLUETOOTH_TRANSPORT_NONE, 0U,
            BLUETOOTH_OVERLAY_UNKNOWN, &missing));
        CHECK(missing == (fact_missing |
                          BLUETOOTH_TRANSPORT_MISSING_BASE_OVERLAY));
    }
    profile = bluetooth_platform_profile_get(BLUETOOTH_PLATFORM_PROFILE_PI5);
    CHECK(bluetooth_transport_candidate(
        profile, BLUETOOTH_TRANSPORT_BCM2712_UARTA_H4, 0x7d50d000ULL,
        BLUETOOTH_OVERLAY_BASE, NULL));
    CHECK(bluetooth_transport_candidate(
        profile, BLUETOOTH_TRANSPORT_PL011_H4, profile->dt_bus_base,
        BLUETOOTH_OVERLAY_BASE, NULL));
    missing = 0U;
    CHECK(!bluetooth_transport_candidate(NULL, BLUETOOTH_TRANSPORT_NONE, 0U,
                                         BLUETOOTH_OVERLAY_BASE, &missing));
    CHECK(missing == BLUETOOTH_TRANSPORT_MISSING_PROFILE);
    forged = *profile;
    forged.known_fact_mask = 0U;
    missing = 0U;
    CHECK(!bluetooth_transport_candidate(&forged, BLUETOOTH_TRANSPORT_NONE,
                                         0U, BLUETOOTH_OVERLAY_BASE,
                                         &missing));
    CHECK(missing == BLUETOOTH_TRANSPORT_MISSING_PROFILE);
}

static void test_activation_gate(void)
{
    const struct bluetooth_platform_profile *profile;
    struct bluetooth_platform_profile forged;
    const u32 base_missing = BLUETOOTH_ACTIVATION_MISSING_PIOS_OWNERSHIP |
                             BLUETOOTH_ACTIVATION_MISSING_PINMUX_CONTROL |
                             BLUETOOTH_ACTIVATION_MISSING_RESET_POWER |
                             BLUETOOTH_ACTIVATION_MISSING_FIRMWARE_BAUD;
    u32 missing;
    u32 i;

    for (i = BLUETOOTH_PLATFORM_PROFILE_PI5;
         i < BLUETOOTH_PLATFORM_PROFILE_COUNT; i++) {
        profile = bluetooth_platform_profile_get(
            (enum bluetooth_platform_profile_id)i);
        missing = 0U;
        CHECK(!bluetooth_activation_permitted(profile, &missing));
        if (profile->id == BLUETOOTH_PLATFORM_PROFILE_PI3_B ||
            profile->id == BLUETOOTH_PLATFORM_PROFILE_PI3_B_PLUS)
            CHECK(missing == (base_missing |
                              BLUETOOTH_ACTIVATION_MISSING_FLOW_CONTROL_PINMUX));
        else
            CHECK(missing == base_missing);
        CHECK((missing & BLUETOOTH_ACTIVATION_MISSING_PIOS_OWNERSHIP) != 0U);
        CHECK((missing & BLUETOOTH_ACTIVATION_MISSING_PINMUX_CONTROL) != 0U);
        CHECK((missing & BLUETOOTH_ACTIVATION_MISSING_RESET_POWER) != 0U);
        CHECK((missing & BLUETOOTH_ACTIVATION_MISSING_FIRMWARE_BAUD) != 0U);
        CHECK(!bluetooth_activation_permitted(profile, NULL));
    }
    missing = 0U;
    CHECK(!bluetooth_activation_permitted(NULL, &missing));
    CHECK(missing == (base_missing | BLUETOOTH_ACTIVATION_MISSING_PROFILE));
    forged = *bluetooth_platform_profile_get(BLUETOOTH_PLATFORM_PROFILE_PI3_B);
    forged.proven_pios_ownership_mask = ~0U;
    forged.proven_activation_mask = ~0U;
    missing = 0U;
    CHECK(!bluetooth_activation_permitted(&forged, &missing));
    CHECK(missing == (base_missing | BLUETOOTH_ACTIVATION_MISSING_PROFILE));
}

int main(void)
{
    test_source_gate();
    test_exact_profiles();
    test_candidate_selection();
    test_activation_gate();
    CHECK(checks > 200);
    if (failures == 0) {
        printf("OK: %d Bluetooth platform contract assertions passed\n", checks);
        return 0;
    }
    printf("%d failure(s) across %d checks\n", failures, checks);
    return 1;
}
