#include <stdio.h>
#include <string.h>

#include "types.h"
#include "wifi_assoc_contract.h"

static int checks;
static int failures;
static u32 host_core;

u32 pios_host_core_id(void)
{
    return host_core;
}

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
    } \
} while (0)

static void fresh(struct wifi_assoc_contract *contract)
{
    memset(contract, 0, sizeof(*contract));
    host_core = 0U;
    CHECK(wifi_assoc_init(contract, 0U));
}

static struct wifi_assoc_observation observation(u64 now, u64 liveness)
{
    struct wifi_assoc_observation value;

    memset(&value, 0, sizeof(value));
    value.now_ms = now;
    value.liveness_sequence = liveness;
    value.liveness_ran = true;
    return value;
}

static void advance_to_wait_assoc(struct wifi_assoc_contract *contract,
                                  const struct wifi_assoc_handle *handle)
{
    struct wifi_assoc_observation value;

    value = observation(1U, 1U);
    CHECK(wifi_assoc_step(contract, 0U, handle, &value) ==
          WIFI_ASSOC_STEP_PROGRESS);
    value = observation(2U, 2U);
    CHECK(wifi_assoc_step(contract, 0U, handle, &value) ==
          WIFI_ASSOC_STEP_NO_PROGRESS);
    value = observation(3U, 3U);
    value.credit_available = true;
    CHECK(wifi_assoc_step(contract, 0U, handle, &value) ==
          WIFI_ASSOC_STEP_PROGRESS);
    value = observation(4U, 4U);
    value.control_publish_verified = true;
    CHECK(wifi_assoc_step(contract, 0U, handle, &value) ==
          WIFI_ASSOC_STEP_PROGRESS);
}

static void test_policy_owner_and_liveness(void)
{
    struct wifi_assoc_contract contract;
    struct wifi_assoc_handle handle;
    struct wifi_assoc_observation value;
    struct wifi_assoc_status status;
    u32 i;

    memset(&contract, 0, sizeof(contract));
    host_core = 1U;
    CHECK(!wifi_assoc_init(&contract, 0U));
    host_core = 0U;
    CHECK(!wifi_assoc_init(&contract, 1U));
    CHECK(wifi_assoc_init(&contract, 0U));
    CHECK(!wifi_assoc_init(&contract, 0U));
    CHECK(!wifi_assoc_start(&contract, 1U, 0U, 10U, &handle));
    CHECK(wifi_assoc_start(&contract, 0U, 0U, 10U, &handle));
    host_core = 1U;
    value = observation(1U, 1U);
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_REJECTED);
    host_core = 0U;
    value = observation(1U, 0U);
    value.liveness_ran = false;
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_NO_PROGRESS);
    CHECK(wifi_assoc_status_copy(&contract, 0U, &status));
    CHECK(status.state == WIFI_ASSOC_PREPARE && status.watchdog_evidence == 0U);
    value = observation(2U, 1U);
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_PROGRESS);
    CHECK(wifi_assoc_status_copy(&contract, 0U, &status));
    CHECK(status.state == WIFI_ASSOC_WAIT_CREDIT && status.watchdog_evidence == 1U);
    value = observation(3U, 1U);
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_NO_PROGRESS);
    CHECK(status.watchdog_evidence == 1U);
    for (i = 0U; i < 80U; i++)
        CHECK(!wifi_assoc_hardware_enable_allowed());
}

static void test_credits_publish_and_authorization(void)
{
    struct wifi_assoc_contract contract;
    struct wifi_assoc_handle handle;
    struct wifi_assoc_observation value;
    struct wifi_assoc_control_publication publication;
    struct wifi_assoc_status status;

    fresh(&contract);
    CHECK(!wifi_assoc_start(&contract, 0U, 0U, 0U, &handle));
    CHECK(!wifi_assoc_start(&contract, 0U, ~0ULL - 1U, 4U, &handle));
    CHECK(wifi_assoc_start(&contract, 0U, 0U, 50U, &handle));
    advance_to_wait_assoc(&contract, &handle);
    CHECK(!wifi_assoc_control_publication_copy(&contract, 0U, &handle,
                                               &publication));
    CHECK(wifi_assoc_status_copy(&contract, 0U, &status));
    CHECK(status.state == WIFI_ASSOC_WAIT_ASSOC && status.watchdog_evidence == 3U);
    value = observation(5U, 5U);
    value.event.type = WIFI_ASSOC_EVENT_LINK;
    value.event.timestamp_ms = 5U;
    value.event.flags = WIFI_ASSOC_EVENT_F_LINK_UP;
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_NO_PROGRESS);
    value = observation(6U, 6U);
    value.event.type = WIFI_ASSOC_EVENT_LINK;
    value.event.timestamp_ms = 6U;
    value.event.flags = WIFI_ASSOC_EVENT_F_LINK_UP |
                        WIFI_ASSOC_EVENT_F_AUTHORIZED;
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_AUTHORIZED);
    CHECK(wifi_assoc_status_copy(&contract, 0U, &status));
    CHECK(status.state == WIFI_ASSOC_AUTHORIZED && status.watchdog_evidence == 4U);

    fresh(&contract);
    CHECK(wifi_assoc_start(&contract, 0U, 0U, 50U, &handle));
    value = observation(1U, 1U);
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_PROGRESS);
    value = observation(2U, 2U);
    value.credit_available = true;
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_PROGRESS);
    CHECK(wifi_assoc_control_publication_copy(&contract, 0U, &handle,
                                               &publication));
    CHECK(publication.state == WIFI_ASSOC_SET_SSID_PUBLISHED &&
          publication.instance_epoch == handle.instance_epoch &&
          publication.attempt_generation == handle.attempt_generation);
    value = observation(3U, 3U);
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_NO_PROGRESS);
    value = observation(4U, 4U);
    value.control_publish_verified = true;
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_PROGRESS);
}

static void test_failures_deadlines_and_handles(void)
{
    struct wifi_assoc_contract contract;
    struct wifi_assoc_handle handle, stale, forged;
    struct wifi_assoc_observation value;
    struct wifi_assoc_status status;

    fresh(&contract);
    CHECK(wifi_assoc_start(&contract, 0U, 0U, 10U, &handle));
    stale = handle;
    value = observation(10U, 1U);
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_FAILED);
    CHECK(wifi_assoc_status_copy(&contract, 0U, &status));
    CHECK(status.state == WIFI_ASSOC_FAILED && status.fault == WIFI_ASSOC_FAULT_DEADLINE);
    CHECK(wifi_assoc_reset(&contract, 0U));
    value = observation(1U, 1U);
    CHECK(!wifi_assoc_step(&contract, 0U, &stale, &value));
    CHECK(wifi_assoc_start(&contract, 0U, 0U, 100U, &handle));
    forged = handle;
    forged.token++;
    value = observation(1U, 1U);
    CHECK(wifi_assoc_step(&contract, 0U, &forged, &value) ==
          WIFI_ASSOC_STEP_REJECTED);
    CHECK(wifi_assoc_cancel(&contract, 0U, &handle));
    CHECK(wifi_assoc_status_copy(&contract, 0U, &status));
    CHECK(status.state == WIFI_ASSOC_FAILED && status.fault == WIFI_ASSOC_FAULT_CANCELLED);
    CHECK(!wifi_assoc_cancel(&contract, 0U, &handle));

    fresh(&contract);
    CHECK(wifi_assoc_start(&contract, 0U, 0U, 100U, &handle));
    value = observation(1U, 1U);
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_PROGRESS);
    value = observation(1U, 2U);
    value.event.type = WIFI_ASSOC_EVENT_LINK;
    value.event.timestamp_ms = 1U;
    value.event.flags = WIFI_ASSOC_EVENT_F_LINK_UP;
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_QUARANTINED);
    CHECK(wifi_assoc_status_copy(&contract, 0U, &status));
    CHECK(status.state == WIFI_ASSOC_QUARANTINED);
}

static void test_events_history_and_psk_states(void)
{
    struct wifi_assoc_contract contract;
    struct wifi_assoc_handle handle;
    struct wifi_assoc_observation value;
    struct wifi_assoc_event_record record;
    u32 state;

    fresh(&contract);
    CHECK(wifi_assoc_start(&contract, 0U, 0U, 100U, &handle));
    advance_to_wait_assoc(&contract, &handle);
    for (state = WIFI_ASSOC_PSK_SUP_WAIT_M1;
         state <= WIFI_ASSOC_PSK_SUP_PREP_G2; state++) {
        if (state != WIFI_ASSOC_PSK_SUP_WAIT_M1) {
            fresh(&contract);
            CHECK(wifi_assoc_start(&contract, 0U, 0U, 100U, &handle));
            advance_to_wait_assoc(&contract, &handle);
        }
        value = observation((u64)state + 10U, (u64)state + 10U);
        value.event.type = WIFI_ASSOC_EVENT_PSK_SUP;
        value.event.timestamp_ms = (u64)state + 10U;
        value.event.status = (u8)state;
        CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
              (state == WIFI_ASSOC_PSK_SUP_TIMEOUT ?
               WIFI_ASSOC_STEP_FAILED : WIFI_ASSOC_STEP_NO_PROGRESS));
        CHECK(wifi_assoc_history_copy(&contract, 0U, 0U, &record));
        CHECK(record.psk_state == state);
    }

    fresh(&contract);
    CHECK(wifi_assoc_start(&contract, 0U, 0U, 100U, &handle));
    advance_to_wait_assoc(&contract, &handle);
    for (state = 0U; state < WIFI_ASSOC_EVENT_HISTORY + 1U; state++) {
        value = observation((u64)state + 10U, (u64)state + 10U);
        value.event.type = WIFI_ASSOC_EVENT_LINK;
        value.event.timestamp_ms = (u64)state + 10U;
        value.event.flags = WIFI_ASSOC_EVENT_F_LINK_UP;
        CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
              WIFI_ASSOC_STEP_NO_PROGRESS);
    }
    CHECK(wifi_assoc_history_copy(&contract, 0U, 0U, &record));
    CHECK(record.timestamp_ms == 11U);
    CHECK(wifi_assoc_history_copy(&contract, 0U, 7U, &record));
    CHECK(record.timestamp_ms == 18U);

    fresh(&contract);
    CHECK(wifi_assoc_start(&contract, 0U, 0U, 100U, &handle));
    advance_to_wait_assoc(&contract, &handle);
    value = observation(5U, 5U);
    value.event.type = WIFI_ASSOC_EVENT_LINK;
    value.event.timestamp_ms = 5U;
    value.event.flags = WIFI_ASSOC_EVENT_F_LINK_UP;
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_NO_PROGRESS);
    value = observation(6U, 6U);
    value.event.type = WIFI_ASSOC_EVENT_DEAUTH;
    value.event.timestamp_ms = 6U;
    value.event.reason = 2U;
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_FAILED);

    fresh(&contract);
    CHECK(wifi_assoc_start(&contract, 0U, 0U, 100U, &handle));
    advance_to_wait_assoc(&contract, &handle);
    value = observation(6U, 5U);
    value.event.type = WIFI_ASSOC_EVENT_PSK_SUP;
    value.event.timestamp_ms = 7U;
    value.event.status = 3U;
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_QUARANTINED);
}

static void build_eapol(u8 packet[99], u16 key_info, u64 replay)
{
    u32 i;

    memset(packet, 0, 99U);
    packet[0] = 2U;
    packet[1] = 3U;
    packet[2] = 0U;
    packet[3] = 95U;
    packet[4] = 2U;
    packet[5] = (u8)(key_info >> 8U);
    packet[6] = (u8)key_info;
    for (i = 0U; i < 8U; i++)
        packet[9U + i] = (u8)(replay >> ((7U - i) * 8U));
}

static void test_eapol_decoder_and_evidence(void)
{
    u8 packet[99];
    struct wifi_assoc_eapol_summary summary;
    struct wifi_assoc_contract contract;
    struct wifi_assoc_handle handle;
    struct wifi_assoc_observation value;

    build_eapol(packet, 0x008AU, 1U);
    CHECK(wifi_assoc_eapol_decode(packet, sizeof(packet), &summary));
    CHECK(summary.valid && summary.classification == WIFI_ASSOC_EAPOL_M1 &&
          summary.replay_counter == 1U);
    build_eapol(packet, 0x03CAU, 2U);
    CHECK(wifi_assoc_eapol_decode(packet, sizeof(packet), &summary));
    CHECK(summary.classification == WIFI_ASSOC_EAPOL_M3);
    build_eapol(packet, 0x0382U, 3U);
    CHECK(wifi_assoc_eapol_decode(packet, sizeof(packet), &summary));
    CHECK(summary.classification == WIFI_ASSOC_EAPOL_G1);
    packet[3]--;
    CHECK(!wifi_assoc_eapol_decode(packet, sizeof(packet), &summary));
    build_eapol(packet, 0x008AU, 0U);
    CHECK(!wifi_assoc_eapol_decode(packet, sizeof(packet), &summary));
    CHECK(!wifi_assoc_eapol_decode(packet, 98U, &summary));
    CHECK(!wifi_assoc_eapol_decode(NULL, 99U, &summary));

    fresh(&contract);
    CHECK(wifi_assoc_start(&contract, 0U, 0U, 100U, &handle));
    value = observation(1U, 1U);
    build_eapol(packet, 0x008AU, 1U);
    CHECK(wifi_assoc_eapol_decode(packet, sizeof(packet), &value.eapol));
    value.eapol_present = true;
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_PROGRESS);
    value = observation(2U, 2U);
    value.eapol = summary;
    value.eapol_present = true;
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_QUARANTINED);
}

static void test_step_bound(void)
{
    struct wifi_assoc_contract contract;
    struct wifi_assoc_handle handle;
    struct wifi_assoc_observation value;
    u32 i;

    fresh(&contract);
    CHECK(wifi_assoc_start(&contract, 0U, 0U, 1000U, &handle));
    for (i = 1U; i <= WIFI_ASSOC_MAX_STEPS; i++) {
        value = observation(i, i);
        CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) >=
              WIFI_ASSOC_STEP_NO_PROGRESS);
    }
    value = observation(WIFI_ASSOC_MAX_STEPS + 1U,
                        WIFI_ASSOC_MAX_STEPS + 1U);
    CHECK(wifi_assoc_step(&contract, 0U, &handle, &value) ==
          WIFI_ASSOC_STEP_FAILED);
}

int main(void)
{
    test_policy_owner_and_liveness();
    test_credits_publish_and_authorization();
    test_failures_deadlines_and_handles();
    test_events_history_and_psk_states();
    test_eapol_decoder_and_evidence();
    test_step_bound();
    CHECK(checks > 180);
    if (failures) {
        printf("wifi_assoc_contract: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("wifi_assoc_contract: all %d checks passed\n", checks);
    return 0;
}
