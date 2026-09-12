#include <stdio.h>
#include <string.h>

#include "types.h"
#include "bt_hci_lifecycle.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
    } \
} while (0)

static void init(struct bt_hci_lifecycle *lifecycle)
{
    memset(lifecycle, 0, sizeof(*lifecycle));
    CHECK(bt_hci_lifecycle_init(lifecycle, BT_HCI_OWNER_CORE));
}

static bool submit(struct bt_hci_lifecycle *lifecycle, u16 opcode, u64 now,
                   u64 timeout, struct bt_hci_handle *handle)
{
    static const u8 payload[] = {0xA1U, 0xB2U, 0xC3U};

    return bt_hci_submit(lifecycle, BT_HCI_OWNER_CORE, opcode, payload,
                         sizeof(payload), now, timeout, handle);
}

static void complete(struct bt_hci_lifecycle *lifecycle, u16 opcode,
                     u8 credits, u8 status)
{
    const u8 event[] = {
        BT_HCI_H4_TYPE_EVENT, BT_HCI_EVENT_COMMAND_COMPLETE, 4U,
        credits, (u8)opcode, (u8)(opcode >> 8), status
    };

    CHECK(bt_hci_event_from_irq(lifecycle, BT_HCI_OWNER_CORE, event,
                                sizeof(event)) == BT_HCI_EVENT_ACCEPTED);
}

static void test_policy_and_init(void)
{
    struct bt_hci_lifecycle lifecycle;
    struct bt_hci_status status;
    u32 i;

    CHECK(bt_hci_selected_first_capability() ==
          BT_HCI_CAPABILITY_LE_PASSIVE_SCAN);
    CHECK(!bt_hci_hardware_enable_allowed());
    CHECK(!bt_hci_capability_authorized(BT_HCI_CAPABILITY_NONE));
    CHECK(!bt_hci_capability_authorized(BT_HCI_CAPABILITY_LE_PASSIVE_SCAN));
    CHECK(!bt_hci_implicit_pairing_allowed());
    memset(&lifecycle, 0, sizeof(lifecycle));
    CHECK(!bt_hci_lifecycle_init(&lifecycle, 1U));
    CHECK(bt_hci_lifecycle_init(&lifecycle, 0U));
    CHECK(!bt_hci_lifecycle_init(&lifecycle, 0U));
    CHECK(bt_hci_status_copy(&lifecycle, 0U, &status));
    CHECK(status.instance_epoch == 1U);
    CHECK(status.credits == 1U);
    CHECK(status.controller_state == BT_HCI_CONTROLLER_READY);
    CHECK(status.queued == 0U && status.in_flight == 0U &&
          status.results == 0U);
    CHECK(!bt_hci_status_copy(&lifecycle, 1U, &status));
    for (i = 0U; i < 80U; i++)
        CHECK(!bt_hci_hardware_enable_allowed());
}

static void test_submit_tx_complete_and_handles(void)
{
    struct bt_hci_lifecycle lifecycle;
    struct bt_hci_handle handle, forged;
    struct bt_hci_tx_command tx;
    struct bt_hci_result result;
    struct bt_hci_status status;
    static const u8 expected[] = {1U, 0x34U, 0x12U, 3U, 0xA1U, 0xB2U, 0xC3U};

    init(&lifecycle);
    CHECK(!bt_hci_submit(&lifecycle, 0U, 0x1234U, NULL, 1U, 0U, 10U,
                         &handle));
    CHECK(!bt_hci_submit(&lifecycle, 0U, 0x1234U, expected,
                         BT_HCI_COMMAND_PAYLOAD_MAX + 1U, 0U, 10U, &handle));
    CHECK(!bt_hci_submit(&lifecycle, 0U, 0x1234U, expected, 1U, 0U, 0U,
                         &handle));
    CHECK(!bt_hci_submit(&lifecycle, 1U, 0x1234U, expected, 1U, 0U, 10U,
                         &handle));
    CHECK(submit(&lifecycle, 0x1234U, 100U, 50U, &handle));
    CHECK(handle.request_id == 1U && handle.generation != 0U &&
          handle.instance_epoch == 1U);
    CHECK(bt_hci_tx_next(&lifecycle, 0U, 101U, &tx));
    CHECK(tx.request_id == handle.request_id && tx.generation == handle.generation);
    CHECK(tx.attempt_id != 0U);
    CHECK(tx.deadline_ms == 150U && tx.opcode == 0x1234U);
    CHECK(tx.byte_count == sizeof(expected));
    CHECK(memcmp(tx.bytes, expected, sizeof(expected)) == 0);
    CHECK(!bt_hci_tx_next(&lifecycle, 0U, 102U, &tx));
    complete(&lifecycle, 0x1234U, 255U, 0U);
    CHECK(bt_hci_status_copy(&lifecycle, 0U, &status));
    CHECK(status.credits == BT_HCI_COMMAND_CREDITS_MAX);
    CHECK(status.credit_clamps == 1U && status.results == 1U);
    CHECK(bt_hci_result_copy(&lifecycle, 0U, &handle, &result));
    CHECK(result.request_id == handle.request_id &&
          result.instance_epoch == handle.instance_epoch);
    CHECK(result.attempt_id == tx.attempt_id);
    CHECK(result.opcode == 0x1234U && result.hci_status == 0U);
    CHECK(result.kind == BT_HCI_RESULT_COMMAND_COMPLETE);
    CHECK(result.credits == BT_HCI_COMMAND_CREDITS_MAX);
    forged = handle;
    forged.request_id++;
    CHECK(!bt_hci_result_copy(&lifecycle, 0U, &forged, &result));
    forged = handle;
    forged.token ^= 1U;
    CHECK(!bt_hci_result_copy(&lifecycle, 0U, &forged, &result));
    forged = handle;
    forged.instance_epoch++;
    CHECK(!bt_hci_result_release(&lifecycle, 0U, &forged));
    CHECK(!bt_hci_result_copy(&lifecycle, 1U, &handle, &result));
    CHECK(bt_hci_result_release(&lifecycle, 0U, &handle));
    CHECK(!bt_hci_result_copy(&lifecycle, 0U, &handle, &result));
}

static void test_status_credit_and_sequential_attempts(void)
{
    struct bt_hci_lifecycle lifecycle;
    struct bt_hci_handle one, two;
    struct bt_hci_tx_command tx;
    struct bt_hci_result result;
    const u8 status[] = {4U, BT_HCI_EVENT_COMMAND_STATUS, 4U,
                         0x0CU, 0U, 0x78U, 0x56U};

    init(&lifecycle);
    CHECK(submit(&lifecycle, 0x5678U, 0U, 100U, &one));
    CHECK(submit(&lifecycle, 0x5679U, 0U, 100U, &two));
    CHECK(bt_hci_tx_next(&lifecycle, 0U, 1U, &tx));
    CHECK(tx.opcode == 0x5678U);
    CHECK(!bt_hci_tx_next(&lifecycle, 0U, 1U, &tx));
    CHECK(bt_hci_event_from_irq(&lifecycle, 0U, status, sizeof(status)) ==
          BT_HCI_EVENT_ACCEPTED);
    CHECK(bt_hci_result_copy(&lifecycle, 0U, &one, &result));
    CHECK(result.kind == BT_HCI_RESULT_COMMAND_STATUS &&
          result.hci_status == 0x0CU && result.credits == 0U);
    CHECK(!bt_hci_tx_next(&lifecycle, 0U, 2U, &tx));
    CHECK(bt_hci_result_release(&lifecycle, 0U, &one));
    CHECK(bt_hci_poll_timeouts(&lifecycle, 0U, 99U) == false);
    CHECK(bt_hci_poll_timeouts(&lifecycle, 0U, 100U));
    CHECK(bt_hci_result_copy(&lifecycle, 0U, &two, &result));
    CHECK(result.kind == BT_HCI_RESULT_TIMEOUT);
}

static void test_event_rejections(void)
{
    static const u8 unknown[] = {4U, 0xFFU, 0U};
    static const u8 malformed_h4[] = {2U, 0x0EU, 0U};
    static const u8 malformed_len[] = {4U, 0x0EU, 4U, 1U, 0U, 0U};
    static const u8 malformed_cc[] = {4U, 0x0EU, 3U, 1U, 0U, 0U};
    static const u8 malformed_cs[] = {4U, 0x0FU, 3U, 0U, 1U, 2U};
    static const u8 unsolicited[] = {4U, 0x0EU, 4U, 1U, 0x34U, 0x12U, 0U};
    static const u8 mismatch[] = {4U, 0x0EU, 4U, 1U, 0x35U, 0x12U, 0U};
    struct bt_hci_lifecycle lifecycle;
    struct bt_hci_handle handle;
    struct bt_hci_status state;
    u32 i;

    init(&lifecycle);
    CHECK(bt_hci_event_from_irq(&lifecycle, 0U, malformed_h4, 2U) ==
          BT_HCI_EVENT_MALFORMED);
    CHECK(bt_hci_status_copy(&lifecycle, 0U, &state));
    CHECK(state.controller_state == BT_HCI_CONTROLLER_QUARANTINED);
    init(&lifecycle);
    CHECK(bt_hci_event_from_irq(&lifecycle, 1U, unsolicited,
                                sizeof(unsolicited)) == BT_HCI_EVENT_WRONG_CORE);
    CHECK(bt_hci_event_from_irq(&lifecycle, 0U, unknown, sizeof(unknown)) ==
          BT_HCI_EVENT_IGNORED);
    CHECK(bt_hci_status_copy(&lifecycle, 0U, &state) && state.ignored_events == 1U);
    for (i = 0U; i < 5U; i++) {
        const u8 *bad[] = {malformed_h4, malformed_len, malformed_cc, malformed_cs, NULL};
        u32 lengths[] = {sizeof(malformed_h4), sizeof(malformed_len),
                         sizeof(malformed_cc), sizeof(malformed_cs), 1U};
        init(&lifecycle);
        CHECK(bt_hci_event_from_irq(&lifecycle, 0U, bad[i], lengths[i]) ==
              (bad[i] ? BT_HCI_EVENT_MALFORMED : BT_HCI_EVENT_ARGUMENT));
        CHECK(bt_hci_status_copy(&lifecycle, 0U, &state));
        CHECK(state.controller_state == BT_HCI_CONTROLLER_QUARANTINED);
            CHECK(state.quarantine_reason == BT_HCI_QUARANTINE_MALFORMED_EVENT);
            CHECK(!bt_hci_submit(&lifecycle, 0U, 1U, NULL, 0U, 0U, 1U, &handle));
        }
        init(&lifecycle);
        CHECK(submit(&lifecycle, 0x1234U, 0U, 10U, &handle));
        CHECK(bt_hci_tx_next(&lifecycle, 0U, 1U, &(struct bt_hci_tx_command){0}));
        complete(&lifecycle, 0x1234U, 1U, 0U);
        CHECK(bt_hci_event_from_irq(&lifecycle, 0U, unsolicited,
                                    sizeof(unsolicited)) == BT_HCI_EVENT_DUPLICATE);
        CHECK(bt_hci_status_copy(&lifecycle, 0U, &state));
        CHECK(state.quarantine_reason == BT_HCI_QUARANTINE_UNSOLICITED_EVENT);
    init(&lifecycle);
    CHECK(bt_hci_event_from_irq(&lifecycle, 0U, unsolicited,
                                sizeof(unsolicited)) == BT_HCI_EVENT_UNSOLICITED);
    CHECK(bt_hci_status_copy(&lifecycle, 0U, &state));
    CHECK(state.quarantine_reason == BT_HCI_QUARANTINE_UNSOLICITED_EVENT);
    init(&lifecycle);
    CHECK(submit(&lifecycle, 0x1234U, 0U, 10U, &handle));
    CHECK(bt_hci_tx_next(&lifecycle, 0U, 1U, &(struct bt_hci_tx_command){0}));
    CHECK(bt_hci_event_from_irq(&lifecycle, 0U, mismatch, sizeof(mismatch)) ==
          BT_HCI_EVENT_MISMATCH);
    CHECK(bt_hci_status_copy(&lifecycle, 0U, &state));
    CHECK(state.controller_state == BT_HCI_CONTROLLER_QUARANTINED);
}

static void test_cancel_reset_and_capacity(void)
{
    struct bt_hci_lifecycle lifecycle;
    struct bt_hci_handle handles[BT_HCI_COMMAND_SLOT_COUNT], stale;
    struct bt_hci_result result;
    struct bt_hci_tx_command tx;
    struct bt_hci_status state;
    u32 i;

    init(&lifecycle);
    for (i = 0U; i < BT_HCI_COMMAND_SLOT_COUNT; i++)
        CHECK(submit(&lifecycle, (u16)(0x2000U + i), 0U, 100U, &handles[i]));
    CHECK(!submit(&lifecycle, 0x3000U, 0U, 100U, &stale));
    CHECK(bt_hci_cancel(&lifecycle, 0U, &handles[2]));
    CHECK(bt_hci_result_copy(&lifecycle, 0U, &handles[2], &result));
    CHECK(result.kind == BT_HCI_RESULT_CANCELLED);
    CHECK(bt_hci_result_release(&lifecycle, 0U, &handles[2]));
    CHECK(submit(&lifecycle, 0x3000U, 0U, 100U, &stale));
    CHECK(bt_hci_tx_next(&lifecycle, 0U, 1U, &tx));
    CHECK(bt_hci_cancel(&lifecycle, 0U, &handles[0]));
    CHECK(bt_hci_result_copy(&lifecycle, 0U, &handles[0], &result));
    CHECK(result.kind == BT_HCI_RESULT_CANCELLED);
    CHECK(bt_hci_result_copy(&lifecycle, 0U, &handles[1], &result));
    CHECK(result.kind == BT_HCI_RESULT_QUARANTINED);
    CHECK(!bt_hci_tx_next(&lifecycle, 0U, 2U, &tx));
    stale = handles[0];
    CHECK(bt_hci_controller_reset(&lifecycle, 0U));
    CHECK(!bt_hci_result_copy(&lifecycle, 0U, &stale, &result));
    CHECK(bt_hci_status_copy(&lifecycle, 0U, &state));
    CHECK(state.instance_epoch == 2U);
    CHECK(state.controller_state == BT_HCI_CONTROLLER_QUARANTINED);
    CHECK(!bt_hci_controller_reset(&lifecycle, 1U));
    init(&lifecycle);
    CHECK(bt_hci_quarantine(&lifecycle, 0U, BT_HCI_QUARANTINE_AER_LIKE_FAULT));
    CHECK(bt_hci_status_copy(&lifecycle, 0U, &state));
    CHECK(state.quarantine_reason == BT_HCI_QUARANTINE_AER_LIKE_FAULT);
    CHECK(!bt_hci_quarantine(&lifecycle, 0U, BT_HCI_QUARANTINE_NONE));
}

static void test_many_generations(void)
{
    struct bt_hci_lifecycle lifecycle;
    struct bt_hci_handle handle, stale;
    struct bt_hci_tx_command tx;
    struct bt_hci_result result;
    u32 i;

    init(&lifecycle);
    for (i = 0U; i < 96U; i++) {
        CHECK(submit(&lifecycle, (u16)(0x4000U + i), i * 10U, 9U, &handle));
        if (i != 0U)
            CHECK(!bt_hci_result_copy(&lifecycle, 0U, &stale, &result));
        CHECK(bt_hci_tx_next(&lifecycle, 0U, i * 10U, &tx));
        CHECK(tx.opcode == (u16)(0x4000U + i));
        complete(&lifecycle, tx.opcode, 1U, (u8)(i & 1U));
        CHECK(bt_hci_result_copy(&lifecycle, 0U, &handle, &result));
        CHECK(result.request_id == (u64)i + 1U);
        CHECK(result.hci_status == (u8)(i & 1U));
        stale = handle;
        CHECK(bt_hci_result_release(&lifecycle, 0U, &handle));
    }
}

int main(void)
{
    test_policy_and_init();
    test_submit_tx_complete_and_handles();
    test_status_credit_and_sequential_attempts();
    test_event_rejections();
    test_cancel_reset_and_capacity();
    test_many_generations();
    CHECK(checks > 300);
    if (failures) {
        printf("bt_hci_lifecycle: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("bt_hci_lifecycle: all %d checks passed\n", checks);
    return 0;
}
