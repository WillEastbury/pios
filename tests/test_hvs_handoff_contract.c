#include <stdio.h>
#include <string.h>

#include "types.h"
#include "hvs_handoff_contract.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static struct hvs_handoff_snapshot_input valid_snapshot(void)
{
    struct hvs_handoff_snapshot_input s;

    memset(&s, 0, sizeof(s));
    s.scanout_base = 0x100000U;
    s.scanout_generation = 7U;
    s.framebuffer_bytes = 1920U * 1080U * 4U;
    s.existing_list_generation = 9U;
    s.width = 1920U;
    s.height = 1080U;
    s.pitch = 1920U * 4U;
    s.snapshot_layout_version = HVS_HANDOFF_VERSION_V1;
    s.hvs_id = 1U;
    s.target_channel = HVS_HANDOFF_CHANNEL_V1;
    s.existing_list_index = 4U;
    s.owner = HVS_HANDOFF_OWNER_MAILBOX;
    s.format = HVS_HANDOFF_FORMAT_RGBA8888;
    s.pixel_order = HVS_HANDOFF_PIXEL_ORDER_RGBA;
    return s;
}

static struct hvs_handoff_list_input valid_list(
    const struct hvs_handoff_snapshot_input *s)
{
    struct hvs_handoff_list_input l;

    memset(&l, 0, sizeof(l));
    l.source_scanout_base = s->scanout_base;
    l.source_scanout_generation = s->scanout_generation;
    l.list_generation = 11U;
    l.element_count = 1U;
    l.staging_index = 18U;
    l.staging_range_start = 16U;
    l.staging_range_count = 4U;
    l.width = s->width;
    l.height = s->height;
    l.pitch = s->pitch;
    l.format = HVS_HANDOFF_FORMAT_RGBA8888;
    l.pixel_order = s->pixel_order;
    return l;
}

static void init_contract(struct hvs_handoff_contract *c,
                          struct hvs_handoff_handle *h, u64 now)
{
    memset(c, 0, sizeof(*c));
    CHECK(hvs_handoff_contract_init(c, 0x173U, HVS_HANDOFF_OWNER_CORE,
                                    now, 1000U));
    CHECK(hvs_handoff_handle_get(c, h));
}

static bool snapshot_contract(struct hvs_handoff_contract *c,
                              const struct hvs_handoff_handle *h, u64 now,
                              struct hvs_handoff_snapshot_input *s)
{
    return hvs_handoff_snapshot(c, h, HVS_HANDOFF_OWNER_CORE, now, s);
}

static bool stage_contract(struct hvs_handoff_contract *c,
                           const struct hvs_handoff_handle *h, u64 now,
                           const struct hvs_handoff_snapshot_input *s,
                           struct hvs_handoff_list_input *l)
{
    return snapshot_contract(c, h, now, (struct hvs_handoff_snapshot_input *)s) &&
           hvs_handoff_stage_list(c, h, HVS_HANDOFF_OWNER_CORE, now, l);
}

static void arm_contract(struct hvs_handoff_contract *c,
                         const struct hvs_handoff_handle *h, u64 now,
                         const struct hvs_handoff_list_input *l)
{
    struct hvs_handoff_observation o;

    memset(&o, 0, sizeof(o));
    o.list_generation = l->list_generation;
    o.channel = HVS_HANDOFF_CHANNEL_V1;
    o.observed_list_index = l->staging_index;
    o.attestation = HVS_HANDOFF_ATTESTATION_MATCHED;
    o.status = 0x44U;
    CHECK(hvs_handoff_report_arm(c, h, HVS_HANDOFF_OWNER_CORE, now, &o));
}

static void test_init_handle_identity(void)
{
    struct hvs_handoff_contract c, other;
    struct hvs_handoff_handle h, forged;
    enum hvs_handoff_state state;

    memset(&c, 0, sizeof(c));
    CHECK(sizeof(struct hvs_handoff_control) == 64U);
    CHECK(__builtin_offsetof(struct hvs_handoff_contract, snapshot) == 64U);
    CHECK(!hvs_handoff_contract_init(NULL, 1U, 0U, 0U, 1U));
    CHECK(!hvs_handoff_contract_init(&c, 0U, 0U, 0U, 1U));
    CHECK(!hvs_handoff_contract_init(&c, 1U, 1U, 0U, 1U));
    CHECK(!hvs_handoff_contract_init(&c, 1U, 0U, 0U, 0U));
    CHECK(!hvs_handoff_contract_init(&c, 1U, 0U, 0U, 1001U));
    CHECK(!hvs_handoff_contract_init(&c, 1U, 0U, ~0ULL, 1U));
    init_contract(&c, &h, 30U);
    CHECK(c.control.controller_id == 0x173U);
    CHECK(c.control.generation == 1U);
    CHECK(c.control.state == HVS_HANDOFF_MAILBOX_OWNER);
    CHECK(c.control.deadline_ms == 1030U);
    CHECK(!hvs_handoff_contract_init(&c, 0x174U, 0U, 0U, 1U));
    CHECK(h._generation == c.control.generation);
    CHECK(h._controller_id == c.control.controller_id);
    CHECK(h._reserved == 0U);
    CHECK(hvs_handoff_state_get(&c, &h, &state));
    CHECK(state == HVS_HANDOFF_MAILBOX_OWNER);
    forged = h;
    forged._generation++;
    CHECK(!hvs_handoff_state_get(&c, &forged, &state));
    forged = h;
    forged._token ^= 1ULL;
    CHECK(!hvs_handoff_state_get(&c, &forged, &state));
    forged = h;
    forged._reserved = 1U;
    CHECK(!hvs_handoff_state_get(&c, &forged, &state));
    memset(&other, 0, sizeof(other));
    CHECK(hvs_handoff_contract_init(&other, 0x174U, 0U, 30U, 1000U));
    CHECK(!hvs_handoff_state_get(&other, &h, &state));
    CHECK(!hvs_handoff_handle_get(NULL, &forged));
    CHECK(!hvs_handoff_handle_get(&c, NULL));
}

static void test_snapshot_validation_and_copy(void)
{
    struct hvs_handoff_contract c;
    struct hvs_handoff_handle h;
    struct hvs_handoff_snapshot_input s, copied;
    u32 i;

    s = valid_snapshot();
    for (i = 0U; i < 19U; i++) {
        init_contract(&c, &h, 0U);
        s = valid_snapshot();
        if (i == 0U) s.owner = HVS_HANDOFF_OWNER_NATIVE;
        if (i == 1U) s.snapshot_layout_version = 99U;
        if (i == 2U) s.hvs_id = 0U;
        if (i == 3U) s.target_channel = 1U;
        if (i == 4U) s.scanout_base = 0U;
        if (i == 5U) s.scanout_generation = 0U;
        if (i == 6U) s.existing_list_generation = 0U;
        if (i == 7U) s.existing_list_index = HVS_HANDOFF_MAX_STAGING_INDEX + 1U;
        if (i == 8U) s.width = 0U;
        if (i == 9U) s.height = HVS_HANDOFF_MAX_DIMENSION + 1U;
        if (i == 10U) s.pitch = s.width * 4U - 1U;
        if (i == 11U) s.pitch = HVS_HANDOFF_MAX_PITCH + 1U;
        if (i == 12U) s.framebuffer_bytes = 1U;
        if (i == 13U) s.scanout_base = HVS_HANDOFF_MAX_SCANOUT_BYTES;
        if (i == 14U) s.framebuffer_bytes = HVS_HANDOFF_MAX_SCANOUT_BYTES;
        if (i == 15U) s.pixel_order = HVS_HANDOFF_PIXEL_ORDER_INVALID;
        if (i == 16U) s._reserved = 1U;
        if (i == 17U) { s.height = 8192U; s.pitch = 32768U; s.framebuffer_bytes = 1U; }
        if (i == 18U) s.format = HVS_HANDOFF_FORMAT_INVALID;
        CHECK(!snapshot_contract(&c, &h, 1U, &s));
        CHECK(c.control.state == HVS_HANDOFF_QUARANTINED);
        CHECK(c.control.fault == HVS_HANDOFF_FAULT_SNAPSHOT);
        CHECK(c.control.handoff_attempt_count == 1U);
        CHECK(c.control.failure_count == 1U);
    }
    init_contract(&c, &h, 0U);
    s = valid_snapshot();
    CHECK(snapshot_contract(&c, &h, 1U, &s));
    s.width = 1U;
    s.scanout_base = 0U;
    CHECK(hvs_handoff_snapshot_get(&c, &h, &copied));
    CHECK(copied.width == 1920U);
    CHECK(copied.scanout_base == 0x100000U);
    CHECK(c.control.state == HVS_HANDOFF_SNAPSHOT_VALID);
    CHECK(!hvs_handoff_snapshot_get(&c, NULL, &copied));
}

static void test_list_validation_copy_and_order(void)
{
    struct hvs_handoff_contract c;
    struct hvs_handoff_handle h;
    struct hvs_handoff_snapshot_input s;
    struct hvs_handoff_list_input l, copied;
    u32 i;

    s = valid_snapshot();
    for (i = 0U; i < 15U; i++) {
        init_contract(&c, &h, 0U);
        l = valid_list(&s);
        if (i == 0U) l.element_count = 0U;
        if (i == 1U) l.element_count = 17U;
        if (i == 2U) l.list_generation = 0U;
        if (i == 3U) l.staging_index = 1024U;
        if (i == 4U) l.staging_range_start = 1024U;
        if (i == 5U) l.staging_range_count = 0U;
        if (i == 6U) l.staging_range_count = 17U;
        if (i == 7U) l.staging_index = 15U;
        if (i == 8U) l.staging_range_start = 1020U, l.staging_range_count = 5U;
        if (i == 9U) l.source_scanout_base++;
        if (i == 10U) l.source_scanout_generation++;
        if (i == 11U) l.width++;
        if (i == 12U) l.format = HVS_HANDOFF_FORMAT_INVALID;
        if (i == 13U) l.pixel_order = HVS_HANDOFF_PIXEL_ORDER_ARGB;
        if (i == 14U) l._reserved = 1U;
        CHECK(snapshot_contract(&c, &h, 1U, &s));
        CHECK(!hvs_handoff_stage_list(&c, &h, 0U, 2U, &l));
        CHECK(c.control.state == HVS_HANDOFF_QUARANTINED);
        CHECK(c.control.fault == HVS_HANDOFF_FAULT_LIST);
    }
    init_contract(&c, &h, 0U);
    l = valid_list(&s);
    CHECK(stage_contract(&c, &h, 1U, &s, &l));
    l.staging_index = 99U;
    l.source_scanout_base = 0U;
    CHECK(hvs_handoff_list_get(&c, &h, &copied));
    CHECK(copied.staging_index == 18U);
    CHECK(copied.source_scanout_base == s.scanout_base);
    CHECK(!hvs_handoff_list_get(&c, NULL, &copied));
}

static void test_state_orders_observations_restore(void)
{
    struct hvs_handoff_contract c;
    struct hvs_handoff_handle h;
    struct hvs_handoff_snapshot_input s = valid_snapshot();
    struct hvs_handoff_list_input l = valid_list(&s);
    struct hvs_handoff_observation o;
    struct hvs_handoff_restore_observation r;

    init_contract(&c, &h, 0U);
    memset(&o, 0, sizeof(o));
    CHECK(!hvs_handoff_stage_list(&c, &h, 0U, 1U, &l));
    CHECK(!hvs_handoff_report_arm(&c, &h, 0U, 1U, &o));
    CHECK(!hvs_handoff_report_present(&c, &h, 0U, 1U, &o));
    CHECK(!hvs_handoff_restore_begin(&c, &h, 0U, 1U));
    CHECK(snapshot_contract(&c, &h, 1U, &s));
    CHECK(!hvs_handoff_snapshot(&c, &h, 0U, 2U, &s));
    CHECK(!hvs_handoff_report_arm(&c, &h, 0U, 2U, &o));
    CHECK(hvs_handoff_stage_list(&c, &h, 0U, 2U, &l));
    CHECK(!hvs_handoff_report_present(&c, &h, 0U, 3U, &o));
    o.list_generation = l.list_generation;
    o.channel = 1U;
    o.observed_list_index = l.staging_index;
    o.attestation = HVS_HANDOFF_ATTESTATION_MATCHED;
    o.status = 0x55U;
    CHECK(!hvs_handoff_report_arm(&c, &h, 0U, 3U, &o));
    CHECK(c.control.fault == HVS_HANDOFF_FAULT_ARM);
    CHECK(!hvs_handoff_stage_list(&c, &h, 0U, 4U, &l));
    CHECK(hvs_handoff_restore_begin(&c, &h, 0U, 4U));

    init_contract(&c, &h, 0U);
    CHECK(stage_contract(&c, &h, 1U, &s, &l));
    arm_contract(&c, &h, 2U, &l);
    o.list_generation = l.list_generation;
    o.channel = 0U;
    o.observed_list_index = l.staging_index;
    o.attestation = HVS_HANDOFF_ATTESTATION_FAILED;
    o.status = 0x77U;
    CHECK(!hvs_handoff_report_present(&c, &h, 0U, 3U, &o));
    CHECK(c.control.fault == HVS_HANDOFF_FAULT_PRESENT);
    CHECK(hvs_handoff_restore_begin(&c, &h, 0U, 4U));
    memset(&r, 0, sizeof(r));
    r.existing_list_generation = s.existing_list_generation;
    r.channel = 0U;
    r.observed_list_index = s.existing_list_index;
    r.observed_owner = HVS_HANDOFF_OWNER_MAILBOX;
    r.attestation = HVS_HANDOFF_ATTESTATION_FAILED;
    r.status = 0x88U;
    CHECK(!hvs_handoff_report_restore(&c, &h, 0U, 5U, &r));
    CHECK(c.control.state == HVS_HANDOFF_QUARANTINED);
    CHECK(c.control.fault == HVS_HANDOFF_FAULT_RESTORE);

    init_contract(&c, &h, 0U);
    CHECK(stage_contract(&c, &h, 1U, &s, &l));
    arm_contract(&c, &h, 2U, &l);
    o.attestation = HVS_HANDOFF_ATTESTATION_PRESENTED;
    o.status = 0x99U;
    CHECK(hvs_handoff_report_present(&c, &h, 0U, 3U, &o));
    CHECK(c.control.state == HVS_HANDOFF_NATIVE_OWNER);
    CHECK(hvs_handoff_restore_begin(&c, &h, 0U, 4U));
    r.attestation = HVS_HANDOFF_ATTESTATION_RESTORED;
    CHECK(hvs_handoff_report_restore(&c, &h, 0U, 5U, &r));
    CHECK(c.control.state == HVS_HANDOFF_MAILBOX_OWNER);
    CHECK(c.control.restore_attempt_count == 1U);
}

static void test_deadlines_owner_rearm_and_generation(void)
{
    struct hvs_handoff_contract c;
    struct hvs_handoff_handle h, new_h;
    struct hvs_handoff_snapshot_input s = valid_snapshot();
    struct hvs_handoff_list_input l = valid_list(&s);
    u32 i;

    for (i = 0U; i < 6U; i++) {
        init_contract(&c, &h, 100U);
        if (i == 0U) CHECK(!snapshot_contract(&c, &h, 1101U, &s));
        if (i == 1U) { CHECK(snapshot_contract(&c, &h, 101U, &s)); CHECK(!hvs_handoff_stage_list(&c, &h, 0U, 1101U, &l)); }
        if (i == 2U) { CHECK(stage_contract(&c, &h, 101U, &s, &l)); CHECK(!hvs_handoff_report_arm(&c, &h, 0U, 1101U, NULL)); }
        if (i == 3U) { CHECK(stage_contract(&c, &h, 101U, &s, &l)); CHECK(!hvs_handoff_report_present(&c, &h, 0U, 1101U, NULL)); }
        if (i == 4U) { CHECK(stage_contract(&c, &h, 101U, &s, &l)); arm_contract(&c, &h, 102U, &l); CHECK(!hvs_handoff_report_present(&c, &h, 0U, 1101U, NULL)); }
        if (i == 5U) { CHECK(!hvs_handoff_restore_begin(&c, &h, 0U, 1101U)); }
        CHECK(c.control.state == HVS_HANDOFF_QUARANTINED);
        CHECK(c.control.fault == HVS_HANDOFF_FAULT_DEADLINE);
    }
    init_contract(&c, &h, 0U);
    CHECK(!hvs_handoff_snapshot(&c, &h, 1U, 1U, &s));
    CHECK(c.control.fault == HVS_HANDOFF_FAULT_OWNER);
    CHECK(hvs_handoff_rearm(&c, &h, 0U, 2U, &new_h));
    CHECK(new_h._generation != h._generation);
    CHECK(!hvs_handoff_snapshot(&c, &h, 0U, 3U, &s));
    CHECK(snapshot_contract(&c, &new_h, 3U, &s));

    init_contract(&c, &h, 0U);
    CHECK(!snapshot_contract(&c, &h, 1U, NULL));
    c.control.generation = ~0ULL;
    h._generation = ~0ULL;
    CHECK(!hvs_handoff_rearm(&c, &h, 0U, 2U, &new_h));

    init_contract(&c, &h, 0U);
    CHECK(!snapshot_contract(&c, &h, 1001U, &s));
    CHECK(!hvs_handoff_rearm(&c, &h, 0U, 1002U, &new_h));
    CHECK(c.control.state == HVS_HANDOFF_QUARANTINED);
}

int main(void)
{
    test_init_handle_identity();
    test_snapshot_validation_and_copy();
    test_list_validation_copy_and_order();
    test_state_orders_observations_restore();
    test_deadlines_owner_rearm_and_generation();
    if (failures) {
        printf("hvs handoff contract: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("hvs handoff contract: %d checks passed\n", checks);
    return checks > 250 ? 0 : 1;
}
