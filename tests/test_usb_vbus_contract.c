#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <string.h>

#include "types.h"
#include "usb_vbus_contract.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

/*
 * This host-only compilation is also the source gate: this test links only
 * the pure contract source and the types shim.  No board or hardware header
 * is available to the unit under test.
 */
static void init_pool(struct usb_vbus_contract_pool *pool)
{
    memset(pool, 0, sizeof(*pool));
    CHECK(usb_vbus_contract_pool_init(pool));
}

static struct usb_vbus_external_attestation valid_attestation(
    u32 board_model, bool observer)
{
    struct usb_vbus_external_attestation a;

    memset(&a, 0, sizeof(a));
    a.topology_id = 0xA500U + board_model;
    a.topology_generation = 7U;
    a.board_model = board_model;
    a.externally_powered = true;
    a.independent_current_protection = true;
    a.port_cable_verified = true;
    a.no_backfeed_verified = true;
    if (observer) {
        a.observer_id = 0xBB01U;
        a.observer_generation = 0x11U;
        a.declared_max_milliamps = 900U;
    }
    return a;
}

static struct usb_vbus_current_observation valid_observation(u32 current)
{
    struct usb_vbus_current_observation o;

    memset(&o, 0, sizeof(o));
    o.observer_id = 0xBB01U;
    o.observer_generation = 0x11U;
    o.current_milliamps = current;
    return o;
}

static void test_source_gate_file(const char *path)
{
    static const char *const forbidden[] = {
        "#include <",
        "platform.h",
        "board_detect",
        "mmio",
        "mailbox",
        "gpio",
        "rp1",
        "timer",
        "watchdog",
        "irq",
        "dma",
        "malloc",
        "calloc",
        "realloc",
        "free(",
    };
    char line[256];
    FILE *file;
    u32 i;

    file = fopen(path, "rb");
    CHECK(file != NULL);
    if (!file)
        return;
    while (fgets(line, sizeof(line), file) != NULL) {
        for (i = 0U; i < sizeof(forbidden) / sizeof(forbidden[0]); i++)
            CHECK(strstr(line, forbidden[i]) == NULL);
    }
    CHECK(fclose(file) == 0);
}

static void test_source_gate(void)
{
    test_source_gate_file("include\\usb_vbus_contract.h");
    test_source_gate_file("src\\usb_vbus_contract.c");
}

static void acquire(struct usb_vbus_contract_pool *pool, u32 controller_id,
                    u32 board_model, struct usb_vbus_handle *handle)
{
    init_pool(pool);
    CHECK(usb_vbus_contract_acquire(pool, controller_id, board_model,
                                    USB_VBUS_OWNER_CORE, handle));
}

static void attest(struct usb_vbus_contract_pool *pool,
                   const struct usb_vbus_handle *handle, bool observer)
{
    struct usb_vbus_external_attestation a =
        valid_attestation(USB_VBUS_PROFILE_PI3_B, observer);

    CHECK(usb_vbus_external_attest(pool, handle, USB_VBUS_OWNER_CORE, &a));
}

static void test_profile_facts_and_software_gate(void)
{
    const struct usb_vbus_profile *p;
    const u32 boards[] = {
        USB_VBUS_PROFILE_PI3_B,
        USB_VBUS_PROFILE_PI3_B_PLUS,
        USB_VBUS_PROFILE_ZERO_2_W,
    };
    const u32 topologies[] = {
        USB_VBUS_TOPOLOGY_PI3_B_LAN9514,
        USB_VBUS_TOPOLOGY_PI3_B_PLUS_LAN7515,
        USB_VBUS_TOPOLOGY_ZERO_2_W_OTG_CONNECTOR,
    };
    struct usb_vbus_profile forged;
    u32 missing;
    u32 i;

    CHECK(USB_VBUS_PROFILE_PI3_B == 0x08U);
    CHECK(USB_VBUS_PROFILE_PI3_B_PLUS == 0x0DU);
    CHECK(USB_VBUS_PROFILE_ZERO_2_W == 0x12U);
    CHECK(USB_VBUS_CONTRACT_CAPACITY == 4U);
    CHECK(USB_VBUS_OWNER_CORE == 0U);
    CHECK(sizeof(struct usb_vbus_record) == 64U);
    CHECK(sizeof(struct usb_vbus_attestation_copy) == 64U);
    CHECK(sizeof(struct usb_vbus_pool_owner) == 64U);
    CHECK(__builtin_offsetof(struct usb_vbus_contract_pool, records) == 64U);
    CHECK(__builtin_offsetof(struct usb_vbus_contract_pool, attestations) ==
          (USB_VBUS_CONTRACT_CAPACITY + 1U) * 64U);
    for (i = 0U; i < 3U; i++) {
        p = usb_vbus_profile_get(boards[i]);
        CHECK(p != NULL);
        CHECK(p->board_model == boards[i]);
        CHECK(p->topology == topologies[i]);
        CHECK(p->software_vbus_control_mask == 0U);
        CHECK(p->software_current_limit_mask == 0U);
        CHECK(p->software_overcurrent_mask == 0U);
        CHECK(p->provenance_revision != 0U);
        missing = 0U;
        CHECK(!usb_vbus_software_enable_permitted(p, &missing));
        CHECK(missing == (USB_VBUS_PROOF_CONTROL |
                          USB_VBUS_PROOF_CURRENT_LIMIT |
                          USB_VBUS_PROOF_OVERCURRENT));
    }
    CHECK(usb_vbus_profile_get(0U) == NULL);
    CHECK(usb_vbus_profile_get(0x09U) == NULL);
    CHECK(usb_vbus_profile_get(0xFFFFFFFFU) == NULL);
    memset(&forged, 0, sizeof(forged));
    forged.software_vbus_control_mask = ~0U;
    forged.software_current_limit_mask = ~0U;
    forged.software_overcurrent_mask = ~0U;
    missing = 0U;
    CHECK(!usb_vbus_software_enable_permitted(&forged, &missing));
    CHECK(missing == (USB_VBUS_PROOF_CONTROL |
                      USB_VBUS_PROOF_CURRENT_LIMIT |
                      USB_VBUS_PROOF_OVERCURRENT));
    CHECK(!usb_vbus_software_enable_permitted(NULL, NULL));
}

static void test_init_acquire_handle_and_owner(void)
{
    struct usb_vbus_contract_pool pool, other;
    struct usb_vbus_handle h, got, other_handle, forged;
    enum usb_vbus_state state;

    memset(&pool, 0, sizeof(pool));
    CHECK(!usb_vbus_contract_pool_init(NULL));
    pool.records[0].generation = 1U;
    CHECK(!usb_vbus_contract_pool_init(&pool));
    memset(&pool, 0, sizeof(pool));
    CHECK(usb_vbus_contract_pool_init(&pool));
    CHECK(!usb_vbus_contract_pool_init(&pool));
    CHECK(pool.records[0].generation == 1U);
    CHECK(pool.records[3].generation == 4U);
    CHECK(pool.records[0].state == USB_VBUS_FREE);
    memset(&pool, 0, sizeof(pool));
    CHECK(!usb_vbus_contract_acquire(&pool, 1U, USB_VBUS_PROFILE_PI3_B,
                                     0U, &h));
    init_pool(&pool);
    memset(&h, 0xA5, sizeof(h));
    CHECK(!usb_vbus_contract_acquire(&pool, 0U, USB_VBUS_PROFILE_PI3_B,
                                     0U, &h));
    CHECK(h.generation == 0U && h.controller_id == 0U);
    CHECK(!usb_vbus_contract_acquire(&pool, 1U, 0x09U, 0U, &h));
    CHECK(!usb_vbus_contract_acquire(&pool, 1U, USB_VBUS_PROFILE_PI3_B,
                                     1U, &h));
    CHECK(!usb_vbus_contract_acquire(&pool, 1U, USB_VBUS_PROFILE_PI3_B,
                                     0U, NULL));
    CHECK(usb_vbus_contract_acquire(&pool, 0x179U, USB_VBUS_PROFILE_PI3_B,
                                    0U, &h));
    CHECK(h.generation == 1U && h.controller_id == 0x179U && h.slot == 0U);
    CHECK(h._reserved == 0U && h._pad == 0U);
    CHECK(usb_vbus_state_get(&pool, &h, &state));
    CHECK(state == USB_VBUS_SAFE_UNKNOWN);
    CHECK(!usb_vbus_contract_acquire(&pool, 0x179U,
                                     USB_VBUS_PROFILE_PI3_B_PLUS, 0U, &got));
    CHECK(usb_vbus_handle_get(&pool, 0x179U, &got));
    CHECK(memcmp(&h, &got, sizeof(h)) == 0);
    CHECK(usb_vbus_contract_acquire(&pool, 0x17AU,
                                    USB_VBUS_PROFILE_PI3_B_PLUS, 0U,
                                    &other_handle));
    forged = other_handle;
    forged.slot = h.slot;
    CHECK(!usb_vbus_state_get(&pool, &forged, &state));
    CHECK(usb_vbus_state_get(&pool, &other_handle, &state));
    CHECK(state == USB_VBUS_SAFE_UNKNOWN);
    CHECK(!usb_vbus_handle_get(&pool, 0U, &got));
    forged = h;
    forged._token ^= 1U;
    CHECK(!usb_vbus_state_get(&pool, &forged, &state));
    forged = h;
    forged.generation++;
    CHECK(!usb_vbus_state_get(&pool, &forged, &state));
    forged = h;
    forged.controller_id++;
    CHECK(!usb_vbus_state_get(&pool, &forged, &state));
    forged = h;
    forged.slot = USB_VBUS_CONTRACT_CAPACITY;
    CHECK(!usb_vbus_state_get(&pool, &forged, &state));
    forged = h;
    forged._reserved = 1U;
    CHECK(!usb_vbus_state_get(&pool, &forged, &state));
    CHECK(!usb_vbus_contract_release(&pool, &h, 1U));
    CHECK(pool.records[0].state == USB_VBUS_SAFE_UNKNOWN);
    memset(&other, 0, sizeof(other));
    CHECK(usb_vbus_contract_pool_init(&other));
    CHECK(!usb_vbus_state_get(&other, &h, &state));
}

static void test_attestation_validation_and_private_copy(void)
{
    struct usb_vbus_contract_pool pool;
    struct usb_vbus_handle h;
    struct usb_vbus_external_attestation a, copied;
    enum usb_vbus_state state;
    u32 i;

    for (i = 0U; i < 14U; i++) {
        acquire(&pool, 0x200U + i, USB_VBUS_PROFILE_PI3_B, &h);
        a = valid_attestation(USB_VBUS_PROFILE_PI3_B, true);
        if (i == 0U) a.topology_id = 0U;
        if (i == 1U) a.topology_generation = 0U;
        if (i == 2U) a.board_model = USB_VBUS_PROFILE_PI3_B_PLUS;
        if (i == 3U) a.externally_powered = false;
        if (i == 4U) a.independent_current_protection = false;
        if (i == 5U) a.port_cable_verified = false;
        if (i == 6U) a.no_backfeed_verified = false;
        if (i == 7U) a._reserved[0] = 1U;
        if (i == 8U) a._reserved[1] = 1U;
        if (i == 9U) { a.observer_id = 0U; a.declared_max_milliamps = 9U; }
        if (i == 10U) a.observer_generation = 0U;
        if (i == 11U) a.declared_max_milliamps = 0U;
        if (i == 12U) a.declared_max_milliamps =
                           USB_VBUS_MAX_ATTESTED_MILLIAMPS + 1U;
        if (i == 13U) a.externally_powered = 2U;
        CHECK(!usb_vbus_external_attest(&pool, &h, USB_VBUS_OWNER_CORE, &a));
        CHECK(usb_vbus_state_get(&pool, &h, &state));
        CHECK(state == USB_VBUS_SAFE_UNKNOWN);
        CHECK(!usb_vbus_probe_permitted(&pool, &h));
    }
    acquire(&pool, 0x300U, USB_VBUS_PROFILE_PI3_B, &h);
    a = valid_attestation(USB_VBUS_PROFILE_PI3_B, false);
    CHECK(usb_vbus_external_attest(&pool, &h, 0U, &a));
    a.topology_id = 0U;
    a.externally_powered = false;
    CHECK(usb_vbus_attestation_get(&pool, &h, &copied));
    CHECK(copied.topology_id == 0xA508U);
    CHECK(copied.externally_powered == true);
    copied.topology_generation = 0U;
    CHECK(usb_vbus_attestation_get(&pool, &h, &a));
    CHECK(a.topology_generation == 7U);
    CHECK(!usb_vbus_attestation_get(&pool, NULL, &a));
    CHECK(!usb_vbus_external_attest(&pool, &h, 0U, &a));
    CHECK(!usb_vbus_external_attest(&pool, &h, 1U, &copied));
    CHECK(usb_vbus_state_get(&pool, &h, &state));
    CHECK(state == USB_VBUS_EXTERNAL_ATTESTED);
}

static void test_probe_and_observer_contract(void)
{
    struct usb_vbus_contract_pool pool;
    struct usb_vbus_handle h;
    struct usb_vbus_current_observation o;
    enum usb_vbus_state state;

    acquire(&pool, 0x400U, USB_VBUS_PROFILE_PI3_B, &h);
    attest(&pool, &h, false);
    CHECK(usb_vbus_probe_permitted(&pool, &h));
    CHECK(!usb_vbus_telemetry_available(&pool, &h));
    o = valid_observation(1U);
    CHECK(!usb_vbus_current_observe(&pool, &h, 0U, &o));
    CHECK(usb_vbus_probe_permitted(&pool, &h));
    CHECK(usb_vbus_state_get(&pool, &h, &state));
    CHECK(state == USB_VBUS_EXTERNAL_ATTESTED);

    acquire(&pool, 0x401U, USB_VBUS_PROFILE_PI3_B, &h);
    attest(&pool, &h, true);
    CHECK(!usb_vbus_probe_permitted(&pool, &h));
    CHECK(usb_vbus_telemetry_available(&pool, &h));
    o = valid_observation(0U);
    CHECK(usb_vbus_current_observe(&pool, &h, 0U, &o));
    CHECK(usb_vbus_probe_permitted(&pool, &h));
    CHECK(usb_vbus_state_get(&pool, &h, &state));
    CHECK(state == USB_VBUS_AVAILABLE_UNCONTROLLED);
    CHECK(pool.records[h.slot].last_current_milliamps == 0U);
    o.current_milliamps = 900U;
    CHECK(usb_vbus_current_observe(&pool, &h, 0U, &o));
    CHECK(pool.records[h.slot].last_current_milliamps == 900U);
    CHECK(pool.records[h.slot].observation_count == 2U);

    acquire(&pool, 0x402U, USB_VBUS_PROFILE_PI3_B, &h);
    attest(&pool, &h, true);
    o = valid_observation(901U);
    CHECK(!usb_vbus_current_observe(&pool, &h, 0U, &o));
    CHECK(usb_vbus_state_get(&pool, &h, &state));
    CHECK(state == USB_VBUS_QUARANTINED);
    CHECK(pool.records[h.slot].fault == USB_VBUS_FAULT_CURRENT_LIMIT);
    CHECK(pool.records[h.slot].fault_count == 1U);
    CHECK(!usb_vbus_probe_permitted(&pool, &h));
    CHECK(!usb_vbus_telemetry_available(&pool, &h));

    acquire(&pool, 0x403U, USB_VBUS_PROFILE_PI3_B, &h);
    attest(&pool, &h, true);
    o = valid_observation(1U);
    o.overcurrent = true;
    CHECK(!usb_vbus_current_observe(&pool, &h, 0U, &o));
    CHECK(pool.records[h.slot].state == USB_VBUS_QUARANTINED);
    CHECK(pool.records[h.slot].fault == USB_VBUS_FAULT_OVERCURRENT);

    acquire(&pool, 0x404U, USB_VBUS_PROFILE_PI3_B, &h);
    attest(&pool, &h, true);
    o = valid_observation(1U);
    o.observer_id++;
    CHECK(!usb_vbus_current_observe(&pool, &h, 0U, &o));
    o = valid_observation(1U);
    o.observer_generation++;
    CHECK(!usb_vbus_current_observe(&pool, &h, 0U, &o));
    o = valid_observation(1U);
    o.observer_id = 0U;
    CHECK(!usb_vbus_current_observe(&pool, &h, 0U, &o));
    o = valid_observation(1U);
    o._reserved = 1U;
    CHECK(!usb_vbus_current_observe(&pool, &h, 0U, &o));
    o = valid_observation(1U);
    o.overcurrent = 2U;
    CHECK(!usb_vbus_current_observe(&pool, &h, 0U, &o));
    CHECK(pool.records[h.slot].state == USB_VBUS_EXTERNAL_ATTESTED);
    CHECK(pool.records[h.slot].observation_count == 0U);
    o = valid_observation(1U);
    CHECK(!usb_vbus_current_observe(&pool, &h, 1U, &o));
    CHECK(pool.records[h.slot].state == USB_VBUS_EXTERNAL_ATTESTED);
}

static void test_lifecycle_stale_handles_and_saturation(void)
{
    struct usb_vbus_contract_pool pool;
    struct usb_vbus_handle h, stale, handles[USB_VBUS_CONTRACT_CAPACITY], extra;
    struct usb_vbus_external_attestation a;
    enum usb_vbus_state state;
    u32 i, j;

    acquire(&pool, 0x500U, USB_VBUS_PROFILE_PI3_B, &h);
    stale = h;
    CHECK(usb_vbus_contract_release(&pool, &h, 0U));
    CHECK(pool.records[h.slot].state == USB_VBUS_FREE);
    CHECK(pool.records[h.slot].generation == 2U);
    CHECK(!usb_vbus_state_get(&pool, &stale, &state));
    CHECK(!usb_vbus_contract_release(&pool, &stale, 0U));
    CHECK(usb_vbus_contract_acquire(&pool, 0x500U, USB_VBUS_PROFILE_PI3_B,
                                    0U, &h));
    CHECK(h.slot == stale.slot && h.generation != stale.generation);
    a = valid_attestation(USB_VBUS_PROFILE_PI3_B, false);
    CHECK(usb_vbus_external_attest(&pool, &h, 0U, &a));
    CHECK(usb_vbus_contract_release(&pool, &h, 0U));
    CHECK(!usb_vbus_attestation_get(&pool, &h, &a));

    acquire(&pool, 0x510U, USB_VBUS_PROFILE_PI3_B, &h);
    attest(&pool, &h, true);
    CHECK(!usb_vbus_probe_permitted(&pool, &h));
    a = valid_attestation(USB_VBUS_PROFILE_PI3_B, true);
    CHECK(!usb_vbus_external_attest(&pool, &h, 0U, &a));
    CHECK(usb_vbus_contract_release(&pool, &h, 0U));
    CHECK(usb_vbus_contract_acquire(&pool, 0x510U, USB_VBUS_PROFILE_PI3_B,
                                    0U, &h));
    CHECK(!usb_vbus_probe_permitted(&pool, &h));

    init_pool(&pool);
    for (i = 0U; i < USB_VBUS_CONTRACT_CAPACITY; i++) {
        CHECK(usb_vbus_contract_acquire(&pool, 0x600U + i,
                                        USB_VBUS_PROFILE_PI3_B, 0U,
                                        &handles[i]));
        for (j = 0U; j < i; j++)
            CHECK(handles[i].slot != handles[j].slot);
    }
    memset(&extra, 0xA5, sizeof(extra));
    CHECK(!usb_vbus_contract_acquire(&pool, 0x700U, USB_VBUS_PROFILE_PI3_B,
                                     0U, &extra));
    CHECK(extra.generation == 0U && extra.controller_id == 0U);
    for (i = 0U; i < USB_VBUS_CONTRACT_CAPACITY; i++)
        CHECK(usb_vbus_contract_release(&pool, &handles[i], 0U));
}

static void test_quarantine_recovery_and_generation_exhaustion(void)
{
    struct usb_vbus_contract_pool pool;
    struct usb_vbus_handle h, fresh;
    struct usb_vbus_current_observation o;
    struct usb_vbus_external_attestation a;
    enum usb_vbus_state state;

    acquire(&pool, 0x800U, USB_VBUS_PROFILE_PI3_B, &h);
    attest(&pool, &h, true);
    o = valid_observation(901U);
    CHECK(!usb_vbus_current_observe(&pool, &h, 0U, &o));
    a = valid_attestation(USB_VBUS_PROFILE_PI3_B, true);
    CHECK(!usb_vbus_external_attest(&pool, &h, 0U, &a));
    CHECK(!usb_vbus_current_observe(&pool, &h, 0U, &o));
    CHECK(usb_vbus_contract_release(&pool, &h, 0U));
    CHECK(!usb_vbus_state_get(&pool, &h, &state));
    CHECK(usb_vbus_contract_acquire(&pool, 0x800U, USB_VBUS_PROFILE_PI3_B,
                                    0U, &fresh));
    CHECK(fresh.generation != h.generation);
    CHECK(usb_vbus_state_get(&pool, &fresh, &state));
    CHECK(state == USB_VBUS_SAFE_UNKNOWN);
    CHECK(!usb_vbus_probe_permitted(&pool, &fresh));
    a = valid_attestation(USB_VBUS_PROFILE_PI3_B, false);
    CHECK(usb_vbus_external_attest(&pool, &fresh, 0U, &a));
    CHECK(usb_vbus_probe_permitted(&pool, &fresh));

    init_pool(&pool);
    pool.records[0].generation = ~0ULL;
    CHECK(usb_vbus_contract_acquire(&pool, 0x900U, USB_VBUS_PROFILE_PI3_B,
                                    0U, &h));
    CHECK(h.slot == 0U && h.generation == ~0ULL);
    CHECK(usb_vbus_contract_release(&pool, &h, 0U));
    CHECK(pool.records[0].state == USB_VBUS_RETIRED);
    CHECK(!usb_vbus_state_get(&pool, &h, &state));
    for (u32 i = 0U; i < USB_VBUS_CONTRACT_CAPACITY - 1U; i++)
        CHECK(usb_vbus_contract_acquire(&pool, 0xA00U + i,
                                        USB_VBUS_PROFILE_PI3_B, 0U, &fresh));
    CHECK(!usb_vbus_contract_acquire(&pool, 0xB00U, USB_VBUS_PROFILE_PI3_B,
                                     0U, &fresh));
}

int main(void)
{
    test_source_gate();
    test_profile_facts_and_software_gate();
    test_init_acquire_handle_and_owner();
    test_attestation_validation_and_private_copy();
    test_probe_and_observer_contract();
    test_lifecycle_stale_handles_and_saturation();
    test_quarantine_recovery_and_generation_exhaustion();
    if (failures) {
        printf("usb vbus contract: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("usb vbus contract: %d checks passed\n", checks);
    return checks > 250 ? 0 : 1;
}
