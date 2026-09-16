#include <stdio.h>
#include <string.h>

#include "types.h"
#include "rp1_cfe_contract.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

_Static_assert(sizeof(struct rp1_cfe_stream_control) == 64U,
               "control cache-line ownership");
_Static_assert(__builtin_offsetof(struct rp1_cfe_contract, controls) == 64U,
               "control separation");
_Static_assert(__builtin_offsetof(struct rp1_cfe_contract, payloads) ==
               64U + RP1_CFE_STREAM_CAPACITY * 64U, "payload separation");

static void init_contract(struct rp1_cfe_contract *contract, u32 controller_id)
{
    memset(contract, 0, sizeof(*contract));
    CHECK(rp1_cfe_contract_init(contract, controller_id));
}

static struct rp1_cfe_stream_descriptor valid_descriptor(u32 instance)
{
    struct rp1_cfe_stream_descriptor descriptor;

    memset(&descriptor, 0, sizeof(descriptor));
    descriptor.layout_version = RP1_CFE_STREAM_LAYOUT_V1;
    descriptor.sensor_connector_profile_id = 0x100U + instance;
    descriptor.sensor_connector_profile_generation = 7U;
    descriptor.instance = instance;
    descriptor.source_stream_id = 0x200U + instance;
    descriptor.source_stream_generation = 11U;
    descriptor.width = 640U;
    descriptor.height = 480U;
    descriptor.stride = 640U;
    descriptor.pixel_format = RP1_CFE_FORMAT_RAW8;
    descriptor.lane_count = 2U;
    descriptor.config_identity = 0x300U;
    descriptor.config_revision = 1U;
    descriptor.config_length = 64U;
    return descriptor;
}

static struct rp1_cfe_sink_span valid_sink(u32 role, u64 allocation_id,
                                           u64 base)
{
    struct rp1_cfe_sink_span sink;

    memset(&sink, 0, sizeof(sink));
    sink.allocation_id = allocation_id;
    sink.allocation_generation = 3U;
    sink.allocation_base = base;
    sink.allocation_capacity = 0x1000U;
    sink.allocation_offset = 0U;
    sink.window_capacity = 0x400U;
    sink.used = 0x200U;
    sink.output_canary = 0xCFE00000ULL + role;
    sink.role = role;
    sink.access = RP1_CFE_SINK_DEVICE_WRITE;
    return sink;
}

static bool prepare_once(const struct rp1_cfe_stream_descriptor *descriptor,
                         const struct rp1_cfe_sink_span *sinks, u32 sink_count)
{
    struct rp1_cfe_contract contract;
    struct rp1_cfe_stream_handle handle;

    memset(&contract, 0, sizeof(contract));
    if (!rp1_cfe_contract_init(&contract, 0x1234U))
        return false;
    return rp1_cfe_contract_prepare(&contract, descriptor, sinks, sink_count,
                                    &handle);
}

static void test_resource_profiles(void)
{
    static const u8 revision[RP1_CFE_SOURCE_REVISION_BYTES] = {
        0xcfU, 0xf5U, 0x33U, 0xaeU, 0xc2U, 0xfaU, 0x60U, 0x18U,
        0x46U, 0x76U, 0x6bU, 0x32U, 0xffU, 0x57U, 0x20U, 0x4eU,
        0x0aU, 0x61U, 0xbeU, 0xd7U,
    };
    static const u64 expected_base[RP1_CFE_INSTANCE_COUNT][RP1_CFE_WINDOW_COUNT] = {
        { 0xc040110000ULL, 0xc040114000ULL, 0xc040120000ULL,
          0xc040124000ULL },
        { 0xc040128000ULL, 0xc04012c000ULL, 0xc040138000ULL,
          0xc04013c000ULL },
    };
    static const u64 expected_size[RP1_CFE_WINDOW_COUNT] = {
        0x100ULL, 0x100ULL, 0x100ULL, 0x1000ULL,
    };
    u32 instance;

    CHECK(rp1_cfe_resource_profile_get(RP1_CFE_INSTANCE_COUNT) == NULL);
    CHECK(rp1_cfe_resource_profile_get(~0U) == NULL);
    for (instance = 0U; instance < RP1_CFE_INSTANCE_COUNT; instance++) {
        const struct rp1_cfe_resource_profile *profile =
            rp1_cfe_resource_profile_get(instance);
        u32 known = ~0U;
        u32 active = ~0U;
        u32 i;

        CHECK(profile != NULL);
        CHECK(profile == rp1_cfe_resource_profile_get(instance));
        CHECK(profile->layout_id == RP1_CFE_LAYOUT_ID_V1);
        CHECK(profile->instance == instance);
        CHECK(memcmp(profile->source_revision, revision, sizeof(revision)) == 0);
        CHECK(profile->clock_id == 22U + instance);
        CHECK(profile->clock_rate_hz == 25000000U);
        CHECK(profile->rp1_source == 47U + instance);
        CHECK(profile->linux_iommu_attachment == 5U);
        CHECK(profile->known_fact_mask == (RP1_CFE_FACT_WINDOWS |
              RP1_CFE_FACT_CLOCK | RP1_CFE_FACT_RP1_SOURCE |
              RP1_CFE_FACT_LINUX_IOMMU_ATTACHMENT));
        CHECK(profile->active_known_mask == 0U);
        CHECK(profile->active_proven_mask == 0U);
        CHECK(profile->access_authorized_mask == 0U);
        CHECK(!rp1_cfe_source_gate(profile, &known, &active));
        CHECK(known == profile->known_fact_mask);
        CHECK(active == 0U);
        for (i = 0U; i < RP1_CFE_WINDOW_COUNT; i++) {
            CHECK(profile->windows[i].name == i);
            CHECK(profile->windows[i]._reserved == 0U);
            CHECK(profile->windows[i].rp1_bus_base == expected_base[instance][i]);
            CHECK(profile->windows[i].bytes == expected_size[i]);
        }
    }
    {
        struct rp1_cfe_resource_profile copied =
            *rp1_cfe_resource_profile_get(0U);
        u32 known = ~0U;
        u32 active = ~0U;

        CHECK(!rp1_cfe_source_gate(&copied, &known, &active));
        CHECK(known == 0U);
        CHECK(active == 0U);
        CHECK(!rp1_cfe_source_gate(NULL, &known, &active));
        CHECK(known == 0U);
        CHECK(active == 0U);
    }
}

static void test_descriptor_validation(void)
{
    struct rp1_cfe_stream_descriptor descriptor = valid_descriptor(0U);
    struct rp1_cfe_sink_span sink =
        valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    u32 i;

    CHECK(prepare_once(&descriptor, &sink, 1U));
    descriptor.layout_version++;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.sensor_connector_profile_id = 0U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.sensor_connector_profile_generation = 0U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.instance = RP1_CFE_INSTANCE_COUNT;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.source_stream_id = 0U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.source_stream_generation = 0U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.width = 1U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.height = 1U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.width = RP1_CFE_MAX_DIMENSION + 1U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.height = RP1_CFE_MAX_DIMENSION + 1U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.stride = 0U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.stride = RP1_CFE_MAX_STRIDE + 1U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.pixel_format = RP1_CFE_FORMAT_INVALID;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.pixel_format = 99U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.lane_count = 0U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.lane_count = 5U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.config_identity = 0U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.config_revision = 0U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.config_length = 0U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.config_length = RP1_CFE_MAX_CONFIG_BYTES + 1U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor.config_flags = 1U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor = valid_descriptor(0U);
    descriptor._reserved[2] = 1U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));

    descriptor = valid_descriptor(0U);
    descriptor.pixel_format = RP1_CFE_FORMAT_RAW10;
    descriptor.width = 3U;
    descriptor.stride = 4U;
    CHECK(prepare_once(&descriptor, &sink, 1U));
    descriptor.stride = 3U;
    CHECK(!prepare_once(&descriptor, &sink, 1U));
    descriptor.width = RP1_CFE_MAX_DIMENSION;
    descriptor.height = RP1_CFE_MAX_DIMENSION;
    descriptor.stride = 10240U;
    CHECK(prepare_once(&descriptor, &sink, 1U));

    for (i = 1U; i <= 4U; i++) {
        descriptor = valid_descriptor(i & 1U);
        descriptor.lane_count = i;
        descriptor.sensor_connector_profile_id = 0x700U + i;
        descriptor.sensor_connector_profile_generation = i;
        descriptor.source_stream_id = 0x800U + i;
        descriptor.source_stream_generation = i + 10U;
        descriptor.config_identity = 0x900U + i;
        descriptor.config_revision = i;
        descriptor.config_length = i * 16U;
        CHECK(prepare_once(&descriptor, &sink, 1U));
    }
}

static void test_sink_validation(void)
{
    struct rp1_cfe_stream_descriptor descriptor = valid_descriptor(0U);
    struct rp1_cfe_sink_span sinks[RP1_CFE_MAX_SINK_SPANS];

    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    CHECK(!prepare_once(&descriptor, NULL, 1U));
    CHECK(!prepare_once(&descriptor, sinks, 0U));
    CHECK(!prepare_once(&descriptor, sinks, RP1_CFE_MAX_SINK_SPANS + 1U));
    CHECK(prepare_once(&descriptor, sinks, 1U));

    sinks[0].allocation_id = 0U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].allocation_generation = 0U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].allocation_base = 0U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].allocation_capacity = 0U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].allocation_offset = sinks[0].allocation_capacity;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].window_capacity = 0U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].used = 0U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].used = sinks[0].window_capacity + 16U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].window_capacity = sinks[0].allocation_capacity + 16U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, ~0ULL - 15U);
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].role = RP1_CFE_SINK_INVALID;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0].role = 4U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].access = 0U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0].access = 2U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].output_canary = 0U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0]._reserved = 1U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0]._reserved = 0U;
    sinks[0]._reserved2 = 1U;
    CHECK(!prepare_once(&descriptor, sinks, 1U));

    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].allocation_base++;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].allocation_capacity++;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].allocation_offset++;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].window_capacity++;
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[0].used++;
    CHECK(!prepare_once(&descriptor, sinks, 1U));

    sinks[0] = valid_sink(RP1_CFE_SINK_STATS, 1U, 0x100000U);
    CHECK(!prepare_once(&descriptor, sinks, 1U));
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[1] = valid_sink(RP1_CFE_SINK_OUTPUT0, 2U, 0x200000U);
    CHECK(!prepare_once(&descriptor, sinks, 2U));
    sinks[1] = valid_sink(RP1_CFE_SINK_OUTPUT1, 2U, 0x100100U);
    CHECK(!prepare_once(&descriptor, sinks, 2U));

    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 7U, 0x300000U);
    sinks[1] = valid_sink(RP1_CFE_SINK_OUTPUT1, 7U, 0x300000U);
    sinks[1].allocation_offset = 0x400U;
    sinks[1].window_capacity = 0x400U;
    sinks[1].used = 0x200U;
    CHECK(prepare_once(&descriptor, sinks, 2U));
    sinks[1].allocation_offset = 0x200U;
    CHECK(!prepare_once(&descriptor, sinks, 2U));
    sinks[1].allocation_offset = 0x400U;
    sinks[1].allocation_capacity = 0x2000U;
    CHECK(!prepare_once(&descriptor, sinks, 2U));
    sinks[1] = valid_sink(RP1_CFE_SINK_OUTPUT1, 8U, 0x300100U);
    CHECK(!prepare_once(&descriptor, sinks, 2U));
    sinks[1] = valid_sink(RP1_CFE_SINK_OUTPUT1, 8U, 0x301000U);
    sinks[2] = valid_sink(RP1_CFE_SINK_STATS, 9U, 0x302000U);
    CHECK(prepare_once(&descriptor, sinks, 3U));
}

static void test_lifecycle_and_private_copy(void)
{
    struct rp1_cfe_contract contract;
    struct rp1_cfe_contract other;
    struct rp1_cfe_stream_descriptor descriptor = valid_descriptor(0U);
    struct rp1_cfe_sink_span sinks[2];
    struct rp1_cfe_stream_handle handle, instance_one, stale, forged;
    struct rp1_cfe_stream_snapshot snapshot;
    struct rp1_cfe_stream_status status;
    enum rp1_cfe_activation_reason reason;
    u32 missing;

    memset(&contract, 0, sizeof(contract));
    CHECK(!rp1_cfe_contract_init(&contract, 0U));
    contract.owner._reserved[0] = 1U;
    CHECK(!rp1_cfe_contract_init(&contract, 0x99U));
    init_contract(&contract, 0x99U);
    CHECK(!rp1_cfe_contract_init(&contract, 0x99U));

    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    sinks[1] = valid_sink(RP1_CFE_SINK_STATS, 2U, 0x200000U);
    handle.token = ~0ULL;
    handle.generation = ~0ULL;
    handle.controller_id = ~0U;
    handle._reserved = ~0U;
    CHECK(rp1_cfe_contract_prepare(&contract, &descriptor, sinks, 2U, &handle));
    CHECK(handle.token != 0U && handle.generation != 0U);
    CHECK(handle.controller_id == 0x99U && handle._reserved == 0U);
    stale = handle;
    CHECK(rp1_cfe_contract_status_get(&contract, &handle, &status));
    CHECK(status.state == RP1_CFE_STREAM_PREPARED);
    CHECK(status.instance == 0U && status.prepare_count == 1U);
    CHECK(rp1_cfe_contract_snapshot_get(&contract, &handle, &snapshot));
    CHECK(snapshot.sink_count == 2U);
    CHECK(snapshot.descriptor.width == 640U);
    CHECK(snapshot.sinks[0].output_canary == sinks[0].output_canary);
    descriptor.width = 2U;
    sinks[0].output_canary = 1U;
    sinks[1].allocation_base = 0x300000U;
    CHECK(rp1_cfe_contract_snapshot_get(&contract, &handle, &snapshot));
    CHECK(snapshot.descriptor.width == 640U);
    CHECK(snapshot.sinks[0].output_canary != sinks[0].output_canary);
    CHECK(snapshot.sinks[1].allocation_base != sinks[1].allocation_base);

    missing = 0U;
    reason = RP1_CFE_ACTIVATION_REASON_NONE;
    CHECK(!rp1_cfe_activation_permitted(&contract, &handle, &missing, &reason));
    CHECK(missing == RP1_CFE_V1_MISSING_PROOFS);
    CHECK(reason == RP1_CFE_ACTIVATION_REASON_HARDWARE_DISABLED_V1);
    CHECK(rp1_cfe_contract_status_get(&contract, &handle, &status));
    CHECK(status.state == RP1_CFE_STREAM_PREPARED);
    CHECK(!rp1_cfe_contract_release(&contract, &handle));
    CHECK(rp1_cfe_contract_report_failure(&contract, &handle));
    CHECK(!rp1_cfe_contract_report_failure(&contract, &handle));
    descriptor = valid_descriptor(0U);
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 3U, 0x400000U);
    CHECK(!rp1_cfe_contract_prepare(&contract, &descriptor, sinks, 1U,
                                    &forged));
    CHECK(rp1_cfe_contract_status_get(&contract, &handle, &status));
    CHECK(status.state == RP1_CFE_STREAM_FAILED && status.failure_count == 1U);
    CHECK(rp1_cfe_contract_release(&contract, &handle));
    CHECK(!rp1_cfe_contract_status_get(&contract, &stale, &status));
    CHECK(!rp1_cfe_contract_snapshot_get(&contract, &stale, &snapshot));
    CHECK(!rp1_cfe_contract_report_failure(&contract, &stale));
    CHECK(!rp1_cfe_contract_release(&contract, &stale));

    descriptor = valid_descriptor(0U);
    sinks[0] = valid_sink(RP1_CFE_SINK_OUTPUT0, 3U, 0x400000U);
    CHECK(rp1_cfe_contract_prepare(&contract, &descriptor, sinks, 1U, &handle));
    CHECK(handle.generation != stale.generation);
    forged = handle;
    forged.token ^= 2U;
    CHECK(!rp1_cfe_contract_report_failure(&contract, &forged));
    forged = handle;
    forged.generation++;
    CHECK(!rp1_cfe_contract_report_failure(&contract, &forged));
    forged = handle;
    forged.controller_id++;
    CHECK(!rp1_cfe_contract_report_failure(&contract, &forged));
    forged = handle;
    forged._reserved = 1U;
    CHECK(!rp1_cfe_contract_report_failure(&contract, &forged));
    descriptor.instance = 1U;
    descriptor.sensor_connector_profile_id++;
    descriptor.source_stream_id++;
    CHECK(rp1_cfe_contract_prepare(&contract, &descriptor, sinks, 1U,
                                   &instance_one));
    CHECK(!rp1_cfe_contract_prepare(&contract, &descriptor, sinks, 1U,
                                    &forged));
    memset(&other, 0, sizeof(other));
    CHECK(rp1_cfe_contract_init(&other, 0x9aU));
    CHECK(!rp1_cfe_contract_report_failure(&other, &handle));
    missing = ~0U;
    reason = RP1_CFE_ACTIVATION_REASON_NONE;
    CHECK(!rp1_cfe_activation_permitted(&other, &handle, &missing, &reason));
    CHECK(missing == 0U);
    CHECK(reason == RP1_CFE_ACTIVATION_REASON_INVALID_HANDLE);
    CHECK(rp1_cfe_contract_report_failure(&contract, &handle));
    CHECK(rp1_cfe_contract_release(&contract, &handle));
    CHECK(rp1_cfe_contract_report_failure(&contract, &instance_one));
    CHECK(rp1_cfe_contract_release(&contract, &instance_one));
}

static void test_saturation_identities_and_exhaustion(void)
{
    struct rp1_cfe_contract contract;
    struct rp1_cfe_stream_descriptor descriptor;
    struct rp1_cfe_sink_span sink;
    struct rp1_cfe_stream_handle handles[RP1_CFE_STREAM_CAPACITY];
    struct rp1_cfe_stream_handle exhausted, next;
    u32 i;

    init_contract(&contract, 0x777U);
    sink = valid_sink(RP1_CFE_SINK_OUTPUT0, 1U, 0x100000U);
    for (i = 0U; i < RP1_CFE_STREAM_CAPACITY; i++) {
        descriptor = valid_descriptor(i);
        descriptor.sensor_connector_profile_id = 0x500U + i;
        descriptor.source_stream_id = 0x600U + i;
        CHECK(rp1_cfe_contract_prepare(&contract, &descriptor, &sink, 1U,
                                       &handles[i]));
    }
    descriptor = valid_descriptor(0U);
    CHECK(!rp1_cfe_contract_prepare(&contract, &descriptor, &sink, 1U, &next));
    for (i = 0U; i < RP1_CFE_STREAM_CAPACITY; i++) {
        CHECK(rp1_cfe_contract_report_failure(&contract, &handles[i]));
        CHECK(rp1_cfe_contract_release(&contract, &handles[i]));
    }

    init_contract(&contract, 0x778U);
    descriptor = valid_descriptor(0U);
    CHECK(rp1_cfe_contract_prepare(&contract, &descriptor, &sink, 1U, &exhausted));
    i = (u32)(exhausted.token & 0xffU);
    contract.controls[i].generation = ~0ULL;
    exhausted.generation = ~0ULL;
    CHECK(rp1_cfe_contract_report_failure(&contract, &exhausted));
    CHECK(rp1_cfe_contract_release(&contract, &exhausted));
    CHECK(contract.controls[i].generation == 0U);
    CHECK(contract.controls[i].state == RP1_CFE_STREAM_RELEASED);
    CHECK(!rp1_cfe_contract_report_failure(&contract, &exhausted));
    CHECK(!rp1_cfe_contract_release(&contract, &exhausted));
    CHECK(rp1_cfe_contract_prepare(&contract, &descriptor, &sink, 1U, &next));
    CHECK((u32)(next.token & 0xffU) != i);
    CHECK(rp1_cfe_contract_report_failure(&contract, &next));
    CHECK(rp1_cfe_contract_release(&contract, &next));
}

static void test_identity_matrix(void)
{
    u32 i;

    /*
     * This exercises independent nonzero profile/source generations and the
     * full prepare/fail/release cleanup path repeatedly without hardware.
     */
    for (i = 1U; i <= 64U; i++) {
        struct rp1_cfe_contract contract;
        struct rp1_cfe_stream_descriptor descriptor =
            valid_descriptor(i & 1U);
        struct rp1_cfe_sink_span sink =
            valid_sink(RP1_CFE_SINK_OUTPUT0, i, 0x100000U + (u64)i * 0x1000U);
        struct rp1_cfe_stream_handle handle;
        struct rp1_cfe_stream_status status;

        descriptor.sensor_connector_profile_id = 0x1000U + i;
        descriptor.sensor_connector_profile_generation = (u64)i;
        descriptor.source_stream_id = 0x2000U + i;
        descriptor.source_stream_generation = (u64)i + 100U;
        descriptor.config_identity = 0x3000U + i;
        descriptor.config_revision = i;
        descriptor.config_length = 16U * i;
        init_contract(&contract, 0x9000U + i);
        CHECK(rp1_cfe_contract_prepare(&contract, &descriptor, &sink, 1U,
                                       &handle));
        CHECK(rp1_cfe_contract_status_get(&contract, &handle, &status));
        CHECK(status.controller_id == 0x9000U + i);
        CHECK(rp1_cfe_contract_report_failure(&contract, &handle));
        CHECK(rp1_cfe_contract_release(&contract, &handle));
    }
}

int main(void)
{
    test_resource_profiles();
    test_descriptor_validation();
    test_sink_validation();
    test_lifecycle_and_private_copy();
    test_saturation_identities_and_exhaustion();
    test_identity_matrix();

    if (failures) {
        printf("rp1 cfe contract: %d/%d checks failed\n", failures, checks);
        return 1;
    }
    printf("rp1 cfe contract: %d checks passed\n", checks);
    return 0;
}
