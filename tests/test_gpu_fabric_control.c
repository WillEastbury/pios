#include <stdio.h>
#include <string.h>

#include "types.h"
#include "gpu_fabric_control.h"

static int failures;
static int checks;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while (0)

static struct gpu_fabric_node_facts facts(u32 id, u64 kernels, u64 capacity)
{
    struct gpu_fabric_node_facts result;

    memset(&result, 0, sizeof(result));
    result.node_id = id;
    result.epoch = 1000U + id;
    result.supported_kernel_mask = kernels;
    result.verified_vram_bytes = capacity;
    result.resident_capacity_bytes = capacity;
    result.resident_used_bytes = 0U;
    result.health = GPU_FABRIC_HEALTH_OK;
    result.load_per_mille = id % 1000U;
    return result;
}

static bool add_node(struct gpu_fabric *fabric, u32 id, u64 kernels,
                     u64 capacity, bool verified,
                     struct gpu_fabric_node_handle *handle)
{
    struct gpu_fabric_node_facts node_facts = facts(id, kernels, capacity);

    return gpu_fabric_node_advertise(fabric, &node_facts, verified, 0U,
                                     handle);
}

static struct gpu_fabric_placement_request placement_request(
    u64 model, u64 generation, u32 count)
{
    struct gpu_fabric_placement_request request;
    u32 i;

    memset(&request, 0, sizeof(request));
    request.model_id = model;
    request.model_generation = generation;
    request.shard_count = count;
    for (i = 0U; i < count; i++) {
        request.shards[i].shard_id = i + 1U;
        request.shards[i].bytes = 64U;
        request.shards[i].required_kernel_mask = 1U;
    }
    return request;
}

static struct gpu_fabric_activation_request activation_request(
    const struct gpu_fabric_placement_handle *placement,
    u32 source_node, u32 source_generation, u32 target_node,
    u32 target_generation, u64 sequence, u64 bytes, u64 credit)
{
    struct gpu_fabric_activation_request request;

    memset(&request, 0, sizeof(request));
    request.placement = *placement;
    request.source_shard_id = 1U;
    request.target_shard_id = 2U;
    request.source_node_id = source_node;
    request.source_node_generation = source_generation;
    request.target_node_id = target_node;
    request.target_node_generation = target_generation;
    request.sequence = sequence;
    request.span_start = 0x1000U + sequence * 64U;
    request.span_bytes = bytes;
    request.credit_capacity_bytes = credit;
    return request;
}

static void test_layout_and_init(void)
{
    struct gpu_fabric fabric;

    CHECK(GPU_FABRIC_MAX_NODES == 8U);
    CHECK(GPU_FABRIC_MAX_PLACEMENTS == 8U);
    CHECK(GPU_FABRIC_MAX_SHARDS == 16U);
    CHECK(GPU_FABRIC_MAX_ACTIVATIONS == 32U);
    CHECK(sizeof(struct gpu_fabric_node_record) == 64U);
    CHECK(sizeof(struct gpu_fabric_placement_record) == 64U);
    CHECK(sizeof(struct gpu_fabric_shard_record) == 64U);
    CHECK(sizeof(struct gpu_fabric_activation_control) == 64U);
    CHECK(sizeof(struct gpu_fabric_activation_route) == 64U);
    CHECK(sizeof(struct gpu_fabric_activation_span) == 64U);
    CHECK(sizeof(struct gpu_fabric_flow_record) == 64U);
    CHECK(!gpu_fabric_init(NULL, 1U, 0U));
    memset(&fabric, 0, sizeof(fabric));
    CHECK(!gpu_fabric_init(&fabric, 0U, 0U));
    CHECK(!gpu_fabric_init(&fabric, 1U, 1U));
    CHECK(gpu_fabric_init(&fabric, 1U, 0U));
    CHECK(!gpu_fabric_init(&fabric, 2U, 0U));
    CHECK(!gpu_fabric_hardware_enable_allowed(&fabric, 0U));
    CHECK(!gpu_fabric_hardware_enable_allowed(&fabric, 1U));
}

static void test_one_through_eight_nodes(void)
{
    u32 count;

    for (count = 1U; count <= GPU_FABRIC_MAX_NODES; count++) {
        struct gpu_fabric fabric;
        struct gpu_fabric_node_handle handles[GPU_FABRIC_MAX_NODES];
        struct gpu_fabric_placement_request request =
            placement_request(500U + count, 1U, 1U);
        struct gpu_fabric_placement_handle placement;
        u32 i;
        u32 node_id;
        u32 generation;

        memset(&fabric, 0, sizeof(fabric));
        CHECK(gpu_fabric_init(&fabric, count + 10U, 0U));
        for (i = 0U; i < count; i++) {
            u32 id = 100U + count - i;
            struct gpu_fabric_node_facts got;

            CHECK(add_node(&fabric, id, 0xFU, 1024U, true, &handles[i]));
            CHECK(handles[i].node_id == id);
            CHECK(handles[i].generation == 1U);
            CHECK(handles[i].epoch == 1000U + id);
            CHECK(gpu_fabric_node_get(&fabric, id, &got, &generation));
            CHECK(generation == handles[i].generation);
            CHECK(got.node_id == id);
            CHECK(got.epoch == handles[i].epoch);
            CHECK(got.supported_kernel_mask == 0xFU);
            CHECK(got.verified_vram_bytes == 1024U);
            CHECK(got.resident_capacity_bytes == 1024U);
            CHECK(got.resident_used_bytes == 0U);
            CHECK(got.health == GPU_FABRIC_HEALTH_OK);
        }
        CHECK(gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
        CHECK(gpu_fabric_shard_assignment_get(&fabric, &placement, 1U,
                                              &node_id, &generation));
        CHECK(node_id == 101U);
        CHECK(generation == 1U);
        CHECK(gpu_fabric_placement_release(&fabric, &placement, 0U));
        CHECK(!gpu_fabric_shard_assignment_get(&fabric, &placement, 1U,
                                               &node_id, &generation));
    }
}

static void test_placement_eligibility_and_atomicity(void)
{
    struct gpu_fabric fabric;
    struct gpu_fabric_node_handle node1;
    struct gpu_fabric_node_handle node2;
    struct gpu_fabric_node_handle node3;
    struct gpu_fabric_placement_request request;
    struct gpu_fabric_placement_handle placement;
    struct gpu_fabric_node_facts before;
    struct gpu_fabric_node_facts after;
    u32 assigned;
    u32 generation;

    memset(&fabric, 0, sizeof(fabric));
    CHECK(gpu_fabric_init(&fabric, 30U, 0U));
    CHECK(add_node(&fabric, 10U, 1U, 128U, false, &node1));
    request = placement_request(1U, 1U, 1U);
    CHECK(!gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
    CHECK(add_node(&fabric, 20U, 2U, 512U, true, &node2));
    CHECK(add_node(&fabric, 30U, 3U, 512U, true, &node3));
    request = placement_request(2U, 1U, 2U);
    request.shards[0].required_kernel_mask = 1U;
    request.shards[1].required_kernel_mask = 2U;
    CHECK(gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
    CHECK(gpu_fabric_shard_assignment_get(&fabric, &placement, 1U,
                                          &assigned, &generation));
    CHECK(assigned == 30U && generation == node3.generation);
    CHECK(gpu_fabric_shard_assignment_get(&fabric, &placement, 2U,
                                          &assigned, &generation));
    CHECK(assigned == 20U && generation == node2.generation);
    CHECK(gpu_fabric_placement_release(&fabric, &placement, 0U));

    CHECK(gpu_fabric_node_get(&fabric, 30U, &before, &generation));
    request = placement_request(3U, 1U, 2U);
    request.shards[0].bytes = 100U;
    request.shards[1].bytes = 1000U;
    CHECK(!gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
    CHECK(gpu_fabric_node_get(&fabric, 30U, &after, &generation));
    CHECK(before.resident_used_bytes == after.resident_used_bytes);

    request = placement_request(4U, 1U, 1U);
    request.shards[0].affinity_node_id = 30U;
    CHECK(gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
    CHECK(gpu_fabric_shard_assignment_get(&fabric, &placement, 1U,
                                          &assigned, &generation));
    CHECK(assigned == 30U);
    CHECK(gpu_fabric_placement_release(&fabric, &placement, 0U));
    request.model_id = 5U;
    request.shards[0].affinity_node_id = 999U;
    CHECK(!gpu_fabric_placement_create(&fabric, &request, 0U, &placement));

    request = placement_request(6U, 1U, 2U);
    request.shards[1].shard_id = request.shards[0].shard_id;
    CHECK(!gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
    request.shards[1].shard_id = 2U;
    request.shards[1].required_kernel_mask = 4U;
    CHECK(!gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
    CHECK(!gpu_fabric_placement_create(&fabric, &request, 1U, &placement));
    CHECK(!gpu_fabric_node_health_report(&fabric, &node1,
                                         GPU_FABRIC_HEALTH_OK, 0U, 1U));
    request = placement_request(7U, 1U, 1U);
    request._reserved = 1U;
    CHECK(!gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
    request._reserved = 0U;
    request.shards[0]._reserved = 1U;
    CHECK(!gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
}

static void test_activation_backpressure_and_sequences(void)
{
    struct gpu_fabric fabric;
    struct gpu_fabric_node_handle node1;
    struct gpu_fabric_node_handle node2;
    struct gpu_fabric_placement_request request = placement_request(50U, 7U, 2U);
    struct gpu_fabric_placement_handle placement;
    struct gpu_fabric_activation_request activation;
    struct gpu_fabric_activation_handle handle;
    struct gpu_fabric_activation_descriptor descriptor;
    enum gpu_fabric_placement_state placement_state;

    memset(&fabric, 0, sizeof(fabric));
    CHECK(gpu_fabric_init(&fabric, 50U, 0U));
    CHECK(add_node(&fabric, 1U, 1U, 2048U, true, &node1));
    CHECK(add_node(&fabric, 2U, 1U, 2048U, true, &node2));
    request.shards[0].affinity_node_id = 1U;
    request.shards[1].affinity_node_id = 2U;
    CHECK(gpu_fabric_placement_create(&fabric, &request, 0U, &placement));

    activation = activation_request(&placement, 1U, node1.generation,
                                    2U, node2.generation, 1U, 128U, 64U);
    CHECK(gpu_fabric_activation_publish(&fabric, &activation, 0U, &handle));
    CHECK(gpu_fabric_activation_get(&fabric, &handle, &descriptor));
    CHECK(descriptor.state == GPU_FABRIC_ACTIVATION_BACKPRESSURED);
    CHECK(descriptor.span_start == activation.span_start);
    CHECK(descriptor.span_bytes == 128U);
    CHECK(descriptor.credit_capacity_bytes == 64U);
    CHECK(!gpu_fabric_activation_dispatch(&fabric, &handle, 0U, &descriptor));
    CHECK(gpu_fabric_activation_retry(&fabric, &handle, 96U, 0U));
    CHECK(gpu_fabric_activation_get(&fabric, &handle, &descriptor));
    CHECK(descriptor.state == GPU_FABRIC_ACTIVATION_BACKPRESSURED);
    CHECK(descriptor.credit_capacity_bytes == 64U);
    CHECK(gpu_fabric_activation_retry(&fabric, &handle, 128U, 0U));
    CHECK(gpu_fabric_activation_dispatch(&fabric, &handle, 0U, &descriptor));
    CHECK(descriptor.state == GPU_FABRIC_ACTIVATION_READY);
    CHECK(gpu_fabric_activation_release(&fabric, &handle, 0U));
    CHECK(!gpu_fabric_activation_get(&fabric, &handle, &descriptor));

    activation.sequence = 2U;
    activation.credit_capacity_bytes = 128U;
    CHECK(gpu_fabric_activation_publish(&fabric, &activation, 0U, &handle));
    CHECK(gpu_fabric_activation_dispatch(&fabric, &handle, 0U, &descriptor));
    CHECK(gpu_fabric_activation_release(&fabric, &handle, 0U));
    activation.sequence = 2U;
    CHECK(!gpu_fabric_activation_publish(&fabric, &activation, 0U, &handle));
    CHECK(gpu_fabric_placement_state_get(&fabric, &placement, &placement_state));
    CHECK(placement_state == GPU_FABRIC_PLACEMENT_QUARANTINED);
}

static void test_stale_removal_and_exhaustion(void)
{
    struct gpu_fabric fabric;
    struct gpu_fabric_node_handle one;
    struct gpu_fabric_node_handle two;
    struct gpu_fabric_placement_request request = placement_request(80U, 1U, 2U);
    struct gpu_fabric_placement_handle placement;
    struct gpu_fabric_placement_handle old_placement;
    struct gpu_fabric_activation_request activation;
    struct gpu_fabric_activation_handle handle;
    struct gpu_fabric_activation_descriptor descriptor;
    enum gpu_fabric_placement_state state;
    u32 i;
    struct gpu_fabric_node_facts rejoin_facts;

    memset(&fabric, 0, sizeof(fabric));
    CHECK(gpu_fabric_init(&fabric, 80U, 0U));
    CHECK(add_node(&fabric, 1U, 1U, 8192U, true, &one));
    CHECK(add_node(&fabric, 2U, 1U, 8192U, true, &two));
    request.shards[0].affinity_node_id = 1U;
    request.shards[1].affinity_node_id = 2U;
    CHECK(gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
    activation = activation_request(&placement, 1U, one.generation, 2U,
                                    two.generation, 1U, 64U, 64U);
    CHECK(gpu_fabric_activation_publish(&fabric, &activation, 0U, &handle));
    CHECK(gpu_fabric_node_remove(&fabric, &two, 0U));
    old_placement = placement;
    CHECK(gpu_fabric_placement_state_get(&fabric, &placement, &state));
    CHECK(state == GPU_FABRIC_PLACEMENT_INVALID);
    CHECK(!gpu_fabric_activation_get(&fabric, &handle, &descriptor));
    CHECK(!gpu_fabric_node_remove(&fabric, &two, 0U));
    CHECK(!gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
    rejoin_facts = facts(2U, 1U, 8192U);
    CHECK(!gpu_fabric_node_advertise(&fabric, &rejoin_facts, true, 0U, &two));
    rejoin_facts.epoch++;
    CHECK(gpu_fabric_node_advertise(&fabric, &rejoin_facts, true, 0U, &two));
    CHECK(two.generation == 3U);
    CHECK(gpu_fabric_placement_release(&fabric, &old_placement, 0U));
    CHECK(gpu_fabric_placement_create(&fabric, &request, 0U, &placement));

    memset(&fabric, 0, sizeof(fabric));
    CHECK(gpu_fabric_init(&fabric, 81U, 0U));
    CHECK(add_node(&fabric, 1U, 1U, 65536U, true, &one));
    for (i = 0U; i < GPU_FABRIC_MAX_PLACEMENTS; i++) {
        request = placement_request(100U + i, 1U, 1U);
        CHECK(gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
    }
    request = placement_request(999U, 1U, 1U);
    CHECK(!gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
}

static void test_activation_exhaustion(void)
{
    struct gpu_fabric fabric;
    struct gpu_fabric_node_handle one;
    struct gpu_fabric_node_handle two;
    struct gpu_fabric_placement_request request = placement_request(200U, 1U, 2U);
    struct gpu_fabric_placement_handle placement;
    struct gpu_fabric_activation_request activation;
    struct gpu_fabric_activation_handle handles[GPU_FABRIC_MAX_ACTIVATIONS];
    u32 i;

    memset(&fabric, 0, sizeof(fabric));
    CHECK(gpu_fabric_init(&fabric, 200U, 0U));
    CHECK(add_node(&fabric, 1U, 1U, 8192U, true, &one));
    CHECK(add_node(&fabric, 2U, 1U, 8192U, true, &two));
    request.shards[0].affinity_node_id = 1U;
    request.shards[1].affinity_node_id = 2U;
    CHECK(gpu_fabric_placement_create(&fabric, &request, 0U, &placement));
    activation = activation_request(&placement, 1U, one.generation, 2U,
                                    two.generation, 1U, 1U, 1U);
    for (i = 0U; i < GPU_FABRIC_MAX_ACTIVATIONS; i++) {
        activation.sequence = (u64)i + 1U;
        CHECK(gpu_fabric_activation_publish(&fabric, &activation, 0U,
                                            &handles[i]));
        CHECK(handles[i].sequence == activation.sequence);
        CHECK(handles[i].generation != 0U);
    }
    activation.sequence++;
    CHECK(!gpu_fabric_activation_publish(&fabric, &activation, 0U, &handles[0]));
}

int main(void)
{
    test_layout_and_init();
    test_one_through_eight_nodes();
    test_placement_eligibility_and_atomicity();
    test_activation_backpressure_and_sequences();
    test_stale_removal_and_exhaustion();
    test_activation_exhaustion();

    printf("gpu fabric control: %d checks, %d failure(s)\n", checks, failures);
    return failures ? 1 : 0;
}
