/*
 * gpu_fabric_control.h - ADR-070 offline GPU-fabric control-plane contract.
 *
 * This is Core-0-owned, bounded control state only. It contains no packet,
 * model, payload, GPU, PCIe, or runtime integration. Public input/output
 * pointers are caller-local copies; no pointer is stored in or transferred by
 * this contract.
 */
#pragma once

#include "types.h"

#define GPU_FABRIC_OWNER_CORE             0U
#define GPU_FABRIC_MAX_NODES              8U
#define GPU_FABRIC_MAX_PLACEMENTS         8U
#define GPU_FABRIC_MAX_SHARDS             16U
#define GPU_FABRIC_MAX_ACTIVATIONS        32U
#define GPU_FABRIC_MAX_FLOWS              32U

enum gpu_fabric_health {
    GPU_FABRIC_HEALTH_UNKNOWN = 0U,
    GPU_FABRIC_HEALTH_OK,
    GPU_FABRIC_HEALTH_DEGRADED,
    GPU_FABRIC_HEALTH_FAILED,
};

enum gpu_fabric_node_state {
    GPU_FABRIC_NODE_EMPTY = 0U,
    GPU_FABRIC_NODE_UNVERIFIED,
    GPU_FABRIC_NODE_ELIGIBLE,
    GPU_FABRIC_NODE_QUARANTINED,
    GPU_FABRIC_NODE_REMOVED,
};

enum gpu_fabric_placement_state {
    GPU_FABRIC_PLACEMENT_EMPTY = 0U,
    GPU_FABRIC_PLACEMENT_ACTIVE,
    GPU_FABRIC_PLACEMENT_INVALID,
    GPU_FABRIC_PLACEMENT_QUARANTINED,
    GPU_FABRIC_PLACEMENT_RELEASED,
    GPU_FABRIC_PLACEMENT_RETIRED,
};

enum gpu_fabric_activation_state {
    GPU_FABRIC_ACTIVATION_EMPTY = 0U,
    GPU_FABRIC_ACTIVATION_BACKPRESSURED,
    GPU_FABRIC_ACTIVATION_READY,
    GPU_FABRIC_ACTIVATION_DISPATCHED,
    GPU_FABRIC_ACTIVATION_INVALID,
    GPU_FABRIC_ACTIVATION_QUARANTINED,
    GPU_FABRIC_ACTIVATION_RELEASED,
    GPU_FABRIC_ACTIVATION_RETIRED,
};

/* Immutable, caller-supplied node facts. `gpu_verified` is supplied separately
 * so a discovered fact can be recorded without making the node eligible. */
struct gpu_fabric_node_facts {
    u32 node_id;
    u32 _reserved;
    u64 epoch;
    u64 supported_kernel_mask;
    u64 verified_vram_bytes;
    u64 resident_capacity_bytes;
    u64 resident_used_bytes;
    u32 health;
    u32 load_per_mille;
};

struct gpu_fabric_shard_request {
    u64 shard_id;
    u64 bytes;
    u64 required_kernel_mask;
    u32 affinity_node_id; /* zero means no fixed-node affinity */
    u32 _reserved;
};

struct gpu_fabric_placement_request {
    u64 model_id;
    u64 model_generation;
    u32 shard_count;
    u32 _reserved;
    struct gpu_fabric_shard_request shards[GPU_FABRIC_MAX_SHARDS];
};

/* Opaque, generation-backed numeric capabilities. */
struct gpu_fabric_node_handle {
    u64 epoch;
    u32 controller_id;
    u32 node_id;
    u32 generation;
    u32 _reserved;
};

struct gpu_fabric_placement_handle {
    u64 model_id;
    u64 model_generation;
    u32 controller_id;
    u32 slot;
    u32 generation;
    u32 _reserved;
};

struct gpu_fabric_activation_handle {
    u64 sequence;
    u32 controller_id;
    u32 slot;
    u32 generation;
    u32 _reserved;
};

/*
 * Numeric activation authority only. `span_start` is an opaque numeric offset,
 * never a virtual, physical, DMA, or payload pointer. Capacity is target credit
 * observed by the control plane for this specific publication.
 */
struct gpu_fabric_activation_request {
    struct gpu_fabric_placement_handle placement;
    u64 source_shard_id;
    u64 target_shard_id;
    u32 source_node_id;
    u32 source_node_generation;
    u32 target_node_id;
    u32 target_node_generation;
    u64 sequence;
    u64 span_start;
    u64 span_bytes;
    u64 credit_capacity_bytes;
};

struct gpu_fabric_activation_descriptor {
    u64 source_shard_id;
    u64 target_shard_id;
    u64 sequence;
    u64 span_start;
    u64 span_bytes;
    u64 credit_capacity_bytes;
    u32 source_node_id;
    u32 source_node_generation;
    u32 target_node_id;
    u32 target_node_generation;
    u32 state;
    u32 _reserved;
};

/* One Core-0-owned mutable node record per cache line. */
struct gpu_fabric_node_record {
    u64 epoch;
    u64 supported_kernel_mask;
    u64 verified_vram_bytes;
    u64 resident_capacity_bytes;
    u64 resident_used_bytes;
    u64 reserved_bytes;
    u32 node_id;
    u32 generation;
    u8 state;
    u8 health;
    u16 load_per_mille;
    u8 gpu_verified;
    u8 _pad[3U];
} ALIGNED(64);

struct gpu_fabric_owner {
    u32 controller_id;
    u32 initialized;
    u32 hardware_enable;
    u32 transition_active;
    u8 _pad[48U];
} ALIGNED(64);

struct gpu_fabric_placement_record {
    u64 model_id;
    u64 model_generation;
    u32 generation;
    u32 state;
    u32 shard_count;
    u32 _reserved;
    u8 _pad[32U];
} ALIGNED(64);

struct gpu_fabric_shard_record {
    u64 shard_id;
    u64 bytes;
    u64 required_kernel_mask;
    u32 affinity_node_id;
    u32 node_id;
    u32 node_generation;
    u32 _reserved;
    u8 _pad[24U];
} ALIGNED(64);

/* Activation state is intentionally separate from immutable route and span. */
struct gpu_fabric_activation_control {
    u32 generation;
    u32 state;
    u32 placement_slot;
    u32 placement_generation;
    u8 _pad[48U];
} ALIGNED(64);

struct gpu_fabric_activation_route {
    u64 source_shard_id;
    u64 target_shard_id;
    u64 sequence;
    u32 source_node_id;
    u32 source_node_generation;
    u32 target_node_id;
    u32 target_node_generation;
    u32 placement_slot;
    u32 placement_generation;
    u8 _pad[16U];
} ALIGNED(64);

struct gpu_fabric_activation_span {
    u64 span_start;
    u64 span_bytes;
    u64 credit_capacity_bytes;
    u8 _pad[40U];
} ALIGNED(64);

/* Per-placement sequence history survives activation release. */
struct gpu_fabric_flow_record {
    u64 source_shard_id;
    u64 target_shard_id;
    u64 last_sequence;
    u32 generation;
    u32 in_use;
    u8 _pad[32U];
} ALIGNED(64);

struct gpu_fabric {
    struct gpu_fabric_owner owner;
    struct gpu_fabric_node_record nodes[GPU_FABRIC_MAX_NODES];
    struct gpu_fabric_placement_record placements[GPU_FABRIC_MAX_PLACEMENTS];
    struct gpu_fabric_shard_record
        shards[GPU_FABRIC_MAX_PLACEMENTS][GPU_FABRIC_MAX_SHARDS];
    struct gpu_fabric_activation_control
        activations[GPU_FABRIC_MAX_ACTIVATIONS];
    struct gpu_fabric_activation_route
        routes[GPU_FABRIC_MAX_ACTIVATIONS];
    struct gpu_fabric_activation_span
        spans[GPU_FABRIC_MAX_ACTIVATIONS];
    struct gpu_fabric_flow_record
        flows[GPU_FABRIC_MAX_PLACEMENTS][GPU_FABRIC_MAX_FLOWS];
} ALIGNED(64);

_Static_assert(sizeof(struct gpu_fabric_owner) == 64U,
               "fabric owner must own one cache line");
_Static_assert(sizeof(struct gpu_fabric_node_record) == 64U,
               "fabric nodes must have cache-line stride");
_Static_assert(sizeof(struct gpu_fabric_placement_record) == 64U,
               "fabric placements must have cache-line stride");
_Static_assert(sizeof(struct gpu_fabric_shard_record) == 64U,
               "fabric shards must have cache-line stride");
_Static_assert(sizeof(struct gpu_fabric_activation_control) == 64U,
               "fabric activation controls must own one cache line");
_Static_assert(sizeof(struct gpu_fabric_activation_route) == 64U,
               "fabric activation routes must own one cache line");
_Static_assert(sizeof(struct gpu_fabric_activation_span) == 64U,
               "fabric activation spans must own one cache line");
_Static_assert(sizeof(struct gpu_fabric_flow_record) == 64U,
               "fabric sequence records must own one cache line");
_Static_assert((sizeof(struct gpu_fabric) % 64U) == 0U,
               "fabric state must retain cache-line stride");

bool gpu_fabric_init(struct gpu_fabric *fabric, u32 controller_id,
                     u32 caller_core);
bool gpu_fabric_node_advertise(struct gpu_fabric *fabric,
                               const struct gpu_fabric_node_facts *facts,
                               bool gpu_verified, u32 caller_core,
                               struct gpu_fabric_node_handle *node_out);
bool gpu_fabric_node_health_report(struct gpu_fabric *fabric,
                                   const struct gpu_fabric_node_handle *node,
                                   enum gpu_fabric_health health,
                                   u32 load_per_mille, u32 caller_core);
bool gpu_fabric_node_remove(struct gpu_fabric *fabric,
                            const struct gpu_fabric_node_handle *node,
                            u32 caller_core);
bool gpu_fabric_node_get(const struct gpu_fabric *fabric, u32 node_id,
                         struct gpu_fabric_node_facts *facts_out,
                         u32 *generation_out);

bool gpu_fabric_placement_create(
    struct gpu_fabric *fabric,
    const struct gpu_fabric_placement_request *request, u32 caller_core,
    struct gpu_fabric_placement_handle *placement_out);
bool gpu_fabric_placement_release(
    struct gpu_fabric *fabric,
    const struct gpu_fabric_placement_handle *placement, u32 caller_core);
bool gpu_fabric_placement_state_get(
    const struct gpu_fabric *fabric,
    const struct gpu_fabric_placement_handle *placement,
    enum gpu_fabric_placement_state *state_out);
bool gpu_fabric_shard_assignment_get(
    const struct gpu_fabric *fabric,
    const struct gpu_fabric_placement_handle *placement, u64 shard_id,
    u32 *node_id_out, u32 *node_generation_out);

bool gpu_fabric_activation_publish(
    struct gpu_fabric *fabric,
    const struct gpu_fabric_activation_request *request, u32 caller_core,
    struct gpu_fabric_activation_handle *activation_out);
bool gpu_fabric_activation_retry(
    struct gpu_fabric *fabric,
    const struct gpu_fabric_activation_handle *activation,
    u64 credit_capacity_bytes, u32 caller_core);
bool gpu_fabric_activation_dispatch(
    struct gpu_fabric *fabric,
    const struct gpu_fabric_activation_handle *activation, u32 caller_core,
    struct gpu_fabric_activation_descriptor *descriptor_out);
bool gpu_fabric_activation_release(
    struct gpu_fabric *fabric,
    const struct gpu_fabric_activation_handle *activation, u32 caller_core);
bool gpu_fabric_activation_get(
    const struct gpu_fabric *fabric,
    const struct gpu_fabric_activation_handle *activation,
    struct gpu_fabric_activation_descriptor *descriptor_out);

/* Permanently false: future execution needs a separate, owner-approved ADR. */
bool gpu_fabric_hardware_enable_allowed(const struct gpu_fabric *fabric,
                                        u32 caller_core);
