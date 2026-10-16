#pragma once
#include "intel_bmg_state.h"
#include "intel_bmg_walker.h"

#define INTEL_BMG_BATCH_BYTES 352U
#define INTEL_BMG_BATCH_WALKER_OFFSET 164U
#define INTEL_BMG_BATCH_FENCE_OFFSET 324U

struct intel_bmg_batch_heap {
    u64 address;
    u32 bytes;
    u32 reserved;
};

/* Caller-local reviewed record. validated==1 and review_id are explicit
 * attestations by the owning caller, not authentication or device discovery.
 * No built-in topology, cache policy or workaround defaults exist. */
struct intel_bmg_batch_policy {
    u32 owner, generation, context, validated;
    u64 review_id;
    u32 eu_count, threads_per_eu, thread_limit, cfe_threads;
    u32 instruction_mocs, general_mocs, stateless_mocs, l1_policy;
    u32 pre_pipeline_flags, pre_sba_flags, completion_flags, reserved;
    struct intel_bmg_batch_heap instruction, indirect, tensor, batch, fence;
};

struct intel_bmg_batch_reference {
    const u8 *data;
    u64 address;
    u32 used, capacity;
    u32 owner, generation, context, immutable;
};

struct intel_bmg_batch_request {
    const struct intel_bmg_batch_policy *policy;
    u32 policy_bytes;
    u32 owner, generation, context;
    struct intel_bmg_tensor_plan plan;
    struct intel_bmg_batch_reference code, indirect;
    u64 fence_value;
};

struct intel_bmg_batch_result {
    u32 owner, generation, context, used;
    u32 walker_offset, fence_offset, execution_gate, reserved;
    u64 policy_review_id;
};

/* Offline assembly only; caller retains immutable policy, references, code
 * and payload through use of the result. Expected tokens are independently
 * owned context state. Both destinations must be disjoint from all full
 * input capacity spans and each other. Failure writes neither destination.
 * Success never sets execution_gate: product ordering/retirement is unproven.
 * No upload, mapping, cache maintenance, submission or live-backend hook. */
bool intel_bmg_batch_prepare(const struct intel_bmg_batch_request *request,
    u32 request_bytes, u32 owner, u32 generation, u32 context,
    u8 *out, u32 capacity, struct intel_bmg_batch_result *result,
    u32 result_bytes);
