#pragma once
#include "intel_bmg_weights.h"
#include "intel_bmg_tensor_plan.h"

struct intel_bmg_tensor_span {
    u8 *data;
    u32 used;
    u32 capacity;
};

struct intel_bmg_tensor_job_handle {
    u32 generation;
};

struct intel_bmg_tensor_job_control {
    struct intel_bmg_weights *registry;
    struct intel_bmg_weight_lease lease;
    u32 owner;
    u32 generation;
    u32 live;
} ALIGNED(64);

struct intel_bmg_tensor_prepared {
    struct intel_bmg_weight_span weight;
    struct intel_bmg_tensor_span activation;
    struct intel_bmg_tensor_span output;
    struct intel_bmg_tensor_plan plan;
    u8 payload[INTEL_BMG_TENSOR_PAYLOAD_BYTES] ALIGNED(64);
} ALIGNED(64);

/* One zero-initialized, kernel-private, core-0-owned job; never reset/reuse
 * storage while tokens exist. No submission or completion API is provided.
 * Handles are scoped to this job's storage, not interchangeable between jobs.
 * Caller-local arguments must not alias job/registry storage. Activation
 * stays immutable and output exclusively owned until abort. Query snapshots
 * expire on abort; neither pointers nor lease may cross ownership domains. */
struct intel_bmg_tensor_job {
    struct intel_bmg_tensor_job_control control;
    struct intel_bmg_tensor_prepared prepared;
} ALIGNED(64);

bool intel_bmg_tensor_job_prepare(struct intel_bmg_tensor_job *job,
    struct intel_bmg_weights *registry, u32 core, u32 owner,
    struct intel_bmg_weight_handle weight, u32 rows, u32 cols,
    const struct intel_bmg_tensor_span *activation,
    const struct intel_bmg_tensor_span *output,
    struct intel_bmg_tensor_job_handle *out);
bool intel_bmg_tensor_job_query(const struct intel_bmg_tensor_job *job,
    u32 core, u32 owner, struct intel_bmg_tensor_job_handle handle,
    struct intel_bmg_tensor_prepared *out);
bool intel_bmg_tensor_job_abort(struct intel_bmg_tensor_job *job,
    u32 core, u32 owner, struct intel_bmg_tensor_job_handle handle);
