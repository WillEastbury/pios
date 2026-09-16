/*
 * pisp_be_gate.h - ADR-055 core-0-only PiSP-BE brick-test state gate.
 *
 * This is a pure state contract. Callers supply trusted observations and
 * numeric time; this module neither obtains observations nor starts work.
 */
#pragma once
#include "types.h"
#include "pisp_be_contract.h"

#define PISP_BE_GATE_OWNER_CORE       0U
#define PISP_BE_GATE_MAX_TIMEOUT_MS   1000U

enum pisp_be_gate_state {
    PISP_BE_GATE_DISABLED = 0,
    PISP_BE_GATE_PASSIVE_ID_OK,
    PISP_BE_GATE_CLOCK_OK,
    PISP_BE_GATE_IOMMU_OK,
    PISP_BE_GATE_IDLE_OK,
    PISP_BE_GATE_JOB_ADMITTED,
    PISP_BE_GATE_JOB_COMPLETED,
    PISP_BE_GATE_QUARANTINED,
};

enum pisp_be_gate_fault {
    PISP_BE_GATE_FAULT_NONE = 0,
    PISP_BE_GATE_FAULT_BAD_OWNER,
    PISP_BE_GATE_FAULT_BAD_ID,
    PISP_BE_GATE_FAULT_CLOCK,
    PISP_BE_GATE_FAULT_IOMMU,
    PISP_BE_GATE_FAULT_STATUS,
    PISP_BE_GATE_FAULT_DEADLINE,
    PISP_BE_GATE_FAULT_JOB,
    PISP_BE_GATE_FAULT_CANARY,
    PISP_BE_GATE_FAULT_INTERNAL,
};

/*
 * A gate handle is a generation-backed capability, not an address. Its
 * controller identity and complete generation bind every mutating action.
 */
struct pisp_be_gate_handle {
    u64 _token;
    u64 _generation;
    u32 _controller_id;
    u32 _reserved;
};

/*
 * The future trusted backend must attest that this exact active lease owns
 * the named IOMMU resource. A true observation alone is insufficient.
 */
struct pisp_be_gate_iommu_evidence {
    struct media_engine_lease lease;
    u32 iommu_resource;
    u32 _reserved;
};

/*
 * Idle requires an explicit trusted attestation as well as the raw queue
 * status and matching batch counters. A zero status alone never implies idle.
 */
struct pisp_be_gate_idle_evidence {
    u32 raw_status;
    u32 batch_started;
    u32 batch_done;
    u32 _reserved;
};

/*
 * Mutable owner state is exactly one cache line and is written by core 0
 * only. It deliberately contains no request payload or external address.
 */
struct pisp_be_gate {
    u64 generation;
    u64 deadline_ms;
    u32 controller_id;
    u32 owner_core;
    u32 state;
    u32 fault;
    u32 transition_count;
    u32 failure_count;
    u32 last_status;
    u32 _reserved0;
    u8 _reserved[16U];
} ALIGNED(64);

_Static_assert(sizeof(struct pisp_be_gate) == 64U,
               "PiSP BE gate must own exactly one cache line");

/*
 * The caller supplies all ownership and time facts. `timeout_ms` is a hard,
 * bounded duration and deadline arithmetic is checked before publication.
 * Storage must be fresh, never-published, zero-initialized storage. Init is
 * intentionally one-shot: it rejects a gate that has ever been initialized.
 * Recover a quarantined gate with pisp_be_gate_rearm(), never by re-init.
 */
bool pisp_be_gate_init(struct pisp_be_gate *gate, u32 owner_core,
                       u32 controller_id, u64 now_ms, u32 timeout_ms,
                       struct pisp_be_gate_handle *handle_out);

/*
 * The gate establishes passive identity through media_engine_contract's
 * public API. The caller acquires the PiSP-BE engine lease after this step.
 */
bool pisp_be_gate_passive_identify(
    struct pisp_be_gate *gate, const struct pisp_be_gate_handle *handle,
    u32 caller_core, struct media_engine_controller *media_controller,
    u64 now_ms, u32 raw_version);

bool pisp_be_gate_clock_verified(
    struct pisp_be_gate *gate, const struct pisp_be_gate_handle *handle,
    u32 caller_core, const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease, u64 now_ms,
    u32 clock_id, bool clock_ok);

bool pisp_be_gate_iommu_verified(
    struct pisp_be_gate *gate, const struct pisp_be_gate_handle *handle,
    u32 caller_core, const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease, u64 now_ms,
    const struct pisp_be_gate_iommu_evidence *evidence, bool iommu_ok);

/*
 * Idle requires explicit trusted `idle_ok`, zero raw status, and balanced
 * batch counters. The evidence is caller-supplied only; this module performs
 * no hardware I/O.
 */
bool pisp_be_gate_idle_verified(
    struct pisp_be_gate *gate, const struct pisp_be_gate_handle *handle,
    u32 caller_core, const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease, u64 now_ms,
    const struct pisp_be_gate_idle_evidence *evidence, bool idle_ok);

/*
 * Admission validates a PREPARED job but intentionally never changes the
 * job state. The caller retains job and engine-lease lifecycle ownership.
 */
bool pisp_be_gate_admit_prepared_job(
    struct pisp_be_gate *gate, const struct pisp_be_gate_handle *handle,
    u32 caller_core, const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease, u64 now_ms,
    const struct pisp_be_contract *job_contract,
    const struct pisp_be_job_handle *job_handle);

/*
 * Completion records only a trusted future backend report. It neither
 * completes/releases a job nor completes/releases an engine lease.
 */
bool pisp_be_gate_report_completion(
    struct pisp_be_gate *gate, const struct pisp_be_gate_handle *handle,
    u32 caller_core, u64 now_ms, bool job_ok, bool canaries_ok);

/*
 * Recovery is explicit: only the current capability may rearm a quarantined
 * gate. A successful rearm invalidates every preceding handle. It does not
 * rearm, release, complete, abort, or otherwise mutate the media-engine
 * lease. The caller must explicitly settle/rearm media_engine_contract before
 * a new passive-identify cycle after an identity or job fault.
 */
bool pisp_be_gate_rearm(struct pisp_be_gate *gate,
                        const struct pisp_be_gate_handle *handle,
                        u32 caller_core, u64 now_ms, u32 timeout_ms,
                        struct pisp_be_gate_handle *handle_out);

bool pisp_be_gate_state_get(const struct pisp_be_gate *gate,
                            const struct pisp_be_gate_handle *handle,
                            u32 caller_core,
                            enum pisp_be_gate_state *state_out,
                            enum pisp_be_gate_fault *fault_out);
