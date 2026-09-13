/*
 * media_admission.h - ADR-072 offline cross-engine media orchestration.
 *
 * This is a fixed-size, Core-0-owned admission model.  It describes numeric
 * resources and dependencies only; it has no media payloads or hardware path.
 */
#pragma once

#include "types.h"
#include "media_engine_contract.h"

#define MEDIA_ADMISSION_OWNER_CORE       0U
#define MEDIA_ADMISSION_MAX_PLANS        4U
#define MEDIA_ADMISSION_MAX_JOBS         8U
#define MEDIA_ADMISSION_MAX_REPLAY       32U
#define MEDIA_ADMISSION_MAX_TIMEOUT_MS   60000U

enum media_admission_plan_state {
    MEDIA_ADMISSION_PLAN_EMPTY = 0,
    MEDIA_ADMISSION_PLAN_ACTIVE,
    MEDIA_ADMISSION_PLAN_COMPLETED,
    MEDIA_ADMISSION_PLAN_FAULTED,
    MEDIA_ADMISSION_PLAN_RETIRED,
};

enum media_admission_job_state {
    MEDIA_ADMISSION_JOB_EMPTY = 0,
    MEDIA_ADMISSION_JOB_PREPARED,
    MEDIA_ADMISSION_JOB_ADMITTED,
    MEDIA_ADMISSION_JOB_COMPLETED,
    MEDIA_ADMISSION_JOB_FAULTED,
    MEDIA_ADMISSION_JOB_RETIRED,
};

enum media_admission_fault {
    MEDIA_ADMISSION_FAULT_NONE = 0,
    MEDIA_ADMISSION_FAULT_ARGUMENT,
    MEDIA_ADMISSION_FAULT_RESOURCE,
    MEDIA_ADMISSION_FAULT_DEPENDENCY,
    MEDIA_ADMISSION_FAULT_ENGINE_LOST,
    MEDIA_ADMISSION_FAULT_TIMEOUT,
    MEDIA_ADMISSION_FAULT_ENGINE_FAILURE,
    MEDIA_ADMISSION_FAULT_CANARY,
    MEDIA_ADMISSION_FAULT_INTERNAL,
};

enum media_admission_replay_type {
    MEDIA_ADMISSION_REPLAY_PLAN_CREATED = 1,
    MEDIA_ADMISSION_REPLAY_JOB_PREPARED,
    MEDIA_ADMISSION_REPLAY_JOB_ADMITTED,
    MEDIA_ADMISSION_REPLAY_JOB_COMPLETED,
    MEDIA_ADMISSION_REPLAY_PLAN_FAULTED,
    MEDIA_ADMISSION_REPLAY_ENGINE_LOST,
    MEDIA_ADMISSION_REPLAY_PLAN_RETIRED,
};

/*
 * A numeric DMA span descriptor is authority metadata, never an address or
 * pointer. `span_start` and `span_bytes` identify an interval within an
 * independently approved numeric namespace; `span_capacity` proves bounds.
 */
struct media_admission_dma_span {
    u64 span_id;
    u64 span_start;
    u64 span_bytes;
    u64 span_capacity;
    u32 generation;
    u32 _reserved;
};

struct media_admission_resources {
    u32 clock_ids[MEDIA_ENGINE_MAX_CLOCKS];
    u32 clock_count;
    u32 iommu_resource;
    u32 gic_spi;
    u32 _reserved;
    struct media_admission_dma_span dma;
};

/* A zero producer_job_id means this job has no input dependency. */
struct media_admission_input {
    u64 producer_job_id;
    u32 producer_generation;
    u32 _reserved;
};

struct media_admission_job_request {
    u64 job_id;
    enum media_engine_kind engine;
    u32 _reserved;
    struct media_admission_resources resources;
    struct media_admission_input input;
};

/* Opaque generation-backed capabilities. */
struct media_admission_plan_handle {
    u64 plan_id;
    u32 controller_id;
    u32 slot;
    u32 generation;
    u32 _reserved;
};

struct media_admission_job_handle {
    u64 plan_id;
    u64 job_id;
    u32 controller_id;
    u32 plan_slot;
    u32 plan_generation;
    u32 job_slot;
    u32 job_generation;
    u32 _reserved;
};

/*
 * Immutable jobs are copied before publication.  They intentionally occupy
 * two cache lines and have no mutable state; controls below are separate.
 */
struct media_admission_job_descriptor {
    u64 job_id;
    u64 input_job_id;
    u64 dma_span_id;
    u64 dma_span_start;
    u64 dma_span_bytes;
    u64 dma_span_capacity;
    u64 lease_token;
    u32 plan_generation;
    u32 job_generation;
    u32 engine;
    u32 input_generation;
    u32 clock_ids[MEDIA_ENGINE_MAX_CLOCKS];
    u32 clock_count;
    u32 iommu_resource;
    u32 gic_spi;
    u32 dma_generation;
    u32 lease_controller_id;
    u32 lease_reserved;
    u8 _pad[24U];
} ALIGNED(64);

_Static_assert(sizeof(struct media_admission_job_descriptor) == 128U,
               "media job descriptors must be two immutable cache lines");

/* Mutable state never shares a cache line with its descriptor or another job. */
struct media_admission_job_control {
    u64 deadline_ms;
    u64 completion_sequence;
    u32 generation;
    u32 state;
    u32 fault;
    u32 canary_valid;
    u32 _reserved0;
    u8 _pad[28U];
} ALIGNED(64);

struct media_admission_plan_control {
    u64 plan_id;
    u64 completion_sequence;
    u32 generation;
    u32 state;
    u32 fault;
    u32 job_count;
    u32 completed_count;
    u32 _reserved0;
    u8 _pad[24U];
} ALIGNED(64);

struct media_admission_owner {
    u32 controller_id;
    u32 initialized;
    u32 hardware_enable;
    u32 transition_active;
    u8 _pad[48U];
} ALIGNED(64);

struct media_admission_replay_event {
    u64 sequence;
    u64 plan_id;
    u64 job_id;
    u64 time_ms;
    u32 plan_generation;
    u32 job_generation;
    u32 type;
    u32 fault;
    u32 engine;
    u32 _reserved;
    u8 _pad[8U];
} ALIGNED(64);

struct media_admission_replay_owner {
    u64 next_sequence;
    u32 oldest_slot;
    u32 count;
    u8 _pad[48U];
} ALIGNED(64);

struct media_admission_snapshot_job {
    u64 job_id;
    u64 deadline_ms;
    u64 completion_sequence;
    u32 generation;
    u32 state;
    u32 fault;
    u32 canary_valid;
    enum media_engine_kind engine;
    u32 _reserved;
};

struct media_admission_snapshot {
    u64 plan_id;
    u64 completion_sequence;
    u32 plan_generation;
    u32 state;
    u32 fault;
    u32 job_count;
    u32 completed_count;
    u32 replay_count;
    u32 _reserved;
    struct media_admission_snapshot_job jobs[MEDIA_ADMISSION_MAX_JOBS];
};

struct media_admission {
    struct media_admission_owner owner;
    const struct media_engine_controller *media_controller;
    u8 _media_pad[56U];
    struct media_admission_plan_control plans[MEDIA_ADMISSION_MAX_PLANS];
    struct media_admission_job_control
        controls[MEDIA_ADMISSION_MAX_PLANS][MEDIA_ADMISSION_MAX_JOBS];
    struct media_admission_job_descriptor
        descriptors[MEDIA_ADMISSION_MAX_PLANS][MEDIA_ADMISSION_MAX_JOBS];
    struct media_admission_replay_owner replay_owner;
    struct media_admission_replay_event replay[MEDIA_ADMISSION_MAX_REPLAY];
} ALIGNED(64);

_Static_assert(sizeof(struct media_admission_owner) == 64U,
               "media admission owner must own one cache line");
_Static_assert(sizeof(struct media_admission_plan_control) == 64U,
               "media plans must have cache-line stride");
_Static_assert(sizeof(struct media_admission_job_control) == 64U,
               "media job controls must have cache-line stride");
_Static_assert(sizeof(struct media_admission_replay_event) == 64U,
               "media replay events must have cache-line stride");
_Static_assert((__builtin_offsetof(struct media_admission, controls) % 64U) ==
               0U,
               "media job controls must start on a cache-line boundary");
_Static_assert((__builtin_offsetof(struct media_admission, descriptors) % 64U) ==
               0U,
               "media job descriptors must start on a cache-line boundary");
_Static_assert(__builtin_offsetof(struct media_admission, descriptors) >
               __builtin_offsetof(struct media_admission, controls),
               "immutable descriptors must follow separate controls");

bool media_admission_init(struct media_admission *admission, u32 controller_id,
                          const struct media_engine_controller *media_controller,
                          u32 caller_core);
bool media_admission_plan_create(struct media_admission *admission, u64 plan_id,
                                 u32 caller_core,
                                 struct media_admission_plan_handle *out);
bool media_admission_job_prepare(
    struct media_admission *admission,
    const struct media_admission_plan_handle *plan,
    const struct media_admission_job_request *request,
    const struct media_engine_lease *lease, u32 caller_core,
    struct media_admission_job_handle *out);
bool media_admission_job_admit(struct media_admission *admission,
                               const struct media_admission_job_handle *job,
                               u64 now_ms, u32 timeout_ms, u32 caller_core);
bool media_admission_job_complete(struct media_admission *admission,
                                  const struct media_admission_job_handle *job,
                                  u64 now_ms, bool engine_ok, bool canary_ok,
                                  u32 caller_core);
bool media_admission_tick(struct media_admission *admission, u64 now_ms,
                          u32 caller_core);
bool media_admission_engine_lost(struct media_admission *admission,
                                 enum media_engine_kind engine, u64 now_ms,
                                 u32 caller_core);
bool media_admission_plan_complete(struct media_admission *admission,
                                   const struct media_admission_plan_handle *plan,
                                   u64 now_ms, u32 caller_core);
bool media_admission_plan_retire(struct media_admission *admission,
                                 const struct media_admission_plan_handle *plan,
                                 u64 now_ms, u32 caller_core);
bool media_admission_plan_state_get(
    const struct media_admission *admission,
    const struct media_admission_plan_handle *plan,
    enum media_admission_plan_state *state_out,
    enum media_admission_fault *fault_out);
bool media_admission_job_state_get(
    const struct media_admission *admission,
    const struct media_admission_job_handle *job,
    enum media_admission_job_state *state_out,
    enum media_admission_fault *fault_out);
bool media_admission_snapshot_get(
    const struct media_admission *admission,
    const struct media_admission_plan_handle *plan,
    struct media_admission_snapshot *out);
bool media_admission_replay_get(
    const struct media_admission *admission, u64 after_sequence,
    struct media_admission_replay_event *out, u32 capacity, u32 *count_out);

/* Permanently false: hardware activation needs a later approved ADR. */
bool media_admission_hardware_enable_allowed(
    const struct media_admission *admission, u32 caller_core);
