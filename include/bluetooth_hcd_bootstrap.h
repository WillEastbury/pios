/*
 * bluetooth_hcd_bootstrap.h - ADR-068 offline HCD/baud bootstrap contract.
 *
 * This is an asynchronous evidence state machine only.  It neither accesses
 * a controller nor grants physical Bluetooth activation authority.
 */
#pragma once
#include "types.h"
#include "bluetooth_platform_contract.h"

#define BLUETOOTH_HCD_BOOTSTRAP_SHA256_BYTES 32U
#define BLUETOOTH_HCD_BOOTSTRAP_MAX_TIMEOUT_MS 60000U

enum bluetooth_hcd_bootstrap_state {
    BLUETOOTH_HCD_BOOTSTRAP_SAFE_OFF = 0,
    BLUETOOTH_HCD_BOOTSTRAP_WAIT_ARTIFACT,
    BLUETOOTH_HCD_BOOTSTRAP_WAIT_SAFE_STATE,
    BLUETOOTH_HCD_BOOTSTRAP_WAIT_HCD_ACK,
    BLUETOOTH_HCD_BOOTSTRAP_REQUEST_BAUD,
    BLUETOOTH_HCD_BOOTSTRAP_WAIT_BAUD_ACK,
    BLUETOOTH_HCD_BOOTSTRAP_COMPLETE,
    BLUETOOTH_HCD_BOOTSTRAP_QUARANTINED,
};

enum bluetooth_hcd_bootstrap_fault {
    BLUETOOTH_HCD_BOOTSTRAP_FAULT_NONE = 0,
    BLUETOOTH_HCD_BOOTSTRAP_FAULT_BAD_PLAN,
    BLUETOOTH_HCD_BOOTSTRAP_FAULT_ARTIFACT,
    BLUETOOTH_HCD_BOOTSTRAP_FAULT_DIGEST,
    BLUETOOTH_HCD_BOOTSTRAP_FAULT_SAFE_STATE,
    BLUETOOTH_HCD_BOOTSTRAP_FAULT_HCD_ACK,
    BLUETOOTH_HCD_BOOTSTRAP_FAULT_BAUD_REQUEST,
    BLUETOOTH_HCD_BOOTSTRAP_FAULT_BAUD_ACK,
    BLUETOOTH_HCD_BOOTSTRAP_FAULT_DEADLINE,
    BLUETOOTH_HCD_BOOTSTRAP_FAULT_SEQUENCE,
};

enum bluetooth_hcd_bootstrap_step_result {
    BLUETOOTH_HCD_BOOTSTRAP_STEP_REJECTED = 0,
    BLUETOOTH_HCD_BOOTSTRAP_STEP_NO_PROGRESS,
    BLUETOOTH_HCD_BOOTSTRAP_STEP_PROGRESS,
    BLUETOOTH_HCD_BOOTSTRAP_STEP_COMPLETE,
    BLUETOOTH_HCD_BOOTSTRAP_STEP_QUARANTINED,
};

/*
 * The selection is injected by the caller.  There is no fallback catalogue,
 * filename convention, or implicit artifact selection in this module.
 */
struct bluetooth_hcd_bootstrap_selection {
    u64 source_id;
    u64 source_generation;
    u64 command_id;
    u32 profile_id;
    u32 controller_id;
    u32 source_bytes;
    u32 command_bytes;
    u8 source_sha256[BLUETOOTH_HCD_BOOTSTRAP_SHA256_BYTES];
    u8 command_sha256[BLUETOOTH_HCD_BOOTSTRAP_SHA256_BYTES];
    u8 _reserved[8U];
} ALIGNED(64);

_Static_assert(sizeof(struct bluetooth_hcd_bootstrap_selection) == 128U,
               "HCD bootstrap selection must retain a cache-line stride");

/*
 * `data` is opaque caller-owned evidence.  The state machine checks that it
 * is present and matches the injected selection but never reads its bytes.
 */
struct bluetooth_hcd_bootstrap_source_blob {
    const u8 *data;
    u64 source_id;
    u64 source_generation;
    u32 byte_count;
    u32 _reserved;
    u8 sha256[BLUETOOTH_HCD_BOOTSTRAP_SHA256_BYTES];
} ALIGNED(64);

struct bluetooth_hcd_bootstrap_command_descriptor {
    u64 command_id;
    u32 byte_count;
    u32 _reserved;
    u8 sha256[BLUETOOTH_HCD_BOOTSTRAP_SHA256_BYTES];
    u8 _pad[16U];
} ALIGNED(64);

_Static_assert(sizeof(struct bluetooth_hcd_bootstrap_source_blob) == 64U,
               "HCD source evidence must own one cache line");
_Static_assert(sizeof(struct bluetooth_hcd_bootstrap_command_descriptor) == 64U,
               "HCD command evidence must own one cache line");

/*
 * Authority is explicitly negative: callers must attest false.  A true
 * claim is malformed and quarantines the offline operation.
 */
struct bluetooth_hcd_bootstrap_plan {
    struct bluetooth_hcd_bootstrap_selection selection;
    u32 initial_baud;
    u32 target_baud;
    u32 state_timeout_ms;
    bool activation_authority_attested;
    u8 _reserved[3U];
};

struct bluetooth_hcd_bootstrap_safe_state_evidence {
    u64 instance_epoch;
    u64 attempt_generation;
    u64 evidence_id;
    u64 evidence_generation;
    u32 profile_id;
    u32 controller_id;
    bool controller_safe_off;
    bool hardware_enable_allowed;
    bool activation_authority_attested;
    u8 _reserved[13U];
} ALIGNED(64);

struct bluetooth_hcd_bootstrap_hcd_ack {
    u64 instance_epoch;
    u64 attempt_generation;
    u64 source_id;
    u64 source_generation;
    u64 command_id;
    u32 profile_id;
    u32 controller_id;
    u32 command_bytes;
    bool accepted;
    u8 _reserved[3U];
    u8 command_sha256[BLUETOOTH_HCD_BOOTSTRAP_SHA256_BYTES];
    u8 _pad[40U];
} ALIGNED(64);

_Static_assert(sizeof(struct bluetooth_hcd_bootstrap_safe_state_evidence) ==
               64U, "Safe-state evidence must own one cache line");
_Static_assert(sizeof(struct bluetooth_hcd_bootstrap_hcd_ack) == 128U,
               "HCD acknowledgement must retain cache-line stride");

struct bluetooth_hcd_bootstrap_baud_request {
    /*
     * Together these fields are the attempt identifier. The epoch is
     * externally unique and the generation is monotonic within that epoch.
     */
    u64 instance_epoch;
    u64 attempt_generation;
    u32 controller_id;
    u32 target_baud;
};

struct bluetooth_hcd_bootstrap_baud_ack {
    u64 instance_epoch;
    u64 attempt_generation;
    u32 controller_id;
    u32 baud;
    bool accepted;
    u8 _reserved[3U];
};

_Static_assert(sizeof(struct bluetooth_hcd_bootstrap_baud_request) == 24U,
               "Baud request must retain its complete attempt identity");
_Static_assert(sizeof(struct bluetooth_hcd_bootstrap_baud_ack) == 32U,
               "Baud acknowledgement must retain its complete attempt identity");

enum bluetooth_hcd_bootstrap_observation_kind {
    BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_NONE = 0,
    BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_ARTIFACT,
    BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_SAFE_STATE,
    BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_HCD_ACK,
    BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_BAUD_REQUEST,
    BLUETOOTH_HCD_BOOTSTRAP_OBSERVE_BAUD_ACK,
};

struct bluetooth_hcd_bootstrap_observation {
    u32 kind;
    u32 _reserved;
    u64 instance_epoch;
    u64 attempt_generation;
    union {
        struct {
            const struct bluetooth_hcd_bootstrap_source_blob *source;
            const struct bluetooth_hcd_bootstrap_command_descriptor *command;
        } artifact;
        struct bluetooth_hcd_bootstrap_safe_state_evidence safe_state;
        struct bluetooth_hcd_bootstrap_hcd_ack hcd_ack;
        struct bluetooth_hcd_bootstrap_baud_request baud_request;
        struct bluetooth_hcd_bootstrap_baud_ack baud_ack;
    } value;
};

struct bluetooth_hcd_bootstrap_handle {
    u64 _token;
    u64 instance_epoch;
    u64 attempt_generation;
};

/*
 * Mutable control and immutable selection reside in independent cache lines.
 * One owner serializes all calls; no pointer or descriptor ownership is
 * transferred to a hardware or interrupt domain.
 */
struct bluetooth_hcd_bootstrap_control {
    u64 instance_epoch;
    u64 deadline_ms;
    u64 attempt_generation;
    u32 state;
    u32 fault;
    u32 state_timeout_ms;
    u32 target_baud;
    u32 progress_count;
    u32 fault_count;
    u8 _reserved[16U];
} ALIGNED(64);

_Static_assert(sizeof(struct bluetooth_hcd_bootstrap_control) == 64U,
               "HCD bootstrap control must own one cache line");

struct bluetooth_hcd_bootstrap {
    struct bluetooth_hcd_bootstrap_control control;
    struct bluetooth_hcd_bootstrap_selection selection;
} ALIGNED(64);

_Static_assert(__builtin_offsetof(struct bluetooth_hcd_bootstrap, selection)
               == 64U, "HCD selection must not share mutable control");

/*
 * `instance_epoch` is caller-assigned, nonzero, and globally unique for the
 * lifetime of this storage. Reinitialization requires a different epoch and
 * invalidates every handle, fact, request, and acknowledgement from the
 * preceding instance.
 */
bool bluetooth_hcd_bootstrap_init(
    struct bluetooth_hcd_bootstrap *bootstrap,
    u64 instance_epoch,
    struct bluetooth_hcd_bootstrap_handle *handle_out);

/*
 * Submitting a valid plan is the first validated input.  It creates a fresh
 * deadline for artifact validation and advances `handle->attempt_generation`.
 * That generation, paired with the instance epoch, identifies every
 * observation and acknowledgement for the attempt.
 */
enum bluetooth_hcd_bootstrap_step_result bluetooth_hcd_bootstrap_submit(
    struct bluetooth_hcd_bootstrap *bootstrap,
    struct bluetooth_hcd_bootstrap_handle *handle, u64 now_ms,
    const struct bluetooth_hcd_bootstrap_plan *plan);

/*
 * A NULL observation is a non-blocking timeout check.  OBSERVE_NONE models a
 * callback/poll that has no newly validated fact; it never creates progress.
 */
enum bluetooth_hcd_bootstrap_step_result bluetooth_hcd_bootstrap_step(
    struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_handle *handle, u64 now_ms,
    const struct bluetooth_hcd_bootstrap_observation *observation);

bool bluetooth_hcd_bootstrap_baud_request_get(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_handle *handle,
    struct bluetooth_hcd_bootstrap_baud_request *request_out);

bool bluetooth_hcd_bootstrap_state_get(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_handle *handle,
    enum bluetooth_hcd_bootstrap_state *state_out,
    enum bluetooth_hcd_bootstrap_fault *fault_out, u32 *progress_count_out);

/*
 * A caller may use this monotonic sequence as watchdog evidence.  It reports
 * true only after a validated state transition; idle polls and rejected input
 * never manufacture progress.
 */
bool bluetooth_hcd_bootstrap_progress_evidence_since(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_handle *handle, u32 since_count,
    u32 *current_count_out);

/* Deliberately false; this pure module cannot grant physical activation. */
bool bluetooth_hcd_bootstrap_hardware_enable_allowed(
    const struct bluetooth_hcd_bootstrap *bootstrap,
    const struct bluetooth_hcd_bootstrap_handle *handle);
