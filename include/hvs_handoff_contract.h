/*
 * hvs_handoff_contract.h - ADR-054 V1 offline framebuffer ownership contract.
 *
 * This is policy only.  It neither discovers, maps, reads, writes, nor enables
 * HVS/display hardware.  A future backend must make bounded readback
 * attestations before it reports an observation to this contract.
 */
#pragma once
#include "types.h"

#define HVS_HANDOFF_OWNER_CORE                 0U
#define HVS_HANDOFF_CHANNEL_V1                 0U
#define HVS_HANDOFF_MAX_CHANNEL                HVS_HANDOFF_CHANNEL_V1
#define HVS_HANDOFF_MAX_DIMENSION              8192U
#define HVS_HANDOFF_MAX_PITCH                  32768U
#define HVS_HANDOFF_MAX_SCANOUT_BYTES          (1ULL << 48)
#define HVS_HANDOFF_MAX_LIST_ELEMENTS          16U
#define HVS_HANDOFF_MAX_STAGING_INDEX          1023U
#define HVS_HANDOFF_MAX_DEADLINE_MS            1000U

enum hvs_handoff_state {
    HVS_HANDOFF_MAILBOX_OWNER = 0,
    HVS_HANDOFF_SNAPSHOT_VALID,
    HVS_HANDOFF_LIST_STAGED,
    HVS_HANDOFF_ARM_VERIFIED,
    HVS_HANDOFF_NATIVE_OWNER,
    HVS_HANDOFF_RESTORING,
    HVS_HANDOFF_QUARANTINED,
};

enum hvs_handoff_fault {
    HVS_HANDOFF_FAULT_NONE = 0,
    HVS_HANDOFF_FAULT_OWNER,
    HVS_HANDOFF_FAULT_SNAPSHOT,
    HVS_HANDOFF_FAULT_LIST,
    HVS_HANDOFF_FAULT_ARM,
    HVS_HANDOFF_FAULT_PRESENT,
    HVS_HANDOFF_FAULT_RESTORE,
    HVS_HANDOFF_FAULT_DEADLINE,
    HVS_HANDOFF_FAULT_INTERNAL,
};

enum hvs_handoff_owner_kind {
    HVS_HANDOFF_OWNER_INVALID = 0,
    HVS_HANDOFF_OWNER_MAILBOX = 1,
    HVS_HANDOFF_OWNER_NATIVE = 2,
};

enum hvs_handoff_version {
    HVS_HANDOFF_VERSION_INVALID = 0,
    HVS_HANDOFF_VERSION_V1 = 1,
};

enum hvs_handoff_pixel_order {
    HVS_HANDOFF_PIXEL_ORDER_INVALID = 0,
    HVS_HANDOFF_PIXEL_ORDER_RGBA = 1,
    HVS_HANDOFF_PIXEL_ORDER_ARGB = 2,
};

/*
 * The only V1 profile is a full-frame, four-byte RGBA-family scanout.  This
 * enum describes policy input; it is deliberately not an HVS command value.
 */
enum hvs_handoff_format {
    HVS_HANDOFF_FORMAT_INVALID = 0,
    HVS_HANDOFF_FORMAT_RGBA8888 = 1,
};

enum hvs_handoff_attestation {
    HVS_HANDOFF_ATTESTATION_INVALID = 0,
    HVS_HANDOFF_ATTESTATION_MATCHED = 1,
    HVS_HANDOFF_ATTESTATION_MISMATCHED = 2,
    HVS_HANDOFF_ATTESTATION_FAILED = 3,
    HVS_HANDOFF_ATTESTATION_PRESENTED = 4,
    HVS_HANDOFF_ATTESTATION_RESTORED = 5,
};

/*
 * All snapshot fields are numeric identities.  `existing_list_index` is an
 * opaque backend index, never a CPU/HVS pointer.  V1 accepts mailbox-owned
 * displays only: an existing native display is intentionally not restorable.
 */
struct hvs_handoff_snapshot_input {
    u64 scanout_base;
    u64 scanout_generation;
    u64 framebuffer_bytes;
    u64 existing_list_generation;
    u32 width;
    u32 height;
    u32 pitch;
    u32 snapshot_layout_version;
    u32 hvs_id;
    u32 target_channel;
    u32 existing_list_index;
    u32 owner;
    u32 format;
    u32 pixel_order;
    u32 _reserved;
};

/*
 * Numeric list authority only.  The contract retains this descriptor by
 * value; `staging_index` and range fields are opaque indices, not addresses.
 */
struct hvs_handoff_list_input {
    u64 source_scanout_base;
    u64 source_scanout_generation;
    u64 list_generation;
    u32 element_count;
    u32 staging_index;
    u32 staging_range_start;
    u32 staging_range_count;
    u32 width;
    u32 height;
    u32 pitch;
    u32 format;
    u32 pixel_order;
    u32 _reserved;
};

/*
 * These attestations are results of a future bounded backend readback, not
 * requests and not booleans which claim that hardware was touched.
 */
struct hvs_handoff_observation {
    u64 list_generation;
    u32 channel;
    u32 observed_list_index;
    u32 attestation;
    u32 status;
};

struct hvs_handoff_restore_observation {
    u64 existing_list_generation;
    u32 channel;
    u32 observed_list_index;
    u32 observed_owner;
    u32 attestation;
    u32 status;
};

/* Opaque, generation-safe capability; it never contains a controller pointer. */
struct hvs_handoff_handle {
    u64 _token;
    u64 _generation;
    u32 _controller_id;
    u32 _reserved;
};

/*
 * The only mutable shared control record.  One controller owns one cache
 * line, and all state/fault/count updates are made by core 0 only.
 */
struct hvs_handoff_control {
    u64 generation;
    u64 deadline_ms;
    u64 last_observed;
    u32 controller_id;
    u32 state;
    u32 fault;
    u32 last_status;
    u32 handoff_attempt_count;
    u32 restore_attempt_count;
    u32 failure_count;
    u32 owner_core;
    u8 _reserved[8U];
} ALIGNED(64);

_Static_assert(sizeof(struct hvs_handoff_control) == 64U,
               "HVS handoff mutable control must own one cache line");

/*
 * Snapshot/list payloads are private copies and are immutable while
 * published.  The controller must be fresh all-zero storage at init.
 */
struct hvs_handoff_contract {
    struct hvs_handoff_control control;
    struct hvs_handoff_snapshot_input snapshot;
    struct hvs_handoff_list_input list;
} ALIGNED(64);

_Static_assert(__builtin_offsetof(struct hvs_handoff_contract, snapshot) == 64U,
               "private payload must not share mutable control cache line");

bool hvs_handoff_contract_init(struct hvs_handoff_contract *contract,
                               u32 controller_id, u32 owner_core,
                               u64 now_ms, u32 deadline_ms);
bool hvs_handoff_snapshot(struct hvs_handoff_contract *contract,
                          const struct hvs_handoff_handle *handle,
                          u32 owner_core, u64 now_ms,
                          const struct hvs_handoff_snapshot_input *snapshot);
bool hvs_handoff_stage_list(struct hvs_handoff_contract *contract,
                            const struct hvs_handoff_handle *handle,
                            u32 owner_core, u64 now_ms,
                            const struct hvs_handoff_list_input *list);
bool hvs_handoff_report_arm(struct hvs_handoff_contract *contract,
                            const struct hvs_handoff_handle *handle,
                            u32 owner_core, u64 now_ms,
                            const struct hvs_handoff_observation *observation);
bool hvs_handoff_report_present(struct hvs_handoff_contract *contract,
                                const struct hvs_handoff_handle *handle,
                                u32 owner_core, u64 now_ms,
                                const struct hvs_handoff_observation *observation);
bool hvs_handoff_restore_begin(struct hvs_handoff_contract *contract,
                               const struct hvs_handoff_handle *handle,
                               u32 owner_core, u64 now_ms);
bool hvs_handoff_report_restore(
    struct hvs_handoff_contract *contract,
    const struct hvs_handoff_handle *handle, u32 owner_core, u64 now_ms,
    const struct hvs_handoff_restore_observation *observation);

/* Rearm clears diagnostics by definition, invalidates the handle, and leaves
 * the controller at the mailbox-owner state without performing a restore. */
bool hvs_handoff_rearm(struct hvs_handoff_contract *contract,
                       const struct hvs_handoff_handle *handle,
                       u32 owner_core, u64 now_ms,
                       struct hvs_handoff_handle *new_handle_out);

bool hvs_handoff_handle_get(const struct hvs_handoff_contract *contract,
                            struct hvs_handoff_handle *handle_out);
bool hvs_handoff_state_get(const struct hvs_handoff_contract *contract,
                           const struct hvs_handoff_handle *handle,
                           enum hvs_handoff_state *state_out);
bool hvs_handoff_snapshot_get(
    const struct hvs_handoff_contract *contract,
    const struct hvs_handoff_handle *handle,
    struct hvs_handoff_snapshot_input *snapshot_out);
bool hvs_handoff_list_get(const struct hvs_handoff_contract *contract,
                          const struct hvs_handoff_handle *handle,
                          struct hvs_handoff_list_input *list_out);
