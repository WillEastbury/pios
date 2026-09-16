/*
 * usb_vbus_contract.h - ADR-059 BCM2837 external-USB-power evidence contract.
 *
 * This is deliberately offline policy.  It does not discover, read, write,
 * source, toggle, or otherwise control board power.  A true result from
 * usb_vbus_probe_permitted() permits only a later passive controller probe;
 * it never permits software VBUS enablement.
 */
#pragma once
#include "types.h"

#define USB_VBUS_PROFILE_PI3_B          0x08U
#define USB_VBUS_PROFILE_PI3_B_PLUS     0x0DU
#define USB_VBUS_PROFILE_ZERO_2_W       0x12U
#define USB_VBUS_CONTRACT_CAPACITY       4U
#define USB_VBUS_OWNER_CORE              0U
#define USB_VBUS_MAX_ATTESTED_MILLIAMPS  5000U

enum usb_vbus_topology {
    USB_VBUS_TOPOLOGY_INVALID = 0,
    USB_VBUS_TOPOLOGY_PI3_B_LAN9514,
    USB_VBUS_TOPOLOGY_PI3_B_PLUS_LAN7515,
    USB_VBUS_TOPOLOGY_ZERO_2_W_OTG_CONNECTOR,
};

enum usb_vbus_state {
    USB_VBUS_FREE = 0,
    USB_VBUS_SAFE_UNKNOWN,
    USB_VBUS_EXTERNAL_ATTESTED,
    USB_VBUS_AVAILABLE_UNCONTROLLED,
    USB_VBUS_QUARANTINED,
    USB_VBUS_RELEASED,
    USB_VBUS_RETIRED,
};

enum usb_vbus_fault {
    USB_VBUS_FAULT_NONE = 0,
    USB_VBUS_FAULT_OVERCURRENT,
    USB_VBUS_FAULT_CURRENT_LIMIT,
};

enum usb_vbus_software_proof {
    USB_VBUS_PROOF_CONTROL = 1U << 0,
    USB_VBUS_PROOF_CURRENT_LIMIT = 1U << 1,
    USB_VBUS_PROOF_OVERCURRENT = 1U << 2,
};

/*
 * These are factual topology classifications, not electrical-control claims.
 * All software authority masks are zero because no PIOS-controllable BCM2837
 * VBUS, current-limit, or overcurrent mechanism has been proven.
 */
struct usb_vbus_profile {
    u32 board_model;
    u32 topology;
    u32 software_vbus_control_mask;
    u32 software_current_limit_mask;
    u32 software_overcurrent_mask;
    u32 provenance_revision;
};

/*
 * Operator/backend evidence only.  It is not a hardware observation.  A
 * current observer is optional, but its identity, generation, and declared
 * bound are an all-or-nothing group.  The bound is evidence metadata, not a
 * board electrical specification, and is constrained to a sane u32 range.
 */
struct usb_vbus_external_attestation {
    u64 topology_id;
    u64 topology_generation;
    u64 observer_id;
    u64 observer_generation;
    u32 board_model;
    u32 declared_max_milliamps;
    u32 externally_powered;
    u32 independent_current_protection;
    u32 port_cable_verified;
    u32 no_backfeed_verified;
    u32 _reserved[2U];
};

struct usb_vbus_current_observation {
    u64 observer_id;
    u64 observer_generation;
    u32 current_milliamps;
    u32 overcurrent;
    u32 _reserved;
};

/*
 * Handles contain no pointer.  A slot, full-width generation, controller
 * identity, and non-forgeable-by-accident token must all match a live record.
 */
struct usb_vbus_handle {
    u64 _token;
    u64 generation;
    u32 controller_id;
    u32 slot;
    u32 _reserved;
    u32 _pad;
};

/*
 * Mutable state occupies one cache line.  Attestations live in separate
 * cache-line records so immutable evidence never shares state ownership.
 */
struct usb_vbus_record {
    u64 generation;
    u32 controller_id;
    u32 board_model;
    u32 owner_core;
    u32 state;
    u32 fault;
    u32 fault_count;
    u32 last_current_milliamps;
    u32 observation_count;
    u32 mutation_guard;
    u8 _pad[20U];
} ALIGNED(64);

struct usb_vbus_attestation_copy {
    struct usb_vbus_external_attestation value;
} ALIGNED(64);

struct usb_vbus_pool_owner {
    u32 initialized;
    u32 mutation_guard;
    u8 _pad[56U];
} ALIGNED(64);

_Static_assert(sizeof(struct usb_vbus_record) == 64U,
               "USB VBUS mutable records must own one cache line");
_Static_assert(sizeof(struct usb_vbus_attestation_copy) == 64U,
               "USB VBUS attestation copies must retain cache-line stride");
_Static_assert(sizeof(struct usb_vbus_pool_owner) == 64U,
               "USB VBUS pool owner must own one cache line");

struct usb_vbus_contract_pool {
    struct usb_vbus_pool_owner owner;
    struct usb_vbus_record records[USB_VBUS_CONTRACT_CAPACITY];
    struct usb_vbus_attestation_copy attestations[USB_VBUS_CONTRACT_CAPACITY];
} ALIGNED(64);

_Static_assert((sizeof(struct usb_vbus_contract_pool) % 64U) == 0U,
               "USB VBUS pool must retain cache-line record stride");
_Static_assert(__builtin_offsetof(struct usb_vbus_contract_pool, records)
               == 64U,
               "USB VBUS records must not share the pool owner line");
_Static_assert(__builtin_offsetof(struct usb_vbus_contract_pool, attestations)
               == (USB_VBUS_CONTRACT_CAPACITY + 1U) * 64U,
               "USB VBUS evidence must not share mutable control lines");

/* Exact-match only; unsupported board IDs have no profile. */
const struct usb_vbus_profile *usb_vbus_profile_get(u32 board_model);

/* Initialization is one-shot over fresh all-zero storage. */
bool usb_vbus_contract_pool_init(struct usb_vbus_contract_pool *pool);

bool usb_vbus_contract_acquire(struct usb_vbus_contract_pool *pool,
                               u32 controller_id, u32 board_model,
                               u32 owner_core,
                               struct usb_vbus_handle *handle_out);
bool usb_vbus_contract_release(struct usb_vbus_contract_pool *pool,
                               const struct usb_vbus_handle *handle,
                               u32 owner_core);

/*
 * This copies externally supplied evidence privately.  It cannot establish
 * that PIOS powers a port or has an electrical control path.
 */
bool usb_vbus_external_attest(
    struct usb_vbus_contract_pool *pool, const struct usb_vbus_handle *handle,
    u32 owner_core, const struct usb_vbus_external_attestation *attestation);

/*
 * A measurement is accepted only for an observer named by the private
 * attestation.  A positive overcurrent observation or a value over the
 * attested bound quarantines the record.  Missing observer evidence is never
 * treated as a safe measurement.
 */
bool usb_vbus_current_observe(
    struct usb_vbus_contract_pool *pool, const struct usb_vbus_handle *handle,
    u32 owner_core, const struct usb_vbus_current_observation *observation);

bool usb_vbus_handle_get(const struct usb_vbus_contract_pool *pool,
                         u32 controller_id, struct usb_vbus_handle *handle_out);
bool usb_vbus_state_get(const struct usb_vbus_contract_pool *pool,
                        const struct usb_vbus_handle *handle,
                        enum usb_vbus_state *state_out);
bool usb_vbus_attestation_get(
    const struct usb_vbus_contract_pool *pool, const struct usb_vbus_handle *handle,
    struct usb_vbus_external_attestation *attestation_out);
bool usb_vbus_telemetry_available(const struct usb_vbus_contract_pool *pool,
                                  const struct usb_vbus_handle *handle);

/*
 * A true result means externally supplied evidence allows only a future
 * passive controller probe.  It does not permit sourcing or toggling VBUS.
 */
bool usb_vbus_probe_permitted(const struct usb_vbus_contract_pool *pool,
                              const struct usb_vbus_handle *handle);

/*
 * Always false for every profile in this contract.  The returned mask names
 * the missing independent proof categories rather than pretending that an
 * externally powered hub gives PIOS a software enable authority.
 */
bool usb_vbus_software_enable_permitted(const struct usb_vbus_profile *profile,
                                        u32 *missing_proof_mask_out);
