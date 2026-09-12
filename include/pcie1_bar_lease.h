/*
 * pcie1_bar_lease.h - offline PCIe1 endpoint BAR/MMIO ownership gate.
 *
 * Numeric configuration and aperture descriptions only. This contract does
 * not map, touch, or enable endpoint hardware.
 */
#pragma once

#include "types.h"
#include "platform.h"

#define PCIE1_BAR_LEASE_OWNER_CORE          0U
#define PCIE1_BAR_LEASE_RESERVATION_CAPACITY 8U
#define PCIE1_BAR_LEASE_RP1_BYTES           0x00800000ULL
#define PCIE1_BAR_LEASE_TOKEN_MAGIC         0xB188ULL
#define PCIE1_BAR_LEASE_TOKEN_SHIFT         48U

enum pcie1_bar_lease_state {
    PCIE1_BAR_LEASE_FREE = 0U,
    PCIE1_BAR_LEASE_ACTIVE,
    PCIE1_BAR_LEASE_QUARANTINED,
    PCIE1_BAR_LEASE_REMOVED,
    PCIE1_BAR_LEASE_RETIRED,
};

enum pcie1_bar_lease_attribute {
    PCIE1_BAR_LEASE_ATTR_INVALID = 0U,
    PCIE1_BAR_LEASE_ATTR_DEVICE_nGnRnE = 1U,
};

enum pcie1_bar_lease_address_domain {
    PCIE1_BAR_LEASE_DOMAIN_CPU_PHYS = 1U,
    PCIE1_BAR_LEASE_DOMAIN_PCI = 2U,
};

enum pcie1_bar_lease_fault {
    PCIE1_BAR_LEASE_FAULT_NONE = 0U,
    PCIE1_BAR_LEASE_FAULT_AER,
    PCIE1_BAR_LEASE_FAULT_REMOVED,
    PCIE1_BAR_LEASE_FAULT_FAILURE,
};

/*
 * Raw configuration and all-ones probe readbacks. `config_hi` and `probe_hi`
 * are zero for a 32-bit BAR. No caller pointer or MMIO address is retained.
 */
struct pcie1_bar_lease_bar {
    u32 config_lo;
    u32 config_hi;
    u32 probe_lo;
    u32 probe_hi;
};

/* Numeric mapping authority only; no CPU pointer is represented here. */
struct pcie1_bar_lease_aperture {
    u64 cpu_base;
    u64 pci_base;
    u64 length;
    u32 attribute;
    u32 _reserved;
};

struct pcie1_bar_lease_reservation {
    u64 base;
    u64 length;
    u32 domain;
    u8 _pad[44U];
} ALIGNED(64);

/*
 * The supplied reservation descriptors are copied before publication. The
 * descriptor pointer itself is not retained and cannot transfer MMIO access.
 */
struct pcie1_bar_lease_request {
    struct pcie1_bar_lease_bar bar;
    struct pcie1_bar_lease_aperture aperture;
    const struct pcie1_bar_lease_reservation *reservations;
    u32 reservation_count;
    u32 _reserved;
};

/* Opaque generation capability, never a CPU or PCI pointer. */
struct pcie1_bar_lease_handle {
    u64 token;
    u64 generation;
    u32 contract_id;
    u32 bdf;
    u64 _reserved;
};

/* Mutable authority is isolated from the immutable mapping payload. */
struct pcie1_bar_lease_control {
    u64 generation;
    u64 capability;
    u32 contract_id;
    u32 bdf;
    u32 owner_core;
    u32 state;
    u32 fault;
    u32 exhausted;
    u32 revoke_count;
    u8 _pad[16U];
} ALIGNED(64);

/*
 * Immutable for the lifetime of an active handle. BAR and aperture fields are
 * retained as numeric evidence for a future, separately-authorized adapter.
 */
struct pcie1_bar_lease_mapping {
    u64 bar_pci_base;
    u64 bar_size;
    u64 cpu_base;
    u64 aperture_pci_base;
    u64 aperture_length;
    u32 bar_is_64;
    u32 attribute;
    u32 reservation_count;
    u32 _reserved;
    u8 _pad[8U];
} ALIGNED(64);

struct pcie1_bar_lease {
    struct pcie1_bar_lease_control control;
    struct pcie1_bar_lease_mapping mapping;
    struct pcie1_bar_lease_reservation
        reservations[PCIE1_BAR_LEASE_RESERVATION_CAPACITY];
} ALIGNED(64);

_Static_assert(sizeof(struct pcie1_bar_lease_handle) == 32U,
               "PCIe1 BAR handle must carry full numeric authority");
_Static_assert(sizeof(struct pcie1_bar_lease_control) == 64U,
               "PCIe1 BAR control must own exactly one cache line");
_Static_assert(sizeof(struct pcie1_bar_lease_mapping) == 64U,
               "PCIe1 BAR mapping must have cache-line stride");
_Static_assert(sizeof(struct pcie1_bar_lease_reservation) == 64U,
               "PCIe1 reservations must have cache-line stride");
_Static_assert(__builtin_offsetof(struct pcie1_bar_lease, mapping) == 64U,
               "PCIe1 BAR mapping must not share the mutable control line");
_Static_assert(__builtin_offsetof(struct pcie1_bar_lease, reservations) ==
               128U, "PCIe1 reservations must not share mapping payload");

/*
 * Initialization is one-shot over fresh zeroed storage. Exactly one endpoint
 * may hold this contract at once; release revokes its mapping authority and
 * generation-bumps before a subsequent lease can be acquired.
 */
bool pcie1_bar_lease_init(struct pcie1_bar_lease *lease, u32 contract_id,
                          u32 caller_core);
bool pcie1_bar_lease_acquire(
    struct pcie1_bar_lease *lease, u32 bdf, u32 caller_core,
    const struct pcie1_bar_lease_request *request,
    struct pcie1_bar_lease_handle *handle_out);
bool pcie1_bar_lease_release(
    struct pcie1_bar_lease *lease,
    const struct pcie1_bar_lease_handle *handle, u32 caller_core);
bool pcie1_bar_lease_revoke(
    struct pcie1_bar_lease *lease,
    const struct pcie1_bar_lease_handle *handle, u32 caller_core,
    enum pcie1_bar_lease_fault fault);
bool pcie1_bar_lease_mapping_get(
    const struct pcie1_bar_lease *lease,
    const struct pcie1_bar_lease_handle *handle, u32 caller_core,
    struct pcie1_bar_lease_mapping *mapping_out);
bool pcie1_bar_lease_state_get(
    const struct pcie1_bar_lease *lease, u32 caller_core,
    enum pcie1_bar_lease_state *state_out,
    enum pcie1_bar_lease_fault *fault_out);

/* Always false: Memory Space and Bus Master remain disabled offline. */
bool pcie1_bar_lease_hardware_enable_allowed(
    const struct pcie1_bar_lease *lease,
    const struct pcie1_bar_lease_handle *handle, u32 caller_core);
