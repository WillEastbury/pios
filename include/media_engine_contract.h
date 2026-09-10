/*
 * media_engine_contract.h - ADR-054 passive resource facts and ownership.
 *
 * This is an offline contract only.  It neither maps nor touches hardware,
 * configures clocks, routes interrupts, or authorizes DMA buffers.
 */
#pragma once
#include "types.h"

#define MEDIA_ENGINE_MAX_CLOCKS  2U

enum media_engine_kind {
    MEDIA_ENGINE_INVALID = 0,
    MEDIA_ENGINE_HEVC = 1,
    MEDIA_ENGINE_PISP_BE = 2,
    MEDIA_ENGINE_PISP_FE = 3,
    MEDIA_ENGINE_HVS = 4,
    MEDIA_ENGINE_COUNT = 4,
};

enum media_engine_known {
    MEDIA_ENGINE_KNOWN_MMIO0   = 1U << 0,
    MEDIA_ENGINE_KNOWN_MMIO1   = 1U << 1,
    MEDIA_ENGINE_KNOWN_VERSION = 1U << 2,
    MEDIA_ENGINE_KNOWN_CLOCKS  = 1U << 3,
    MEDIA_ENGINE_KNOWN_GIC_SPI = 1U << 4,
    MEDIA_ENGINE_KNOWN_IOMMU   = 1U << 5,
};

enum media_engine_capability {
    MEDIA_ENGINE_CAP_PASSIVE_IDENTIFY = 1U << 0,
    MEDIA_ENGINE_CAP_ACTIVE_LEASE      = 1U << 1,
    MEDIA_ENGINE_CAP_DMA_EXCLUSIVE     = 1U << 2,
    MEDIA_ENGINE_CAP_PASSIVE_ONLY      = 1U << 3,
    MEDIA_ENGINE_CAP_INCOMPLETE        = 1U << 4,
};

enum media_engine_version_match {
    MEDIA_ENGINE_VERSION_NONE = 0,
    MEDIA_ENGINE_VERSION_EXACT,
    MEDIA_ENGINE_VERSION_MASKED,
    MEDIA_ENGINE_VERSION_LOW_BYTE_53_OR_54,
};

enum media_engine_state {
    MEDIA_ENGINE_DISABLED = 0,
    MEDIA_ENGINE_PASSIVELY_IDENTIFIED,
    MEDIA_ENGINE_LEASED,
    MEDIA_ENGINE_COMPLETED,
    MEDIA_ENGINE_RELEASED,
    MEDIA_ENGINE_QUARANTINED,
};

struct media_engine_mmio_window {
    u64 cpu_phys_base;
    u32 bytes;
    u32 _reserved;
};

/*
 * Every optional numeric property is guarded by known_mask.  In particular,
 * a zero field never means that an unapproved resource has address zero.
 */
struct media_engine_descriptor {
    enum media_engine_kind kind;
    const char *name;
    u32 known_mask;
    u32 capabilities;
    struct media_engine_mmio_window mmio[2];
    u32 version_offset;
    u32 version_mask;
    u32 version_value;
    enum media_engine_version_match version_match;
    u32 clock_ids[MEDIA_ENGINE_MAX_CLOCKS];
    u32 clock_count;
    u32 gic_spi;
    u32 iommu_resource;
};

/*
 * A lease is an opaque generation-backed capability, never a controller or
 * hardware pointer.  Only this module creates or validates its token.
 */
struct media_engine_lease {
    u64 _token;
    u32 _controller_id;
    u32 _reserved;
};

/*
 * Controller identity is supplied by its owner and binds every issued lease
 * to this instance. One engine's mutable ownership state occupies precisely
 * one cache line. Future cross-core wiring must use a message-passing owner
 * rather than mutate these records remotely.
 */
struct media_engine_controller_owner {
    u32 controller_id;
    u8 _reserved[60U];
} ALIGNED(64);

_Static_assert(sizeof(struct media_engine_controller_owner) == 64U,
               "media engine controller owner must own exactly one cache line");

/*
 * One engine's mutable ownership state occupies precisely one cache line.
 */
struct media_engine_owner_record {
    u32 generation;
    u32 lease_generation;
    u32 state;
    u32 identified_version;
    u8 _reserved[48U];
} ALIGNED(64);

_Static_assert(sizeof(struct media_engine_owner_record) == 64U,
               "media engine owner record must own exactly one cache line");

/* IOMMU resource 2 has separate cache-line ownership from engine state. */
struct media_engine_iommu2_owner {
    u32 kind;
    u32 generation;
    u8 _reserved[56U];
} ALIGNED(64);

_Static_assert(sizeof(struct media_engine_iommu2_owner) == 64U,
               "IOMMU2 owner must own exactly one cache line");

struct media_engine_controller {
    struct media_engine_controller_owner owner;
    struct media_engine_owner_record engines[MEDIA_ENGINE_COUNT];
    struct media_engine_iommu2_owner iommu2;
} ALIGNED(64);

_Static_assert((sizeof(struct media_engine_controller) % 64U) == 0U,
               "media engine controller must retain cache-line record stride");
_Static_assert(__builtin_offsetof(struct media_engine_controller, iommu2) ==
               (MEDIA_ENGINE_COUNT + 1U) * 64U,
               "IOMMU2 ownership must start on its own cache line");

const struct media_engine_descriptor *
media_engine_descriptor(enum media_engine_kind kind);

bool media_engine_version_matches(enum media_engine_kind kind, u32 raw_version);

/*
 * `controller_id` is a nonzero stable identity unique among concurrently
 * active controllers. Every lease is bound to it as well as to its generation.
 */
bool media_engine_controller_init(struct media_engine_controller *controller,
                                  u32 controller_id);

bool media_engine_passive_identify(struct media_engine_controller *controller,
                                   enum media_engine_kind kind, u32 raw_version);

/*
 * A lease only records exclusive ownership.  It carries no DMA address or
 * buffer authority; an approved mapping contract is required before DMA.
 */
bool media_engine_lease_acquire(struct media_engine_controller *controller,
                                enum media_engine_kind kind,
                                struct media_engine_lease *lease_out);
bool media_engine_lease_complete(struct media_engine_controller *controller,
                                 const struct media_engine_lease *lease);
bool media_engine_lease_release(struct media_engine_controller *controller,
                                const struct media_engine_lease *lease);
bool media_engine_lease_abort(struct media_engine_controller *controller,
                              const struct media_engine_lease *lease);

/*
 * Read-only capability check for contracts layered above engine ownership.
 * It exposes neither the owner record nor a mutation path. A true result
 * means `lease` is current and the specified engine remains actively leased.
 */
bool media_engine_lease_active_for(
    const struct media_engine_controller *controller,
    const struct media_engine_lease *lease, enum media_engine_kind kind);

/* Quarantine can only be left through this explicit administrative action. */
bool media_engine_rearm(struct media_engine_controller *controller,
                        enum media_engine_kind kind);

bool media_engine_state_get(const struct media_engine_controller *controller,
                            enum media_engine_kind kind,
                            enum media_engine_state *state_out);
