/*
 * rp1_cfe_contract.h - offline RP1 CFE / PiSP-FE resource and stream contract.
 *
 * This is a passive, hardware-disabled contract. Resource facts describe a
 * Linux-source layout only; they neither map nor authorize PIOS hardware.
 */
#pragma once
#include "types.h"

#define RP1_CFE_SOURCE_REVISION_BYTES       20U
#define RP1_CFE_LAYOUT_ID_V1                1U
#define RP1_CFE_INSTANCE_COUNT              2U
#define RP1_CFE_WINDOW_COUNT                4U
#define RP1_CFE_STREAM_CAPACITY             2U
#define RP1_CFE_MAX_SINK_SPANS              3U
#define RP1_CFE_STREAM_LAYOUT_V1            1U
#define RP1_CFE_MAX_DIMENSION               8192U
#define RP1_CFE_MAX_STRIDE                  16384U
#define RP1_CFE_MAX_CONFIG_BYTES            65536U
#define RP1_CFE_SINK_ALIGNMENT              16U

enum rp1_cfe_resource_window_name {
    RP1_CFE_WINDOW_CSI_DMA = 0,
    RP1_CFE_WINDOW_CSI_HOST,
    RP1_CFE_WINDOW_MIPI_CFG,
    RP1_CFE_WINDOW_PISP_FE,
};

enum rp1_cfe_resource_fact {
    RP1_CFE_FACT_WINDOWS = 1U << 0,
    RP1_CFE_FACT_CLOCK = 1U << 1,
    RP1_CFE_FACT_RP1_SOURCE = 1U << 2,
    RP1_CFE_FACT_LINUX_IOMMU_ATTACHMENT = 1U << 3,
};

/*
 * These bits intentionally remain absent from every V1 profile. In
 * particular, a Linux IRQ routing derivation or IOMMU attachment does not
 * establish a PIOS IRQ, translation, IOMMU, DMA, sensor, or MIPI claim.
 * No CFE-local reset binding is published here; that absence does not prove a
 * future active backend will not require one. MIPI_CFG is shared with DSI in
 * its island, so an active backend additionally requires its proof below.
 */
enum rp1_cfe_active_property {
    RP1_CFE_ACTIVE_CPU_MMIO = 1U << 0,
    RP1_CFE_ACTIVE_GIC_ROUTE = 1U << 1,
    RP1_CFE_ACTIVE_PIOS_IOMMU = 1U << 2,
    RP1_CFE_ACTIVE_DMA = 1U << 3,
    RP1_CFE_ACTIVE_SENSOR_PROFILE = 1U << 4,
    RP1_CFE_ACTIVE_MIPI_EXCLUSIVITY = 1U << 5,
};

enum rp1_cfe_activation_proof {
    RP1_CFE_PROOF_TRANSLATION = 1U << 0,
    RP1_CFE_PROOF_IRQ_ROUTE = 1U << 1,
    RP1_CFE_PROOF_PIOS_IOMMU = 1U << 2,
    RP1_CFE_PROOF_SENSOR_PROFILE = 1U << 3,
    RP1_CFE_PROOF_MIPI_EXCLUSIVITY = 1U << 4,
};

#define RP1_CFE_V1_MISSING_PROOFS \
    (RP1_CFE_PROOF_TRANSLATION | RP1_CFE_PROOF_IRQ_ROUTE | \
     RP1_CFE_PROOF_PIOS_IOMMU | RP1_CFE_PROOF_SENSOR_PROFILE | \
     RP1_CFE_PROOF_MIPI_EXCLUSIVITY)

enum rp1_cfe_activation_reason {
    RP1_CFE_ACTIVATION_REASON_NONE = 0,
    RP1_CFE_ACTIVATION_REASON_INVALID_HANDLE,
    RP1_CFE_ACTIVATION_REASON_HARDWARE_DISABLED_V1,
};

struct rp1_cfe_bus_window {
    u32 name;
    u32 _reserved;
    u64 rp1_bus_base;
    u64 bytes;
};

/*
 * `source_revision` is the 20-byte Git object ID
 * cff533aec2fa601846766b32ff57204e0a61bed7. `linux_iommu_attachment` is
 * source provenance only, not a PIOS IOMMU resource ID.
 */
struct rp1_cfe_resource_profile {
    u8 source_revision[RP1_CFE_SOURCE_REVISION_BYTES];
    u32 layout_id;
    u32 instance;
    struct rp1_cfe_bus_window windows[RP1_CFE_WINDOW_COUNT];
    u32 clock_id;
    u32 clock_rate_hz;
    u32 rp1_source;
    u32 linux_iommu_attachment;
    u32 known_fact_mask;
    u32 active_known_mask;
    u32 active_proven_mask;
    u32 access_authorized_mask;
};

enum rp1_cfe_pixel_format {
    RP1_CFE_FORMAT_INVALID = 0,
    RP1_CFE_FORMAT_RAW8 = 1,
    RP1_CFE_FORMAT_RAW10 = 2,
};

/*
 * This is numeric request metadata, not a PiSP configuration blob. The
 * config identity/revision/length can identify an external future format but
 * grants no activation authority and V1 retains no configuration pointer.
 * The connector identity is deliberately required rather than inferred:
 * sensor power, MCLK, I2C, and lane details vary with the selected overlay.
 */
struct rp1_cfe_stream_descriptor {
    u32 layout_version;
    u32 sensor_connector_profile_id;
    u64 sensor_connector_profile_generation;
    u32 instance;
    u32 source_stream_id;
    u64 source_stream_generation;
    u32 width;
    u32 height;
    u32 stride;
    u32 pixel_format;
    u32 lane_count;
    u32 config_identity;
    u32 config_revision;
    u32 config_length;
    u32 config_flags;
    u32 _reserved[3];
};

enum rp1_cfe_sink_role {
    RP1_CFE_SINK_INVALID = 0,
    RP1_CFE_SINK_OUTPUT0 = 1,
    RP1_CFE_SINK_OUTPUT1 = 2,
    RP1_CFE_SINK_STATS = 3,
};

enum rp1_cfe_sink_access {
    RP1_CFE_SINK_DEVICE_WRITE = 1U << 0,
};

/*
 * Numeric allocation authority only. Neither a CPU virtual address nor a
 * hardware DMA-width assertion crosses this API.
 */
struct rp1_cfe_sink_span {
    u64 allocation_id;
    u64 allocation_generation;
    u64 allocation_base;
    u64 allocation_capacity;
    u64 allocation_offset;
    u64 window_capacity;
    u64 used;
    u64 output_canary;
    u32 role;
    u32 access;
    u32 _reserved;
    u32 _reserved2;
};

enum rp1_cfe_stream_state {
    RP1_CFE_STREAM_FREE = 0,
    RP1_CFE_STREAM_PREPARED,
    RP1_CFE_STREAM_FAILED,
    RP1_CFE_STREAM_RELEASED,
};

struct rp1_cfe_stream_handle {
    u64 token;
    u64 generation;
    u32 controller_id;
    u32 _reserved;
};

/* Each mutable stream record exclusively owns one cache line. */
struct rp1_cfe_stream_control {
    u64 generation;
    u32 state;
    u32 controller_id;
    u32 instance;
    u32 prepare_count;
    u32 failure_count;
    u32 release_count;
    u32 _reserved0;
    u8 _reserved[28U];
} ALIGNED(64);

_Static_assert(sizeof(struct rp1_cfe_stream_control) == 64U,
               "RP1 CFE mutable stream control must own one cache line");

/* Private payload data is separate from cache-line-isolated mutable control. */
struct rp1_cfe_stream_payload {
    struct rp1_cfe_stream_descriptor descriptor;
    struct rp1_cfe_sink_span sinks[RP1_CFE_MAX_SINK_SPANS];
    u32 sink_count;
    u32 _reserved;
} ALIGNED(64);

struct rp1_cfe_contract_owner {
    u32 controller_id;
    u32 initialized;
    u8 _reserved[56U];
} ALIGNED(64);

struct rp1_cfe_contract {
    struct rp1_cfe_contract_owner owner;
    struct rp1_cfe_stream_control controls[RP1_CFE_STREAM_CAPACITY];
    struct rp1_cfe_stream_payload payloads[RP1_CFE_STREAM_CAPACITY];
} ALIGNED(64);

_Static_assert(__builtin_offsetof(struct rp1_cfe_contract, controls) == 64U,
               "RP1 CFE controls require a dedicated cache-line region");
_Static_assert(__builtin_offsetof(struct rp1_cfe_contract, payloads) ==
               64U + RP1_CFE_STREAM_CAPACITY * 64U,
               "RP1 CFE payloads must not share mutable control cache lines");
_Static_assert((sizeof(struct rp1_cfe_contract) % 64U) == 0U,
               "RP1 CFE contract must retain cache-line stride");

struct rp1_cfe_stream_snapshot {
    struct rp1_cfe_stream_descriptor descriptor;
    struct rp1_cfe_sink_span sinks[RP1_CFE_MAX_SINK_SPANS];
    u32 sink_count;
    u32 _reserved;
};

struct rp1_cfe_stream_status {
    u64 generation;
    u32 state;
    u32 controller_id;
    u32 instance;
    u32 prepare_count;
    u32 failure_count;
    u32 release_count;
    u32 _reserved;
};

/* Invalid instances return NULL. These immutable facts do not authorize I/O. */
const struct rp1_cfe_resource_profile *
rp1_cfe_resource_profile_get(u32 instance);

/*
 * A supplementary source-fact gate. It has no access side effects and always
 * reports no active authorization in V1.
 */
bool rp1_cfe_source_gate(const struct rp1_cfe_resource_profile *profile,
                         u32 *known_facts_out, u32 *active_proofs_out);

/*
 * `contract` must be freshly all-zero storage. `controller_id` is a nonzero
 * domain-unique caller-provided identity; initialization is one-shot.
 */
bool rp1_cfe_contract_init(struct rp1_cfe_contract *contract, u32 controller_id);

bool rp1_cfe_contract_prepare(struct rp1_cfe_contract *contract,
                              const struct rp1_cfe_stream_descriptor *descriptor,
                              const struct rp1_cfe_sink_span *sinks, u32 sink_count,
                              struct rp1_cfe_stream_handle *handle_out);

/*
 * There is deliberately no activate or complete operation. This query never
 * changes state and always rejects V1 with all required proofs missing.
 */
bool rp1_cfe_activation_permitted(
    const struct rp1_cfe_contract *contract,
    const struct rp1_cfe_stream_handle *handle,
    u32 *missing_proofs_out, enum rp1_cfe_activation_reason *reason_out);

/* Administrative failure attestation only: PREPARED -> FAILED -> RELEASED. */
bool rp1_cfe_contract_report_failure(struct rp1_cfe_contract *contract,
                                     const struct rp1_cfe_stream_handle *handle);
bool rp1_cfe_contract_release(struct rp1_cfe_contract *contract,
                              const struct rp1_cfe_stream_handle *handle);

bool rp1_cfe_contract_status_get(const struct rp1_cfe_contract *contract,
                                 const struct rp1_cfe_stream_handle *handle,
                                 struct rp1_cfe_stream_status *status_out);
bool rp1_cfe_contract_snapshot_get(const struct rp1_cfe_contract *contract,
                                   const struct rp1_cfe_stream_handle *handle,
                                   struct rp1_cfe_stream_snapshot *snapshot_out);
