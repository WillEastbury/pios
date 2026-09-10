/*
 * pisp_be_contract.h - ADR-055 offline PiSP-BE request ownership contract.
 *
 * This module validates only bounded PIOS request metadata. It does not
 * identify hardware, map addresses, perform DMA, or interpret PiSP tiles.
 */
#pragma once
#include "types.h"
#include "media_engine_contract.h"

#define PISP_BE_REQUEST_LAYOUT_V1            1U
#define PISP_BE_CONFIG_BYTES                 16720U
#define PISP_BE_CONFIG_INLINE_TILES_OFFSET   6476U
#define PISP_BE_CONFIG_INLINE_TILE_COUNT     64U
#define PISP_BE_CONFIG_INLINE_TILE_BYTES     160U
#define PISP_BE_CONFIG_NUM_TILES_OFFSET      16716U
#define PISP_BE_CONFIG_GLOBAL_BAYER_OFFSET   112U
#define PISP_BE_CONFIG_GLOBAL_RGB_OFFSET     116U
#define PISP_BE_CONFIG_ROOT_BIT              1U
#define PISP_BE_DMA_ADDRESS_BITS             36U
#define PISP_BE_DMA_ADDRESS_LIMIT            (1ULL << PISP_BE_DMA_ADDRESS_BITS)
#define PISP_BE_MAX_SPANS                    14U
#define PISP_BE_JOB_CAPACITY                 4U

_Static_assert(PISP_BE_CONFIG_NUM_TILES_OFFSET + sizeof(u32) ==
               PISP_BE_CONFIG_BYTES,
               "PiSP BE num_tiles must be the final u32");
_Static_assert(PISP_BE_CONFIG_INLINE_TILES_OFFSET +
               PISP_BE_CONFIG_INLINE_TILE_COUNT *
               PISP_BE_CONFIG_INLINE_TILE_BYTES == PISP_BE_CONFIG_NUM_TILES_OFFSET,
               "PiSP BE inline tile region must end before num_tiles");

enum pisp_be_span_access {
    PISP_BE_DEVICE_READ = 1U << 0,
    PISP_BE_DEVICE_WRITE = 1U << 1,
};

enum pisp_be_span_role {
    PISP_BE_SPAN_INVALID = 0,
    PISP_BE_SPAN_CONFIG,
    PISP_BE_SPAN_MAIN_INPUT,
    PISP_BE_SPAN_AUX_INPUT,
    PISP_BE_SPAN_OUTPUT,
};

enum pisp_be_job_state {
    PISP_BE_JOB_FREE = 0,
    PISP_BE_JOB_PREPARED,
    PISP_BE_JOB_IN_FLIGHT,
    PISP_BE_JOB_COMPLETE,
    PISP_BE_JOB_FAILED,
    PISP_BE_JOB_RELEASED,
};

/*
 * `layout_version` describes this PIOS request API only. The supplied raw
 * PiSP configuration byte sequence is not self-describing; unknown PIOS API
 * versions reject before it is copied.
 */
struct pisp_be_request {
    u32 layout_version;
    const void *config_bytes;
    u32 config_bytes_len;
    u32 _reserved;
};

/*
 * Numeric DMA authority only: no CPU pointer crosses this boundary.
 * A repeated (allocation_id, allocation_generation) denotes the same
 * allocation and is permitted only when base/capacity agree and declared
 * allocation windows do not overlap. All actual device-use intervals must
 * also be pairwise non-overlapping, including across allocation identities.
 * `output_canary` is required and nonzero only for output spans.
 */
struct pisp_be_dma_span {
    u64 allocation_id;
    u64 dma_allocation_base;
    u64 allocation_capacity;
    u64 allocation_offset;
    u64 window_capacity;
    u64 used;
    u64 output_canary;
    u32 allocation_generation;
    u32 access;
    u32 logical_role;
    u32 _reserved;
};

struct pisp_be_output_canary {
    u32 span_index;
    u32 _reserved;
    u64 observed_canary;
};

/*
 * Handles are generation-backed capabilities, never pointers. The controller
 * ID binds a job to both this contract instance and its media controller.
 */
struct pisp_be_job_handle {
    u64 _token;
    u64 _generation;
    u32 _controller_id;
    u32 _reserved;
};

/* Mutable state is isolated one job per cache line. */
struct pisp_be_job_control {
    u64 generation;
    u32 state;
    u32 canaries_intact;
    u8 _reserved[48U];
} ALIGNED(64);

_Static_assert(sizeof(struct pisp_be_job_control) == 64U,
               "PiSP BE job control must own exactly one cache line");

/*
 * This payload is private to the contract and immutable after PREPARED
 * publication. Its config is a copy, never the caller's raw pointer.
 */
struct pisp_be_job_payload {
    u8 config[PISP_BE_CONFIG_BYTES];
    struct pisp_be_dma_span spans[PISP_BE_MAX_SPANS];
    u32 span_count;
    u32 _reserved;
} ALIGNED(64);

struct pisp_be_contract_owner {
    u32 controller_id;
    u8 _reserved[60U];
} ALIGNED(64);

struct pisp_be_contract {
    struct pisp_be_contract_owner owner;
    struct pisp_be_job_control jobs[PISP_BE_JOB_CAPACITY];
    struct pisp_be_job_payload payloads[PISP_BE_JOB_CAPACITY];
} ALIGNED(64);

_Static_assert(__builtin_offsetof(struct pisp_be_contract, jobs) == 64U,
               "PiSP BE controls require a dedicated cache-line region");
_Static_assert(__builtin_offsetof(struct pisp_be_contract, payloads) ==
               64U + PISP_BE_JOB_CAPACITY * 64U,
               "PiSP BE payload must be separate from mutable controls");

/*
 * `controller_id` must be nonzero and unique among concurrently active
 * contracts. Job handles are bound to both this identity and a full generation.
 */
bool pisp_be_contract_init(struct pisp_be_contract *contract, u32 controller_id);

/*
 * Validation is complete before the exact config copy occurs. The caller
 * retains ownership of both input arrays and may change them after return.
 * The passed engine lease remains caller-owned and must remain active until
 * this job is released or aborted then released.
 */
bool pisp_be_contract_prepare(
    struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct pisp_be_request *request,
    const struct pisp_be_dma_span *spans, u32 span_count,
    struct pisp_be_job_handle *handle_out);

bool pisp_be_contract_mark_in_flight(
    struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct pisp_be_job_handle *handle);
bool pisp_be_contract_complete(
    struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct pisp_be_job_handle *handle);
bool pisp_be_contract_report_output_canaries(
    struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct pisp_be_job_handle *handle,
    const struct pisp_be_output_canary *reports, u32 report_count);
bool pisp_be_contract_abort(
    struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct pisp_be_job_handle *handle);
bool pisp_be_contract_release(
    struct pisp_be_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct pisp_be_job_handle *handle);

bool pisp_be_contract_state_get(
    const struct pisp_be_contract *contract,
    const struct pisp_be_job_handle *handle,
    enum pisp_be_job_state *state_out);

/* Safe private-copy inspection for tests and diagnostics; no payload pointer. */
bool pisp_be_contract_config_byte(
    const struct pisp_be_contract *contract,
    const struct pisp_be_job_handle *handle, u32 offset, u8 *byte_out);
