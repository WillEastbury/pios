/*
 * hevc_contract.h - ADR-056 offline HEVC request and lifetime contract.
 *
 * This is original PIOS metadata, not an H.265 bitstream or hardware ABI.
 * It validates numeric authority and ownership only; it never interprets
 * compressed input, maps memory, or enables a device.
 */
#pragma once
#include "types.h"
#include "media_engine_contract.h"

#define HEVC_REQUEST_LAYOUT_V1        1U
#define HEVC_JOB_CAPACITY             4U
#define HEVC_FRAME_CAPACITY           16U
#define HEVC_MAX_SLICES               16U
#define HEVC_MAX_ENTRY_OFFSETS        64U
#define HEVC_DMA_ADDRESS_BITS         36U
#define HEVC_DMA_ADDRESS_LIMIT        (1ULL << HEVC_DMA_ADDRESS_BITS)
/* Conservative PIOS admission ceiling, independent of a hardware limit. */
#define HEVC_MAX_CTB_COUNT            65536U

enum hevc_span_access {
    HEVC_DEVICE_READ = 1U << 0,
    HEVC_DEVICE_WRITE = 1U << 1,
};

enum hevc_chroma_format {
    HEVC_CHROMA_INVALID = 0,
    HEVC_CHROMA_420 = 1,
};

enum hevc_scaling_mode {
    HEVC_SCALING_INVALID = 0,
    HEVC_SCALING_DEFAULT = 1,
};

enum hevc_slice_type {
    HEVC_SLICE_INVALID = 0,
    HEVC_SLICE_I = 1,
    HEVC_SLICE_P = 2,
    HEVC_SLICE_B = 3,
};

enum hevc_job_state {
    HEVC_JOB_FREE = 0,
    HEVC_JOB_PREPARED,
    HEVC_JOB_IN_FLIGHT,
    HEVC_JOB_COMPLETE,
    HEVC_JOB_FAILED,
    HEVC_JOB_RELEASED,
};

enum hevc_frame_state {
    HEVC_FRAME_FREE = 0,
    HEVC_FRAME_CAPTURE_READY,
    HEVC_FRAME_PRODUCING,
    HEVC_FRAME_REFERENCEABLE,
    HEVC_FRAME_RELEASED,
};

/*
 * Numeric authority only. `allocation_offset` names the start of the
 * declared window and `used` names the device-use prefix of that window.
 * No CPU address or mapping authority is represented here.
 */
struct hevc_dma_span {
    u64 allocation_id;
    u64 dma_allocation_base;
    u64 allocation_capacity;
    u64 allocation_offset;
    u64 window_capacity;
    u64 used;
    u64 output_canary;
    u32 allocation_generation;
    u32 access;
    u32 _reserved0;
    u32 _reserved1;
};

struct hevc_picture_info {
    u32 width;
    u32 height;
    u32 ctb_count;
    u32 chroma_format;
    u32 luma_bit_depth;
    u32 chroma_bit_depth;
    u32 scaling_mode;
    u32 syntax_flags;
    u32 _reserved;
};

/*
 * Full generations are deliberately out-of-band from the compact slot token:
 * no lifecycle check may rely on truncated generation bits.
 */
struct hevc_frame_handle {
    u64 _token;
    u64 _generation;
    u32 _controller_id;
    u32 _reserved;
};

/*
 * Source ranges are numerical offsets into the one opaque compressed span.
 * They deliberately describe no compressed-stream syntax.
 */
struct hevc_slice {
    u64 source_byte_offset;
    u64 bit_length;
    u32 ctb_address;
    u32 slice_type;
    struct hevc_frame_handle references[HEVC_FRAME_CAPACITY];
    u32 reference_count;
    u32 _reserved;
};

struct hevc_job_handle {
    u64 _token;
    u64 _generation;
    u32 _controller_id;
    u32 _reserved;
};

/*
 * V1 has exactly one opaque source and one registered capture output. Entry
 * offsets are reserved for a future layout, so both the reference count and
 * entry-offset count must be zero now. References use only inline
 * generation-safe handles; the I-only V1 policy rejects them.
 */
struct hevc_request {
    u32 layout_version;
    struct hevc_picture_info picture;
    const struct hevc_slice *slices;
    u32 slice_count;
    const u32 *entry_offsets;
    u32 entry_offset_count;
    u32 _reserved;
    struct hevc_dma_span source;
    struct hevc_dma_span capture;
    struct hevc_frame_handle capture_frame;
};

struct hevc_job_control {
    u64 generation;
    u32 state;
    u32 canary_intact;
    u32 output_slot;
    u32 _reserved0;
    u64 output_generation;
    u8 _reserved[32U];
} ALIGNED(64);

_Static_assert(sizeof(struct hevc_job_control) == 64U,
               "HEVC job control must own exactly one cache line");

struct hevc_frame_control {
    u64 generation;
    u32 state;
    u32 job_users;
    u32 producing_jobs;
    u32 reference_users;
    u8 _reserved[40U];
} ALIGNED(64);

_Static_assert(sizeof(struct hevc_frame_control) == 64U,
               "HEVC frame control must own exactly one cache line");

struct hevc_slice_payload {
    u64 source_byte_offset;
    u64 bit_length;
    u32 ctb_address;
    u32 slice_type;
};

/*
 * Private payloads are copied before PREPARED is published. They contain no
 * caller pointer, preserving the contract if a request array is later reused.
 */
struct hevc_job_payload {
    struct hevc_picture_info picture;
    struct hevc_slice_payload slices[HEVC_MAX_SLICES];
    struct hevc_dma_span source;
    struct hevc_dma_span capture;
    u64 output_canary;
    u32 slice_count;
    u32 _reserved;
} ALIGNED(64);

struct hevc_frame_payload {
    struct hevc_dma_span capture;
} ALIGNED(64);

struct hevc_contract_owner {
    u32 controller_id;
    u8 _reserved[60U];
} ALIGNED(64);

struct hevc_contract {
    struct hevc_contract_owner owner;
    struct hevc_job_control jobs[HEVC_JOB_CAPACITY];
    struct hevc_frame_control frames[HEVC_FRAME_CAPACITY];
    struct hevc_job_payload job_payloads[HEVC_JOB_CAPACITY];
    struct hevc_frame_payload frame_payloads[HEVC_FRAME_CAPACITY];
} ALIGNED(64);

_Static_assert(__builtin_offsetof(struct hevc_contract, jobs) == 64U,
               "HEVC job controls require a dedicated cache-line region");
_Static_assert(__builtin_offsetof(struct hevc_contract, frames) ==
               64U + HEVC_JOB_CAPACITY * 64U,
               "HEVC frame controls require a dedicated cache-line region");
_Static_assert(__builtin_offsetof(struct hevc_contract, job_payloads) ==
               64U + (HEVC_JOB_CAPACITY + HEVC_FRAME_CAPACITY) * 64U,
               "HEVC immutable payloads must follow mutable controls");

bool hevc_contract_init(struct hevc_contract *contract, u32 controller_id);

bool hevc_contract_register_capture(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_dma_span *capture,
    struct hevc_frame_handle *frame_out);

/*
 * The caller retains its request arrays. This contract copies accepted
 * metadata and retains no caller pointer. The media lease remains
 * caller-owned and must stay active through job/frame release.
 */
bool hevc_contract_prepare(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_request *request,
    struct hevc_job_handle *job_out);

bool hevc_contract_mark_in_flight(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_job_handle *job);

/*
 * `success` controls COMPLETE versus FAILED. A successful retained output
 * becomes referenceable only after its capture canary is independently
 * reported; failed/aborted jobs take the strict failed-release path.
 */
bool hevc_contract_complete(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_job_handle *job, bool success, bool retain);

bool hevc_contract_report_output_canary(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_job_handle *job, u64 observed_canary);

bool hevc_contract_abort(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_job_handle *job);

bool hevc_contract_release_job(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_job_handle *job);

bool hevc_contract_release_frame(
    struct hevc_contract *contract,
    const struct media_engine_controller *media_controller,
    const struct media_engine_lease *engine_lease,
    const struct hevc_frame_handle *frame);

bool hevc_contract_job_state_get(const struct hevc_contract *contract,
                                 const struct hevc_job_handle *job,
                                 enum hevc_job_state *state_out);
bool hevc_contract_frame_state_get(const struct hevc_contract *contract,
                                   const struct hevc_frame_handle *frame,
                                   enum hevc_frame_state *state_out);
bool hevc_contract_picture_get(const struct hevc_contract *contract,
                               const struct hevc_job_handle *job,
                               struct hevc_picture_info *picture_out);
bool hevc_contract_slice_get(const struct hevc_contract *contract,
                             const struct hevc_job_handle *job, u32 index,
                             struct hevc_slice_payload *slice_out);
