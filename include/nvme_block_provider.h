/*
 * nvme_block_provider.h - offline, callback-backed block-provider contract.
 *
 * This models exactly one already-validated namespace. It is deliberately
 * independent from controller discovery and storage consumers.
 */
#pragma once

#include "types.h"

#define NVME_BLOCK_PROVIDER_OWNER_CORE       0U
#define NVME_BLOCK_PROVIDER_MAGIC            0x4E564D45424C4B31ULL
#define NVME_BLOCK_PROVIDER_HANDLE_MAGIC     0x4E564D45484E4431ULL
#define NVME_BLOCK_PROVIDER_BLOCK_BYTES_MIN  512U
#define NVME_BLOCK_PROVIDER_BLOCK_BYTES_MAX  65536U

enum nvme_block_provider_state {
    NVME_BLOCK_PROVIDER_EMPTY = 0U,
    NVME_BLOCK_PROVIDER_ACTIVE,
    NVME_BLOCK_PROVIDER_QUARANTINED,
    NVME_BLOCK_PROVIDER_REMOVED,
    NVME_BLOCK_PROVIDER_RETIRED,
};

enum nvme_block_provider_fault {
    NVME_BLOCK_PROVIDER_FAULT_NONE = 0U,
    NVME_BLOCK_PROVIDER_FAULT_CALLBACK,
    NVME_BLOCK_PROVIDER_FAULT_PARTIAL,
    NVME_BLOCK_PROVIDER_FAULT_TIMEOUT,
    NVME_BLOCK_PROVIDER_FAULT_AER,
    NVME_BLOCK_PROVIDER_FAULT_REMOVED,
    NVME_BLOCK_PROVIDER_FAULT_CORRUPT,
};

enum nvme_block_provider_result {
    NVME_BLOCK_PROVIDER_OK = 0U,
    NVME_BLOCK_PROVIDER_INVALID,
    NVME_BLOCK_PROVIDER_OWNER,
    NVME_BLOCK_PROVIDER_INACTIVE,
    NVME_BLOCK_PROVIDER_STALE,
    NVME_BLOCK_PROVIDER_RANGE,
    NVME_BLOCK_PROVIDER_READ_ONLY,
    NVME_BLOCK_PROVIDER_IO,
    NVME_BLOCK_PROVIDER_PARTIAL,
    NVME_BLOCK_PROVIDER_BUSY,
    NVME_BLOCK_PROVIDER_RETIRED_RESULT,
    NVME_BLOCK_PROVIDER_CORRUPT,
};

enum nvme_block_provider_revoke_reason {
    NVME_BLOCK_PROVIDER_REVOKE_TIMEOUT = 1U,
    NVME_BLOCK_PROVIDER_REVOKE_AER,
    NVME_BLOCK_PROVIDER_REVOKE_REMOVAL,
};

/*
 * Geometry is copied at initialization and is immutable for the provider
 * lifetime. `capacity_bytes` must exactly equal logical_blocks * block_bytes.
 * `instance_epoch` is a nonzero, externally allocated never-reused epoch:
 * reinitialization requires fresh zeroed provider storage and a new epoch.
 */
struct nvme_block_provider_geometry {
    u64 logical_blocks;
    u64 capacity_bytes;
    u64 namespace_generation;
    u64 instance_epoch;
    u32 namespace_id;
    u32 block_bytes;
    u32 max_transfer_blocks;
    u8 read_only;
    u8 _reserved[19U];
} ALIGNED(64);

/*
 * The callback owns no provider state. It synchronously receives exactly one
 * requested span (never more than geometry.max_transfer_blocks) and must
 * preserve bytes/capacity, set used to its exact transfer length, then return
 * true only for a completed operation.
 */
struct nvme_block_provider_span {
    u8 *bytes;
    u64 capacity;
    u64 used;
};

typedef bool (*nvme_block_provider_transfer_fn)(
    void *context, u64 lba, u32 block_count,
    struct nvme_block_provider_span *span);

struct nvme_block_provider_callbacks {
    nvme_block_provider_transfer_fn read;
    nvme_block_provider_transfer_fn write;
    void *context;
};

/* Immutable callback configuration is copied; callback context is never retained elsewhere. */
struct nvme_block_provider_backend {
    nvme_block_provider_transfer_fn read;
    nvme_block_provider_transfer_fn write;
    void *context;
    u8 _pad[40U];
} ALIGNED(64);

struct nvme_block_provider_control {
    u64 magic;
    u64 provider_generation;
    u64 geometry_fingerprint;
    u64 completed_operations;
    u32 owner_core;
    u32 state;
    u32 fault;
    u32 callback_active;
    u8 _pad[16U];
} ALIGNED(64);

struct nvme_block_provider_handle {
    u64 magic;
    u64 instance_epoch;
    u64 namespace_generation;
    u64 provider_generation;
    u32 namespace_id;
    u32 _reserved[7U];
} ALIGNED(64);

struct nvme_block_provider {
    struct nvme_block_provider_control control;
    struct nvme_block_provider_geometry geometry;
    struct nvme_block_provider_backend backend;
} ALIGNED(64);

struct nvme_block_provider_status {
    u64 instance_epoch;
    u64 namespace_generation;
    u64 provider_generation;
    u64 capacity_bytes;
    u32 namespace_id;
    u32 block_bytes;
    u32 max_transfer_blocks;
    u32 state;
    u32 fault;
    u32 owner_core;
    u8 read_only;
    u8 _pad[7U];
} ALIGNED(64);

_Static_assert(sizeof(struct nvme_block_provider_geometry) == 64U,
               "block geometry must own one cache line");
_Static_assert(sizeof(struct nvme_block_provider_backend) == 64U,
               "block callbacks must own one cache line");
_Static_assert(sizeof(struct nvme_block_provider_control) == 64U,
               "block control must own one cache line");
_Static_assert(sizeof(struct nvme_block_provider_handle) == 64U,
               "block handles must have a cache-line stride");
_Static_assert(sizeof(struct nvme_block_provider_status) == 64U,
               "block status must own one cache line");
_Static_assert(__builtin_offsetof(struct nvme_block_provider, geometry) == 64U,
               "geometry must not share mutable control storage");
_Static_assert(__builtin_offsetof(struct nvme_block_provider, backend) == 128U,
               "callback configuration must not share geometry storage");

/*
 * This synchronous contract has one mutable owner: core 0. Every public call
 * supplies that owner identity and must be serialized by its caller. Calls
 * from an IRQ or recursively from a transfer callback are forbidden; the
 * contract detects callback reentry but intentionally provides no locking.
 */
enum nvme_block_provider_result nvme_block_provider_init(
    struct nvme_block_provider *provider,
    const struct nvme_block_provider_geometry *geometry,
    const struct nvme_block_provider_callbacks *callbacks, u32 caller_core,
    struct nvme_block_provider_handle *handle_out);

enum nvme_block_provider_result nvme_block_provider_open(
    const struct nvme_block_provider *provider, u32 caller_core,
    struct nvme_block_provider_handle *handle_out);
enum nvme_block_provider_result nvme_block_provider_geometry_get(
    const struct nvme_block_provider *provider, u32 caller_core,
    struct nvme_block_provider_geometry *geometry_out);
enum nvme_block_provider_result nvme_block_provider_status_get(
    const struct nvme_block_provider *provider, u32 caller_core,
    struct nvme_block_provider_status *status_out);

enum nvme_block_provider_result nvme_block_provider_read(
    struct nvme_block_provider *provider,
    const struct nvme_block_provider_handle *handle, u32 caller_core,
    u64 lba, u32 block_count, struct nvme_block_provider_span *span);
enum nvme_block_provider_result nvme_block_provider_write(
    struct nvme_block_provider *provider,
    const struct nvme_block_provider_handle *handle, u32 caller_core,
    u64 lba, u32 block_count, struct nvme_block_provider_span *span);

/*
 * Revocation is serialized core-0 lifecycle work after any active synchronous
 * callback returns. It generation-invalidates every prior handle. At u64
 * generation exhaustion the provider is permanently retired.
 */
enum nvme_block_provider_result nvme_block_provider_revoke(
    struct nvme_block_provider *provider, u32 caller_core,
    enum nvme_block_provider_revoke_reason reason);
