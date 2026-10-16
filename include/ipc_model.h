#pragma once
#include "types.h"
#include "ipc_queue.h"

/* Management operations require all participating VMs to be quiescent.
 * Hot operations require exactly one execution owner per VM, not per core.
 * All storage must use one consistent shared Normal-NC mapping on hardware. */
#define IPC_MODEL_VMS 8U
#define IPC_MODEL_ROUTES 16U
#define IPC_MODEL_LANES 32U
#define IPC_MODEL_DEPTH 8U
#define IPC_MODEL_REQUESTS 128U
#define IPC_MODEL_ERR_STALE -9
#define IPC_MODEL_ERR_OWNER -10
#define IPC_MODEL_ERR_PHASE -11

struct ipc_model_id { u32 slot, generation; };
struct ipc_model_vm { u32 slot, generation; };
struct ipc_model_request_id { u32 slot, generation; };
struct ipc_model_lane_id { u32 slot, generation; };

/* Authority IDs are resolved by the runtime; never cross-domain pointers.
 * Request metadata occupies one immutable line, separate from owner state. */
struct ipc_model_request {
    struct ipc_model_id model;
    u32 shard;
    u32 tag;
    struct ipc_model_vm source, target;
    u32 span_id, span_generation, offset, length, capacity;
    u32 reserved[3];
} ALIGNED(64);
struct ipc_model_owner {
    u32 generation, phase;
    struct ipc_model_vm owner;
    struct ipc_model_lane_id lane;
    u32 reserved[10];
} ALIGNED(64);
struct ipc_model_slot {
    struct ipc_model_owner control;
    struct ipc_model_request request;
} ALIGNED(64);
struct ipc_model_vm_record {
    u32 generation, live, attached, cleanup_pending;
    u32 reserved[12];
} ALIGNED(64);
struct ipc_model_route {
    struct ipc_model_id model;
    u32 shard, live;
    struct ipc_model_vm target;
    u32 reserved[10];
} ALIGNED(64);
struct ipc_model_lane_config {
    u32 generation, live;
    struct ipc_model_vm source, target;
    struct ipc_model_id model;
    u32 shard;
    u32 reserved[7];
} ALIGNED(64);
struct ipc_model_cursor { u32 sequence; u32 reserved[15]; } ALIGNED(64);
struct ipc_model_entry {
    struct ipc_model_request_id request;
    u32 sequence;
    u32 reserved[13];
} ALIGNED(64);
struct ipc_model_lane {
    struct ipc_model_lane_config config;
    struct ipc_model_cursor producer;
    struct ipc_model_cursor consumer;
    struct ipc_model_entry entries[IPC_MODEL_DEPTH];
} ALIGNED(64);
struct ipc_model_registry {
    struct ipc_model_vm_record vms[IPC_MODEL_VMS];
    struct ipc_model_cursor model_generations[IPC_MODEL_ROUTES];
    struct ipc_model_route routes[IPC_MODEL_ROUTES];
    struct ipc_model_lane lanes[IPC_MODEL_LANES];
    struct ipc_model_slot requests[IPC_MODEL_REQUESTS];
} ALIGNED(64);

void ipc_model_init(struct ipc_model_registry *r);
i32 ipc_model_vm_start(struct ipc_model_registry *r, u32 slot, struct ipc_model_vm *out);
i32 ipc_model_vm_retire(struct ipc_model_registry *r, struct ipc_model_vm vm);
i32 ipc_model_bind(struct ipc_model_registry *r, struct ipc_model_id model,
                   u32 shard, struct ipc_model_vm target);
i32 ipc_model_lane_open(struct ipc_model_registry *r, struct ipc_model_vm source,
                        struct ipc_model_id model, u32 shard,
                        struct ipc_model_lane_id *out);
i32 ipc_model_request_create(struct ipc_model_registry *r, struct ipc_model_vm caller,
                             const struct ipc_model_request *metadata,
                             struct ipc_model_request_id *out);
i32 ipc_model_publish(struct ipc_model_registry *r, struct ipc_model_vm caller,
                      struct ipc_model_lane_id lane, struct ipc_model_request_id request,
                      u32 *published_sequence);
/* One acquire and one credit publication per bounded batch. Returned descriptors
 * belong to target; stale entries consume credit but are never returned. */
i32 ipc_model_receive(struct ipc_model_registry *r, struct ipc_model_vm caller,
                      struct ipc_model_lane_id lane, struct ipc_model_request_id *out,
                      u32 ceiling, u32 *count, u32 *rejected);
i32 ipc_model_read(struct ipc_model_registry *r, struct ipc_model_vm caller,
                   struct ipc_model_request_id request, struct ipc_model_request *out);
i32 ipc_model_release(struct ipc_model_registry *r, struct ipc_model_vm caller,
                      struct ipc_model_request_id request);
/* Cancellation is legal only before publication or after receipt. Queued
 * cancellation requires a runtime command to the target, never remote mutation. */
i32 ipc_model_cancel(struct ipc_model_registry *r, struct ipc_model_vm caller,
                     struct ipc_model_request_id request);
i32 ipc_model_sequences(struct ipc_model_registry *r, struct ipc_model_vm caller,
                        struct ipc_model_lane_id lane, u32 *published, u32 *consumed);
