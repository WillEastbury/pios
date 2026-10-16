#pragma once
#include "types.h"
#include "lease.h"

#define TB_DEPTH 4U
#define TB_BACKENDS 8U

#define TB_OP_MATVEC 1U
#define TB_ABI_VERSION 1U

#define TB_DTYPE_I8 1U
#define TB_DTYPE_I32 2U
#define TB_LAYOUT_ROW_MAJOR 1U

#define TB_POLICY_AUTO 0U
#define TB_POLICY_EXACT 1U

#define TB_BACKEND_CPU 1U
#define TB_BACKEND_QPU 2U
#define TB_BACKEND_XE2 3U
#define TB_BACKEND_XMX 4U

#define TB_CAP_CPU 0x0001U
#define TB_CAP_QPU 0x0002U
#define TB_CAP_XE2 0x0004U
#define TB_CAP_XMX 0x0008U

#define TB_OK 0
#define TB_INVALID (-1)
#define TB_FULL (-2)
#define TB_STALE (-3)
#define TB_UNAVAILABLE (-4)
#define TB_EXECUTION (-5)
#define TB_AUTHORITY (-6)
#define TB_STATE (-7)

struct tensor_descriptor {
    u32 abi_version;
    u32 operation;
    u32 input_dtype;
    u32 output_dtype;
    u32 layout;
    u32 rows;
    u32 cols;
    u32 policy;
    u32 target_backend;
    u32 target_device_generation;
    u32 owner_capsule;
    u32 reserved;
    struct lease_handle matrix;
    struct lease_handle vector;
    struct lease_handle output;
    u32 _pad[4];
} ALIGNED(64);

struct tb_capability {
    u32 backend_id;
    u32 backend_generation;
    u32 capability;
    u32 operation;
    u32 input_dtype;
    u32 output_dtype;
    u32 layout;
    u32 max_rows;
    u32 max_cols;
    u32 col_multiple;
    u32 priority;
    u32 device_generation;
    u32 live;
    u32 _pad[3];
} ALIGNED(64);

typedef bool (*tb_execute)(const struct tensor_descriptor *,
                           const i8 *, const i8 *, i32 *);

struct tb_backend {
    struct tb_capability capability;
    tb_execute execute;
    u64 _pad[7];
} ALIGNED(64);

struct tb_job {
    u32 generation;
    u32 state;
    u32 backend_slot;
    u32 backend_generation;
    i32 status;
    u32 selected_backend;
    u32 _pad[10];
    struct tensor_descriptor descriptor;
} ALIGNED(64);

struct tensor_broker {
    struct tb_backend backends[TB_BACKENDS];
    struct tb_job jobs[TB_DEPTH];
    u32 queue[TB_DEPTH];
    u32 head;
    u32 count;
    u32 _pad[10];
} ALIGNED(64);

i32 tb_register(struct tensor_broker *, const struct tb_capability *, tb_execute);
i32 tb_retire(struct tensor_broker *, u32 backend_id, u32 device_generation);
i32 tb_submit(struct tensor_broker *, const struct tensor_descriptor *, u32 *);
bool tb_step(struct tensor_broker *);
i32 tb_result(struct tensor_broker *, u32 handle, u32 *selected_backend);
i32 tb_release(struct tensor_broker *, u32 handle);
