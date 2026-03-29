#pragma once

#include "types.h"

#define V3D_QPU_MAX_DISPATCH 12U

typedef enum {
    V3D_STATUS_OK = 0,
    V3D_STATUS_UNSUPPORTED = -1,
    V3D_STATUS_INVALID = -2,
    V3D_STATUS_TIMEOUT = -3,
    V3D_STATUS_FAILED = -4,
    V3D_STATUS_NOT_IMPLEMENTED = -5,
} v3d_status_t;

typedef enum {
    V3D_BACKEND_AUTO = 0,
    V3D_BACKEND_MAILBOX = 1,
    V3D_BACKEND_MMIO_CSD = 2,
} v3d_backend_t;

typedef enum {
    V3D_KERNEL_MATMUL = 0,
    V3D_KERNEL_ADD = 1,
    V3D_KERNEL_MUL = 2,
    V3D_KERNEL_RELU = 3,
    V3D_KERNEL_DOT = 4,
    V3D_KERNEL_SCALE = 5,
    V3D_KERNEL_SOFTMAX = 6,
    V3D_KERNEL_MAX
} v3d_kernel_id_t;

struct v3d_kernel_desc {
    v3d_kernel_id_t id;
    const char *name;
    u32 qpu_count;
    u32 control_list_bus;
    bool ready;
    bool noflush;
};

struct v3d_caps {
    bool mailbox_qpu;
    bool mmio_probe_ok;
    bool mmio_csd;
    bool dispatch_supported;
    u64 reg_base;
    u32 ident0;
    u32 ident1;
    u32 ident2;
};

struct v3d_dispatch_cfg {
    u32 qpu_count;
    u32 control_list_bus;
    bool noflush;
    u32 timeout_ms;
    v3d_backend_t backend;
};

void v3d_init(void);
const struct v3d_caps *v3d_caps_get(void);
bool v3d_available(void);
bool v3d_dispatch_supported(void);
const struct v3d_kernel_desc *v3d_kernel_desc_get(v3d_kernel_id_t id);
v3d_status_t v3d_dispatch_kernel(v3d_kernel_id_t id, u32 timeout_ms);
v3d_status_t v3d_kernel_bind(v3d_kernel_id_t id, u32 uniform_bus, u32 shader_bus);
v3d_status_t v3d_kernel_bind_blob(v3d_kernel_id_t id,
                                  const void *uniform_data, u32 uniform_bytes,
                                  const u64 *shader_code, u32 shader_insts);

u32 v3d_reg_read(u32 reg_off, bool *ok_out);
v3d_status_t v3d_reg_write(u32 reg_off, u32 val);
v3d_status_t v3d_dispatch_compute(const struct v3d_dispatch_cfg *cfg);
