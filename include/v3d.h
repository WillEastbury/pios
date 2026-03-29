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

u32 v3d_reg_read(u32 reg_off, bool *ok_out);
v3d_status_t v3d_reg_write(u32 reg_off, u32 val);
v3d_status_t v3d_dispatch_compute(const struct v3d_dispatch_cfg *cfg);
