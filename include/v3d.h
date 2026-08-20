#pragma once

#include "types.h"

#define V3D_QPU_MAX_DISPATCH 12U

typedef enum {
    V3D_STATUS_OK = 0,
    V3D_STATUS_IN_PROGRESS = 1,
    V3D_STATUS_UNSUPPORTED = -1,
    V3D_STATUS_INVALID = -2,
    V3D_STATUS_TIMEOUT = -3,
    V3D_STATUS_FAILED = -4,
    V3D_STATUS_NOT_IMPLEMENTED = -5,
    V3D_STATUS_NOT_READY = -6,
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
    V3D_KERNEL_STORE_CONST = 7,
    V3D_KERNEL_LOAD_STORE = 8,
    V3D_KERNEL_STORE_SSBO = 9,
    V3D_KERNEL_NOOP = 10,
    V3D_KERNEL_PICOVM_ALU = 11,
    V3D_KERNEL_MATVEC16 = 12,
    V3D_KERNEL_GRAY_XOR64 = 13,
    V3D_KERNEL_GRAY_RESIDUAL64 = 14,
    V3D_KERNEL_MATVEC64 = 15,
    V3D_KERNEL_MATMUL64X16 = 16,
    V3D_KERNEL_BITNET_BITMAP64X16 = 17,
    V3D_KERNEL_GRAY_RESTORE64 = 18,
    V3D_KERNEL_MAX
} v3d_kernel_id_t;

struct v3d_kernel_desc {
    v3d_kernel_id_t id;
    const char *name;
    u32 qpu_count;
    u32 control_list_bus;
    u32 csd_cfg[7];
    bool ready;
    bool noflush;
    bool native_csd;
    bool verified;
};

struct v3d_caps {
    bool mailbox_qpu;
    bool native_probe_ok;
    bool native_compute_enabled;
    bool mmio_probe_ok;
    bool mmio_csd;
    bool native_selftest_ok;
    bool native_mmu_ready;
    bool native_tiny_kernels_ready;
    bool dispatch_supported;
    u64 reg_base;
    u64 hub_base;
    u64 core0_base;
    u64 sms_base;
    u32 ident0;
    u32 ident1;
    u32 ident2;
    u32 hub_ident1;
    u32 hub_ident2;
    u32 hub_ident3;
    u32 mmu_debug;
    u32 tech_version;
    u32 core_count;
    u32 qpus_per_slice;
    u32 slice_count;
    u32 mmu_va_bits;
    u32 mmu_pa_bits;
    u32 csd_status;
    u32 mmu_ctl;
    u32 mmuc_control;
    u32 tiny_ready_mask;
    u32 tiny_verified_mask;
    u64 pt_paddr;
    u64 scratch_paddr;
    v3d_status_t native_selftest_status;
};

struct v3d_dispatch_cfg {
    u32 qpu_count;
    u32 control_list_bus;
    u32 csd_cfg[7];
    bool csd_cfg_valid;
    bool noflush;
    u32 timeout_ms;
    v3d_backend_t backend;
};

struct v3d_csd_debug {
    u32 status_before;
    u32 status_after_kick;
    u32 status_after_wait;
    u32 core_int_sts;
    u32 hub_int_sts;
    u32 err_stat;
    u32 mmu_ctl;
    u32 mmu_illegal_addr;
    u32 mmu_vio_addr;
    u32 mmu_vio_id;
    u32 gmp_status;
    u32 gmp_cfg;
    u32 gmp_vio_addr;
    u32 current_cfg0;
    u32 current_cfg5;
    u32 current_cfg6;
    u32 l2t_before_invalidate;
    u32 l2t_after_invalidate;
    u32 l2t_after_tmuwcf;
    u32 l2t_after_clean;
    u32 l2t_invalidate_wait_us;
    u32 tmuwcf_wait_us;
    u32 l2t_clean_wait_us;
    u32 cache_clean_ok;
};

struct v3d_reset_debug {
    v3d_status_t status;
    u32 attempts;
    u32 err_before;
    u32 err_after_clear;
    u32 err_after_reset;
    u32 core_int_before;
    u32 hub_int_before;
    u32 core_int_after;
    u32 hub_int_after;
    u32 sms_before;
    u32 sms_after;
    u32 mmu_ctl_before;
    u32 mmu_ctl_after;
    u32 mmuc_before;
    u32 mmuc_after;
    u32 pm_before;
    u32 pm_asserted;
    u32 pm_after;
};

void v3d_init(void);
const struct v3d_caps *v3d_caps_get(void);
bool v3d_available(void);
bool v3d_dispatch_supported(void);
v3d_status_t v3d_native_selftest(void);
v3d_status_t v3d_native_mmu_setup(void);
const struct v3d_kernel_desc *v3d_kernel_desc_get(v3d_kernel_id_t id);
v3d_status_t v3d_dispatch_kernel(v3d_kernel_id_t id, u32 timeout_ms);
v3d_status_t v3d_dispatch_kernel_begin(v3d_kernel_id_t id, u32 timeout_ms);
v3d_status_t v3d_dispatch_poll(bool *done);
bool v3d_dispatch_in_flight(void);
v3d_status_t v3d_kernel_bind(v3d_kernel_id_t id, u32 uniform_bus, u32 shader_bus);
v3d_status_t v3d_kernel_bind_csd(v3d_kernel_id_t id, const u32 *csd_cfg, u32 qpu_count);
v3d_status_t v3d_kernel_bind_builtin_qpu(v3d_kernel_id_t id,
                                         const void *uniform_data,
                                         u32 uniform_bytes);
v3d_status_t v3d_kernel_bind_builtin_qpu_grid(v3d_kernel_id_t id,
                                              const void *uniform_data,
                                              u32 uniform_bytes,
                                              u32 workgroups_x);
void v3d_kernel_mark_verified(v3d_kernel_id_t id, bool verified);
v3d_status_t v3d_kernel_bind_blob(v3d_kernel_id_t id,
                                  const void *uniform_data, u32 uniform_bytes,
                                  const u64 *shader_code, u32 shader_insts);

u32 v3d_reg_read(u32 reg_off, bool *ok_out);
v3d_status_t v3d_reg_write(u32 reg_off, u32 val);
v3d_status_t v3d_dispatch_compute(const struct v3d_dispatch_cfg *cfg);
void v3d_csd_debug_last(struct v3d_csd_debug *out);
v3d_status_t v3d_soft_reset(void);
v3d_status_t v3d_pm_reset(void);
void v3d_reset_debug_last(struct v3d_reset_debug *out);
