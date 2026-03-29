#include "v3d.h"

#include "gpu.h"
#include "mmio.h"
#include "timer.h"
#include "uart.h"

#define V3D_BASE_ADDR            (PERIPH_BASE + 0x00C04000UL)
#define V3D_IDENT0_OFF           0x000U
#define V3D_IDENT1_OFF           0x004U
#define V3D_IDENT2_OFF           0x008U
#define V3D_MAX_TIMEOUT_MS       5000U
#define V3D_DEFAULT_TIMEOUT_MS   25U
#define V3D_CSD_STATUS_OFF       0x3C00U
#define V3D_CSD_QUEUED_CFG0_OFF  0x3C20U
#define V3D_CSD_QUEUED_CFG1_OFF  0x3C24U
#define V3D_CSD_QUEUED_CFG2_OFF  0x3C28U
#define V3D_CSD_QUEUED_CFG3_OFF  0x3C2CU
#define V3D_CSD_QUEUED_CFG4_OFF  0x3C30U
#define V3D_CSD_QUEUED_CFG5_OFF  0x3C34U
#define V3D_CSD_QUEUED_CFG6_OFF  0x3C38U
#define V3D_CSD_STATUS_BUSY_MASK (1U << 0)

static struct v3d_caps g_v3d_caps;
static bool g_mmio_auto_quarantined;
static bool g_mmio_auto_warned;
static u32 g_kernel_uniform_handle[V3D_KERNEL_MAX];
static u32 g_kernel_uniform_bus[V3D_KERNEL_MAX];
static u32 g_kernel_shader_handle[V3D_KERNEL_MAX];
static u32 g_kernel_shader_bus[V3D_KERNEL_MAX];
static struct v3d_kernel_desc g_kernels[V3D_KERNEL_MAX] = {
    { V3D_KERNEL_MATMUL, "matmul", 12, 0, false, false },
    { V3D_KERNEL_ADD,    "add",     4, 0, false, false },
    { V3D_KERNEL_MUL,    "mul",     4, 0, false, false },
    { V3D_KERNEL_RELU,   "relu",    4, 0, false, false },
    { V3D_KERNEL_DOT,    "dot",     8, 0, false, false },
    { V3D_KERNEL_SCALE,  "scale",   4, 0, false, false },
    { V3D_KERNEL_SOFTMAX,"softmax", 8, 0, false, false },
};
struct v3d_kernel_blob {
    u32 control_handle;
    u32 control_bus;
    u32 *control_ptr;
};
static struct v3d_kernel_blob g_kernel_blobs[V3D_KERNEL_MAX];

static bool v3d_ident_plausible(u32 ident0)
{
    if (ident0 == 0U || ident0 == 0xFFFFFFFFU)
        return false;
    return true;
}

static bool v3d_reg_allowed(u32 reg_off)
{
    if ((reg_off & 3U) != 0U)
        return false;
    if (reg_off <= V3D_IDENT2_OFF)
        return true;
    if (reg_off == V3D_CSD_STATUS_OFF)
        return true;
    return (reg_off >= V3D_CSD_QUEUED_CFG0_OFF && reg_off <= V3D_CSD_QUEUED_CFG6_OFF);
}

static void v3d_kernel_blobs_init(void)
{
    for (u32 i = 0; i < V3D_KERNEL_MAX; i++) {
        g_kernel_uniform_handle[i] = 0;
        g_kernel_uniform_bus[i] = 0;
        g_kernel_shader_handle[i] = 0;
        g_kernel_shader_bus[i] = 0;
        g_kernel_blobs[i].control_handle = 0;
        g_kernel_blobs[i].control_bus = 0;
        g_kernel_blobs[i].control_ptr = NULL;
        g_kernels[i].control_list_bus = 0;
        g_kernels[i].ready = false;

        u32 bytes = g_kernels[i].qpu_count * 8U;
        u32 handle = gpu_mem_alloc(bytes, 16, GPU_MEM_FLAG_COHERENT | GPU_MEM_FLAG_ZERO);
        if (!handle)
            continue;
        u32 control_bus = gpu_mem_lock(handle);
        if (!control_bus) {
            gpu_mem_free(handle);
            continue;
        }

        g_kernel_blobs[i].control_handle = handle;
        g_kernel_blobs[i].control_bus = control_bus;
        g_kernel_blobs[i].control_ptr = (u32 *)(usize)(control_bus & 0x3FFFFFFF);
        g_kernels[i].control_list_bus = control_bus;
    }
}

void v3d_init(void)
{
    g_v3d_caps.mailbox_qpu = false;
    g_v3d_caps.mmio_probe_ok = false;
    g_v3d_caps.mmio_csd = false;
    g_v3d_caps.dispatch_supported = false;
    g_v3d_caps.reg_base = V3D_BASE_ADDR;
    g_v3d_caps.ident0 = 0;
    g_v3d_caps.ident1 = 0;
    g_v3d_caps.ident2 = 0;
    g_mmio_auto_quarantined = false;
    g_mmio_auto_warned = false;

    if (qpu_enable(true)) {
        g_v3d_caps.mailbox_qpu = true;
        g_v3d_caps.dispatch_supported = true;
        (void)qpu_enable(false);
    }

    u32 ident0 = mmio_read(V3D_BASE_ADDR + V3D_IDENT0_OFF);
    if (v3d_ident_plausible(ident0)) {
        g_v3d_caps.mmio_probe_ok = true;
        g_v3d_caps.mmio_csd = true;
        g_v3d_caps.ident0 = ident0;
        g_v3d_caps.ident1 = mmio_read(V3D_BASE_ADDR + V3D_IDENT1_OFF);
        g_v3d_caps.ident2 = mmio_read(V3D_BASE_ADDR + V3D_IDENT2_OFF);
    }
    g_v3d_caps.dispatch_supported = g_v3d_caps.mailbox_qpu || g_v3d_caps.mmio_csd;
    v3d_kernel_blobs_init();

    uart_puts("[v3d] mailbox_qpu=");
    uart_hex(g_v3d_caps.mailbox_qpu ? 1 : 0);
    uart_puts(" mmio_probe=");
    uart_hex(g_v3d_caps.mmio_probe_ok ? 1 : 0);
    uart_puts(" mmio_csd=");
    uart_hex(g_v3d_caps.mmio_csd ? 1 : 0);
    uart_puts(" ident0=");
    uart_hex(g_v3d_caps.ident0);
    uart_puts("\n");
}

const struct v3d_caps *v3d_caps_get(void)
{
    return &g_v3d_caps;
}

bool v3d_available(void)
{
    return g_v3d_caps.mailbox_qpu;
}

bool v3d_dispatch_supported(void)
{
    return g_v3d_caps.dispatch_supported;
}

u32 v3d_reg_read(u32 reg_off, bool *ok_out)
{
    bool ok = false;
    u32 val = 0;

    if (g_v3d_caps.mmio_probe_ok && v3d_reg_allowed(reg_off)) {
        val = mmio_read(g_v3d_caps.reg_base + reg_off);
        ok = true;
    }

    if (ok_out)
        *ok_out = ok;
    return val;
}

v3d_status_t v3d_reg_write(u32 reg_off, u32 val)
{
    if (!g_v3d_caps.mmio_probe_ok || !v3d_reg_allowed(reg_off))
        return V3D_STATUS_UNSUPPORTED;
    mmio_write(g_v3d_caps.reg_base + reg_off, val);
    return V3D_STATUS_OK;
}

static v3d_status_t v3d_dispatch_mmio_csd(const struct v3d_dispatch_cfg *cfg, u32 timeout_ms)
{
    if (!g_v3d_caps.mmio_csd)
        return V3D_STATUS_UNSUPPORTED;

    /* Wait for CSD idle before submitting a new queue entry. */
    for (u32 i = 0; i < timeout_ms * 1000U; i++) {
        u32 st = mmio_read(g_v3d_caps.reg_base + V3D_CSD_STATUS_OFF);
        if ((st & V3D_CSD_STATUS_BUSY_MASK) == 0U)
            break;
        if (i + 1U == timeout_ms * 1000U)
            return V3D_STATUS_TIMEOUT;
        timer_delay_us(1);
    }

    /* Minimal CSD queue programming: control list pointer + launch config. */
    mmio_write(g_v3d_caps.reg_base + V3D_CSD_QUEUED_CFG0_OFF, cfg->control_list_bus);
    mmio_write(g_v3d_caps.reg_base + V3D_CSD_QUEUED_CFG1_OFF, cfg->qpu_count);
    mmio_write(g_v3d_caps.reg_base + V3D_CSD_QUEUED_CFG2_OFF, cfg->noflush ? 1U : 0U);
    mmio_write(g_v3d_caps.reg_base + V3D_CSD_QUEUED_CFG3_OFF, 0U);
    mmio_write(g_v3d_caps.reg_base + V3D_CSD_QUEUED_CFG4_OFF, 0U);
    mmio_write(g_v3d_caps.reg_base + V3D_CSD_QUEUED_CFG5_OFF, 0U);
    dmb();
    mmio_write(g_v3d_caps.reg_base + V3D_CSD_QUEUED_CFG6_OFF, 1U); /* queue kick */

    for (u32 i = 0; i < timeout_ms * 1000U; i++) {
        u32 st = mmio_read(g_v3d_caps.reg_base + V3D_CSD_STATUS_OFF);
        if ((st & V3D_CSD_STATUS_BUSY_MASK) == 0U) {
            dmb();
            return V3D_STATUS_OK;
        }
        timer_delay_us(1);
    }
    return V3D_STATUS_TIMEOUT;
}

static v3d_status_t v3d_dispatch_mailbox(const struct v3d_dispatch_cfg *cfg, u32 timeout_ms)
{
    if (!g_v3d_caps.mailbox_qpu)
        return V3D_STATUS_UNSUPPORTED;
    if (!qpu_enable(true))
        return V3D_STATUS_FAILED;
    bool ok = qpu_execute_timeout(cfg->qpu_count, cfg->control_list_bus,
                                  cfg->noflush, timeout_ms);
    if (!qpu_enable(false))
        return V3D_STATUS_FAILED;
    return ok ? V3D_STATUS_OK : V3D_STATUS_TIMEOUT;
}

v3d_status_t v3d_dispatch_compute(const struct v3d_dispatch_cfg *cfg)
{
    if (!cfg)
        return V3D_STATUS_INVALID;
    if (!g_v3d_caps.dispatch_supported)
        return V3D_STATUS_UNSUPPORTED;
    if (cfg->qpu_count == 0 || cfg->qpu_count > V3D_QPU_MAX_DISPATCH)
        return V3D_STATUS_INVALID;
    if ((cfg->control_list_bus & 0xFU) != 0U)
        return V3D_STATUS_INVALID;

    u32 timeout_ms = cfg->timeout_ms;
    if (timeout_ms == 0)
        timeout_ms = V3D_DEFAULT_TIMEOUT_MS;
    if (timeout_ms > V3D_MAX_TIMEOUT_MS)
        return V3D_STATUS_INVALID;
    if (cfg->backend == V3D_BACKEND_MMIO_CSD)
        return v3d_dispatch_mmio_csd(cfg, timeout_ms);
    if (cfg->backend == V3D_BACKEND_MAILBOX)
        return v3d_dispatch_mailbox(cfg, timeout_ms);

    v3d_status_t r = V3D_STATUS_UNSUPPORTED;
    if (!g_mmio_auto_quarantined) {
        r = v3d_dispatch_mmio_csd(cfg, timeout_ms);
        if (r == V3D_STATUS_OK)
            return r;
        if ((r == V3D_STATUS_TIMEOUT || r == V3D_STATUS_FAILED) &&
            g_v3d_caps.mailbox_qpu) {
            g_mmio_auto_quarantined = true;
            if (!g_mmio_auto_warned) {
                uart_puts("[v3d] auto: quarantining MMIO CSD, using mailbox fallback\n");
                g_mmio_auto_warned = true;
            }
        }
    }
    return v3d_dispatch_mailbox(cfg, timeout_ms);
}

const struct v3d_kernel_desc *v3d_kernel_desc_get(v3d_kernel_id_t id)
{
    if (id >= V3D_KERNEL_MAX)
        return NULL;
    return &g_kernels[id];
}

v3d_status_t v3d_dispatch_kernel(v3d_kernel_id_t id, u32 timeout_ms)
{
    if (id >= V3D_KERNEL_MAX)
        return V3D_STATUS_INVALID;
    const struct v3d_kernel_desc *k = &g_kernels[id];
    if (k->qpu_count == 0 || k->control_list_bus == 0 || !k->ready)
        return V3D_STATUS_NOT_IMPLEMENTED;

    struct v3d_dispatch_cfg cfg;
    cfg.qpu_count = k->qpu_count;
    cfg.control_list_bus = k->control_list_bus;
    cfg.noflush = k->noflush;
    cfg.timeout_ms = timeout_ms;
    cfg.backend = V3D_BACKEND_AUTO;
    return v3d_dispatch_compute(&cfg);
}

v3d_status_t v3d_kernel_bind(v3d_kernel_id_t id, u32 uniform_bus, u32 shader_bus)
{
    if (id >= V3D_KERNEL_MAX)
        return V3D_STATUS_INVALID;
    if ((uniform_bus & 0xFU) != 0U || (shader_bus & 0xFU) != 0U)
        return V3D_STATUS_INVALID;
    struct v3d_kernel_blob *b = &g_kernel_blobs[id];
    if (!b->control_ptr || b->control_bus == 0)
        return V3D_STATUS_FAILED;

    for (u32 i = 0; i < g_kernels[id].qpu_count; i++) {
        b->control_ptr[i * 2 + 0] = uniform_bus;
        b->control_ptr[i * 2 + 1] = shader_bus;
    }
    dmb();
    g_kernels[id].ready = true;
    g_kernels[id].control_list_bus = b->control_bus;
    return V3D_STATUS_OK;
}

v3d_status_t v3d_kernel_bind_blob(v3d_kernel_id_t id,
                                  const void *uniform_data, u32 uniform_bytes,
                                  const u64 *shader_code, u32 shader_insts)
{
    if (id >= V3D_KERNEL_MAX)
        return V3D_STATUS_INVALID;
    if (!uniform_data || uniform_bytes == 0 || !shader_code || shader_insts == 0)
        return V3D_STATUS_INVALID;
    if (!g_v3d_caps.dispatch_supported)
        return V3D_STATUS_UNSUPPORTED;

    u32 shader_bytes = shader_insts * 8U;
    u32 uniform_alloc = (uniform_bytes + 15U) & ~15U;
    u32 shader_alloc = (shader_bytes + 15U) & ~15U;

    u32 uniform_handle = gpu_mem_alloc(uniform_alloc, 16, GPU_MEM_FLAG_COHERENT);
    if (!uniform_handle)
        return V3D_STATUS_FAILED;
    u32 uniform_bus = gpu_mem_lock(uniform_handle);
    if (!uniform_bus) {
        gpu_mem_free(uniform_handle);
        return V3D_STATUS_FAILED;
    }

    u32 shader_handle = gpu_mem_alloc(shader_alloc, 16, GPU_MEM_FLAG_COHERENT);
    if (!shader_handle) {
        gpu_mem_unlock(uniform_handle);
        gpu_mem_free(uniform_handle);
        return V3D_STATUS_FAILED;
    }
    u32 shader_bus = gpu_mem_lock(shader_handle);
    if (!shader_bus) {
        gpu_mem_free(shader_handle);
        gpu_mem_unlock(uniform_handle);
        gpu_mem_free(uniform_handle);
        return V3D_STATUS_FAILED;
    }

    void *uniform_ptr = (void *)(usize)(uniform_bus & 0x3FFFFFFF);
    void *shader_ptr = (void *)(usize)(shader_bus & 0x3FFFFFFF);
    memcpy(uniform_ptr, uniform_data, uniform_bytes);
    memcpy(shader_ptr, shader_code, shader_bytes);
    dsb();

    v3d_status_t r = v3d_kernel_bind(id, uniform_bus, shader_bus);
    if (r != V3D_STATUS_OK) {
        gpu_mem_unlock(shader_handle);
        gpu_mem_free(shader_handle);
        gpu_mem_unlock(uniform_handle);
        gpu_mem_free(uniform_handle);
        return r;
    }

    if (g_kernel_uniform_handle[id]) {
        gpu_mem_unlock(g_kernel_uniform_handle[id]);
        gpu_mem_free(g_kernel_uniform_handle[id]);
    }
    if (g_kernel_shader_handle[id]) {
        gpu_mem_unlock(g_kernel_shader_handle[id]);
        gpu_mem_free(g_kernel_shader_handle[id]);
    }
    g_kernel_uniform_handle[id] = uniform_handle;
    g_kernel_uniform_bus[id] = uniform_bus;
    g_kernel_shader_handle[id] = shader_handle;
    g_kernel_shader_bus[id] = shader_bus;
    return V3D_STATUS_OK;
}
