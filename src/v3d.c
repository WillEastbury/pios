#include "v3d.h"

#include "gpu.h"
#include "mmu.h"
#include "mmio.h"
#include "platform.h"
#include "timer.h"
#include "uart.h"
#include "videocore.h"
#include "v3d_tiny_qpu.h"

/* Legacy pre-Pi5 guess. Kept for comparison only; native Pi5 probing uses the
 * bcm2712-ds.dtsi V3D 7.1 ranges decoded by videocore.c. */
#define V3D_LEGACY_BASE_ADDR     (PERIPH_BASE + 0x00C04000UL)
#define V3D_PI5_HUB_BASE         0x1002000000UL
#define V3D_PI5_CORE0_BASE       0x1002008000UL
#define V3D_PI5_SMS_BASE         0x1002030800UL
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
#define V3D_SHADER_INSTS_MAX     8192U
#define V3D_UNIFORM_BYTES_MAX    4096U
#define V3D_MMU_PAGE_SHIFT       12U
#define V3D_MMU_PT_ENTRIES       1048576U
#define V3D_MMU_IDENTITY_BYTES   0x40000000UL
#define V3D_MMU_SCRATCH_BYTES    4096U

#define V3D_PTE_WRITEABLE        0x20000000U
#define V3D_PTE_VALID            0x10000000U

#define V3D_MMUC_CONTROL_OFF     0x00001000U
#define V3D_MMUC_CONTROL_FLUSHING 0x00000004U
#define V3D_MMUC_CONTROL_FLUSH   0x00000002U
#define V3D_MMUC_CONTROL_ENABLE  0x00000001U

#define V3D_MMU_CTL_OFF          0x00001200U
#define V3D_MMU_CTL_CAP_EXCEEDED_ABORT 0x04000000U
#define V3D_MMU_CTL_CAP_EXCEEDED_INT   0x02000000U
#define V3D_MMU_CTL_PT_INVALID_ENABLE  0x00010000U
#define V3D_MMU_CTL_PT_INVALID_ABORT   0x00080000U
#define V3D_MMU_CTL_PT_INVALID_INT     0x00040000U
#define V3D_MMU_CTL_WRITE_VIOLATION_ABORT 0x00000800U
#define V3D_MMU_CTL_WRITE_VIOLATION_INT   0x00000400U
#define V3D_MMU_CTL_TLB_CLEARING 0x00000080U
#define V3D_MMU_CTL_TLB_CLEAR    0x00000004U
#define V3D_MMU_CTL_ENABLE       0x00000001U
#define V3D_MMU_PT_PA_BASE_OFF   0x00001204U
#define V3D_MMU_ILLEGAL_ADDR_OFF 0x00001230U
#define V3D_MMU_ILLEGAL_ADDR_ENABLE 0x80000000U

#define V3D_CSD_STATUS_OLD_OFF       0x00000900U
#define V3D_CSD_QUEUED_CFG0_OLD_OFF  0x00000904U
#define V3D_CSD_QUEUED_CFG1_OLD_OFF  0x00000908U
#define V3D_CSD_QUEUED_CFG2_OLD_OFF  0x0000090CU
#define V3D_CSD_QUEUED_CFG3_OLD_OFF  0x00000910U
#define V3D_CSD_QUEUED_CFG4_OLD_OFF  0x00000914U
#define V3D_CSD_QUEUED_CFG5_OLD_OFF  0x00000918U
#define V3D_CSD_QUEUED_CFG6_OLD_OFF  0x0000091CU

#define V3D_CSD_QUEUED_CFG0_V71_OFF  0x00000930U
#define V3D_CSD_QUEUED_CFG1_V71_OFF  0x00000934U
#define V3D_CSD_QUEUED_CFG2_V71_OFF  0x00000938U
#define V3D_CSD_QUEUED_CFG3_V71_OFF  0x0000093CU
#define V3D_CSD_QUEUED_CFG4_V71_OFF  0x00000940U
#define V3D_CSD_QUEUED_CFG5_V71_OFF  0x00000944U
#define V3D_CSD_QUEUED_CFG6_V71_OFF  0x00000948U
#define V3D_CSD_STATUS_BUSY_MASK     0x0000000FU

static struct v3d_caps g_v3d_caps;
static bool g_mmio_auto_quarantined;
static bool g_mmio_auto_warned;
static u32 g_kernel_uniform_handle[V3D_KERNEL_MAX];
static u32 g_kernel_uniform_bus[V3D_KERNEL_MAX];
static u32 g_kernel_shader_handle[V3D_KERNEL_MAX];
static u32 g_kernel_shader_bus[V3D_KERNEL_MAX];
static struct v3d_kernel_desc g_kernels[V3D_KERNEL_MAX] = {
    [V3D_KERNEL_MATMUL]  = { .id = V3D_KERNEL_MATMUL,  .name = "matmul",  .qpu_count = 12 },
    [V3D_KERNEL_ADD]     = { .id = V3D_KERNEL_ADD,     .name = "add",     .qpu_count = 4 },
    [V3D_KERNEL_MUL]     = { .id = V3D_KERNEL_MUL,     .name = "mul",     .qpu_count = 4 },
    [V3D_KERNEL_RELU]    = { .id = V3D_KERNEL_RELU,    .name = "relu",    .qpu_count = 4 },
    [V3D_KERNEL_DOT]     = { .id = V3D_KERNEL_DOT,     .name = "dot",     .qpu_count = 8 },
    [V3D_KERNEL_SCALE]   = { .id = V3D_KERNEL_SCALE,   .name = "scale",   .qpu_count = 4 },
    [V3D_KERNEL_SOFTMAX] = { .id = V3D_KERNEL_SOFTMAX, .name = "softmax", .qpu_count = 8 },
};
struct v3d_kernel_blob {
    u32 control_handle;
    u32 control_bus;
    u32 *control_ptr;
};
static struct v3d_kernel_blob g_kernel_blobs[V3D_KERNEL_MAX];
#if PIOS_ENABLE_NATIVE_V3D_COMPUTE
static u32 g_v3d_pt[V3D_MMU_PT_ENTRIES] ALIGNED(4096);
static u8 g_v3d_scratch[V3D_MMU_SCRATCH_BYTES] ALIGNED(4096);
#endif

#if PIOS_ENABLE_NATIVE_V3D_COMPUTE
static bool v3d_ident_plausible(u32 ident0)
{
    if (ident0 == 0U || ident0 == 0xFFFFFFFFU)
        return false;
    return true;
}
#endif

static void v3d_update_tiny_kernel_state(void)
{
    g_v3d_caps.tiny_ready_mask = 0;
    g_v3d_caps.tiny_verified_mask = 0;

    if (g_kernels[V3D_KERNEL_ADD].ready)
        g_v3d_caps.tiny_ready_mask |= (1U << V3D_KERNEL_ADD);
    if (g_kernels[V3D_KERNEL_RELU].ready)
        g_v3d_caps.tiny_ready_mask |= (1U << V3D_KERNEL_RELU);
    if (g_kernels[V3D_KERNEL_ADD].verified)
        g_v3d_caps.tiny_verified_mask |= (1U << V3D_KERNEL_ADD);
    if (g_kernels[V3D_KERNEL_RELU].verified)
        g_v3d_caps.tiny_verified_mask |= (1U << V3D_KERNEL_RELU);

    g_v3d_caps.native_tiny_kernels_ready =
        ((g_v3d_caps.tiny_verified_mask & ((1U << V3D_KERNEL_ADD) | (1U << V3D_KERNEL_RELU))) ==
         ((1U << V3D_KERNEL_ADD) | (1U << V3D_KERNEL_RELU)));
}

static bool v3d_reg_allowed(u32 reg_off)
{
    if ((reg_off & 3U) != 0U)
        return false;
    if (reg_off <= V3D_IDENT2_OFF)
        return true;
    if (reg_off == V3D_CSD_STATUS_OLD_OFF)
        return true;
    if (g_v3d_caps.tech_version >= 71U &&
        reg_off >= V3D_CSD_QUEUED_CFG0_V71_OFF &&
        reg_off <= V3D_CSD_QUEUED_CFG6_V71_OFF)
        return true;
    return (reg_off >= V3D_CSD_QUEUED_CFG0_OLD_OFF && reg_off <= V3D_CSD_QUEUED_CFG6_OLD_OFF);
}

static u32 v3d_csd_cfg0_off(void) { return g_v3d_caps.tech_version >= 71U ? V3D_CSD_QUEUED_CFG0_V71_OFF : V3D_CSD_QUEUED_CFG0_OLD_OFF; }
static u32 v3d_csd_cfg1_off(void) { return g_v3d_caps.tech_version >= 71U ? V3D_CSD_QUEUED_CFG1_V71_OFF : V3D_CSD_QUEUED_CFG1_OLD_OFF; }
static u32 v3d_csd_cfg2_off(void) { return g_v3d_caps.tech_version >= 71U ? V3D_CSD_QUEUED_CFG2_V71_OFF : V3D_CSD_QUEUED_CFG2_OLD_OFF; }
static u32 v3d_csd_cfg3_off(void) { return g_v3d_caps.tech_version >= 71U ? V3D_CSD_QUEUED_CFG3_V71_OFF : V3D_CSD_QUEUED_CFG3_OLD_OFF; }
static u32 v3d_csd_cfg4_off(void) { return g_v3d_caps.tech_version >= 71U ? V3D_CSD_QUEUED_CFG4_V71_OFF : V3D_CSD_QUEUED_CFG4_OLD_OFF; }
static u32 v3d_csd_cfg5_off(void) { return g_v3d_caps.tech_version >= 71U ? V3D_CSD_QUEUED_CFG5_V71_OFF : V3D_CSD_QUEUED_CFG5_OLD_OFF; }
static u32 v3d_csd_cfg6_off(void) { return g_v3d_caps.tech_version >= 71U ? V3D_CSD_QUEUED_CFG6_V71_OFF : V3D_CSD_QUEUED_CFG6_OLD_OFF; }

#if PIOS_ENABLE_NATIVE_V3D_COMPUTE
static bool v3d_wait_hub_clear(u32 reg_off, u32 mask, u32 timeout_us)
{
    for (u32 i = 0; i < timeout_us; i++) {
        if ((mmio_read(g_v3d_caps.hub_base + reg_off) & mask) == 0U)
            return true;
        timer_delay_us(1);
    }
    return false;
}

static bool v3d_mmu_flush_all(void)
{
    mmio_write(g_v3d_caps.hub_base + V3D_MMUC_CONTROL_OFF,
               V3D_MMUC_CONTROL_FLUSH | V3D_MMUC_CONTROL_ENABLE);
    if (!v3d_wait_hub_clear(V3D_MMUC_CONTROL_OFF, V3D_MMUC_CONTROL_FLUSHING, 100U))
        return false;

    mmio_write(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF,
               mmio_read(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF) | V3D_MMU_CTL_TLB_CLEAR);
    return v3d_wait_hub_clear(V3D_MMU_CTL_OFF, V3D_MMU_CTL_TLB_CLEARING, 100U);
}
#endif

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
        for (u32 j = 0; j < 7U; j++)
            g_kernels[i].csd_cfg[j] = 0;
        g_kernels[i].ready = false;
        g_kernels[i].native_csd = false;
        g_kernels[i].verified = false;

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
    g_v3d_caps.native_probe_ok = false;
    g_v3d_caps.native_compute_enabled = PIOS_ENABLE_NATIVE_V3D_COMPUTE ? true : false;
    g_v3d_caps.mmio_probe_ok = false;
    g_v3d_caps.mmio_csd = false;
    g_v3d_caps.native_selftest_ok = false;
    g_v3d_caps.native_mmu_ready = false;
    g_v3d_caps.native_tiny_kernels_ready = false;
    g_v3d_caps.dispatch_supported = false;
    g_v3d_caps.reg_base = 0;
    g_v3d_caps.hub_base = 0;
    g_v3d_caps.core0_base = 0;
    g_v3d_caps.sms_base = 0;
    g_v3d_caps.ident0 = 0;
    g_v3d_caps.ident1 = 0;
    g_v3d_caps.ident2 = 0;
    g_v3d_caps.hub_ident1 = 0;
    g_v3d_caps.hub_ident2 = 0;
    g_v3d_caps.hub_ident3 = 0;
    g_v3d_caps.mmu_debug = 0;
    g_v3d_caps.tech_version = 0;
    g_v3d_caps.core_count = 0;
    g_v3d_caps.qpus_per_slice = 0;
    g_v3d_caps.slice_count = 0;
    g_v3d_caps.mmu_va_bits = 0;
    g_v3d_caps.mmu_pa_bits = 0;
    g_v3d_caps.csd_status = 0;
    g_v3d_caps.mmu_ctl = 0;
    g_v3d_caps.mmuc_control = 0;
    g_v3d_caps.pt_paddr = 0;
    g_v3d_caps.scratch_paddr = 0;
    g_v3d_caps.tiny_ready_mask = 0;
    g_v3d_caps.tiny_verified_mask = 0;
    g_v3d_caps.native_selftest_status = V3D_STATUS_UNSUPPORTED;
    g_mmio_auto_quarantined = false;
    g_mmio_auto_warned = false;

    if (qpu_enable(true)) {
        g_v3d_caps.mailbox_qpu = true;
        g_v3d_caps.dispatch_supported = true;
        (void)qpu_enable(false);
    }

    const struct videocore_probe *vc = videocore_probe_get();
    if (vc && vc->enabled && vc->v3d_seen && vc->v3d_is_71) {
        g_v3d_caps.native_probe_ok = true;
        g_v3d_caps.hub_base = V3D_PI5_HUB_BASE;
        g_v3d_caps.core0_base = V3D_PI5_CORE0_BASE;
        g_v3d_caps.sms_base = V3D_PI5_SMS_BASE;
        g_v3d_caps.reg_base = V3D_PI5_CORE0_BASE;
        g_v3d_caps.ident0 = vc->v3d_core_ident0;
        g_v3d_caps.ident1 = vc->v3d_core_ident1;
        g_v3d_caps.ident2 = vc->v3d_core_ident2;
        g_v3d_caps.hub_ident1 = vc->v3d_hub_ident1;
        g_v3d_caps.hub_ident2 = vc->v3d_hub_ident2;
        g_v3d_caps.hub_ident3 = vc->v3d_hub_ident3;
        g_v3d_caps.mmu_debug = vc->v3d_mmu_debug;
        g_v3d_caps.tech_version = vc->v3d_tech_version;
        g_v3d_caps.core_count = vc->v3d_core_count;
        g_v3d_caps.qpus_per_slice = vc->v3d_qpus_per_slice;
        g_v3d_caps.slice_count = vc->v3d_slice_count;
        g_v3d_caps.mmu_va_bits = vc->v3d_mmu_va_bits;
        g_v3d_caps.mmu_pa_bits = vc->v3d_mmu_pa_bits;
#if PIOS_ENABLE_NATIVE_V3D_COMPUTE
        g_v3d_caps.mmio_probe_ok = true;
        g_v3d_caps.mmio_csd = true;
#endif
    }

#if PIOS_ENABLE_NATIVE_V3D_COMPUTE
    if (!g_v3d_caps.mmio_probe_ok) {
        u32 ident0 = mmio_read(V3D_LEGACY_BASE_ADDR + V3D_IDENT0_OFF);
        if (v3d_ident_plausible(ident0)) {
            g_v3d_caps.mmio_probe_ok = true;
            g_v3d_caps.mmio_csd = true;
            g_v3d_caps.reg_base = V3D_LEGACY_BASE_ADDR;
            g_v3d_caps.ident0 = ident0;
            g_v3d_caps.ident1 = mmio_read(V3D_LEGACY_BASE_ADDR + V3D_IDENT1_OFF);
            g_v3d_caps.ident2 = mmio_read(V3D_LEGACY_BASE_ADDR + V3D_IDENT2_OFF);
        }
    }
#endif
#if PIOS_ENABLE_NATIVE_V3D_COMPUTE
    if (g_v3d_caps.mmio_probe_ok)
        (void)v3d_native_mmu_setup();
#endif
    g_v3d_caps.native_selftest_status = v3d_native_selftest();
    g_v3d_caps.dispatch_supported = g_v3d_caps.mailbox_qpu || g_v3d_caps.mmio_csd;
    v3d_kernel_blobs_init();

    uart_puts("[v3d] mbox=");
    uart_hex(g_v3d_caps.mailbox_qpu ? 1 : 0);
    uart_puts(" mmio=");
    uart_hex(g_v3d_caps.mmio_probe_ok ? 1 : 0);
    uart_puts(" csd=");
    uart_hex(g_v3d_caps.mmio_csd ? 1 : 0);
    uart_puts(" native=");
    uart_hex(g_v3d_caps.native_probe_ok ? 1 : 0);
    uart_puts(" comp=");
    uart_hex(g_v3d_caps.native_compute_enabled ? 1 : 0);
    uart_puts(" id0=");
    uart_hex(g_v3d_caps.ident0);
    uart_puts(" tv=");
    uart_hex(g_v3d_caps.tech_version);
    uart_puts(" self=");
    uart_hex((u32)g_v3d_caps.native_selftest_status);
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

v3d_status_t v3d_native_selftest(void)
{
    g_v3d_caps.native_selftest_ok = false;

    if (!g_v3d_caps.native_probe_ok)
        return V3D_STATUS_UNSUPPORTED;
    if (g_v3d_caps.tech_version != 71U || g_v3d_caps.core_count == 0U)
        return V3D_STATUS_FAILED;
    if (g_v3d_caps.qpus_per_slice == 0U || g_v3d_caps.slice_count == 0U)
        return V3D_STATUS_FAILED;
    if (g_v3d_caps.mmu_va_bits < 30U || g_v3d_caps.mmu_pa_bits < 30U)
        return V3D_STATUS_FAILED;

#if PIOS_ENABLE_NATIVE_V3D_COMPUTE
    if (!g_v3d_caps.mmio_probe_ok || g_v3d_caps.reg_base == 0U)
        return V3D_STATUS_NOT_READY;
    if (!g_v3d_caps.native_mmu_ready)
        return V3D_STATUS_NOT_READY;

    g_v3d_caps.csd_status = mmio_read(g_v3d_caps.reg_base + V3D_CSD_STATUS_OLD_OFF);
    if ((g_v3d_caps.csd_status & V3D_CSD_STATUS_BUSY_MASK) != 0U)
        return V3D_STATUS_NOT_READY;
#endif

    g_v3d_caps.native_selftest_ok = true;
    return V3D_STATUS_OK;
}

v3d_status_t v3d_kernel_bind_csd(v3d_kernel_id_t id, const u32 *csd_cfg, u32 qpu_count)
{
    if (id >= V3D_KERNEL_MAX || !csd_cfg)
        return V3D_STATUS_INVALID;
    if (id != V3D_KERNEL_ADD && id != V3D_KERNEL_RELU)
        return V3D_STATUS_NOT_IMPLEMENTED;
    if (qpu_count == 0U || qpu_count > V3D_QPU_MAX_DISPATCH)
        return V3D_STATUS_INVALID;
    if (!g_v3d_caps.native_selftest_ok)
        return V3D_STATUS_NOT_READY;

    for (u32 i = 0; i < 7U; i++)
        g_kernels[id].csd_cfg[i] = csd_cfg[i];
    g_kernels[id].qpu_count = qpu_count;
    g_kernels[id].control_list_bus = 1U; /* Native CSD configs do not use mailbox control lists. */
    g_kernels[id].native_csd = true;
    g_kernels[id].verified = false;
    g_kernels[id].ready = true;
    v3d_update_tiny_kernel_state();
    return V3D_STATUS_OK;
}

v3d_status_t v3d_kernel_bind_builtin_qpu(v3d_kernel_id_t id,
                                         const void *uniform_data,
                                         u32 uniform_bytes)
{
    switch (id) {
    case V3D_KERNEL_ADD:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_vector_add_qpu,
                                    V3D_TINY_VECTOR_ADD_QPU_WORDS);
    case V3D_KERNEL_RELU:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_relu_qpu,
                                    V3D_TINY_RELU_QPU_WORDS);
    default:
        return V3D_STATUS_NOT_IMPLEMENTED;
    }
}

void v3d_kernel_mark_verified(v3d_kernel_id_t id, bool verified)
{
    if (id >= V3D_KERNEL_MAX)
        return;
    g_kernels[id].verified = verified ? true : false;
    v3d_update_tiny_kernel_state();
}

v3d_status_t v3d_native_mmu_setup(void)
{
#if !PIOS_ENABLE_NATIVE_V3D_COMPUTE
    return V3D_STATUS_UNSUPPORTED;
#else
    if (!g_v3d_caps.native_probe_ok || !g_v3d_caps.mmio_probe_ok || g_v3d_caps.hub_base == 0U)
        return V3D_STATUS_NOT_READY;

    for (u32 page = 0; page < (u32)(V3D_MMU_IDENTITY_BYTES >> V3D_MMU_PAGE_SHIFT); page++)
        g_v3d_pt[page] = V3D_PTE_VALID | V3D_PTE_WRITEABLE | page;
    for (u32 page = (u32)(V3D_MMU_IDENTITY_BYTES >> V3D_MMU_PAGE_SHIFT);
         page < V3D_MMU_PT_ENTRIES; page++)
        g_v3d_pt[page] = 0;

    g_v3d_caps.pt_paddr = (u64)(usize)g_v3d_pt;
    g_v3d_caps.scratch_paddr = (u64)(usize)g_v3d_scratch;
    dcache_clean_range((u64)(usize)g_v3d_pt, sizeof(g_v3d_pt));
    dcache_clean_range((u64)(usize)g_v3d_scratch, sizeof(g_v3d_scratch));

    mmio_write(g_v3d_caps.hub_base + V3D_MMU_PT_PA_BASE_OFF,
               (u32)(g_v3d_caps.pt_paddr >> V3D_MMU_PAGE_SHIFT));
    mmio_write(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF,
               V3D_MMU_CTL_ENABLE |
               V3D_MMU_CTL_PT_INVALID_ENABLE |
               V3D_MMU_CTL_PT_INVALID_ABORT |
               V3D_MMU_CTL_PT_INVALID_INT |
               V3D_MMU_CTL_WRITE_VIOLATION_ABORT |
               V3D_MMU_CTL_WRITE_VIOLATION_INT |
               V3D_MMU_CTL_CAP_EXCEEDED_ABORT |
               V3D_MMU_CTL_CAP_EXCEEDED_INT);
    mmio_write(g_v3d_caps.hub_base + V3D_MMU_ILLEGAL_ADDR_OFF,
               (u32)(g_v3d_caps.scratch_paddr >> V3D_MMU_PAGE_SHIFT) |
               V3D_MMU_ILLEGAL_ADDR_ENABLE);
    mmio_write(g_v3d_caps.hub_base + V3D_MMUC_CONTROL_OFF, V3D_MMUC_CONTROL_ENABLE);

    if (!v3d_mmu_flush_all())
        return V3D_STATUS_TIMEOUT;

    g_v3d_caps.mmu_ctl = mmio_read(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF);
    g_v3d_caps.mmuc_control = mmio_read(g_v3d_caps.hub_base + V3D_MMUC_CONTROL_OFF);
    if ((g_v3d_caps.mmu_ctl & V3D_MMU_CTL_ENABLE) == 0U ||
        (g_v3d_caps.mmuc_control & V3D_MMUC_CONTROL_ENABLE) == 0U)
        return V3D_STATUS_FAILED;

    g_v3d_caps.native_mmu_ready = true;
    return V3D_STATUS_OK;
#endif
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
    if (!g_v3d_caps.native_compute_enabled ||
        !g_v3d_caps.mmio_probe_ok ||
        !v3d_reg_allowed(reg_off))
        return V3D_STATUS_UNSUPPORTED;
    mmio_write(g_v3d_caps.reg_base + reg_off, val);
    return V3D_STATUS_OK;
}

static v3d_status_t v3d_dispatch_mmio_csd(const struct v3d_dispatch_cfg *cfg, u32 timeout_ms)
{
    if (!g_v3d_caps.native_compute_enabled)
        return V3D_STATUS_UNSUPPORTED;
    if (!g_v3d_caps.native_selftest_ok)
        return V3D_STATUS_NOT_READY;
    if (!g_v3d_caps.mmio_csd)
        return V3D_STATUS_UNSUPPORTED;
    if (!cfg->csd_cfg_valid)
        return V3D_STATUS_NOT_IMPLEMENTED;

    /* Wait for CSD idle before submitting a new queue entry. */
    for (u32 i = 0; i < timeout_ms * 1000U; i++) {
        u32 st = mmio_read(g_v3d_caps.reg_base + V3D_CSD_STATUS_OLD_OFF);
        if ((st & V3D_CSD_STATUS_BUSY_MASK) == 0U)
            break;
        if (i + 1U == timeout_ms * 1000U)
            return V3D_STATUS_TIMEOUT;
        timer_delay_us(1);
    }

    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg1_off(), cfg->csd_cfg[1]);
    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg2_off(), cfg->csd_cfg[2]);
    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg3_off(), cfg->csd_cfg[3]);
    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg4_off(), cfg->csd_cfg[4]);
    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg5_off(), cfg->csd_cfg[5]);
    dmb();
    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg6_off(), cfg->csd_cfg[6]);
    dmb();
    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg0_off(), cfg->csd_cfg[0]);

    for (u32 i = 0; i < timeout_ms * 1000U; i++) {
        u32 st = mmio_read(g_v3d_caps.reg_base + V3D_CSD_STATUS_OLD_OFF);
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
                uart_puts("[v3d] auto: MMIO->mbox fallback\n");
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
    for (u32 i = 0; i < 7U; i++)
        cfg.csd_cfg[i] = k->csd_cfg[i];
    cfg.csd_cfg_valid = k->native_csd;
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
    g_kernels[id].native_csd = false;
    g_kernels[id].verified = false;
    g_kernels[id].control_list_bus = b->control_bus;
    v3d_update_tiny_kernel_state();
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
    if (uniform_bytes > V3D_UNIFORM_BYTES_MAX || shader_insts > V3D_SHADER_INSTS_MAX)
        return V3D_STATUS_INVALID;
    if (!g_v3d_caps.dispatch_supported)
        return V3D_STATUS_UNSUPPORTED;

    u64 shader_bytes64 = (u64)shader_insts * 8U;
    if (shader_bytes64 > 0xFFFFFFFFU)
        return V3D_STATUS_INVALID;
    u32 shader_bytes = (u32)shader_bytes64;
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
    uart_puts("[v3d] bind: ");
    uart_puts(g_kernels[id].name);
    uart_puts(" q=");
    uart_hex(g_kernels[id].qpu_count);
    uart_puts("\n");
    return V3D_STATUS_OK;
}
