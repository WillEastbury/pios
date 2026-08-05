#include "v3d.h"

#include "gpu.h"
#include "highmem.h"
#include "mailbox.h"
#include "mmu.h"
#include "mmio.h"
#include "platform.h"
#include "simd.h"
#include "timer.h"
#include "uart.h"
#include "videocore.h"
#include "v3d_tiny_qpu.h"
#include "v3d_gray_xor_qpu.h"
#include "v3d_gray_residual_qpu.h"
#include "v3d_matvec64_qpu.h"
#include "v3d_matmul64x16_qpu.h"
#include "v3d_bitnet_bitmap_qpu.h"

/* Legacy pre-Pi5 guess. Kept for comparison only; native Pi5 probing uses the
 * bcm2712-ds.dtsi V3D 7.1 ranges decoded by videocore.c. */
#define V3D_LEGACY_BASE_ADDR     (PERIPH_BASE + 0x00C04000UL)
#define V3D_PI5_HUB_BASE         0x1002000000UL
#define V3D_PI5_CORE0_BASE       0x1002008000UL
#define V3D_PI5_SMS_BASE         0x1002030800UL
#define V3D_PM_BASE              (PERIPH_BASE + 0x01200000UL)
#define V3D_PM_GRAFX_2712_OFF    0x00000304U
#define V3D_PM_PASSWORD          0x5A000000U
#define V3D_PM_V3DRSTN           (1U << 6)
#define V3D_FW_CLOCK_V3D         5U
#define V3D_TAG_SET_CLOCK_STATE  0x00038001U
#define V3D_IDENT0_OFF           0x000U
#define V3D_IDENT1_OFF           0x004U
#define V3D_IDENT2_OFF           0x008U
#define V3D_MAX_TIMEOUT_MS       5000U
#define V3D_DEFAULT_TIMEOUT_MS   25U
#define V3D_CACHE_TIMEOUT_US     100000U
#define V3D_L2TCACTL_OFF         0x00000030U
#define V3D_L2TCACTL_TMUWCF      0x00000100U
#define V3D_L2TCACTL_FLM_CLEAN   0x00000004U
#define V3D_L2TCACTL_L2TFLS      0x00000001U
#define V3D_SLCACTL_OFF          0x00000024U
#define V3D_SLCACTL_INV_ALL      0x0F0F0F0FU
#define V3D_L2TFLSTA_OFF         0x00000034U
#define V3D_L2TFLEND_OFF         0x00000038U
#define V3D_CORE_INT_STS_OFF     0x00000050U
#define V3D_CORE_INT_CLR_OFF     0x00000058U
#define V3D_CORE_INT_MSK_SET_OFF 0x00000060U
#define V3D_CORE_INT_MSK_CLR_OFF 0x00000064U
#define V3D_CORE_INT_CSDDONE_V71 (1U << 6)
#define V3D_CORE_INT_FLDONE      (1U << 1)
#define V3D_CORE_INT_FRDONE      (1U << 0)
#define V3D_CORE_INT_OUTOMEM     (1U << 2)
#define V3D_HUB_INT_STS_OFF      0x00000050U
#define V3D_HUB_INT_CLR_OFF      0x00000058U
#define V3D_HUB_INT_MSK_SET_OFF  0x00000060U
#define V3D_HUB_INT_MSK_CLR_OFF  0x00000064U
#define V3D_HUB_INT_TFUC         (1U << 1)
#define V3D_HUB_INT_MMU_CAP      (1U << 3)
#define V3D_HUB_INT_MMU_PTI      (1U << 4)
#define V3D_HUB_INT_MMU_WRV      (1U << 5)
#define V3D_HUB_INT_GMPV_V71     (1U << 6)
#define V3D_ERR_STAT_OFF         0x00000F20U
#define V3D_GMP_STATUS_V71_OFF   0x00000600U
#define V3D_GMP_STATUS_CFG_BUSY  0x00000008U
#define V3D_GMP_CFG_V71_OFF      0x00000604U
#define V3D_GMP_CFG_PROT_ENABLE  0x00000001U
#define V3D_GMP_VIO_ADDR_V71_OFF 0x00000608U
#define V3D_GMP_TABLE_ADDR_OFF   0x00000810U
#define V3D_GMP_CLEAR_LOAD_OFF   0x00000814U
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
#define V3D_MMU_PT_LOW_BASE      0x06000000UL
#define V3D_TINY_BLOB_BASE       (PIOS_FB_BACK_BASE + 0x00F00000UL)
#define V3D_TINY_BLOB_STRIDE     0x00000800UL

#define V3D_PTE_WRITEABLE        0x20000000U
#define V3D_PTE_VALID            0x10000000U

#define V3D_MMUC_CONTROL_OFF     0x00001000U
#define V3D_MMUC_CONTROL_FLUSHING 0x00000004U
#define V3D_MMUC_CONTROL_FLUSH   0x00000002U
#define V3D_MMUC_CONTROL_ENABLE  0x00000001U

#define V3D_MMU_CTL_OFF          0x00001200U
#define V3D_MMU_CTL_CAP_EXCEEDED_ABORT 0x04000000U
#define V3D_MMU_CTL_CAP_EXCEEDED_INT   0x02000000U
#define V3D_MMU_CTL_CAP_EXCEEDED       0x08000000U
#define V3D_MMU_CTL_PT_INVALID         0x00100000U
#define V3D_MMU_CTL_PT_INVALID_ENABLE  0x00010000U
#define V3D_MMU_CTL_PT_INVALID_ABORT   0x00080000U
#define V3D_MMU_CTL_PT_INVALID_INT     0x00040000U
#define V3D_MMU_CTL_WRITE_VIOLATION_ABORT 0x00000800U
#define V3D_MMU_CTL_WRITE_VIOLATION_INT   0x00000400U
#define V3D_MMU_CTL_WRITE_VIOLATION       0x00001000U
#define V3D_MMU_CTL_TLB_CLEARING 0x00000080U
#define V3D_MMU_CTL_TLB_CLEAR    0x00000004U
#define V3D_MMU_CTL_ENABLE       0x00000001U
#define V3D_MMU_PT_PA_BASE_OFF   0x00001204U
#define V3D_MMU_VIO_ID_OFF       0x0000122CU
#define V3D_MMU_ILLEGAL_ADDR_OFF 0x00001230U
#define V3D_MMU_ILLEGAL_ADDR_ENABLE 0x80000000U
#define V3D_MMU_VIO_ADDR_OFF     0x00001234U

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
#define V3D_CSD_QUEUED_CFG7_V71_OFF  0x0000094CU
#define V3D_CSD_CURRENT_CFG0_V71_OFF 0x00000958U
#define V3D_CSD_CURRENT_CFG5_V71_OFF 0x0000096CU
#define V3D_CSD_CURRENT_CFG6_V71_OFF 0x00000970U
#define V3D_CSD_STATUS_BUSY_MASK     0x0000000FU
#define V3D_CSD_STATUS_COMPLETED_MASK 0x00000FF0U
#define V3D_CSD_CFG012_WG_COUNT_SHIFT 16U
#define V3D_CSD_CFG3_WGS_PER_SG_SHIFT 8U
#define V3D_CSD_CFG3_BATCHES_PER_SG_M1_SHIFT 12U
#define V3D_CSD_CFG3_MAX_SG_ID_SHIFT 20U
#define V3D_CSD_CFG3_WG_SIZE_SHIFT 0U
#define V3D_CSD_CFG5_SINGLE_SEG      (1U << 1)
#define V3D_CSD_CFG5_THREADING       (1U << 0)
#define V3D_SMS_REE_CS_OFF           0x00000000U
#define V3D_SMS_STATE_READY          0x00000004U
#define V3D_SMS_STATE_MASK           0x0000000FU

static struct v3d_caps g_v3d_caps;
static struct v3d_csd_debug g_csd_debug;
static struct v3d_reset_debug g_reset_debug;
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
    [V3D_KERNEL_STORE_CONST] = { .id = V3D_KERNEL_STORE_CONST, .name = "store_const", .qpu_count = 4 },
    [V3D_KERNEL_LOAD_STORE]  = { .id = V3D_KERNEL_LOAD_STORE,  .name = "load_store",  .qpu_count = 4 },
    [V3D_KERNEL_STORE_SSBO]  = { .id = V3D_KERNEL_STORE_SSBO,  .name = "store_ssbo",  .qpu_count = 4 },
    [V3D_KERNEL_NOOP]        = { .id = V3D_KERNEL_NOOP,        .name = "noop",        .qpu_count = 4 },
    [V3D_KERNEL_PICOVM_ALU]  = { .id = V3D_KERNEL_PICOVM_ALU,  .name = "picovm_alu",  .qpu_count = 4 },
    [V3D_KERNEL_MATVEC16]    = { .id = V3D_KERNEL_MATVEC16,    .name = "matvec16",    .qpu_count = 4 },
    [V3D_KERNEL_GRAY_XOR64]   = { .id = V3D_KERNEL_GRAY_XOR64,   .name = "gray_xor64", .qpu_count = 4 },
    [V3D_KERNEL_GRAY_RESIDUAL64] = { .id = V3D_KERNEL_GRAY_RESIDUAL64, .name = "gray_residual64", .qpu_count = 4 },
    [V3D_KERNEL_MATVEC64]    = { .id = V3D_KERNEL_MATVEC64,    .name = "matvec64",    .qpu_count = 4 },
    [V3D_KERNEL_MATMUL64X16] = { .id = V3D_KERNEL_MATMUL64X16, .name = "matmul64x16", .qpu_count = 4 },
    [V3D_KERNEL_BITNET_BITMAP64X16] = { .id = V3D_KERNEL_BITNET_BITMAP64X16, .name = "bitnet_bitmap64x16", .qpu_count = 4 },
};
struct v3d_kernel_blob {
    u32 control_handle;
    u32 control_bus;
    u32 *control_ptr;
};
static struct v3d_kernel_blob g_kernel_blobs[V3D_KERNEL_MAX];
#if PIOS_ENABLE_NATIVE_V3D_COMPUTE
static u32 *g_v3d_pt;
static u8 g_v3d_scratch[V3D_MMU_SCRATCH_BYTES] ALIGNED(4096);
static u32 g_tiny_store_uniforms[4] ALIGNED(64);
static u32 g_tiny_store_ssbo_uniforms[4] ALIGNED(64);
static u32 g_tiny_load_store_uniforms[4] ALIGNED(64);
static u32 g_tiny_add_uniforms[4] ALIGNED(64);
static u32 g_tiny_relu_uniforms[4] ALIGNED(64);
static u64 g_tiny_store_shader[V3D_TINY_STORE_CONST_QPU_WORDS] ALIGNED(64);
static u64 g_tiny_store_ssbo_shader[V3D_TINY_STORE_SSBO_QPU_WORDS] ALIGNED(64);
static u64 g_tiny_load_store_shader[V3D_TINY_LOAD_STORE_QPU_WORDS] ALIGNED(64);
static u64 g_tiny_add_shader[V3D_TINY_VECTOR_ADD_QPU_WORDS] ALIGNED(64);
static u64 g_tiny_relu_shader[V3D_TINY_RELU_QPU_WORDS] ALIGNED(64);
static u64 g_tiny_noop_shader[V3D_TINY_NOOP_QPU_WORDS] ALIGNED(64);
static u8 g_bitnet_spill[65536U] ALIGNED(4096);
#endif

#if PIOS_ENABLE_NATIVE_V3D_COMPUTE
static bool v3d_firmware_clock_enable(void)
{
    static volatile u32 mb[8] ALIGNED(16);
    mb[0] = 8U * 4U;
    mb[1] = 0;
    mb[2] = V3D_TAG_SET_CLOCK_STATE;
    mb[3] = 8;
    mb[4] = 8;
    mb[5] = V3D_FW_CLOCK_V3D;
    mb[6] = 3; /* on + wait */
    mb[7] = 0;
    return mbox_call(MBOX_CH_PROP, mb);
}

static bool v3d_ident_plausible(u32 ident0)
{
    if (ident0 == 0U || ident0 == 0xFFFFFFFFU)
        return false;
    return true;
}

/*
 * Blind power-domain reset of the V3D block via the PM GRAFX register only.
 * Touches NO V3D core MMIO (a wedged CSD queue can stall those reads, which is
 * why the full v3d_pm_reset() path hangs). Asserting/deasserting PM_V3DRSTN
 * power-cycles the V3D core and clears a CSD-queue wedge that survives a warm
 * reboot (e.g. after an oversized synchronous dispatch storm). Safe to run on
 * every boot before native init: the 3D/compute core reset is independent of
 * the firmware-owned HVS/HDMI display path.
 */
static void v3d_blind_pm_reset(void)
{
    u64 pm_reg = V3D_PM_BASE + V3D_PM_GRAFX_2712_OFF;
    u32 pm_before = mmio_read(pm_reg);
    mmio_write(pm_reg, V3D_PM_PASSWORD | (pm_before & ~V3D_PM_V3DRSTN));
    dsb();
    timer_delay_ms(10);
    mmio_write(pm_reg, V3D_PM_PASSWORD | (mmio_read(pm_reg) | V3D_PM_V3DRSTN));
    dsb();
    timer_delay_ms(10);
    (void)v3d_firmware_clock_enable();
}
#endif

static void v3d_update_tiny_kernel_state(void)
{
    g_v3d_caps.tiny_ready_mask = 0;
    g_v3d_caps.tiny_verified_mask = 0;

    if (g_kernels[V3D_KERNEL_ADD].ready)
        g_v3d_caps.tiny_ready_mask |= (1U << V3D_KERNEL_ADD);
    if (g_kernels[V3D_KERNEL_MUL].ready)
        g_v3d_caps.tiny_ready_mask |= (1U << V3D_KERNEL_MUL);
    if (g_kernels[V3D_KERNEL_RELU].ready)
        g_v3d_caps.tiny_ready_mask |= (1U << V3D_KERNEL_RELU);
    if (g_kernels[V3D_KERNEL_ADD].verified)
        g_v3d_caps.tiny_verified_mask |= (1U << V3D_KERNEL_ADD);
    if (g_kernels[V3D_KERNEL_MUL].verified)
        g_v3d_caps.tiny_verified_mask |= (1U << V3D_KERNEL_MUL);
    if (g_kernels[V3D_KERNEL_RELU].verified)
        g_v3d_caps.tiny_verified_mask |= (1U << V3D_KERNEL_RELU);

    g_v3d_caps.native_tiny_kernels_ready =
        ((g_v3d_caps.tiny_verified_mask & ((1U << V3D_KERNEL_ADD) |
                                           (1U << V3D_KERNEL_MUL) |
                                           (1U << V3D_KERNEL_RELU))) ==
         ((1U << V3D_KERNEL_ADD) | (1U << V3D_KERNEL_MUL) | (1U << V3D_KERNEL_RELU)));
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

static u32 v3d_core_irq_mask(void)
{
    return V3D_CORE_INT_FLDONE | V3D_CORE_INT_FRDONE |
           V3D_CORE_INT_OUTOMEM | V3D_CORE_INT_CSDDONE_V71;
}

static u32 v3d_hub_irq_mask(void)
{
    return V3D_HUB_INT_TFUC | V3D_HUB_INT_MMU_CAP | V3D_HUB_INT_MMU_PTI |
           V3D_HUB_INT_MMU_WRV | V3D_HUB_INT_GMPV_V71;
}

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

static bool v3d_gmp_allow_all(void)
{
    if (!g_v3d_caps.mmio_probe_ok || g_v3d_caps.reg_base == 0U)
        return false;
    if (g_v3d_caps.tech_version < 71U)
        return true;
    mmio_write(g_v3d_caps.reg_base + V3D_GMP_CFG_V71_OFF, V3D_GMP_CFG_PROT_ENABLE);
    mmio_write(g_v3d_caps.reg_base + V3D_GMP_TABLE_ADDR_OFF, 0U);
    mmio_write(g_v3d_caps.reg_base + V3D_GMP_CLEAR_LOAD_OFF, 0xFFFFFFFFU);
    for (u32 i = 0; i < 1000U; i++) {
        if ((mmio_read(g_v3d_caps.reg_base + V3D_GMP_STATUS_V71_OFF) &
             V3D_GMP_STATUS_CFG_BUSY) == 0U)
            return true;
        timer_delay_us(1);
    }
    return false;
}

static bool v3d_clean_caches(void)
{
    if (!g_v3d_caps.mmio_probe_ok || g_v3d_caps.reg_base == 0U)
        return false;

    g_csd_debug.tmuwcf_wait_us = 0U;
    g_csd_debug.l2t_clean_wait_us = 0U;
    g_csd_debug.cache_clean_ok = 0U;
    mmio_write(g_v3d_caps.reg_base + V3D_L2TCACTL_OFF, V3D_L2TCACTL_TMUWCF);
    dsb();
    for (u32 i = 0; i < V3D_CACHE_TIMEOUT_US; i++) {
        u32 state = mmio_read(g_v3d_caps.reg_base + V3D_L2TCACTL_OFF);
        g_csd_debug.l2t_after_tmuwcf = state;
        if ((state & V3D_L2TCACTL_TMUWCF) == 0U) {
            g_csd_debug.tmuwcf_wait_us = i;
            break;
        }
        timer_delay_us(1);
        if (i + 1U == V3D_CACHE_TIMEOUT_US) {
            g_csd_debug.tmuwcf_wait_us = V3D_CACHE_TIMEOUT_US;
            return false;
        }
    }

    mmio_write(g_v3d_caps.reg_base + V3D_L2TCACTL_OFF,
               V3D_L2TCACTL_L2TFLS | V3D_L2TCACTL_FLM_CLEAN);
    dsb();
    for (u32 i = 0; i < V3D_CACHE_TIMEOUT_US; i++) {
        u32 state = mmio_read(g_v3d_caps.reg_base + V3D_L2TCACTL_OFF);
        g_csd_debug.l2t_after_clean = state;
        if ((state & V3D_L2TCACTL_L2TFLS) == 0U) {
            g_csd_debug.l2t_clean_wait_us = i;
            g_csd_debug.cache_clean_ok = 1U;
            return true;
        }
        timer_delay_us(1);
    }
    g_csd_debug.l2t_clean_wait_us = V3D_CACHE_TIMEOUT_US;
    return false;
}

static void v3d_init_core_cache_state(void)
{
    if (!g_v3d_caps.mmio_probe_ok || g_v3d_caps.reg_base == 0U)
        return;
    mmio_write(g_v3d_caps.reg_base + V3D_L2TFLSTA_OFF, 0U);
    mmio_write(g_v3d_caps.reg_base + V3D_L2TFLEND_OFF, 0xFFFFFFFFU);
}

static bool v3d_invalidate_caches(void)
{
    if (!g_v3d_caps.mmio_probe_ok || g_v3d_caps.reg_base == 0U)
        return false;
    g_csd_debug.l2t_before_invalidate =
        mmio_read(g_v3d_caps.reg_base + V3D_L2TCACTL_OFF);
    g_csd_debug.l2t_invalidate_wait_us = 0U;
    mmio_write(g_v3d_caps.reg_base + V3D_L2TCACTL_OFF, V3D_L2TCACTL_L2TFLS);
    mmio_write(g_v3d_caps.reg_base + V3D_SLCACTL_OFF, V3D_SLCACTL_INV_ALL);
    dsb();
    for (u32 i = 0; i < V3D_CACHE_TIMEOUT_US; i++) {
        u32 state = mmio_read(g_v3d_caps.reg_base + V3D_L2TCACTL_OFF);
        g_csd_debug.l2t_after_invalidate = state;
        if ((state & V3D_L2TCACTL_L2TFLS) == 0U) {
            g_csd_debug.l2t_invalidate_wait_us = i;
            return true;
        }
        timer_delay_us(1);
    }
    g_csd_debug.l2t_invalidate_wait_us = V3D_CACHE_TIMEOUT_US;
    return false;
}

static void v3d_sms_enter_ree(void)
{
    if (g_v3d_caps.tech_version < 71U || g_v3d_caps.sms_base == 0U)
        return;
    mmio_write(g_v3d_caps.sms_base + V3D_SMS_REE_CS_OFF, V3D_SMS_STATE_READY);
    for (u32 i = 0; i < 100U; i++) {
        u32 state = mmio_read(g_v3d_caps.sms_base + V3D_SMS_REE_CS_OFF) & V3D_SMS_STATE_MASK;
        if (state != 5U && state != 6U)
            return;
        timer_delay_us(1);
    }
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
    simd_zero(&g_csd_debug, sizeof(g_csd_debug));
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
#endif
    }

#if PIOS_ENABLE_NATIVE_V3D_COMPUTE
    if (!g_v3d_caps.mmio_probe_ok) {
        u32 ident0 = mmio_read(V3D_LEGACY_BASE_ADDR + V3D_IDENT0_OFF);
        if (v3d_ident_plausible(ident0)) {
            g_v3d_caps.mmio_probe_ok = true;
            g_v3d_caps.reg_base = V3D_LEGACY_BASE_ADDR;
            g_v3d_caps.ident0 = ident0;
            g_v3d_caps.ident1 = mmio_read(V3D_LEGACY_BASE_ADDR + V3D_IDENT1_OFF);
            g_v3d_caps.ident2 = mmio_read(V3D_LEGACY_BASE_ADDR + V3D_IDENT2_OFF);
        }
    }
#endif
#if PIOS_ENABLE_NATIVE_V3D_COMPUTE && PIOS_ENABLE_NATIVE_V3D_MMU_BOOT
    if (g_v3d_caps.mmio_probe_ok) {
        v3d_blind_pm_reset();
        (void)v3d_firmware_clock_enable();
        v3d_sms_enter_ree();
        (void)v3d_native_mmu_setup();
        (void)v3d_gmp_allow_all();
    }
#endif
    v3d_init_core_cache_state();
    g_v3d_caps.native_selftest_status = v3d_native_selftest();
    g_v3d_caps.dispatch_supported = g_v3d_caps.mailbox_qpu ||
                                    (g_v3d_caps.mmio_csd &&
                                     g_v3d_caps.native_mmu_ready);
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
    if (id != V3D_KERNEL_ADD && id != V3D_KERNEL_MUL && id != V3D_KERNEL_RELU &&
        id != V3D_KERNEL_STORE_CONST && id != V3D_KERNEL_LOAD_STORE &&
        id != V3D_KERNEL_STORE_SSBO && id != V3D_KERNEL_NOOP &&
        id != V3D_KERNEL_PICOVM_ALU && id != V3D_KERNEL_MATVEC16 &&
        id != V3D_KERNEL_GRAY_XOR64 && id != V3D_KERNEL_GRAY_RESIDUAL64 &&
        id != V3D_KERNEL_MATVEC64 && id != V3D_KERNEL_MATMUL64X16 &&
        id != V3D_KERNEL_BITNET_BITMAP64X16)
        return V3D_STATUS_NOT_IMPLEMENTED;
    if (qpu_count == 0U || qpu_count > V3D_QPU_MAX_DISPATCH)
        return V3D_STATUS_INVALID;
    if (!g_v3d_caps.native_selftest_ok)
        return V3D_STATUS_NOT_READY;

    for (u32 i = 0; i < 7U; i++)
        g_kernels[id].csd_cfg[i] = csd_cfg[i];
    g_kernels[id].qpu_count = qpu_count;
    g_kernels[id].control_list_bus = 0U; /* Native CSD configs do not use mailbox control lists. */
    g_kernels[id].native_csd = true;
    g_kernels[id].verified = false;
    g_kernels[id].ready = true;
    v3d_update_tiny_kernel_state();
    return V3D_STATUS_OK;
}

/*
 * Mesa-parity supergroup packing for direct CSD. When the workgroup size is a
 * multiple of 16 lanes are already saturated (1 wg/supergroup); otherwise pack
 * multiple workgroups per supergroup to fill the 16-lane batch. Mirrors
 * v3d_csd_choose_workgroups_per_supergroup (no TSY barrier path).
 */
static u32 v3d_choose_wgs_per_sg(u32 num_wgs, u32 wg_size)
{
    if (wg_size % 16U == 0U)
        return 1U;
    u32 best = 1U;
    u32 best_unused = 16U;
    for (u32 w = 1U; w <= 16U; w++) {
        if (w > num_wgs)
            return best;
        u32 unused = (16U - ((w * wg_size) % 16U)) & 0x0fU;
        if (unused == 0U)
            return w;
        if (unused < best_unused) {
            best = w;
            best_unused = unused;
        }
    }
    return best;
}

v3d_status_t v3d_kernel_bind_builtin_qpu_grid(v3d_kernel_id_t id,
                                              const void *uniform_data,
                                              u32 uniform_bytes,
                                              u32 workgroups_x)
{
#if PIOS_ENABLE_NATIVE_V3D_COMPUTE && PIOS_ENABLE_TINY_QPU_KERNELS
    if (g_v3d_caps.native_mmu_ready && g_v3d_caps.mmio_csd) {
        const u64 *src_shader = NULL;
        u64 *dst_shader = NULL;
        u32 *dst_uniforms = NULL;
        u32 shader_words = 0;
        u32 uniform_count = 0;
        u32 wg_size = 1U;

        if (workgroups_x == 0U || workgroups_x > 1024U)
            return V3D_STATUS_INVALID;

        if (id == V3D_KERNEL_STORE_CONST) {
            src_shader = v3d_tiny_store_const_qpu;
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 0U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 1U * V3D_TINY_BLOB_STRIDE);
            shader_words = V3D_TINY_STORE_CONST_QPU_WORDS;
            uniform_count = 2U; /* relocated destination + value */
        } else if (id == V3D_KERNEL_STORE_SSBO) {
            src_shader = v3d_tiny_store_ssbo_qpu;
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 2U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 3U * V3D_TINY_BLOB_STRIDE);
            shader_words = V3D_TINY_STORE_SSBO_QPU_WORDS;
            uniform_count = 3U;
        } else if (id == V3D_KERNEL_LOAD_STORE) {
            src_shader = v3d_tiny_load_store_qpu;
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 4U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 5U * V3D_TINY_BLOB_STRIDE);
            shader_words = V3D_TINY_LOAD_STORE_QPU_WORDS;
            uniform_count = 2U; /* relocated source + destination */
        } else if (id == V3D_KERNEL_NOOP) {
            src_shader = v3d_tiny_noop_qpu;
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 6U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 7U * V3D_TINY_BLOB_STRIDE);
            shader_words = V3D_TINY_NOOP_QPU_WORDS;
            uniform_count = 0U;
        } else if (id == V3D_KERNEL_PICOVM_ALU) {
            src_shader = v3d_tiny_picovm_alu_qpu;
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 14U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 15U * V3D_TINY_BLOB_STRIDE);
            shader_words = V3D_TINY_PICOVM_ALU_QPU_WORDS;
            uniform_count = 2U;
        } else if (id == V3D_KERNEL_MATVEC16) {
            src_shader = v3d_tiny_matvec16_qpu;
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 16U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 17U * V3D_TINY_BLOB_STRIDE);
            shader_words = V3D_TINY_MATVEC16_QPU_WORDS;
            uniform_count = 23U;
        } else if (id == V3D_KERNEL_GRAY_XOR64) {
            src_shader = v3d_tiny_gray_xor64_qpu;
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 18U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 19U * V3D_TINY_BLOB_STRIDE);
            shader_words = V3D_TINY_GRAY_XOR64_QPU_WORDS;
            uniform_count = 15U;
        } else if (id == V3D_KERNEL_GRAY_RESIDUAL64) {
            src_shader = v3d_tiny_gray_residual64_qpu;
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 20U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 21U * V3D_TINY_BLOB_STRIDE);
            shader_words = V3D_TINY_GRAY_RESIDUAL64_QPU_WORDS;
            uniform_count = 6U;
        } else if (id == V3D_KERNEL_MATVEC64) {
            src_shader = v3d_tiny_matvec64_qpu;
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 22U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 23U * V3D_TINY_BLOB_STRIDE);
            shader_words = V3D_TINY_MATVEC64_QPU_WORDS;
            uniform_count = V3D_TINY_MATVEC64_UNIFORMS;
        } else if (id == V3D_KERNEL_MATMUL64X16) {
            src_shader = v3d_tiny_matmul64x16_qpu;
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 24U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 25U * V3D_TINY_BLOB_STRIDE);
            shader_words = V3D_TINY_MATMUL64X16_QPU_WORDS;
            uniform_count = V3D_TINY_MATMUL64X16_UNIFORMS;
        } else if (id == V3D_KERNEL_BITNET_BITMAP64X16) {
            src_shader = v3d_bitnet_bitmap64x16_qpu;
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 26U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 30U * V3D_TINY_BLOB_STRIDE);
            shader_words = V3D_BITNET_BITMAP64X16_QPU_WORDS;
            uniform_count = V3D_BITNET_BITMAP64X16_UNIFORMS;
        } else if (id == V3D_KERNEL_ADD) {
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 8U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 9U * V3D_TINY_BLOB_STRIDE);
            if (uniform_bytes == 7U * sizeof(u32)) {
                src_shader = v3d_tiny_vector_add_n_qpu;
                shader_words = V3D_TINY_VECTOR_ADD_N_QPU_WORDS;
                uniform_count = 7U;
            } else {
                src_shader = v3d_tiny_vector_add_qpu;
                shader_words = V3D_TINY_VECTOR_ADD_QPU_WORDS;
                uniform_count = 3U;
            }
            wg_size = 16U;
        } else if (id == V3D_KERNEL_MUL) {
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 12U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 13U * V3D_TINY_BLOB_STRIDE);
            if (uniform_bytes == 7U * sizeof(u32)) {
                src_shader = v3d_tiny_vector_mul_n_qpu;
                shader_words = V3D_TINY_VECTOR_MUL_N_QPU_WORDS;
                uniform_count = 7U;
            } else {
                src_shader = v3d_tiny_vector_mul_qpu;
                shader_words = V3D_TINY_VECTOR_MUL_QPU_WORDS;
                uniform_count = 3U;
            }
            wg_size = 16U;
        } else if (id == V3D_KERNEL_RELU) {
            dst_shader = (u64 *)(usize)(V3D_TINY_BLOB_BASE + 10U * V3D_TINY_BLOB_STRIDE);
            dst_uniforms = (u32 *)(usize)(V3D_TINY_BLOB_BASE + 11U * V3D_TINY_BLOB_STRIDE);
            if (uniform_bytes == 6U * sizeof(u32)) {
                src_shader = v3d_tiny_relu_n_qpu;
                shader_words = V3D_TINY_RELU_N_QPU_WORDS;
                uniform_count = 6U;
            } else {
                src_shader = v3d_tiny_relu_qpu;
                shader_words = V3D_TINY_RELU_QPU_WORDS;
                uniform_count = 2U;
            }
            wg_size = 16U;
        }

        if (!src_shader || (!uniform_data && uniform_count != 0U))
            return V3D_STATUS_INVALID;
        u32 uniform_storage_bytes = (uniform_count * sizeof(u32) + 63U) & ~63U;
        if (dst_uniforms)
            simd_zero(dst_uniforms, uniform_storage_bytes);
        if (id == V3D_KERNEL_BITNET_BITMAP64X16 &&
            uniform_bytes == 3U * sizeof(u32)) {
            const u32 *buffers = (const u32 *)uniform_data;
            simd_zero(g_bitnet_spill, sizeof(g_bitnet_spill));
            dcache_clean_range((u64)(usize)g_bitnet_spill, sizeof(g_bitnet_spill));
            for (u32 i = 0; i < uniform_count; i++) {
                u8 kind = v3d_bitnet_bitmap64x16_uniform_kind[i];
                u32 data = v3d_bitnet_bitmap64x16_uniform_data[i];
                dst_uniforms[i] = kind == 53U ? buffers[data] :
                                  (kind == 65U ? (u32)(usize)g_bitnet_spill :
                                   (kind == 66U ?
                                    V3D_BITNET_BITMAP64X16_SPILL_PER_THREAD : data));
            }
        } else if (uniform_bytes == uniform_count * sizeof(u32)) {
            const u32 *src32 = (const u32 *)uniform_data;
            for (u32 i = 0; i < uniform_count; i++)
                dst_uniforms[i] = src32[i];
        } else if (uniform_bytes == uniform_count * sizeof(u64)) {
            const u64 *src64 = (const u64 *)uniform_data;
            for (u32 i = 0; i < uniform_count; i++)
                dst_uniforms[i] = (u32)src64[i];
        } else {
            return V3D_STATUS_INVALID;
        }

        memcpy(dst_shader, src_shader, shader_words * sizeof(u64));
        if (dst_uniforms)
            dcache_clean_range((u64)(usize)dst_uniforms, uniform_storage_bytes);
        dcache_clean_range((u64)(usize)dst_shader, shader_words * sizeof(u64));

        u32 cfg[7] = { 0 };
        cfg[0] = workgroups_x << V3D_CSD_CFG012_WG_COUNT_SHIFT;
        cfg[1] = 1U << V3D_CSD_CFG012_WG_COUNT_SHIFT;
        cfg[2] = 1U << V3D_CSD_CFG012_WG_COUNT_SHIFT;
        if (id == V3D_KERNEL_STORE_SSBO)
            wg_size = 16U;
        /*
         * Mesa-parity supergroup/batch packing. For wg_size=16 this reduces to
         * 1 wg/supergroup, num_batches=num_wgs, max_sg_id=num_wgs-1 (the proven
         * vector-N config). For wg_size=1 (matvec) it packs up to 16 rows per
         * supergroup so the 16-lane batch is filled, otherwise CSD completes but
         * writes nothing.
         */
        u32 num_wgs = workgroups_x;
        u32 wgs_per_sg = v3d_choose_wgs_per_sg(num_wgs, wg_size);
        if (wgs_per_sg == 0U)
            wgs_per_sg = 1U;
        u32 batches_per_sg = (wgs_per_sg * wg_size + 15U) / 16U;
        if (batches_per_sg == 0U)
            batches_per_sg = 1U;
        u32 whole_sgs = num_wgs / wgs_per_sg;
        u32 rem_wgs = num_wgs - whole_sgs * wgs_per_sg;
        u32 num_sgs = whole_sgs + (rem_wgs ? 1U : 0U);
        u32 num_batches = batches_per_sg * whole_sgs +
                          (rem_wgs * wg_size + 15U) / 16U;
        cfg[3] = ((wgs_per_sg & 0xfU) << V3D_CSD_CFG3_WGS_PER_SG_SHIFT) |
                 ((batches_per_sg - 1U) << V3D_CSD_CFG3_BATCHES_PER_SG_M1_SHIFT) |
                 (((num_sgs - 1U) & 0x3fU) << V3D_CSD_CFG3_MAX_SG_ID_SHIFT) |
                 ((wg_size & 0xffU) << V3D_CSD_CFG3_WG_SIZE_SHIFT);
        /*
         * Live BCM2712 CSD requires the raw batch count. Using the older
         * count-minus-one encoding completes with core interrupt 0x40 but
         * executes no QPU instructions, including for a one-batch dispatch.
         */
        cfg[4] = num_batches;
        /* Metadata from the Mesa 26.2/26.3 wrappers:
         * all current memory kernels are threads=4, single_seg=0; only the
         * three-instruction noop is threads=4, single_seg=1. */
        cfg[5] = (u32)(usize)dst_shader;
        if (id != V3D_KERNEL_BITNET_BITMAP64X16)
            cfg[5] |= V3D_CSD_CFG5_THREADING;
        if (id == V3D_KERNEL_NOOP)
            cfg[5] |= V3D_CSD_CFG5_SINGLE_SEG;
        cfg[6] = (u32)(usize)dst_uniforms;
        return v3d_kernel_bind_csd(id, cfg, 1U);
    }
#endif

    if (workgroups_x != 1U)
        return V3D_STATUS_NOT_IMPLEMENTED;

    switch (id) {
    case V3D_KERNEL_STORE_CONST:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_store_const_qpu,
                                    V3D_TINY_STORE_CONST_QPU_WORDS);
    case V3D_KERNEL_STORE_SSBO:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_store_ssbo_qpu,
                                    V3D_TINY_STORE_SSBO_QPU_WORDS);
    case V3D_KERNEL_NOOP:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_noop_qpu,
                                    V3D_TINY_NOOP_QPU_WORDS);
    case V3D_KERNEL_PICOVM_ALU:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_picovm_alu_qpu,
                                    V3D_TINY_PICOVM_ALU_QPU_WORDS);
    case V3D_KERNEL_MATVEC16:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_matvec16_qpu,
                                    V3D_TINY_MATVEC16_QPU_WORDS);
    case V3D_KERNEL_GRAY_XOR64:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_gray_xor64_qpu,
                                    V3D_TINY_GRAY_XOR64_QPU_WORDS);
    case V3D_KERNEL_GRAY_RESIDUAL64:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_gray_residual64_qpu,
                                    V3D_TINY_GRAY_RESIDUAL64_QPU_WORDS);
    case V3D_KERNEL_MATVEC64:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_matvec64_qpu,
                                    V3D_TINY_MATVEC64_QPU_WORDS);
    case V3D_KERNEL_MATMUL64X16:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_matmul64x16_qpu,
                                    V3D_TINY_MATMUL64X16_QPU_WORDS);
    case V3D_KERNEL_BITNET_BITMAP64X16:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_bitnet_bitmap64x16_qpu,
                                    V3D_BITNET_BITMAP64X16_QPU_WORDS);
    case V3D_KERNEL_LOAD_STORE:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_load_store_qpu,
                                    V3D_TINY_LOAD_STORE_QPU_WORDS);
    case V3D_KERNEL_ADD:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_vector_add_qpu,
                                    V3D_TINY_VECTOR_ADD_QPU_WORDS);
    case V3D_KERNEL_MUL:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_vector_mul_qpu,
                                    V3D_TINY_VECTOR_MUL_QPU_WORDS);
    case V3D_KERNEL_RELU:
        return v3d_kernel_bind_blob(id,
                                    uniform_data, uniform_bytes,
                                    v3d_tiny_relu_qpu,
                                    V3D_TINY_RELU_QPU_WORDS);
    default:
        return V3D_STATUS_NOT_IMPLEMENTED;
    }
}

v3d_status_t v3d_kernel_bind_builtin_qpu(v3d_kernel_id_t id,
                                         const void *uniform_data,
                                         u32 uniform_bytes)
{
    return v3d_kernel_bind_builtin_qpu_grid(id, uniform_data, uniform_bytes, 1U);
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

    if (!g_v3d_pt)
        g_v3d_pt = (u32 *)(usize)V3D_MMU_PT_LOW_BASE;

    for (u32 page = 0; page < V3D_MMU_PT_ENTRIES; page++)
        g_v3d_pt[page] = V3D_PTE_VALID | V3D_PTE_WRITEABLE | page;

    g_v3d_caps.pt_paddr = (u64)(usize)g_v3d_pt;
    g_v3d_caps.scratch_paddr = (u64)(usize)g_v3d_scratch;
    dcache_clean_range((u64)(usize)g_v3d_pt,
                       (u64)V3D_MMU_PT_ENTRIES * sizeof(u32));
    dcache_clean_range((u64)(usize)g_v3d_scratch, sizeof(g_v3d_scratch));

    mmio_write(g_v3d_caps.hub_base + V3D_MMU_PT_PA_BASE_OFF,
               (u32)(g_v3d_caps.pt_paddr >> V3D_MMU_PAGE_SHIFT));
    /*
     * Bring-up must not arm V3D MMU abort/interrupt reporting before PIOS has
     * a V3D fault ISR. The first hardware trial proved identity table install
     * and selftest could pass, but the board later PiSODed/rebooted without any
     * tiny kernel ever becoming ready/dispatched. That points at an async V3D
     * MMU fault path surfacing after boot. Keep fault reporting poll-only for
     * now; dispatch paths already fail closed via status/timeout/quarantine.
     */
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
    g_v3d_caps.mmio_csd = true;
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

    g_csd_debug.status_before = mmio_read(g_v3d_caps.reg_base + V3D_CSD_STATUS_OLD_OFF);
    u32 completed_before = g_csd_debug.status_before & V3D_CSD_STATUS_COMPLETED_MASK;
    g_csd_debug.mmu_vio_addr = 0U;
    g_csd_debug.mmu_vio_id = 0U;
    (void)v3d_gmp_allow_all();
    mmio_write(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF, 0xFFFFFFFFU);
    /* MMU fault status bits are W1C. Linux clears them by writing the current
     * MMU_CTL value back before the next job; preserve the configured abort/
     * interrupt policy while clearing stale WRV/PTI/CAP status. */
    mmio_write(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF,
               mmio_read(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF));
    mmio_write(g_v3d_caps.reg_base + V3D_CORE_INT_CLR_OFF, V3D_CORE_INT_CSDDONE_V71);
    if (!v3d_invalidate_caches())
        return V3D_STATUS_FAILED;

    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg1_off(), cfg->csd_cfg[1]);
    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg2_off(), cfg->csd_cfg[2]);
    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg3_off(), cfg->csd_cfg[3]);
    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg4_off(), cfg->csd_cfg[4]);
    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg5_off(), cfg->csd_cfg[5]);
    dmb();
    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg6_off(), cfg->csd_cfg[6]);
    if (g_v3d_caps.tech_version >= 71U)
        mmio_write(g_v3d_caps.reg_base + V3D_CSD_QUEUED_CFG7_V71_OFF, 0);
    dmb();
    mmio_write(g_v3d_caps.reg_base + v3d_csd_cfg0_off(), cfg->csd_cfg[0]);
    dmb();
    g_csd_debug.status_after_kick = mmio_read(g_v3d_caps.reg_base + V3D_CSD_STATUS_OLD_OFF);

    for (u32 i = 0; i < timeout_ms * 1000U; i++) {
        u32 st = mmio_read(g_v3d_caps.reg_base + V3D_CSD_STATUS_OLD_OFF);
        u32 mmu_st = mmio_read(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF);
        if ((mmu_st & (V3D_MMU_CTL_WRITE_VIOLATION |
                       V3D_MMU_CTL_PT_INVALID |
                       V3D_MMU_CTL_CAP_EXCEEDED)) != 0U &&
            g_csd_debug.mmu_vio_id == 0U) {
            g_csd_debug.mmu_ctl = mmu_st;
            g_csd_debug.mmu_vio_addr =
                mmio_read(g_v3d_caps.hub_base + V3D_MMU_VIO_ADDR_OFF);
            g_csd_debug.mmu_vio_id =
                mmio_read(g_v3d_caps.hub_base + V3D_MMU_VIO_ID_OFF);
        }
        if ((st & V3D_CSD_STATUS_COMPLETED_MASK) != completed_before &&
            (st & V3D_CSD_STATUS_BUSY_MASK) == 0U) {
            g_csd_debug.status_after_wait = st;
            g_csd_debug.core_int_sts = mmio_read(g_v3d_caps.reg_base + V3D_CORE_INT_STS_OFF);
            g_csd_debug.hub_int_sts = mmio_read(g_v3d_caps.hub_base + V3D_HUB_INT_STS_OFF);
            g_csd_debug.err_stat = mmio_read(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF);
            if (g_csd_debug.mmu_ctl == 0U)
                g_csd_debug.mmu_ctl = mmio_read(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF);
            g_csd_debug.mmu_illegal_addr = mmio_read(g_v3d_caps.hub_base + V3D_MMU_ILLEGAL_ADDR_OFF);
            if (g_csd_debug.mmu_vio_id == 0U) {
                g_csd_debug.mmu_vio_addr = mmio_read(g_v3d_caps.hub_base + V3D_MMU_VIO_ADDR_OFF);
                g_csd_debug.mmu_vio_id = mmio_read(g_v3d_caps.hub_base + V3D_MMU_VIO_ID_OFF);
            }
            g_csd_debug.gmp_status = mmio_read(g_v3d_caps.reg_base + V3D_GMP_STATUS_V71_OFF);
            g_csd_debug.gmp_cfg = mmio_read(g_v3d_caps.reg_base + V3D_GMP_CFG_V71_OFF);
            g_csd_debug.gmp_vio_addr = mmio_read(g_v3d_caps.reg_base + V3D_GMP_VIO_ADDR_V71_OFF);
            g_csd_debug.current_cfg0 = mmio_read(g_v3d_caps.reg_base + V3D_CSD_CURRENT_CFG0_V71_OFF);
            g_csd_debug.current_cfg5 = mmio_read(g_v3d_caps.reg_base + V3D_CSD_CURRENT_CFG5_V71_OFF);
            g_csd_debug.current_cfg6 = mmio_read(g_v3d_caps.reg_base + V3D_CSD_CURRENT_CFG6_V71_OFF);
            if (!v3d_clean_caches())
                return V3D_STATUS_FAILED;
            dmb();
            return V3D_STATUS_OK;
        }
        timer_delay_us(1);
    }
    g_csd_debug.status_after_wait = mmio_read(g_v3d_caps.reg_base + V3D_CSD_STATUS_OLD_OFF);
    g_csd_debug.core_int_sts = mmio_read(g_v3d_caps.reg_base + V3D_CORE_INT_STS_OFF);
    g_csd_debug.hub_int_sts = mmio_read(g_v3d_caps.hub_base + V3D_HUB_INT_STS_OFF);
    g_csd_debug.err_stat = mmio_read(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF);
    g_csd_debug.mmu_ctl = mmio_read(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF);
    g_csd_debug.mmu_vio_addr = mmio_read(g_v3d_caps.hub_base + V3D_MMU_VIO_ADDR_OFF);
    g_csd_debug.mmu_vio_id = mmio_read(g_v3d_caps.hub_base + V3D_MMU_VIO_ID_OFF);
    g_csd_debug.gmp_status = mmio_read(g_v3d_caps.reg_base + V3D_GMP_STATUS_V71_OFF);
    g_csd_debug.gmp_cfg = mmio_read(g_v3d_caps.reg_base + V3D_GMP_CFG_V71_OFF);
    g_csd_debug.gmp_vio_addr = mmio_read(g_v3d_caps.reg_base + V3D_GMP_VIO_ADDR_V71_OFF);
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

void v3d_csd_debug_last(struct v3d_csd_debug *out)
{
    if (out)
        *out = g_csd_debug;
}

void v3d_reset_debug_last(struct v3d_reset_debug *out)
{
    if (out)
        *out = g_reset_debug;
}

v3d_status_t v3d_soft_reset(void)
{
#if !PIOS_ENABLE_NATIVE_V3D_COMPUTE
    return V3D_STATUS_UNSUPPORTED;
#else
    if (!g_v3d_caps.native_compute_enabled || !g_v3d_caps.mmio_probe_ok ||
        !g_v3d_caps.native_mmu_ready || g_v3d_caps.reg_base == 0U ||
        g_v3d_caps.hub_base == 0U)
        return V3D_STATUS_NOT_READY;

    simd_zero(&g_reset_debug, sizeof(g_reset_debug));
    g_reset_debug.attempts = 1;
    g_reset_debug.err_before = mmio_read(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF);
    g_reset_debug.core_int_before = mmio_read(g_v3d_caps.reg_base + V3D_CORE_INT_STS_OFF);
    g_reset_debug.hub_int_before = mmio_read(g_v3d_caps.hub_base + V3D_HUB_INT_STS_OFF);
    if (g_v3d_caps.sms_base != 0U)
        g_reset_debug.sms_before = mmio_read(g_v3d_caps.sms_base + V3D_SMS_REE_CS_OFF);
    g_reset_debug.mmu_ctl_before = mmio_read(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF);
    g_reset_debug.mmuc_before = mmio_read(g_v3d_caps.hub_base + V3D_MMUC_CONTROL_OFF);

    for (u32 i = 0; i < 25000U; i++) {
        if ((mmio_read(g_v3d_caps.reg_base + V3D_CSD_STATUS_OLD_OFF) &
             V3D_CSD_STATUS_BUSY_MASK) == 0U)
            break;
        if (i + 1U == 25000U) {
            g_reset_debug.status = V3D_STATUS_TIMEOUT;
            return V3D_STATUS_TIMEOUT;
        }

        timer_delay_us(1);
    }

    mmio_write(g_v3d_caps.reg_base + V3D_CORE_INT_MSK_SET_OFF, 0xFFFFFFFFU);
    mmio_write(g_v3d_caps.hub_base + V3D_HUB_INT_MSK_SET_OFF, 0xFFFFFFFFU);
    mmio_write(g_v3d_caps.reg_base + V3D_CORE_INT_CLR_OFF, v3d_core_irq_mask());
    mmio_write(g_v3d_caps.hub_base + V3D_HUB_INT_CLR_OFF, v3d_hub_irq_mask());
    mmio_write(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF, 0xFFFFFFFFU);
    dsb();
    g_reset_debug.err_after_clear = mmio_read(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF);

    v3d_sms_enter_ree();
    (void)v3d_gmp_allow_all();
    v3d_init_core_cache_state();
    if (v3d_native_mmu_setup() != V3D_STATUS_OK) {
        g_reset_debug.status = V3D_STATUS_FAILED;
        return V3D_STATUS_FAILED;
    }
    (void)v3d_mmu_flush_all();
    (void)v3d_clean_caches();
    (void)v3d_invalidate_caches();

    mmio_write(g_v3d_caps.reg_base + V3D_CORE_INT_CLR_OFF, v3d_core_irq_mask());
    mmio_write(g_v3d_caps.hub_base + V3D_HUB_INT_CLR_OFF, v3d_hub_irq_mask());
    mmio_write(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF, 0xFFFFFFFFU);
    dsb();

    g_reset_debug.err_after_reset = mmio_read(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF);
    g_reset_debug.core_int_after = mmio_read(g_v3d_caps.reg_base + V3D_CORE_INT_STS_OFF);
    g_reset_debug.hub_int_after = mmio_read(g_v3d_caps.hub_base + V3D_HUB_INT_STS_OFF);
    if (g_v3d_caps.sms_base != 0U)
        g_reset_debug.sms_after = mmio_read(g_v3d_caps.sms_base + V3D_SMS_REE_CS_OFF);
    g_reset_debug.mmu_ctl_after = mmio_read(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF);
    g_reset_debug.mmuc_after = mmio_read(g_v3d_caps.hub_base + V3D_MMUC_CONTROL_OFF);

    g_v3d_caps.native_selftest_status = v3d_native_selftest();
    g_v3d_caps.dispatch_supported = g_v3d_caps.mailbox_qpu ||
                                    (g_v3d_caps.mmio_csd &&
                                     g_v3d_caps.native_mmu_ready);

    g_reset_debug.status = (g_reset_debug.err_after_reset == 0U) ?
                           V3D_STATUS_OK : V3D_STATUS_FAILED;
    return g_reset_debug.status;
#endif
}

v3d_status_t v3d_pm_reset(void)
{
#if !PIOS_ENABLE_NATIVE_V3D_COMPUTE
    return V3D_STATUS_UNSUPPORTED;
#else
    if (!g_v3d_caps.native_compute_enabled || !g_v3d_caps.mmio_probe_ok ||
        g_v3d_caps.reg_base == 0U || g_v3d_caps.hub_base == 0U ||
        g_v3d_caps.tech_version < 71U)
        return V3D_STATUS_NOT_READY;

    (void)v3d_firmware_clock_enable();

    simd_zero(&g_reset_debug, sizeof(g_reset_debug));
    g_reset_debug.attempts = 1;
    g_reset_debug.err_before = mmio_read(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF);
    g_reset_debug.core_int_before = mmio_read(g_v3d_caps.reg_base + V3D_CORE_INT_STS_OFF);
    g_reset_debug.hub_int_before = mmio_read(g_v3d_caps.hub_base + V3D_HUB_INT_STS_OFF);
    if (g_v3d_caps.sms_base != 0U)
        g_reset_debug.sms_before = mmio_read(g_v3d_caps.sms_base + V3D_SMS_REE_CS_OFF);
    g_reset_debug.mmu_ctl_before = mmio_read(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF);
    g_reset_debug.mmuc_before = mmio_read(g_v3d_caps.hub_base + V3D_MMUC_CONTROL_OFF);

    for (u32 i = 0; i < 25000U; i++) {
        if ((mmio_read(g_v3d_caps.reg_base + V3D_CSD_STATUS_OLD_OFF) &
             V3D_CSD_STATUS_BUSY_MASK) == 0U)
            break;
        if (i + 1U == 25000U) {
            g_reset_debug.status = V3D_STATUS_TIMEOUT;
            return V3D_STATUS_TIMEOUT;
        }
        timer_delay_us(1);
    }

    mmio_write(g_v3d_caps.reg_base + V3D_CORE_INT_MSK_SET_OFF, 0xFFFFFFFFU);
    mmio_write(g_v3d_caps.hub_base + V3D_HUB_INT_MSK_SET_OFF, 0xFFFFFFFFU);
    mmio_write(g_v3d_caps.reg_base + V3D_CORE_INT_CLR_OFF, v3d_core_irq_mask());
    mmio_write(g_v3d_caps.hub_base + V3D_HUB_INT_CLR_OFF, v3d_hub_irq_mask());
    mmio_write(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF, 0xFFFFFFFFU);
    dsb();
    g_reset_debug.err_after_clear = mmio_read(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF);

    u64 pm_reg = V3D_PM_BASE + V3D_PM_GRAFX_2712_OFF;
    u32 pm_before = mmio_read(pm_reg);
    g_reset_debug.pm_before = pm_before;
    mmio_write(pm_reg, V3D_PM_PASSWORD | (pm_before & ~V3D_PM_V3DRSTN));
    dsb();
    g_reset_debug.pm_asserted = mmio_read(pm_reg);
    (void)v3d_firmware_clock_enable();
    timer_delay_ms(10);
    mmio_write(pm_reg, V3D_PM_PASSWORD | (mmio_read(pm_reg) | V3D_PM_V3DRSTN));
    dsb();
    (void)v3d_firmware_clock_enable();
    timer_delay_ms(10);
    g_reset_debug.pm_after = mmio_read(pm_reg);

    g_v3d_caps.ident0 = mmio_read(g_v3d_caps.reg_base + V3D_IDENT0_OFF);
    g_v3d_caps.ident1 = mmio_read(g_v3d_caps.reg_base + V3D_IDENT1_OFF);
    g_v3d_caps.ident2 = mmio_read(g_v3d_caps.reg_base + V3D_IDENT2_OFF);
    if (!v3d_ident_plausible(g_v3d_caps.ident0)) {
        g_reset_debug.status = V3D_STATUS_FAILED;
        return V3D_STATUS_FAILED;
    }

    v3d_sms_enter_ree();
    v3d_init_core_cache_state();
    g_v3d_caps.native_mmu_ready = false;
    g_v3d_caps.mmio_csd = false;
    if (v3d_native_mmu_setup() != V3D_STATUS_OK) {
        g_reset_debug.status = V3D_STATUS_FAILED;
        return V3D_STATUS_FAILED;
    }
    (void)v3d_gmp_allow_all();
    (void)v3d_mmu_flush_all();
    (void)v3d_clean_caches();
    (void)v3d_invalidate_caches();

    mmio_write(g_v3d_caps.reg_base + V3D_CORE_INT_CLR_OFF, v3d_core_irq_mask());
    mmio_write(g_v3d_caps.hub_base + V3D_HUB_INT_CLR_OFF, v3d_hub_irq_mask());
    mmio_write(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF, 0xFFFFFFFFU);
    dsb();

    g_reset_debug.err_after_reset = mmio_read(g_v3d_caps.reg_base + V3D_ERR_STAT_OFF);
    g_reset_debug.core_int_after = mmio_read(g_v3d_caps.reg_base + V3D_CORE_INT_STS_OFF);
    g_reset_debug.hub_int_after = mmio_read(g_v3d_caps.hub_base + V3D_HUB_INT_STS_OFF);
    if (g_v3d_caps.sms_base != 0U)
        g_reset_debug.sms_after = mmio_read(g_v3d_caps.sms_base + V3D_SMS_REE_CS_OFF);
    g_reset_debug.mmu_ctl_after = mmio_read(g_v3d_caps.hub_base + V3D_MMU_CTL_OFF);
    g_reset_debug.mmuc_after = mmio_read(g_v3d_caps.hub_base + V3D_MMUC_CONTROL_OFF);

    g_v3d_caps.native_selftest_status = v3d_native_selftest();
    g_v3d_caps.dispatch_supported = g_v3d_caps.mailbox_qpu ||
                                    (g_v3d_caps.mmio_csd &&
                                     g_v3d_caps.native_mmu_ready);

    g_reset_debug.status = (g_reset_debug.err_after_reset == 0U &&
                            g_v3d_caps.native_selftest_status == V3D_STATUS_OK) ?
                           V3D_STATUS_OK : V3D_STATUS_FAILED;
    return g_reset_debug.status;
#endif
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
    if (k->qpu_count == 0 || (!k->native_csd && k->control_list_bus == 0) || !k->ready)
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
    if ((uniform_bus & 0x3FFFFFFFU) + uniform_alloc > V3D_MMU_IDENTITY_BYTES) {
        gpu_mem_unlock(uniform_handle);
        gpu_mem_free(uniform_handle);
        return V3D_STATUS_NOT_READY;
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
    if ((shader_bus & 0x3FFFFFFFU) + shader_alloc > V3D_MMU_IDENTITY_BYTES) {
        gpu_mem_unlock(shader_handle);
        gpu_mem_free(shader_handle);
        gpu_mem_unlock(uniform_handle);
        gpu_mem_free(uniform_handle);
        return V3D_STATUS_NOT_READY;
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
