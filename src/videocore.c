#include "videocore.h"

#include "mmio.h"
#include "uart.h"

/*
 * Native Pi5 VideoCore trial driver, phase 0.
 *
 * This deliberately does read-only probing only. It does not stop the firmware
 * display driver, touch HVS/HDMI control registers, enable IRQs, or submit V3D
 * jobs. The stable mailbox framebuffer remains the display owner.
 */

#define VC6_HVS_BASE             0x107C580000UL
#define VC6_HVS_VERSION_OFF      0x00000000U
#define VC6_HVS_ID_OFF           0x000000FCU
#define VC6_HVS_VERSION_C0       0x53U
#define VC6_HVS_VERSION_D0       0x54U

/* From bcm2712-ds.dtsi: brcm,2712-v3d reg-names "hub", "core0", "sms". */
#define VC7_V3D_HUB_BASE         0x1002000000UL
#define VC7_V3D_CORE0_BASE       0x1002008000UL
#define VC7_V3D_SMS_BASE         0x1002030800UL

#define V3D_HUB_IDENT1_OFF       0x0000000CU
#define V3D_HUB_IDENT2_OFF       0x00000010U
#define V3D_HUB_IDENT3_OFF       0x00000014U
#define V3D_CTL_IDENT0_OFF       0x00000000U
#define V3D_CTL_IDENT1_OFF       0x00000004U
#define V3D_CTL_IDENT2_OFF       0x00000008U

#define V3D_HUB_IDENT1_TVER_MASK 0x0000000FU
#define V3D_HUB_IDENT1_REV_MASK  0x000000F0U
#define V3D_HUB_IDENT1_REV_SHIFT 4U
#define V3D_HUB_IDENT1_NCORES_MASK 0x00000F00U
#define V3D_HUB_IDENT1_NCORES_SHIFT 8U
#define V3D_HUB_IDENT1_NHOSTS_MASK 0x0000F000U
#define V3D_HUB_IDENT1_NHOSTS_SHIFT 12U
#define V3D_HUB_IDENT1_WITH_L3C  0x00010000U
#define V3D_HUB_IDENT1_WITH_TFU  0x00020000U
#define V3D_HUB_IDENT1_WITH_TSY  0x00040000U
#define V3D_HUB_IDENT1_WITH_MSO  0x00080000U
#define V3D_HUB_IDENT2_L3C_NKB_MASK 0x000000FFU
#define V3D_HUB_IDENT2_WITH_MMU  0x00000100U
#define V3D_HUB_IDENT3_IPIDX_MASK 0x000000FFU
#define V3D_HUB_IDENT3_IPREV_MASK 0x0000FF00U
#define V3D_HUB_IDENT3_IPREV_SHIFT 8U

#define V3D_MMU_DEBUG_INFO_OFF   0x00001238U
#define V3D_MMU_VERSION_MASK     0x0000000FU
#define V3D_MMU_VA_WIDTH_MASK    0x000000F0U
#define V3D_MMU_VA_WIDTH_SHIFT   4U
#define V3D_MMU_PA_WIDTH_MASK    0x00000F00U
#define V3D_MMU_PA_WIDTH_SHIFT   8U

#define V3D_IDENT0_VER_MASK      0xFF000000U
#define V3D_IDENT0_VER_SHIFT     24U
#define V3D_IDENT1_REV_MASK      0x0000000FU
#define V3D_IDENT1_NSLC_MASK     0x000000F0U
#define V3D_IDENT1_NSLC_SHIFT    4U
#define V3D_IDENT1_QUPS_MASK     0x00000F00U
#define V3D_IDENT1_QUPS_SHIFT    8U
#define V3D_IDENT1_NTMU_MASK     0x0000F000U
#define V3D_IDENT1_NTMU_SHIFT    12U
#define V3D_IDENT1_NSEM_MASK     0x00FF0000U
#define V3D_IDENT1_NSEM_SHIFT    16U
#define V3D_IDENT1_VPM_SIZE_MASK 0xF0000000U
#define V3D_IDENT1_VPM_SIZE_SHIFT 28U

static struct videocore_probe g_videocore_probe;

#if PIOS_ENABLE_NATIVE_VIDEOCORE
static bool vc_reg_plausible(u32 v)
{
    return v != 0U && v != 0xFFFFFFFFU;
}

static void vc_log_hex(const char *name, u64 value)
{
    uart_puts(name);
    uart_hex(value);
}

static void vc_log_bool(const char *name, bool value)
{
    uart_puts(name);
    uart_puts(value ? "1" : "0");
}
#endif

const struct videocore_probe *videocore_probe_get(void)
{
    return &g_videocore_probe;
}

void videocore_dump(void)
{
#if !PIOS_ENABLE_NATIVE_VIDEOCORE
    uart_puts("[vc] native probe disabled\n");
#else
    const struct videocore_probe *p = &g_videocore_probe;

    uart_puts("[vc] hvs ");
    uart_puts(p->hvs_seen ? (p->hvs_is_d0 ? "D0 " : "C0 ") : "missing ");
    vc_log_hex("ver=", p->hvs_version);
    vc_log_hex(" id=", p->hvs_id);
    uart_puts("\n");

    uart_puts("[vc] v3d ");
    uart_puts(p->v3d_seen ? "seen " : "missing ");
    vc_log_hex("tech=", p->v3d_tech_version);
    vc_log_hex(" rev=", p->v3d_hub_revision);
    vc_log_hex(" iprev=", p->v3d_ip_revision);
    vc_log_hex(" ipidx=", p->v3d_ip_index);
    vc_log_hex(" cores=", p->v3d_core_count);
    vc_log_hex(" hosts=", p->v3d_host_count);
    uart_puts("\n");

    uart_puts("[vc] caps ");
    vc_log_bool("mmu=", p->v3d_has_mmu);
    vc_log_bool(" tfu=", p->v3d_has_tfu);
    vc_log_bool(" tsy=", p->v3d_has_tsy);
    vc_log_bool(" mso=", p->v3d_has_mso);
    vc_log_bool(" l3c=", p->v3d_has_l3c);
    vc_log_hex(" l3kb=", p->v3d_l3c_kb);
    uart_puts("\n");

    uart_puts("[vc] mmu ");
    vc_log_hex("dbg=", p->v3d_mmu_debug);
    vc_log_hex(" va=", p->v3d_mmu_va_bits);
    vc_log_hex(" pa=", p->v3d_mmu_pa_bits);
    vc_log_hex(" ver=", p->v3d_mmu_version);
    uart_puts("\n");

    uart_puts("[vc] core0 ");
    vc_log_hex("id0=", p->v3d_core_ident0);
    vc_log_hex(" id1=", p->v3d_core_ident1);
    vc_log_hex(" id2=", p->v3d_core_ident2);
    vc_log_hex(" qps=", p->v3d_qpus_per_slice);
    vc_log_hex(" slices=", p->v3d_slice_count);
    vc_log_hex(" tmu=", p->v3d_tmu_count);
    vc_log_hex(" sem=", p->v3d_sem_count);
    vc_log_hex(" vpmkb=", p->v3d_vpm_kb);
    uart_puts("\n");
#endif
}

void videocore_init(void)
{
    g_videocore_probe.enabled = PIOS_ENABLE_NATIVE_VIDEOCORE ? true : false;

#if !PIOS_ENABLE_NATIVE_VIDEOCORE
    uart_puts("[vc] native probe disabled\n");
#else
    g_videocore_probe.hvs_version = mmio_read(VC6_HVS_BASE + VC6_HVS_VERSION_OFF);
    g_videocore_probe.hvs_id = mmio_read(VC6_HVS_BASE + VC6_HVS_ID_OFF);
    g_videocore_probe.hvs_seen =
        ((g_videocore_probe.hvs_version & 0xFFU) == VC6_HVS_VERSION_C0) ||
        ((g_videocore_probe.hvs_version & 0xFFU) == VC6_HVS_VERSION_D0);
    g_videocore_probe.hvs_is_d0 =
        ((g_videocore_probe.hvs_version & 0xFFU) == VC6_HVS_VERSION_D0);

    g_videocore_probe.v3d_hub_ident1 = mmio_read(VC7_V3D_HUB_BASE + V3D_HUB_IDENT1_OFF);
    g_videocore_probe.v3d_hub_ident2 = mmio_read(VC7_V3D_HUB_BASE + V3D_HUB_IDENT2_OFF);
    g_videocore_probe.v3d_hub_ident3 = mmio_read(VC7_V3D_HUB_BASE + V3D_HUB_IDENT3_OFF);
    g_videocore_probe.v3d_core_ident0 = mmio_read(VC7_V3D_CORE0_BASE + V3D_CTL_IDENT0_OFF);
    g_videocore_probe.v3d_core_ident1 = mmio_read(VC7_V3D_CORE0_BASE + V3D_CTL_IDENT1_OFF);
    g_videocore_probe.v3d_core_ident2 = mmio_read(VC7_V3D_CORE0_BASE + V3D_CTL_IDENT2_OFF);
    g_videocore_probe.v3d_mmu_debug = mmio_read(VC7_V3D_HUB_BASE + V3D_MMU_DEBUG_INFO_OFF);

    {
        u32 ident1 = g_videocore_probe.v3d_hub_ident1;
        u32 tver = ident1 & V3D_HUB_IDENT1_TVER_MASK;
        u32 rev = (ident1 & V3D_HUB_IDENT1_REV_MASK) >> V3D_HUB_IDENT1_REV_SHIFT;
        g_videocore_probe.v3d_tech_version = tver * 10U + rev;
        g_videocore_probe.v3d_hub_revision = rev;
        g_videocore_probe.v3d_core_count =
            (ident1 & V3D_HUB_IDENT1_NCORES_MASK) >> V3D_HUB_IDENT1_NCORES_SHIFT;
        g_videocore_probe.v3d_host_count =
            (ident1 & V3D_HUB_IDENT1_NHOSTS_MASK) >> V3D_HUB_IDENT1_NHOSTS_SHIFT;
        g_videocore_probe.v3d_has_l3c = (ident1 & V3D_HUB_IDENT1_WITH_L3C) != 0U;
        g_videocore_probe.v3d_has_tfu = (ident1 & V3D_HUB_IDENT1_WITH_TFU) != 0U;
        g_videocore_probe.v3d_has_tsy = (ident1 & V3D_HUB_IDENT1_WITH_TSY) != 0U;
        g_videocore_probe.v3d_has_mso = (ident1 & V3D_HUB_IDENT1_WITH_MSO) != 0U;
    }

    g_videocore_probe.v3d_has_mmu =
        (g_videocore_probe.v3d_hub_ident2 & V3D_HUB_IDENT2_WITH_MMU) != 0U;
    g_videocore_probe.v3d_l3c_kb =
        g_videocore_probe.v3d_hub_ident2 & V3D_HUB_IDENT2_L3C_NKB_MASK;
    g_videocore_probe.v3d_ip_index =
        g_videocore_probe.v3d_hub_ident3 & V3D_HUB_IDENT3_IPIDX_MASK;
    g_videocore_probe.v3d_ip_revision =
        (g_videocore_probe.v3d_hub_ident3 & V3D_HUB_IDENT3_IPREV_MASK) >>
        V3D_HUB_IDENT3_IPREV_SHIFT;
    g_videocore_probe.v3d_mmu_version =
        g_videocore_probe.v3d_mmu_debug & V3D_MMU_VERSION_MASK;
    g_videocore_probe.v3d_mmu_va_bits =
        30U + ((g_videocore_probe.v3d_mmu_debug & V3D_MMU_VA_WIDTH_MASK) >>
               V3D_MMU_VA_WIDTH_SHIFT);
    g_videocore_probe.v3d_mmu_pa_bits =
        30U + ((g_videocore_probe.v3d_mmu_debug & V3D_MMU_PA_WIDTH_MASK) >>
               V3D_MMU_PA_WIDTH_SHIFT);
    g_videocore_probe.v3d_core_ident_version =
        (g_videocore_probe.v3d_core_ident0 & V3D_IDENT0_VER_MASK) >>
        V3D_IDENT0_VER_SHIFT;
    g_videocore_probe.v3d_core_revision =
        g_videocore_probe.v3d_core_ident1 & V3D_IDENT1_REV_MASK;
    g_videocore_probe.v3d_slice_count =
        (g_videocore_probe.v3d_core_ident1 & V3D_IDENT1_NSLC_MASK) >>
        V3D_IDENT1_NSLC_SHIFT;
    g_videocore_probe.v3d_qpus_per_slice =
        (g_videocore_probe.v3d_core_ident1 & V3D_IDENT1_QUPS_MASK) >>
        V3D_IDENT1_QUPS_SHIFT;
    g_videocore_probe.v3d_tmu_count =
        (g_videocore_probe.v3d_core_ident1 & V3D_IDENT1_NTMU_MASK) >>
        V3D_IDENT1_NTMU_SHIFT;
    g_videocore_probe.v3d_sem_count =
        (g_videocore_probe.v3d_core_ident1 & V3D_IDENT1_NSEM_MASK) >>
        V3D_IDENT1_NSEM_SHIFT;
    g_videocore_probe.v3d_vpm_kb =
        (g_videocore_probe.v3d_core_ident1 & V3D_IDENT1_VPM_SIZE_MASK) >>
        V3D_IDENT1_VPM_SIZE_SHIFT;

    g_videocore_probe.v3d_seen =
        vc_reg_plausible(g_videocore_probe.v3d_hub_ident1) &&
        vc_reg_plausible(g_videocore_probe.v3d_core_ident0);
    g_videocore_probe.v3d_is_71 = (g_videocore_probe.v3d_tech_version == 71U);

    videocore_dump();

    (void)VC7_V3D_SMS_BASE;
#endif
}
