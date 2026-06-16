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
#define V3D_HUB_IDENT2_WITH_MMU  0x00000100U

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
#endif

const struct videocore_probe *videocore_probe_get(void)
{
    return &g_videocore_probe;
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

    {
        u32 ident1 = g_videocore_probe.v3d_hub_ident1;
        u32 tver = ident1 & V3D_HUB_IDENT1_TVER_MASK;
        u32 rev = (ident1 & V3D_HUB_IDENT1_REV_MASK) >> V3D_HUB_IDENT1_REV_SHIFT;
        g_videocore_probe.v3d_tech_version = tver * 10U + rev;
        g_videocore_probe.v3d_core_count =
            (ident1 & V3D_HUB_IDENT1_NCORES_MASK) >> V3D_HUB_IDENT1_NCORES_SHIFT;
    }

    g_videocore_probe.v3d_seen =
        vc_reg_plausible(g_videocore_probe.v3d_hub_ident1) &&
        vc_reg_plausible(g_videocore_probe.v3d_core_ident0);
    g_videocore_probe.v3d_is_71 = (g_videocore_probe.v3d_tech_version == 71U);

    uart_puts("[vc] native probe ");
    uart_puts((g_videocore_probe.hvs_seen || g_videocore_probe.v3d_seen) ? "ok " : "miss ");
    vc_log_hex("hvs_ver=", g_videocore_probe.hvs_version);
    vc_log_hex(" hvs_id=", g_videocore_probe.hvs_id);
    uart_puts(g_videocore_probe.hvs_is_d0 ? " hvs=D0" : " hvs=C/unknown");
    vc_log_hex(" hub1=", g_videocore_probe.v3d_hub_ident1);
    vc_log_hex(" hub2=", g_videocore_probe.v3d_hub_ident2);
    vc_log_hex(" core0=", g_videocore_probe.v3d_core_ident0);
    uart_puts(" v3d=");
    uart_hex(g_videocore_probe.v3d_tech_version);
    uart_puts(" cores=");
    uart_hex(g_videocore_probe.v3d_core_count);
    uart_puts((g_videocore_probe.v3d_hub_ident2 & V3D_HUB_IDENT2_WITH_MMU) ? " mmu=1\n" : " mmu=0\n");

    (void)VC7_V3D_SMS_BASE;
#endif
}
