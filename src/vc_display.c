#include "vc_display.h"

#include "fb.h"
#include "platform.h"
#include "mmio.h"
#include "videocore.h"

#define VC6_HVS_BASE             0x107C580000UL
#define VC6_HVS_VERSION_OFF      0x00000000U
#define VC6_HVS_ID_OFF           0x000000FCU
#define VC6_HVS_VERSION_D0       0x54U
#define VC6D_HVS_CONTROL_OFF     0x00000020U
#define VC6D_HVS_FETCHER_STATUS_OFF 0x00000024U
#define VC6D_HVS_FETCH_STATUS_OFF 0x00000028U
#define VC6D_HVS_HANDLE_ERROR_OFF 0x0000002CU
#define VC6_HVS_DL_STATUS_OFF    0x00000094U
#define VC6D_HVS_DL_STATUS_OFF   0x00000034U
#define VC6_HVS_CONTROL_ENABLE   (1U << 31)
#define VC6_HVS_DISP0_CTRL0_OFF  0x00000030U
#define VC6_HVS_DISP_STRIDE      0x00000020U
#define VC6D_HVS_DISP0_CTRL0_OFF 0x00000100U
#define VC6D_HVS_DISP_STRIDE     0x00000040U

#define VC6D_DLIST_MAX_WORDS        16U
#define VC6D_HVS_DLIST_BASE_OFF     0x00004000U
#define VC6D_HVS_DLIST_BOOT_RESERVED_WORDS 32U
#define SCALER6_CTL0_END            (1U << 31)
#define SCALER6_CTL0_VALID          (1U << 30)
#define SCALER6_CTL0_NEXT_SHIFT     24U
#define SCALER6_CTL0_ALPHA_MASK_SHIFT 18U
#define SCALER6D_CTL0_ALPHA_MASK_FIXED 3U
#define SCALER6_CTL0_UNITY          (1U << 15)
#define SCALER6_CTL0_ORDER_SHIFT    13U
#define SCALER6_CTL0_PIXEL_FORMAT_RGBA8888 7U
#define HVS_PIXEL_ORDER_ARGB        2U
#define SCALER5_CTL2_ALPHA_SHIFT    4U
#define SCALER6_POS2_LINES_SHIFT    16U

static struct vc_display_status g_vc_display;

static void vc_display_attach(enum vc_display_backend backend,
                              u64 base, u32 width, u32 height,
                              u32 pitch, u32 size_bytes)
{
    g_vc_display.backend = backend;
    g_vc_display.scanout_base = base;
    g_vc_display.width = width;
    g_vc_display.height = height;
    g_vc_display.pitch = pitch;
    g_vc_display.size_bytes = size_bytes;
    g_vc_display.ready = (base != 0 && width != 0 && height != 0 && pitch != 0);
    g_vc_display.native_owner = backend == VC_DISPLAY_BACKEND_NATIVE;
}

void vc_display_init(void)
{
    g_vc_display.enabled = PIOS_ENABLE_NATIVE_VIDEOCORE ? true : false;
    g_vc_display.ready = false;
    g_vc_display.native_probe_ready = false;
    g_vc_display.native_owner = false;
    g_vc_display.hvs_seen = false;
    g_vc_display.hvs_is_d0 = false;
    g_vc_display.v3d_seen = false;
    g_vc_display.backend = VC_DISPLAY_BACKEND_NONE;
    g_vc_display.scanout_base = 0;
    g_vc_display.width = 0;
    g_vc_display.height = 0;
    g_vc_display.pitch = 0;
    g_vc_display.size_bytes = 0;
    g_vc_display.hvs_version = 0;
    g_vc_display.hvs_id = 0;
    g_vc_display.v3d_tech_version = 0;
    g_vc_display.present_count = 0;
    g_vc_display.previous_backend = VC_DISPLAY_BACKEND_NONE;
    g_vc_display.takeover_attempts = 0;
    g_vc_display.fallback_count = 0;
    g_vc_display.last_takeover_status = VC_DISPLAY_TAKEOVER_IDLE;
    g_vc_display.snapshot_hvs_version = 0;
    g_vc_display.snapshot_hvs_id = 0;
    g_vc_display.snapshot_count = 0;
    g_vc_display.hvs_control = 0;
    g_vc_display.hvs_fetcher_status = 0;
    g_vc_display.hvs_fetch_status = 0;
    g_vc_display.hvs_handle_error = 0;
    g_vc_display.hvs_dl_status = 0;
    for (u32 i = 0; i < 3U; i++) {
        g_vc_display.hvs_disp_ctrl0[i] = 0;
        g_vc_display.hvs_disp_ctrl1[i] = 0;
        g_vc_display.hvs_disp_lptrs[i] = 0;
        g_vc_display.hvs_disp_cob[i] = 0;
        g_vc_display.hvs_disp_status[i] = 0;
        g_vc_display.hvs_disp_dl[i] = 0;
        g_vc_display.hvs_disp_run[i] = 0;
    }
    g_vc_display.dlist_status = VC_DISPLAY_DLIST_IDLE;
    g_vc_display.dlist_count = 0;
    g_vc_display.dlist_format = 0;
    g_vc_display.dlist_order = 0;
    g_vc_display.dlist_stage_index = 0;
    g_vc_display.dlist_stage_count = 0;
    g_vc_display.dlist_stage_readback_ok = 0;
    g_vc_display.dlist_arm_channel = 0;
    g_vc_display.dlist_arm_readback_ok = 0;
    g_vc_display.dlist_arm_lptrs_before = 0;
    g_vc_display.dlist_arm_lptrs_after = 0;
    g_vc_display.dlist_restore_count = 0;
    g_vc_display.dlist_restore_readback_ok = 0;
    g_vc_display.dlist_restore_lptrs = 0;
    g_vc_display.channel_reapply_status = VC_DISPLAY_CHANNEL_IDLE;
    g_vc_display.channel_reapply_count = 0;
    g_vc_display.channel_reapply_channel = 0;
    g_vc_display.channel_reapply_readback_ok = 0;
    g_vc_display.channel_reapply_ctrl0 = 0;
    g_vc_display.channel_reapply_ctrl1 = 0;
    g_vc_display.channel_reapply_cob = 0;
    g_vc_display.channel_reapply_rb_ctrl0 = 0;
    g_vc_display.channel_reapply_rb_ctrl1 = 0;
    g_vc_display.channel_reapply_rb_cob = 0;
    g_vc_display.global_reapply_status = VC_DISPLAY_GLOBAL_IDLE;
    g_vc_display.global_reapply_count = 0;
    g_vc_display.global_reapply_readback_ok = 0;
    g_vc_display.global_reapply_control = 0;
    g_vc_display.global_reapply_rb_control = 0;
    for (u32 i = 0; i < VC6D_DLIST_MAX_WORDS; i++) {
        g_vc_display.dlist_words[i] = 0;
        g_vc_display.dlist_readback[i] = 0;
    }
}

void vc_display_probe_native(void)
{
    const struct videocore_probe *vc = videocore_probe_get();
    u64 fb_base = 0;
    u32 fb_width = 0, fb_height = 0, fb_pitch = 0, fb_size = 0;

    g_vc_display.enabled = PIOS_ENABLE_NATIVE_VIDEOCORE ? true : false;
    if (!vc)
        return;
    g_vc_display.hvs_seen = vc->hvs_seen;
    g_vc_display.hvs_is_d0 = (vc->hvs_version & 0xFFU) == VC6_HVS_VERSION_D0;
    g_vc_display.v3d_seen = vc->v3d_seen;
    g_vc_display.hvs_version = vc->hvs_version;
    g_vc_display.hvs_id = vc->hvs_id;
    g_vc_display.v3d_tech_version = vc->v3d_tech_version;
    g_vc_display.native_probe_ready = vc->enabled && vc->hvs_seen;

    fb_display_info(&fb_base, &fb_width, &fb_height, &fb_pitch, &fb_size);
    if (fb_base && fb_width && fb_height && fb_pitch) {
#if PIOS_HAS_BOOTINFO_FB && !PIOS_HAS_MAILBOX_FB
        vc_display_attach(VC_DISPLAY_BACKEND_BOOTINFO, fb_base, fb_width, fb_height, fb_pitch, fb_size);
#elif PIOS_HAS_MAILBOX_FB
        vc_display_attach(VC_DISPLAY_BACKEND_MAILBOX, fb_base, fb_width, fb_height, fb_pitch, fb_size);
#else
        vc_display_attach(VC_DISPLAY_BACKEND_NONE, fb_base, fb_width, fb_height, fb_pitch, fb_size);
#endif
    }
}

static bool vc_display_mode_valid(void)
{
    if (!g_vc_display.ready)
        return false;
    if (g_vc_display.width == 0 || g_vc_display.height == 0 || g_vc_display.pitch == 0)
        return false;
    if (g_vc_display.width > 4096U || g_vc_display.height > 2160U)
        return false;
    if (g_vc_display.pitch < g_vc_display.width * 4U || g_vc_display.pitch > 32768U)
        return false;
    if (g_vc_display.size_bytes &&
        (u64)g_vc_display.pitch * (u64)g_vc_display.height > g_vc_display.size_bytes)
        return false;
    return true;
}

static void vc_display_snapshot_hvs(void)
{
    if (!g_vc_display.hvs_seen)
        return;
    g_vc_display.snapshot_hvs_version = mmio_read(VC6_HVS_BASE + VC6_HVS_VERSION_OFF);
    g_vc_display.snapshot_hvs_id = mmio_read(VC6_HVS_BASE + VC6_HVS_ID_OFF);
    g_vc_display.hvs_is_d0 = (g_vc_display.snapshot_hvs_version & 0xFFU) == VC6_HVS_VERSION_D0;
}

static u32 vc_display_hvs_dl_status_off(void)
{
    return g_vc_display.hvs_is_d0 ? VC6D_HVS_DL_STATUS_OFF : VC6_HVS_DL_STATUS_OFF;
}

static u32 vc_display_hvs_disp_base_off(u32 ch)
{
    return g_vc_display.hvs_is_d0 ?
        (VC6D_HVS_DISP0_CTRL0_OFF + ch * VC6D_HVS_DISP_STRIDE) :
        (VC6_HVS_DISP0_CTRL0_OFF + ch * VC6_HVS_DISP_STRIDE);
}

static u32 vc_display_hvs_disp_ctrl0_off(u32 ch) { return vc_display_hvs_disp_base_off(ch); }
static u32 vc_display_hvs_disp_ctrl1_off(u32 ch) { return vc_display_hvs_disp_base_off(ch) + 0x04U; }
static u32 vc_display_hvs_disp_lptrs_off(u32 ch) { return vc_display_hvs_disp_base_off(ch) + (g_vc_display.hvs_is_d0 ? 0x10U : 0x0CU); }
static u32 vc_display_hvs_disp_cob_off(u32 ch) { return vc_display_hvs_disp_base_off(ch) + (g_vc_display.hvs_is_d0 ? 0x14U : 0x10U); }
static u32 vc_display_hvs_disp_status_off(u32 ch) { return vc_display_hvs_disp_base_off(ch) + (g_vc_display.hvs_is_d0 ? 0x18U : 0x14U); }
static u32 vc_display_hvs_disp_dl_off(u32 ch) { return vc_display_hvs_disp_base_off(ch) + (g_vc_display.hvs_is_d0 ? 0x1CU : 0x18U); }
static u32 vc_display_hvs_disp_run_off(u32 ch) { return vc_display_hvs_disp_base_off(ch) + (g_vc_display.hvs_is_d0 ? 0x20U : 0x1CU); }

bool vc_display_snapshot(void)
{
    if (!g_vc_display.hvs_seen)
        return false;

    vc_display_snapshot_hvs();
    g_vc_display.hvs_control = mmio_read(VC6_HVS_BASE + VC6D_HVS_CONTROL_OFF);
    g_vc_display.hvs_fetcher_status = mmio_read(VC6_HVS_BASE + VC6D_HVS_FETCHER_STATUS_OFF);
    g_vc_display.hvs_fetch_status = mmio_read(VC6_HVS_BASE + VC6D_HVS_FETCH_STATUS_OFF);
    g_vc_display.hvs_handle_error = mmio_read(VC6_HVS_BASE + VC6D_HVS_HANDLE_ERROR_OFF);
    g_vc_display.hvs_dl_status = mmio_read(VC6_HVS_BASE + vc_display_hvs_dl_status_off());
    for (u32 ch = 0; ch < 3U; ch++) {
        g_vc_display.hvs_disp_ctrl0[ch] = mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_ctrl0_off(ch));
        g_vc_display.hvs_disp_ctrl1[ch] = mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_ctrl1_off(ch));
        g_vc_display.hvs_disp_lptrs[ch] = mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_lptrs_off(ch));
        g_vc_display.hvs_disp_cob[ch] = mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_cob_off(ch));
        g_vc_display.hvs_disp_status[ch] = mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_status_off(ch));
        g_vc_display.hvs_disp_dl[ch] = mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_dl_off(ch));
        g_vc_display.hvs_disp_run[ch] = mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_run_off(ch));
    }
    g_vc_display.snapshot_count++;
    return true;
}

static bool vc_display_dlist_mode_valid(void)
{
    if (!vc_display_mode_valid())
        return false;
    if ((g_vc_display.scanout_base & 0x3ULL) != 0ULL)
        return false;
    if ((g_vc_display.pitch & 0x3U) != 0U)
        return false;
    if (g_vc_display.pitch > 0x1FFFFU)
        return false;
    if (g_vc_display.width == 0 || g_vc_display.width > 8192U)
        return false;
    if (g_vc_display.height == 0 || g_vc_display.height > 8192U)
        return false;
    return true;
}

bool vc_display_dlist_dryrun(void)
{
    u32 count = 0;
    u64 base = g_vc_display.scanout_base;

    g_vc_display.dlist_count = 0;
    for (u32 i = 0; i < VC6D_DLIST_MAX_WORDS; i++)
        g_vc_display.dlist_words[i] = 0;

    if (!g_vc_display.native_probe_ready || !g_vc_display.hvs_seen || !g_vc_display.ready) {
        g_vc_display.dlist_status = VC_DISPLAY_DLIST_NOT_READY;
        return false;
    }
    if (!vc_display_dlist_mode_valid()) {
        g_vc_display.dlist_status = VC_DISPLAY_DLIST_BAD_MODE;
        return false;
    }

    g_vc_display.dlist_format = SCALER6_CTL0_PIXEL_FORMAT_RGBA8888;
    g_vc_display.dlist_order = HVS_PIXEL_ORDER_ARGB;

    /* VC6D, linear, unscaled, opaque 32bpp primary plane over the full mode. */
    g_vc_display.dlist_words[count++] =
        SCALER6_CTL0_VALID |
        (SCALER6D_CTL0_ALPHA_MASK_FIXED << SCALER6_CTL0_ALPHA_MASK_SHIFT) |
        SCALER6_CTL0_UNITY |
        (HVS_PIXEL_ORDER_ARGB << SCALER6_CTL0_ORDER_SHIFT) |
        SCALER6_CTL0_PIXEL_FORMAT_RGBA8888;
    g_vc_display.dlist_words[count++] = 0U; /* POS0: x=0, y=0 */
    g_vc_display.dlist_words[count++] = 0xFFFU << SCALER5_CTL2_ALPHA_SHIFT;
    g_vc_display.dlist_words[count++] =
        ((g_vc_display.height - 1U) << SCALER6_POS2_LINES_SHIFT) |
        (g_vc_display.width - 1U);
    g_vc_display.dlist_words[count++] = 0xC0C0C0C0U; /* HVS-owned context */
    g_vc_display.dlist_words[count++] = (u32)((base >> 32) & 0xFFU); /* PTR0 upper */
    g_vc_display.dlist_words[count++] = (u32)base; /* PTR1 lower */
    g_vc_display.dlist_words[count++] = g_vc_display.pitch; /* PTR2 pitch */
    g_vc_display.dlist_words[count++] = SCALER6_CTL0_END;

    g_vc_display.dlist_words[0] |= count << SCALER6_CTL0_NEXT_SHIFT;
    g_vc_display.dlist_count = count;
    g_vc_display.dlist_status = VC_DISPLAY_DLIST_OK;
    return true;
}

static bool vc_display_dlist_stage_conflicts(u32 start, u32 count)
{
    u32 end = start + count;
    for (u32 ch = 0; ch < 3U; ch++) {
        u32 active = g_vc_display.hvs_disp_dl[ch] & 0xFFFU;
        u32 head = g_vc_display.hvs_disp_lptrs[ch] & 0xFFFU;
        if ((active >= start && active < end) || (head >= start && head < end))
            return true;
    }
    return false;
}

bool vc_display_dlist_stage(void)
{
    const u32 start = VC6D_HVS_DLIST_BOOT_RESERVED_WORDS;

    if (g_vc_display.dlist_status != VC_DISPLAY_DLIST_OK) {
        if (!vc_display_dlist_dryrun())
            return false;
    }
    if (g_vc_display.dlist_count == 0 || g_vc_display.dlist_count > VC6D_DLIST_MAX_WORDS) {
        g_vc_display.dlist_status = VC_DISPLAY_DLIST_BAD_MODE;
        return false;
    }
    if (!vc_display_snapshot()) {
        g_vc_display.dlist_status = VC_DISPLAY_DLIST_NOT_READY;
        return false;
    }
    if (vc_display_dlist_stage_conflicts(start, g_vc_display.dlist_count)) {
        g_vc_display.dlist_status = VC_DISPLAY_DLIST_STAGE_CONFLICT;
        return false;
    }

    for (u32 i = 0; i < g_vc_display.dlist_count; i++) {
        u64 off = VC6D_HVS_DLIST_BASE_OFF + ((u64)(start + i) * sizeof(u32));
        mmio_write(VC6_HVS_BASE + off, g_vc_display.dlist_words[i]);
    }
    dsb();

    g_vc_display.dlist_stage_readback_ok = 1U;
    for (u32 i = 0; i < g_vc_display.dlist_count; i++) {
        u64 off = VC6D_HVS_DLIST_BASE_OFF + ((u64)(start + i) * sizeof(u32));
        u32 got = mmio_read(VC6_HVS_BASE + off);
        g_vc_display.dlist_readback[i] = got;
        if (got != g_vc_display.dlist_words[i])
            g_vc_display.dlist_stage_readback_ok = 0U;
    }

    g_vc_display.dlist_stage_index = start;
    g_vc_display.dlist_stage_count = g_vc_display.dlist_count;
    g_vc_display.dlist_status = g_vc_display.dlist_stage_readback_ok ?
        VC_DISPLAY_DLIST_STAGED : VC_DISPLAY_DLIST_STAGE_VERIFY_FAILED;
    return g_vc_display.dlist_stage_readback_ok != 0U;
}

bool vc_display_dlist_arm(bool dry_run)
{
    const u32 ch = 0U;
    if (g_vc_display.dlist_status != VC_DISPLAY_DLIST_STAGED ||
        g_vc_display.dlist_stage_readback_ok == 0U ||
        g_vc_display.dlist_stage_count == 0U) {
        if (!vc_display_dlist_stage())
            return false;
    }
    if (g_vc_display.dlist_stage_index > 0xFFFU) {
        g_vc_display.dlist_status = VC_DISPLAY_DLIST_BAD_MODE;
        return false;
    }

    g_vc_display.dlist_arm_channel = ch;
    g_vc_display.dlist_arm_lptrs_before =
        mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_lptrs_off(ch));

    if (dry_run) {
        g_vc_display.dlist_status = VC_DISPLAY_DLIST_ARM_DRY_RUN_OK;
        return true;
    }

    mmio_write(VC6_HVS_BASE + vc_display_hvs_disp_lptrs_off(ch),
               g_vc_display.dlist_stage_index & 0xFFFU);
    dsb();
    g_vc_display.dlist_arm_lptrs_after =
        mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_lptrs_off(ch));
    g_vc_display.dlist_arm_readback_ok =
        ((g_vc_display.dlist_arm_lptrs_after & 0xFFFU) ==
         (g_vc_display.dlist_stage_index & 0xFFFU)) ? 1U : 0U;
    g_vc_display.dlist_status = g_vc_display.dlist_arm_readback_ok ?
        VC_DISPLAY_DLIST_ARMED : VC_DISPLAY_DLIST_ARM_VERIFY_FAILED;
    (void)vc_display_snapshot();
    return g_vc_display.dlist_arm_readback_ok != 0U;
}

bool vc_display_global_reapply(bool dry_run)
{
    if (!vc_display_snapshot()) {
        g_vc_display.global_reapply_status = VC_DISPLAY_GLOBAL_NOT_READY;
        return false;
    }

    g_vc_display.global_reapply_control = g_vc_display.hvs_control;
    if ((g_vc_display.global_reapply_control & VC6_HVS_CONTROL_ENABLE) == 0U) {
        g_vc_display.global_reapply_status = VC_DISPLAY_GLOBAL_NOT_READY;
        return false;
    }

    if (dry_run) {
        g_vc_display.global_reapply_status = VC_DISPLAY_GLOBAL_REAPPLY_DRY_RUN_OK;
        return true;
    }

    mmio_write(VC6_HVS_BASE + VC6D_HVS_CONTROL_OFF,
               g_vc_display.global_reapply_control);
    dsb();
    g_vc_display.global_reapply_rb_control =
        mmio_read(VC6_HVS_BASE + VC6D_HVS_CONTROL_OFF);
    g_vc_display.global_reapply_readback_ok =
        (g_vc_display.global_reapply_rb_control == g_vc_display.global_reapply_control) ? 1U : 0U;
    g_vc_display.global_reapply_count++;
    g_vc_display.global_reapply_status = g_vc_display.global_reapply_readback_ok ?
        VC_DISPLAY_GLOBAL_REAPPLIED : VC_DISPLAY_GLOBAL_VERIFY_FAILED;
    (void)vc_display_snapshot();
    return g_vc_display.global_reapply_readback_ok != 0U;
}

bool vc_display_channel_reapply(bool dry_run)
{
    const u32 ch = 0U;

    if (!vc_display_snapshot()) {
        g_vc_display.channel_reapply_status = VC_DISPLAY_CHANNEL_NOT_READY;
        return false;
    }

    g_vc_display.channel_reapply_channel = ch;
    g_vc_display.channel_reapply_ctrl0 = g_vc_display.hvs_disp_ctrl0[ch];
    g_vc_display.channel_reapply_ctrl1 = g_vc_display.hvs_disp_ctrl1[ch];
    g_vc_display.channel_reapply_cob = g_vc_display.hvs_disp_cob[ch];

    if (!g_vc_display.channel_reapply_ctrl0 || !g_vc_display.channel_reapply_cob) {
        g_vc_display.channel_reapply_status = VC_DISPLAY_CHANNEL_NOT_READY;
        return false;
    }

    if (dry_run) {
        g_vc_display.channel_reapply_status = VC_DISPLAY_CHANNEL_REAPPLY_DRY_RUN_OK;
        return true;
    }

    mmio_write(VC6_HVS_BASE + vc_display_hvs_disp_ctrl1_off(ch),
               g_vc_display.channel_reapply_ctrl1);
    mmio_write(VC6_HVS_BASE + vc_display_hvs_disp_cob_off(ch),
               g_vc_display.channel_reapply_cob);
    mmio_write(VC6_HVS_BASE + vc_display_hvs_disp_ctrl0_off(ch),
               g_vc_display.channel_reapply_ctrl0);
    dsb();

    g_vc_display.channel_reapply_rb_ctrl0 =
        mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_ctrl0_off(ch));
    g_vc_display.channel_reapply_rb_ctrl1 =
        mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_ctrl1_off(ch));
    g_vc_display.channel_reapply_rb_cob =
        mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_cob_off(ch));
    g_vc_display.channel_reapply_readback_ok =
        (g_vc_display.channel_reapply_rb_ctrl0 == g_vc_display.channel_reapply_ctrl0 &&
         g_vc_display.channel_reapply_rb_ctrl1 == g_vc_display.channel_reapply_ctrl1 &&
         g_vc_display.channel_reapply_rb_cob == g_vc_display.channel_reapply_cob) ? 1U : 0U;
    g_vc_display.channel_reapply_count++;
    g_vc_display.channel_reapply_status = g_vc_display.channel_reapply_readback_ok ?
        VC_DISPLAY_CHANNEL_REAPPLIED : VC_DISPLAY_CHANNEL_VERIFY_FAILED;
    (void)vc_display_snapshot();
    return g_vc_display.channel_reapply_readback_ok != 0U;
}

bool vc_display_takeover_current(bool dry_run)
{
    g_vc_display.takeover_attempts++;
    vc_display_snapshot_hvs();

    if (!g_vc_display.native_probe_ready || !g_vc_display.hvs_seen) {
        g_vc_display.last_takeover_status = VC_DISPLAY_TAKEOVER_NOT_READY;
        return false;
    }
    if (!vc_display_mode_valid()) {
        g_vc_display.last_takeover_status = VC_DISPLAY_TAKEOVER_BAD_MODE;
        return false;
    }

    if (dry_run) {
        g_vc_display.last_takeover_status = VC_DISPLAY_TAKEOVER_DRY_RUN_OK;
        return true;
    }

    g_vc_display.previous_backend = g_vc_display.backend;
    g_vc_display.backend = VC_DISPLAY_BACKEND_NATIVE;
    g_vc_display.native_owner = true;
    g_vc_display.last_takeover_status = VC_DISPLAY_TAKEOVER_OK;
    return true;
}

bool vc_display_fallback(void)
{
    enum vc_display_backend target = g_vc_display.previous_backend;
    bool restore_ok = true;

    if (g_vc_display.dlist_status == VC_DISPLAY_DLIST_ARMED &&
        g_vc_display.dlist_arm_readback_ok &&
        g_vc_display.dlist_arm_lptrs_before != g_vc_display.dlist_arm_lptrs_after) {
        u32 ch = g_vc_display.dlist_arm_channel;
        mmio_write(VC6_HVS_BASE + vc_display_hvs_disp_lptrs_off(ch),
                   g_vc_display.dlist_arm_lptrs_before);
        dsb();
        g_vc_display.dlist_restore_lptrs =
            mmio_read(VC6_HVS_BASE + vc_display_hvs_disp_lptrs_off(ch));
        g_vc_display.dlist_restore_readback_ok =
            ((g_vc_display.dlist_restore_lptrs & 0xFFFU) ==
             (g_vc_display.dlist_arm_lptrs_before & 0xFFFU)) ? 1U : 0U;
        g_vc_display.dlist_restore_count++;
        restore_ok = g_vc_display.dlist_restore_readback_ok != 0U;
        if (restore_ok) {
            g_vc_display.dlist_arm_lptrs_after = g_vc_display.dlist_restore_lptrs;
            g_vc_display.dlist_status = VC_DISPLAY_DLIST_STAGED;
            (void)vc_display_snapshot();
        }
    }

    if (target == VC_DISPLAY_BACKEND_NONE || target == VC_DISPLAY_BACKEND_NATIVE) {
#if PIOS_HAS_BOOTINFO_FB && !PIOS_HAS_MAILBOX_FB
        target = VC_DISPLAY_BACKEND_BOOTINFO;
#elif PIOS_HAS_MAILBOX_FB
        target = VC_DISPLAY_BACKEND_MAILBOX;
#else
        target = VC_DISPLAY_BACKEND_NONE;
#endif
    }
    g_vc_display.backend = target;
    g_vc_display.native_owner = false;
    g_vc_display.fallback_count++;
    g_vc_display.last_takeover_status = VC_DISPLAY_TAKEOVER_IDLE;
    return g_vc_display.ready && restore_ok;
}

void vc_display_note_present(void)
{
    g_vc_display.present_count++;
}

const struct vc_display_status *vc_display_status_get(void)
{
    return &g_vc_display;
}

const char *vc_display_backend_name(enum vc_display_backend backend)
{
    switch (backend) {
    case VC_DISPLAY_BACKEND_BOOTINFO: return "bootinfo";
    case VC_DISPLAY_BACKEND_MAILBOX: return "mailbox";
    case VC_DISPLAY_BACKEND_NATIVE: return "native";
    case VC_DISPLAY_BACKEND_NONE:
    default:
        return "none";
    }
}

const char *vc_display_takeover_status_name(u32 status)
{
    switch (status) {
    case VC_DISPLAY_TAKEOVER_OK: return "ok";
    case VC_DISPLAY_TAKEOVER_DRY_RUN_OK: return "dry-run-ok";
    case VC_DISPLAY_TAKEOVER_NOT_READY: return "not-ready";
    case VC_DISPLAY_TAKEOVER_BAD_MODE: return "bad-mode";
    case VC_DISPLAY_TAKEOVER_UNSUPPORTED: return "unsupported";
    case VC_DISPLAY_TAKEOVER_IDLE:
    default:
        return "idle";
    }
}

const char *vc_display_dlist_status_name(u32 status)
{
    switch (status) {
    case VC_DISPLAY_DLIST_OK: return "ok";
    case VC_DISPLAY_DLIST_STAGED: return "staged";
    case VC_DISPLAY_DLIST_ARM_DRY_RUN_OK: return "arm-dry-run-ok";
    case VC_DISPLAY_DLIST_ARMED: return "armed";
    case VC_DISPLAY_DLIST_ARM_VERIFY_FAILED: return "arm-verify-failed";
    case VC_DISPLAY_DLIST_STAGE_CONFLICT: return "stage-conflict";
    case VC_DISPLAY_DLIST_STAGE_VERIFY_FAILED: return "stage-verify-failed";
    case VC_DISPLAY_DLIST_NOT_READY: return "not-ready";
    case VC_DISPLAY_DLIST_BAD_MODE: return "bad-mode";
    case VC_DISPLAY_DLIST_UNSUPPORTED: return "unsupported";
    case VC_DISPLAY_DLIST_IDLE:
    default:
        return "idle";
    }
}

const char *vc_display_channel_status_name(u32 status)
{
    switch (status) {
    case VC_DISPLAY_CHANNEL_REAPPLY_DRY_RUN_OK: return "dry-run-ok";
    case VC_DISPLAY_CHANNEL_REAPPLIED: return "reapplied";
    case VC_DISPLAY_CHANNEL_NOT_READY: return "not-ready";
    case VC_DISPLAY_CHANNEL_VERIFY_FAILED: return "verify-failed";
    case VC_DISPLAY_CHANNEL_IDLE:
    default:
        return "idle";
    }
}

const char *vc_display_global_status_name(u32 status)
{
    switch (status) {
    case VC_DISPLAY_GLOBAL_REAPPLY_DRY_RUN_OK: return "dry-run-ok";
    case VC_DISPLAY_GLOBAL_REAPPLIED: return "reapplied";
    case VC_DISPLAY_GLOBAL_NOT_READY: return "not-ready";
    case VC_DISPLAY_GLOBAL_VERIFY_FAILED: return "verify-failed";
    case VC_DISPLAY_GLOBAL_IDLE:
    default:
        return "idle";
    }
}
