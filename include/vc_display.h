#pragma once

#include "types.h"

enum vc_display_backend {
    VC_DISPLAY_BACKEND_NONE = 0,
    VC_DISPLAY_BACKEND_BOOTINFO = 1,
    VC_DISPLAY_BACKEND_MAILBOX = 2,
    VC_DISPLAY_BACKEND_NATIVE = 3,
};

enum vc_display_takeover_status {
    VC_DISPLAY_TAKEOVER_IDLE = 0,
    VC_DISPLAY_TAKEOVER_OK = 1,
    VC_DISPLAY_TAKEOVER_DRY_RUN_OK = 2,
    VC_DISPLAY_TAKEOVER_NOT_READY = 3,
    VC_DISPLAY_TAKEOVER_BAD_MODE = 4,
    VC_DISPLAY_TAKEOVER_UNSUPPORTED = 5,
};

enum vc_display_dlist_status {
    VC_DISPLAY_DLIST_IDLE = 0,
    VC_DISPLAY_DLIST_OK = 1,
    VC_DISPLAY_DLIST_NOT_READY = 2,
    VC_DISPLAY_DLIST_BAD_MODE = 3,
    VC_DISPLAY_DLIST_UNSUPPORTED = 4,
    VC_DISPLAY_DLIST_STAGED = 5,
    VC_DISPLAY_DLIST_STAGE_CONFLICT = 6,
    VC_DISPLAY_DLIST_STAGE_VERIFY_FAILED = 7,
    VC_DISPLAY_DLIST_ARM_DRY_RUN_OK = 8,
    VC_DISPLAY_DLIST_ARMED = 9,
    VC_DISPLAY_DLIST_ARM_VERIFY_FAILED = 10,
};

enum vc_display_channel_status {
    VC_DISPLAY_CHANNEL_IDLE = 0,
    VC_DISPLAY_CHANNEL_REAPPLY_DRY_RUN_OK = 1,
    VC_DISPLAY_CHANNEL_REAPPLIED = 2,
    VC_DISPLAY_CHANNEL_NOT_READY = 3,
    VC_DISPLAY_CHANNEL_VERIFY_FAILED = 4,
};

enum vc_display_global_status {
    VC_DISPLAY_GLOBAL_IDLE = 0,
    VC_DISPLAY_GLOBAL_REAPPLY_DRY_RUN_OK = 1,
    VC_DISPLAY_GLOBAL_REAPPLIED = 2,
    VC_DISPLAY_GLOBAL_NOT_READY = 3,
    VC_DISPLAY_GLOBAL_VERIFY_FAILED = 4,
};

struct vc_display_status {
    bool enabled;
    bool ready;
    bool native_probe_ready;
    bool native_owner;
    bool hvs_seen;
    bool hvs_is_d0;
    bool v3d_seen;
    enum vc_display_backend backend;
    u64 scanout_base;
    u32 width;
    u32 height;
    u32 pitch;
    u32 size_bytes;
    u32 hvs_version;
    u32 hvs_id;
    u32 v3d_tech_version;
    u32 present_count;
    enum vc_display_backend previous_backend;
    u32 takeover_attempts;
    u32 fallback_count;
    u32 last_takeover_status;
    u32 snapshot_hvs_version;
    u32 snapshot_hvs_id;
    u32 snapshot_count;
    u32 hvs_control;
    u32 hvs_fetcher_status;
    u32 hvs_fetch_status;
    u32 hvs_handle_error;
    u32 hvs_dl_status;
    u32 hvs_disp_ctrl0[3];
    u32 hvs_disp_ctrl1[3];
    u32 hvs_disp_lptrs[3];
    u32 hvs_disp_cob[3];
    u32 hvs_disp_status[3];
    u32 hvs_disp_dl[3];
    u32 hvs_disp_run[3];
    u32 dlist_status;
    u32 dlist_count;
    u32 dlist_format;
    u32 dlist_order;
    u32 dlist_stage_index;
    u32 dlist_stage_count;
    u32 dlist_stage_readback_ok;
    u32 dlist_arm_channel;
    u32 dlist_arm_readback_ok;
    u32 dlist_arm_lptrs_before;
    u32 dlist_arm_lptrs_after;
    u32 dlist_restore_count;
    u32 dlist_restore_readback_ok;
    u32 dlist_restore_lptrs;
    u32 channel_reapply_status;
    u32 channel_reapply_count;
    u32 channel_reapply_channel;
    u32 channel_reapply_readback_ok;
    u32 channel_reapply_ctrl0;
    u32 channel_reapply_ctrl1;
    u32 channel_reapply_cob;
    u32 channel_reapply_rb_ctrl0;
    u32 channel_reapply_rb_ctrl1;
    u32 channel_reapply_rb_cob;
    u32 global_reapply_status;
    u32 global_reapply_count;
    u32 global_reapply_readback_ok;
    u32 global_reapply_control;
    u32 global_reapply_rb_control;
    u32 dlist_words[16];
    u32 dlist_readback[16];
};

void vc_display_init(void);
void vc_display_probe_native(void);
bool vc_display_snapshot(void);
bool vc_display_dlist_dryrun(void);
bool vc_display_dlist_stage(void);
bool vc_display_dlist_arm(bool dry_run);
bool vc_display_global_reapply(bool dry_run);
bool vc_display_channel_reapply(bool dry_run);
bool vc_display_takeover_current(bool dry_run);
bool vc_display_fallback(void);
void vc_display_note_present(void);
const struct vc_display_status *vc_display_status_get(void);
const char *vc_display_backend_name(enum vc_display_backend backend);
const char *vc_display_takeover_status_name(u32 status);
const char *vc_display_dlist_status_name(u32 status);
const char *vc_display_channel_status_name(u32 status);
const char *vc_display_global_status_name(u32 status);
