#pragma once
#include "types.h"

/* Operator-only core-0 lifecycle. No queue, MSI, or BME activation. */
const char *b50_native_command(u32 operation);
void b50_native_service(void);
bool b50_native_blocks_legacy(void);
struct b50_native_diag {
    u32 attached;
    u32 ggtt_state;
    u32 ggtt_page;
    u32 ggtt_offset;
    u32 ggtt_detail;
    u32 ggtt_framework_reason;
    u32 prep_state;
    u32 prep_offset;
    u32 prep_detail;
    u32 prep_framework_reason;
    u32 boot_state;
    u32 boot_framework_reason;
    u64 ggtt_expected;
    u64 ggtt_actual;
};
void b50_native_diag(struct b50_native_diag *out);
struct b50_native_boot_diag {
    u32 state;
    u32 framework_reason;
    u32 failure;
    u32 ready;
    u32 last_status;
    u32 dma_ctrl;
    u32 wopcm_size_raw;
    u32 wopcm_size_expected;
    u32 wopcm_offset_raw;
    u32 wopcm_offset_expected;
    u32 reserved[6];
};
void b50_native_boot_diag(struct b50_native_boot_diag *out);
struct b50_native_ct_diag {
    u32 state;
    u32 index;
    u32 retries;
    u32 response;
    u32 failure;
    u32 h2g_head;
    u32 h2g_tail;
    u32 h2g_status;
    u32 g2h_head;
    u32 g2h_tail;
    u32 g2h_status;
    u32 hwconfig0;
    u32 context_state;
    u32 context_response;
    u32 context_failure;
    u32 context_ring_tail;
};
void b50_native_ct_diag(struct b50_native_ct_diag *out);
struct b50_native_submit_diag {
    u32 state;
    u32 response;
    u32 failure;
    u32 result;
    u32 completion;
    u32 g2h_head;
    u32 g2h_tail;
    u32 g2h_status;
    u32 engine_head;
    u32 engine_tail;
    u32 engine_start;
    u32 engine_control;
    u32 engine_hws_pga;
    u32 engine_context_control;
    u32 engine_acthd;
    u32 engine_ipehr;
};
void b50_native_submit_diag(struct b50_native_submit_diag *out);

#define B50_GGTT_DIAG_NONE         0U
#define B50_GGTT_DIAG_PRECONDITION 1U
#define B50_GGTT_DIAG_PLAN         2U
#define B50_GGTT_DIAG_READBACK     3U
#define B50_GGTT_DIAG_STEP_LATE    4U
#define B50_GGTT_DIAG_STATE        5U

#define B50_PREP_DIAG_NONE         0U
#define B50_PREP_DIAG_PRECONDITION 1U
#define B50_PREP_DIAG_STEP_LATE    2U
#define B50_PREP_DIAG_BUILD        3U
#define B50_PREP_DIAG_STATE        4U

#define B50_NATIVE_STATUS    0U
#define B50_NATIVE_ATTACH    1U
#define B50_NATIVE_PREFLIGHT 2U
#define B50_NATIVE_REVOKE    3U
#define B50_NATIVE_CANARY    4U
#define B50_NATIVE_FIRMWARE  5U
#define B50_NATIVE_STAGE     6U
#define B50_NATIVE_FORCEWAKE 7U
#define B50_NATIVE_GGTT      8U
#define B50_NATIVE_PREPARE   9U
#define B50_NATIVE_BOOT      10U
#define B50_NATIVE_VALIDATE  11U
#define B50_NATIVE_CT        12U
#define B50_NATIVE_CONTEXT   13U
#define B50_NATIVE_SUBMIT    14U
