#pragma once
#include "types.h"

#define KSVC_MAX_SERVICES 16

#define KSVC_KIND_POLL     0x00000001U
#define KSVC_KIND_NETWORK  0x00000002U
#define KSVC_KIND_STORAGE  0x00000004U
#define KSVC_KIND_TLS      0x00000008U
#define KSVC_KIND_CRYPTO   0x00000010U
#define KSVC_KIND_CONSOLE  0x00000020U
#define KSVC_KIND_UI       0x00000040U
#define KSVC_KIND_TIMER    0x00000080U

#define KSVC_STATE_EMPTY      0U
#define KSVC_STATE_REGISTERED 1U
#define KSVC_STATE_RUNNING    2U
#define KSVC_STATE_PAUSED     3U
#define KSVC_STATE_FAULTED    4U

struct ksvc_snapshot_entry {
    u32 id;
    u32 owner_core;
    u32 kind;
    u32 state;
    u32 priority;
    u64 calls;
    u64 errors;
    u64 total_ticks;
    u64 last_run_ms;
    u64 last_duration_ticks;
    u64 max_duration_ticks;
    char name[32];
} PACKED;

void ksvc_init_core(void);
i32  ksvc_register(const char *name, u32 kind, u32 owner_core, u32 priority);
u64  ksvc_begin(i32 id);
void ksvc_end(i32 id, u64 start_ticks, bool error);
void ksvc_mark_error(i32 id);
u32  ksvc_snapshot(struct ksvc_snapshot_entry *out, u32 max_entries);
const char *ksvc_state_name(u32 state);
