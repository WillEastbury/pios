#pragma once
#include "types.h"

#define KSVC_MAX_SERVICES 16
#define KSVC_MBOX_DEPTH 16
#define KSVC_MSG_DATA_MAX 32

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
    u64 messages_sent;
    u64 messages_recv;
    u64 mailbox_drops;
    u32 mailbox_pending;
    char name[32];
} PACKED;

struct ksvc_msg {
    u32 type;
    u32 src;
    u32 dst;
    u32 param;
    u32 len;
    u64 tag;
    u8  data[KSVC_MSG_DATA_MAX];
} PACKED;

void ksvc_init_core(void);
i32  ksvc_register(const char *name, u32 kind, u32 owner_core, u32 priority);
u64  ksvc_begin(i32 id);
void ksvc_end(i32 id, u64 start_ticks, bool error);
void ksvc_mark_error(i32 id);
bool ksvc_send(i32 dst_id, const struct ksvc_msg *msg);
bool ksvc_recv(i32 service_id, struct ksvc_msg *out);
u32  ksvc_mailbox_pending(i32 service_id);
bool ksvc_mailbox_selftest(i32 service_id);
u32  ksvc_snapshot(struct ksvc_snapshot_entry *out, u32 max_entries);
const char *ksvc_state_name(u32 state);
