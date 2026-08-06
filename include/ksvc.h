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

typedef bool (*ksvc_poll_fn_t)(void *ctx);

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
    u64 restarts;
    u32 last_error;
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
i32  ksvc_register_poll(const char *name, u32 kind, u32 owner_core, u32 priority,
                        ksvc_poll_fn_t poll_fn, void *ctx);
bool ksvc_run(i32 id);
u64  ksvc_begin(i32 id);
void ksvc_end(i32 id, u64 start_ticks, bool error);

/*
 * Hot-path variants for callers inside the scheduler loop (ADR-017/Q7).
 *
 * ksvc_begin/ksvc_end each read the cycle counter, and ksvc_end additionally
 * calls timer_monotonic_ms() -- a 64-bit divide. Two nested begin/end pairs per
 * scheduler iteration therefore cost 4 counter reads and 2 divides, paid at full
 * spin rate on an idle core. That perturbs the very thing it measures, which the
 * "diagnostics must not perturb scheduling" invariant forbids.
 *
 * ksvc_now_ticks() lets a caller read the counter ONCE and hand the same value
 * to several ksvc_end_at() calls. ksvc_end_at() skips both the redundant counter
 * read and the divide; last_run_ms is not updated, which is meaningless anyway
 * for a service that runs continuously.
 */
u64  ksvc_now_ticks(void);
void ksvc_begin_at(i32 id);
void ksvc_end_at(i32 id, u64 start_ticks, u64 now_ticks, bool error);
void ksvc_mark_error(i32 id);
bool ksvc_mark_error_code(i32 id, u32 error_code);
bool ksvc_pause(i32 id);
bool ksvc_resume(i32 id);
bool ksvc_restart(i32 id);
bool ksvc_send(i32 dst_id, const struct ksvc_msg *msg);
bool ksvc_recv(i32 service_id, struct ksvc_msg *out);
u32  ksvc_mailbox_pending(i32 service_id);
bool ksvc_mailbox_selftest(i32 service_id);
bool ksvc_fault_policy_selftest(i32 service_id);
u32  ksvc_snapshot(struct ksvc_snapshot_entry *out, u32 max_entries);
const char *ksvc_state_name(u32 state);
