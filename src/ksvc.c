#include "ksvc.h"
#include "core.h"
#include "simd.h"
#include "timer.h"

struct ksvc_entry {
    bool used;
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
    volatile u32 mb_head;
    volatile u32 mb_tail;
    u64 messages_sent;
    u64 messages_recv;
    u64 mailbox_drops;
    struct ksvc_msg mailbox[KSVC_MBOX_DEPTH];
    char name[32];
};

static struct ksvc_entry services[NUM_CORES][KSVC_MAX_SERVICES];

static inline u64 ksvc_counter_ticks(void)
{
    u64 cnt;
    __asm__ volatile("mrs %0, cntvct_el0" : "=r"(cnt));
    return cnt;
}

static void copy_name(char *dst, u32 cap, const char *src)
{
    u32 i = 0;
    if (!dst || cap == 0) return;
    if (src) {
        while (i + 1U < cap && src[i]) {
            dst[i] = src[i];
            i++;
        }
    }
    dst[i] = 0;
}

static struct ksvc_entry *ksvc_get(i32 id)
{
    if (id < 0) return NULL;
    u32 core = ((u32)id >> 8) & 0xFFU;
    u32 slot = (u32)id & 0xFFU;
    if (core >= NUM_CORES || slot >= KSVC_MAX_SERVICES) return NULL;
    struct ksvc_entry *e = &services[core][slot];
    return e->used ? e : NULL;
}

void ksvc_init_core(void)
{
    u32 c = core_id() & 3U;
    simd_zero(services[c], sizeof(services[c]));
}

i32 ksvc_register(const char *name, u32 kind, u32 owner_core, u32 priority)
{
    if (!name || owner_core >= NUM_CORES) return -1;
    struct ksvc_entry *tab = services[owner_core];
    for (u32 i = 0; i < KSVC_MAX_SERVICES; i++) {
        if (!tab[i].used) {
            tab[i].used = true;
            tab[i].id = ((owner_core & 0xFFU) << 8) | (i & 0xFFU);
            tab[i].owner_core = owner_core;
            tab[i].kind = kind;
            tab[i].state = KSVC_STATE_REGISTERED;
            tab[i].priority = priority;
            copy_name(tab[i].name, sizeof(tab[i].name), name);
            return (i32)tab[i].id;
        }
    }
    return -1;
}

u64 ksvc_begin(i32 id)
{
    struct ksvc_entry *e = ksvc_get(id);
    if (e && e->state != KSVC_STATE_PAUSED && e->state != KSVC_STATE_FAULTED)
        e->state = KSVC_STATE_RUNNING;
    return ksvc_counter_ticks();
}

void ksvc_end(i32 id, u64 start_ticks, bool error)
{
    u64 now = ksvc_counter_ticks();
    struct ksvc_entry *e = ksvc_get(id);
    if (!e) return;
    u64 dt = now >= start_ticks ? (now - start_ticks) : 0;
    e->calls++;
    e->total_ticks += dt;
    e->last_duration_ticks = dt;
    if (dt > e->max_duration_ticks) e->max_duration_ticks = dt;
    e->last_run_ms = timer_monotonic_ms();
    if (error) {
        e->errors++;
        e->state = KSVC_STATE_FAULTED;
    } else if (e->state == KSVC_STATE_RUNNING) {
        e->state = KSVC_STATE_REGISTERED;
    }
}

void ksvc_mark_error(i32 id)
{
    struct ksvc_entry *e = ksvc_get(id);
    if (!e) return;
    e->errors++;
    e->state = KSVC_STATE_FAULTED;
}

static u32 ksvc_mailbox_count(const struct ksvc_entry *e)
{
    return (e->mb_head - e->mb_tail) & (KSVC_MBOX_DEPTH - 1U);
}

bool ksvc_send(i32 dst_id, const struct ksvc_msg *msg)
{
    struct ksvc_entry *e = ksvc_get(dst_id);
    if (!e || !msg) return false;
    if (e->owner_core != (core_id() & 3U)) return false;
    u32 head = e->mb_head;
    u32 next = (head + 1U) & (KSVC_MBOX_DEPTH - 1U);
    if (next == e->mb_tail) {
        e->mailbox_drops++;
        return false;
    }
    struct ksvc_msg m = *msg;
    m.dst = e->id;
    if (m.len > KSVC_MSG_DATA_MAX) m.len = KSVC_MSG_DATA_MAX;
    e->mailbox[head] = m;
    dmb();
    e->mb_head = next;
    e->messages_sent++;
    sev();
    return true;
}

bool ksvc_recv(i32 service_id, struct ksvc_msg *out)
{
    struct ksvc_entry *e = ksvc_get(service_id);
    if (!e || !out) return false;
    if (e->owner_core != (core_id() & 3U)) return false;
    u32 tail = e->mb_tail;
    if (tail == e->mb_head) return false;
    *out = e->mailbox[tail];
    e->mb_tail = (tail + 1U) & (KSVC_MBOX_DEPTH - 1U);
    e->messages_recv++;
    dmb();
    return true;
}

u32 ksvc_mailbox_pending(i32 service_id)
{
    struct ksvc_entry *e = ksvc_get(service_id);
    if (!e) return 0;
    return ksvc_mailbox_count(e);
}

bool ksvc_mailbox_selftest(i32 service_id)
{
    struct ksvc_msg msg;
    struct ksvc_msg got;
    simd_zero(&msg, sizeof(msg));
    simd_zero(&got, sizeof(got));
    msg.type = 0x4B535643U; /* KSVC */
    msg.src = (u32)service_id;
    msg.param = 0x12345678U;
    msg.tag = 0x1122334455667788ULL;
    msg.len = 4;
    msg.data[0] = 'P';
    msg.data[1] = 'I';
    msg.data[2] = 'O';
    msg.data[3] = 'S';
    if (!ksvc_send(service_id, &msg)) return false;
    if (!ksvc_recv(service_id, &got)) return false;
    return got.type == msg.type && got.src == msg.src && got.dst == (u32)service_id &&
           got.param == msg.param && got.tag == msg.tag && got.len == msg.len &&
           got.data[0] == 'P' && got.data[1] == 'I' &&
           got.data[2] == 'O' && got.data[3] == 'S';
}

u32 ksvc_snapshot(struct ksvc_snapshot_entry *out, u32 max_entries)
{
    if (!out || max_entries == 0) return 0;
    u32 n = 0;
    for (u32 c = 0; c < NUM_CORES && n < max_entries; c++) {
        for (u32 i = 0; i < KSVC_MAX_SERVICES && n < max_entries; i++) {
            struct ksvc_entry *e = &services[c][i];
            if (!e->used) continue;
            out[n].id = e->id;
            out[n].owner_core = e->owner_core;
            out[n].kind = e->kind;
            out[n].state = e->state;
            out[n].priority = e->priority;
            out[n].calls = e->calls;
            out[n].errors = e->errors;
            out[n].total_ticks = e->total_ticks;
            out[n].last_run_ms = e->last_run_ms;
            out[n].last_duration_ticks = e->last_duration_ticks;
            out[n].max_duration_ticks = e->max_duration_ticks;
            out[n].messages_sent = e->messages_sent;
            out[n].messages_recv = e->messages_recv;
            out[n].mailbox_drops = e->mailbox_drops;
            out[n].mailbox_pending = ksvc_mailbox_count(e);
            copy_name(out[n].name, sizeof(out[n].name), e->name);
            n++;
        }
    }
    return n;
}

const char *ksvc_state_name(u32 state)
{
    if (state == KSVC_STATE_EMPTY) return "empty";
    if (state == KSVC_STATE_REGISTERED) return "registered";
    if (state == KSVC_STATE_RUNNING) return "running";
    if (state == KSVC_STATE_PAUSED) return "paused";
    if (state == KSVC_STATE_FAULTED) return "faulted";
    return "unknown";
}
