#include "ksvc.h"
#include "core.h"
#include "simd.h"
#include "timer.h"

struct ksvc_entry {
    bool used;
    u32 generation;
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
    u64 restarts;
    u32 last_error;
    ksvc_poll_fn_t poll_fn;
    void *poll_ctx;
    struct ksvc_msg mailbox[KSVC_MBOX_DEPTH];
    char name[32];
} ALIGNED(64);

static struct ksvc_entry services[NUM_CORES][KSVC_MAX_SERVICES];
_Static_assert((sizeof(struct ksvc_entry) & 63U) == 0U,
               "KSVC service descriptors must have cache-line stride");

#define KSVC_POISON_U32 0xDEAD5E12U

static u32 ksvc_bump_generation(u32 old)
{
    u32 g = old + 1U;
    return g ? g : 1U;
}

static void ksvc_poison(struct ksvc_entry *e)
{
    if (!e)
        return;
    u32 gen = ksvc_bump_generation(e->generation);
    simd_zero(e, sizeof(*e));
    e->used = false;
    e->generation = gen;
    e->id = KSVC_POISON_U32;
    e->owner_core = KSVC_POISON_U32;
    e->kind = KSVC_POISON_U32;
    e->state = KSVC_STATE_EMPTY;
    e->priority = KSVC_POISON_U32;
    e->last_error = KSVC_POISON_U32;
}

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
    for (u32 i = 0; i < KSVC_MAX_SERVICES; i++)
        ksvc_poison(&services[c][i]);
}

i32 ksvc_register(const char *name, u32 kind, u32 owner_core, u32 priority)
{
    if (!name || owner_core >= NUM_CORES) return -1;
    struct ksvc_entry *tab = services[owner_core];
    for (u32 i = 0; i < KSVC_MAX_SERVICES; i++) {
        if (!tab[i].used) {
            u32 gen = ksvc_bump_generation(tab[i].generation);
            simd_zero(&tab[i], sizeof(tab[i]));
            tab[i].used = true;
            tab[i].generation = gen;
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

i32 ksvc_register_poll(const char *name, u32 kind, u32 owner_core, u32 priority,
                       ksvc_poll_fn_t poll_fn, void *ctx)
{
    i32 id = ksvc_register(name, kind, owner_core, priority);
    struct ksvc_entry *e = ksvc_get(id);
    if (!e) return -1;
    e->poll_fn = poll_fn;
    e->poll_ctx = ctx;
    return id;
}

bool ksvc_run(i32 id)
{
    struct ksvc_entry *e = ksvc_get(id);
    if (!e || !e->poll_fn) return false;
    if (e->state == KSVC_STATE_PAUSED || e->state == KSVC_STATE_FAULTED)
        return false;
    u64 start = ksvc_begin(id);
    bool ok = e->poll_fn(e->poll_ctx);
    ksvc_end(id, start, !ok);
    return ok;
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

u64 ksvc_now_ticks(void)
{
    return ksvc_counter_ticks();
}

/* Mark RUNNING without reading the counter; caller already has `now`. */
void ksvc_begin_at(i32 id)
{
    struct ksvc_entry *e = ksvc_get(id);
    if (e && e->state != KSVC_STATE_PAUSED && e->state != KSVC_STATE_FAULTED)
        e->state = KSVC_STATE_RUNNING;
}

/* Close an interval using a counter value the caller already read. Deliberately
 * does NOT touch last_run_ms: that call is a 64-bit divide and is meaningless
 * for a continuously-running service. See ksvc.h. */
void ksvc_end_at(i32 id, u64 start_ticks, u64 now_ticks, bool error)
{
    struct ksvc_entry *e = ksvc_get(id);
    if (!e) return;
    u64 dt = now_ticks >= start_ticks ? (now_ticks - start_ticks) : 0;
    e->calls++;
    e->total_ticks += dt;
    e->last_duration_ticks = dt;
    if (dt > e->max_duration_ticks) e->max_duration_ticks = dt;
    if (error) {
        e->errors++;
        e->state = KSVC_STATE_FAULTED;
    } else if (e->state == KSVC_STATE_RUNNING) {
        e->state = KSVC_STATE_REGISTERED;
    }
}

void ksvc_mark_error(i32 id){
    ksvc_mark_error_code(id, 1U);
}

bool ksvc_mark_error_code(i32 id, u32 error_code)
{
    struct ksvc_entry *e = ksvc_get(id);
    if (!e) return false;
    e->errors++;
    e->last_error = error_code;
    e->state = KSVC_STATE_FAULTED;
    return true;
}

bool ksvc_pause(i32 id)
{
    struct ksvc_entry *e = ksvc_get(id);
    if (!e || e->state == KSVC_STATE_FAULTED) return false;
    e->state = KSVC_STATE_PAUSED;
    return true;
}

bool ksvc_resume(i32 id)
{
    struct ksvc_entry *e = ksvc_get(id);
    if (!e || e->state != KSVC_STATE_PAUSED) return false;
    e->state = KSVC_STATE_REGISTERED;
    return true;
}

bool ksvc_restart(i32 id)
{
    struct ksvc_entry *e = ksvc_get(id);
    if (!e) return false;
    e->mb_head = 0;
    e->mb_tail = 0;
    e->last_error = 0;
    e->restarts++;
    e->state = KSVC_STATE_REGISTERED;
    dmb();
    return true;
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

bool ksvc_fault_policy_selftest(i32 service_id)
{
    struct ksvc_entry *e = ksvc_get(service_id);
    if (!e) return false;
    u32 before_state = e->state;
    u64 before_errors = e->errors;
    u64 before_restarts = e->restarts;
    if (!ksvc_pause(service_id)) return false;
    if (e->state != KSVC_STATE_PAUSED) return false;
    if (!ksvc_resume(service_id)) return false;
    if (e->state != KSVC_STATE_REGISTERED) return false;
    ksvc_mark_error_code(service_id, 0x51564331U);
    if (e->state != KSVC_STATE_FAULTED || e->errors != before_errors + 1U ||
        e->last_error != 0x51564331U) return false;
    if (!ksvc_restart(service_id)) return false;
    if (e->state != KSVC_STATE_REGISTERED || e->restarts != before_restarts + 1U ||
        e->last_error != 0) return false;
    if (before_state == KSVC_STATE_PAUSED)
        (void)ksvc_pause(service_id);
    return true;
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
            out[n].restarts = e->restarts;
            out[n].last_error = e->last_error;
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
