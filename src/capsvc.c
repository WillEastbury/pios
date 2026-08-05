/*
 * capsvc.c - generic capsule-service dispatcher (core 0 side).
 *
 * Terminates TCP for every registered service (accept/read/write, same as
 * admin_http_service/uhttp_bridge do today) and forwards decoded request
 * bytes to that service's capsule through a shared Normal-NC arena slot,
 * then relays the capsule's reply back to the TCP connection. Zero
 * service-specific logic lives here: capsvc_register(port, target_core) is
 * the entire integration surface for a new service.
 *
 * See include/capsvc.h for the full design rationale (shared-arena
 * transport, why raw fifo.h/ipc_proc_fifo_* are not used, wire format).
 *
 * Kernel-only runtime state (g_rt[]) is parallel to, and indexed identically
 * to, the shared arena (capsvc_arena()->slots[]) -- one arena slot is
 * permanently paired with one connection-tracking slot, so "allocate a
 * connection" and "allocate an arena slot" are the same operation. The
 * capsule only ever sees the arena half (data + the opaque tag); tcp_conn_t
 * and timing state never cross into shared memory.
 */
#include "capsvc.h"
#include "core.h"
#include "tcp.h"
#include "timer.h"
#include "proc.h"
#include "simd.h"
#include "capsule_store.h"

enum {
    CAPSVC_RT_IDLE = 0,
    CAPSVC_RT_READING,
    CAPSVC_RT_WAIT_REPLY,
    CAPSVC_RT_SENDING,
    CAPSVC_RT_EXTERNAL_WAIT,
};

struct capsvc_registration {
    bool active;
    u16 port;
    u32 target_core;
    tcp_conn_t listen_conn;
    u32 capsule_pid;   /* 0 until capsvc_attach() is called */
    char host[CAPSVC_HOST_MAX];
};

struct capsvc_rt {
    bool inuse;
    u32 svc_idx;
    tcp_conn_t conn;
    u32 state;
    u32 generation;    /* mirrors the arena slot's generation for this request cycle */
    u32 resp_off;
    u64 phase_ms;
};

static struct capsvc_registration g_svc[CAPSVC_MAX_SERVICES];
static u32 g_svc_count;
static struct capsvc_rt g_rt[CAPSVC_SLOT_COUNT];
static u32 g_generation_ctr;
static u32 g_dbg_accept_calls;
static u32 g_dbg_accept_success;
static u32 g_dbg_slot_alloc_fail;
static u32 g_dbg_poll_calls;

i32 capsvc_register(u16 port, u32 target_core)
{
    if (g_svc_count >= CAPSVC_MAX_SERVICES)
        return -1;
    u32 idx = g_svc_count++;
    struct capsvc_registration *s = &g_svc[idx];
    s->active = true;
    s->port = port;
    s->target_core = target_core;
    s->listen_conn = -1;
    s->capsule_pid = 0;
    s->host[0] = 0;
    return (i32)idx;
}

bool capsvc_bind_host(u32 svc_idx, const char *host)
{
    if (svc_idx >= g_svc_count || !host || !*host)
        return false;
    u32 n = 0;
    while (host[n]) {
        char ch = host[n];
        bool alpha = (ch >= 'a' && ch <= 'z') ||
                     (ch >= 'A' && ch <= 'Z');
        bool digit = ch >= '0' && ch <= '9';
        if ((!alpha && !digit && ch != '-' && ch != '.') ||
            n + 1U >= CAPSVC_HOST_MAX)
            return false;
        g_svc[svc_idx].host[n] =
            ch >= 'A' && ch <= 'Z' ? (char)(ch + ('a' - 'A')) : ch;
        n++;
    }
    g_svc[svc_idx].host[n] = 0;
    return true;
}

i32 capsvc_find_host(const char *host)
{
    if (!host || !*host)
        return -1;
    for (u32 i = 0; i < g_svc_count; i++) {
        const char *a = host;
        const char *b = g_svc[i].host;
        bool equal = true;
        if (!*b)
            continue;
        while (*a && *b) {
            char ac = *a++;
            char bc = *b++;
            if (ac >= 'A' && ac <= 'Z') ac = (char)(ac + ('a' - 'A'));
            if (bc >= 'A' && bc <= 'Z') bc = (char)(bc + ('a' - 'A'));
            if (ac != bc) {
                equal = false;
                break;
            }
        }
        if (equal && *a == 0 && *b == 0)
            return (i32)i;
    }
    return -1;
}

void capsvc_attach(u32 svc_idx, u32 capsule_pid)
{
    if (svc_idx < g_svc_count)
        g_svc[svc_idx].capsule_pid = capsule_pid;
}

static bool capsvc_streq(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) { if (*a != *b) return false; a++; b++; }
    return *a == 0 && *b == 0;
}

/* Preload this service's PicoScript bytecode from a WALFS/picowal capsule
 * manifest (capsule_store.h), matched by the manifest's `io = tcp/<port>`
 * binding -- the exact same frozen manifest format and card-pairing
 * convention uhttp_bridge.c's uhttp_load_capsule_process() already uses for
 * the port-82/83 workers. Generic and service-agnostic: this function has no
 * idea what "admin" or any other service means, it only matches ports. Runs
 * once at capsvc_init(); len stays 0 (capsule falls back to its own
 * compiled-in default program) if no manifest/process/port match is found --
 * this is the expected, non-error state until an operator installs a card
 * via the WALFS terminal ops (tracked separately). */
static void capsvc_preload_program(struct capsvc_registration *s, struct capsvc_program *prog)
{
    prog->len = 0;
    struct capsule_manifest m;
    char err[64];
    if (!capsule_store_load_manifest(CAPSULE_PACK_MIN, &m, err, sizeof(err)))
        return;
    for (u32 i = 0; i < m.process_count; i++) {
        struct capsule_process *p = &m.processes[i];
        if (!p->has_tcp || p->tcp_port != s->port)
            continue;
        if (p->entry[0] && !capsvc_streq(p->entry, "http"))
            continue;
        if (p->has_host)
            (void)capsvc_bind_host((u32)(s - g_svc), p->host);
        i32 n = capsule_store_read(CAPSULE_PACK_MIN, p->bytecode, prog->data, CAPSVC_PROG_MAX);
        if (n > 0 && ((u32)n & 3U) == 0U)
            prog->len = (u32)n;
        return;
    }
}

void capsvc_init(void)
{
    for (u32 i = 0; i < g_svc_count; i++) {
        struct capsvc_registration *s = &g_svc[i];
        if (!s->active)
            continue;
        s->listen_conn = tcp_listen(s->port);
    }
    struct capsvc_arena *a = capsvc_arena();
    for (u32 i = 0; i < CAPSVC_MAX_SERVICES; i++) {
        a->attach[i].magic = 0;
        a->attach[i].svc_idx = i;
        a->attach[i].pid = 0;
        if (i < g_svc_count && g_svc[i].active)
            capsvc_preload_program(&g_svc[i], &a->programs[i]);
        else
            a->programs[i].len = 0;
    }
    for (u32 i = 0; i < CAPSVC_SLOT_COUNT; i++) {
        g_rt[i].inuse = false;
        g_rt[i].conn = -1;
        g_rt[i].state = CAPSVC_RT_IDLE;
        a->slots[i].hdr.generation = 0;
        a->slots[i].hdr.state = CAPSVC_SLOT_FREE;
    }
    capsvc_clean(a, sizeof(*a));
}

/* Poll every registered service's attach block for a not-yet-discovered
 * capsule pid. Cheap (CAPSVC_MAX_SERVICES cache lines) and safe to call every
 * pass -- once capsule_pid is set it is never re-read, so a capsule that
 * restarts with a new pid would need an explicit re-attach path (not needed
 * yet: capsules are expected to run for the kernel's lifetime, same as
 * uhttp_bridge's workers). */
static void capsvc_poll_attach(void)
{
    struct capsvc_arena *a = capsvc_arena();
    for (u32 i = 0; i < g_svc_count; i++) {
        if (g_svc[i].capsule_pid != 0)
            continue;
        capsvc_inval(&a->attach[i], CAPSVC_LINE);
        if (a->attach[i].magic == CAPSVC_ATTACH_MAGIC && a->attach[i].pid != 0)
            capsvc_attach(i, a->attach[i].pid);
    }
}

static bool capsvc_req_headers_complete(const u8 *p, u32 n)
{
    for (u32 i = 3; i < n; i++) {
        if (p[i - 3] == '\r' && p[i - 2] == '\n' && p[i - 1] == '\r' && p[i] == '\n')
            return true;
    }
    return false;
}

/* Allocate a free arena slot + its paired runtime tracker for a new
 * connection. One free-list scan covers both halves since they share an
 * index -- fail closed (refuse the connection) if the table is full rather
 * than silently reusing a slot still owned by someone else. */
static i32 capsvc_slot_alloc(u32 svc_idx, tcp_conn_t c)
{
    struct capsvc_arena *a = capsvc_arena();
    for (u32 i = 0; i < CAPSVC_SLOT_COUNT; i++) {
        if (g_rt[i].inuse)
            continue;
        struct capsvc_rt *rt = &g_rt[i];
        rt->inuse = true;
        rt->svc_idx = svc_idx;
        rt->conn = c;
        rt->state = CAPSVC_RT_READING;
        rt->resp_off = 0;
        rt->phase_ms = timer_monotonic_ms();
        rt->generation = ++g_generation_ctr;
        struct capsvc_slot *slot = &a->slots[i];
        slot->hdr.generation = rt->generation;
        slot->hdr.svc_idx = svc_idx;
        slot->hdr.conn_slot = i;
        slot->hdr.req_len = 0;
        slot->hdr.resp_len = 0;
        slot->hdr.state = CAPSVC_SLOT_FREE;   /* not REQUEST until headers are complete */
        capsvc_clean(&slot->hdr, CAPSVC_LINE);
        return (i32)i;
    }
    return -1;
}

static void capsvc_slot_release(i32 slot_idx)
{
    struct capsvc_rt *rt = &g_rt[(u32)slot_idx];
    struct capsvc_slot *slot = &capsvc_arena()->slots[(u32)slot_idx];
    if (rt->conn >= 0)
        tcp_close(rt->conn);
    /* Poison + bump generation before returning to the free pool, so a
     * stale/replayed reply for this slot can never be mistaken for live
     * (the next slot_alloc() bumps it again, but doing it here too closes
     * the window between release and reuse). */
    slot->hdr.state = CAPSVC_SLOT_FREE;
    slot->hdr.generation++;
    slot->hdr.req_len = 0;
    slot->hdr.resp_len = 0;
    capsvc_clean(&slot->hdr, CAPSVC_LINE);
    rt->inuse = false;
    rt->conn = -1;
    rt->state = CAPSVC_RT_IDLE;
}

static void capsvc_poll_slot(i32 slot_idx)
{
    struct capsvc_rt *rt = &g_rt[(u32)slot_idx];
    struct capsvc_registration *s = &g_svc[rt->svc_idx];
    struct capsvc_slot *slot = &capsvc_arena()->slots[(u32)slot_idx];
    u64 now = timer_monotonic_ms();

    switch (rt->state) {
    case CAPSVC_RT_READING: {
        u32 st = tcp_state(rt->conn);
        if (st == TCP_CLOSED || st >= TCP_CLOSING) {
            capsvc_slot_release(slot_idx);
            return;
        }
        u32 avail = tcp_readable(rt->conn);
        u32 req_len = slot->hdr.req_len;
        if (avail > 0 && req_len < CAPSVC_SLOT_DATA_MAX) {
            u32 rd = tcp_read(rt->conn, slot->data + req_len, CAPSVC_SLOT_DATA_MAX - req_len);
            req_len += rd;
            slot->hdr.req_len = req_len;
        }
        bool eoh = (req_len >= 4) && capsvc_req_headers_complete(slot->data, req_len);
        bool full = req_len >= CAPSVC_SLOT_DATA_MAX;
        bool stalled = (req_len > 0) && (now - rt->phase_ms > 40ULL);
        if (eoh || full || stalled) {
            /* Publish the request: payload bytes cleaned before the header
             * flip + wake (producer writes payload before head/state, per
             * the FIFO publication contract), so the capsule reads them
             * coherently once woken. */
            capsvc_clean(slot->data, req_len);
            slot->hdr.tag = capsvc_tag_pack(s->port, (u16)slot_idx, rt->generation);
            slot->hdr.state = CAPSVC_SLOT_REQUEST;
            capsvc_clean(&slot->hdr, CAPSVC_LINE);
            if (s->capsule_pid)
                proc_post_remote_wake(s->target_core, s->capsule_pid);
            rt->state = CAPSVC_RT_WAIT_REPLY;
            rt->phase_ms = now;
        } else if (now - rt->phase_ms > 2000ULL) {
            capsvc_slot_release(slot_idx);   /* client opened but sent nothing */
        }
        return;
    }
    case CAPSVC_RT_WAIT_REPLY: {
        capsvc_inval(&slot->hdr, CAPSVC_LINE);
        if (slot->hdr.state == CAPSVC_SLOT_REPLY && slot->hdr.generation == rt->generation) {
            u32 rl = slot->hdr.resp_len;
            if (rl > CAPSVC_SLOT_DATA_MAX)
                rl = CAPSVC_SLOT_DATA_MAX;
            if (rl)
                capsvc_inval(slot->data, rl);
            rt->resp_off = 0;
            rt->state = CAPSVC_RT_SENDING;
            rt->phase_ms = now;
        } else if (now - rt->phase_ms > 5000ULL) {
            static const char to[] =
                "HTTP/1.0 504 Gateway Timeout\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
            if (rt->conn >= 0)
                tcp_write(rt->conn, to, sizeof(to) - 1U);
            capsvc_slot_release(slot_idx);
        }
        return;
    }
    case CAPSVC_RT_SENDING: {
        u32 st = tcp_state(rt->conn);
        if (st == TCP_CLOSED || st >= TCP_CLOSING) {
            capsvc_slot_release(slot_idx);
            return;
        }
        u32 resp_len = slot->hdr.resp_len;
        if (resp_len > CAPSVC_SLOT_DATA_MAX)
            resp_len = CAPSVC_SLOT_DATA_MAX;
        if (rt->resp_off < resp_len) {
            u32 writable = tcp_writable(rt->conn);
            if (writable > 0) {
                u32 remain = resp_len - rt->resp_off;
                u32 chunk = remain < writable ? remain : writable;
                u32 n = tcp_write(rt->conn, slot->data + rt->resp_off, chunk);
                rt->resp_off += n;
                if (n) rt->phase_ms = now;
            }
        }
        if (rt->resp_off >= resp_len && tcp_tx_pending(rt->conn) == 0) {
            capsvc_slot_release(slot_idx);
        } else if (now - rt->phase_ms > 5000ULL) {
            capsvc_slot_release(slot_idx);
        }
        return;
    }
    default:
        return;
    }
}

i32 capsvc_external_begin(u32 svc_idx, const u8 *request, u32 request_len,
                          u64 *token_out)
{
    if (svc_idx >= g_svc_count || !request || request_len == 0U ||
        request_len > CAPSVC_SLOT_DATA_MAX || !token_out)
        return -1;
    i32 slot_idx = capsvc_slot_alloc(svc_idx, -1);
    if (slot_idx < 0)
        return -1;
    struct capsvc_rt *rt = &g_rt[(u32)slot_idx];
    struct capsvc_registration *s = &g_svc[svc_idx];
    struct capsvc_slot *slot = &capsvc_arena()->slots[(u32)slot_idx];
    simd_memcpy(slot->data, request, request_len);
    slot->hdr.req_len = request_len;
    slot->hdr.tag = capsvc_tag_pack(s->port, (u16)slot_idx, rt->generation);
    capsvc_clean(slot->data, request_len);
    slot->hdr.state = CAPSVC_SLOT_REQUEST;
    capsvc_clean(&slot->hdr, CAPSVC_LINE);
    if (s->capsule_pid)
        proc_post_remote_wake(s->target_core, s->capsule_pid);
    rt->state = CAPSVC_RT_EXTERNAL_WAIT;
    rt->phase_ms = timer_monotonic_ms();
    *token_out = slot->hdr.tag;
    return 0;
}

i32 capsvc_external_poll(u64 token, u8 *response, u32 response_max,
                         u32 *response_len_out)
{
    u32 idx = capsvc_tag_slot(token);
    if (idx >= CAPSVC_SLOT_COUNT || !response || !response_len_out)
        return -1;
    struct capsvc_rt *rt = &g_rt[idx];
    struct capsvc_slot *slot = &capsvc_arena()->slots[idx];
    if (!rt->inuse || rt->state != CAPSVC_RT_EXTERNAL_WAIT ||
        rt->generation != capsvc_tag_generation(token))
        return -1;
    capsvc_inval(&slot->hdr, CAPSVC_LINE);
    if (slot->hdr.state != CAPSVC_SLOT_REPLY)
        return 0;
    u32 len = slot->hdr.resp_len;
    if (len > CAPSVC_SLOT_DATA_MAX || len > response_max) {
        capsvc_slot_release((i32)idx);
        return -1;
    }
    if (len) {
        capsvc_inval(slot->data, len);
        simd_memcpy(response, slot->data, len);
    }
    *response_len_out = len;
    capsvc_slot_release((i32)idx);
    return 1;
}

void capsvc_external_cancel(u64 token)
{
    u32 idx = capsvc_tag_slot(token);
    if (idx >= CAPSVC_SLOT_COUNT)
        return;
    struct capsvc_rt *rt = &g_rt[idx];
    if (rt->inuse && rt->state == CAPSVC_RT_EXTERNAL_WAIT &&
        rt->generation == capsvc_tag_generation(token))
        capsvc_slot_release((i32)idx);
}

void capsvc_poll(void)
{
    g_dbg_poll_calls++;
    capsvc_poll_attach();
    for (u32 i = 0; i < g_svc_count; i++) {
        struct capsvc_registration *s = &g_svc[i];
        if (!s->active || s->listen_conn < 0)
            continue;
        g_dbg_accept_calls++;
        tcp_conn_t c = tcp_accept(s->listen_conn);
        if (c >= 0) {
            g_dbg_accept_success++;
            i32 slot = capsvc_slot_alloc(i, c);
            if (slot < 0) {
                g_dbg_slot_alloc_fail++;
                tcp_close(c);   /* no free slot: fail closed, never silently drop ownership */
            }
        }
    }
    for (u32 i = 0; i < CAPSVC_SLOT_COUNT; i++) {
        if (g_rt[i].inuse)
            capsvc_poll_slot((i32)i);
    }
}

static void dbg_append(char *dst, u32 *off, u32 cap, const char *src)
{
    while (*src && *off + 1U < cap) dst[(*off)++] = *src++;
    dst[*off] = 0;
}
static void dbg_append_u32(char *dst, u32 *off, u32 cap, u32 v)
{
    char tmp[12]; u32 i = 0;
    if (v == 0) tmp[i++] = '0';
    while (v) { tmp[i++] = (char)('0' + (v % 10U)); v /= 10U; }
    while (i) dbg_append(dst, off, cap, (const char[]){ tmp[--i], 0 });
}

void capsvc_debug_status(char *out, u32 *len, u32 max)
{
    u32 off = *len;
    struct capsvc_arena *a = capsvc_arena();
    dbg_append(out, &off, max, "capsvc poll_calls=");
    dbg_append_u32(out, &off, max, g_dbg_poll_calls);
    dbg_append(out, &off, max, " accept_calls=");
    dbg_append_u32(out, &off, max, g_dbg_accept_calls);
    dbg_append(out, &off, max, " accept_success=");
    dbg_append_u32(out, &off, max, g_dbg_accept_success);
    dbg_append(out, &off, max, " slot_alloc_fail=");
    dbg_append_u32(out, &off, max, g_dbg_slot_alloc_fail);
    dbg_append(out, &off, max, "\n");
    {
        tcp_snapshot_entry_t snap[16];
        u32 n = tcp_snapshot(snap, 16);
        for (u32 i = 0; i < g_svc_count; i++) {
            for (u32 j = 0; j < n; j++) {
                if (snap[j].conn != g_svc[i].listen_conn)
                    continue;
                dbg_append(out, &off, max, "  listener svc=");
                dbg_append_u32(out, &off, max, i);
                dbg_append(out, &off, max, " conn=");
                dbg_append_u32(out, &off, max, (u32)g_svc[i].listen_conn);
                dbg_append(out, &off, max, " state=");
                dbg_append_u32(out, &off, max, snap[j].state);
                dbg_append(out, &off, max, " pending=");
                dbg_append_u32(out, &off, max, snap[j].pending_count);
                dbg_append(out, &off, max, "\n");
            }
        }
    }
    for (u32 i = 0; i < g_svc_count; i++) {
        capsvc_inval(&a->attach[i], CAPSVC_LINE);
        dbg_append(out, &off, max, "capsvc svc=");
        dbg_append_u32(out, &off, max, i);
        dbg_append(out, &off, max, " port=");
        dbg_append_u32(out, &off, max, g_svc[i].port);
        dbg_append(out, &off, max, " host=");
        dbg_append(out, &off, max, g_svc[i].host[0] ? g_svc[i].host : "*");
        dbg_append(out, &off, max, " kernel_pid=");
        dbg_append_u32(out, &off, max, g_svc[i].capsule_pid);
        dbg_append(out, &off, max, " arena_magic=");
        dbg_append_u32(out, &off, max, a->attach[i].magic);
        dbg_append(out, &off, max, " arena_pid=");
        dbg_append_u32(out, &off, max, a->attach[i].pid);
        dbg_append(out, &off, max, " prog_len=");
        dbg_append_u32(out, &off, max, a->programs[i].len);
        dbg_append(out, &off, max, "\n");
    }
    for (u32 i = 0; i < CAPSVC_SLOT_COUNT; i++) {
        if (!g_rt[i].inuse)
            continue;
        struct capsvc_slot *slot = &a->slots[i];
        capsvc_inval(&slot->hdr, CAPSVC_LINE);
        dbg_append(out, &off, max, "  slot=");
        dbg_append_u32(out, &off, max, i);
        dbg_append(out, &off, max, " rt_state=");
        dbg_append_u32(out, &off, max, g_rt[i].state);
        dbg_append(out, &off, max, " hdr_state=");
        dbg_append_u32(out, &off, max, slot->hdr.state);
        dbg_append(out, &off, max, " req_len=");
        dbg_append_u32(out, &off, max, slot->hdr.req_len);
        dbg_append(out, &off, max, " resp_len=");
        dbg_append_u32(out, &off, max, slot->hdr.resp_len);
        dbg_append(out, &off, max, " gen=");
        dbg_append_u32(out, &off, max, slot->hdr.generation);
        dbg_append(out, &off, max, "/");
        dbg_append_u32(out, &off, max, g_rt[i].generation);
        dbg_append(out, &off, max, "\n");
    }
    *len = off;
}

u32 capsvc_service_count(void)
{
    return g_svc_count;
}

bool capsvc_service_info(u32 idx, u16 *port_out, u32 *target_core_out,
                          u32 *capsule_pid_out, bool *attached_out,
                          char *desc_out, u32 desc_max)
{
    if (idx >= g_svc_count || !g_svc[idx].active)
        return false;
    struct capsvc_registration *s = &g_svc[idx];
    if (port_out) *port_out = s->port;
    if (target_core_out) *target_core_out = s->target_core;
    if (capsule_pid_out) *capsule_pid_out = s->capsule_pid;
    if (attached_out) *attached_out = (s->capsule_pid != 0);
    if (desc_out && desc_max) {
        struct capsvc_arena *a = capsvc_arena();
        capsvc_inval(&a->attach[idx], 2U * CAPSVC_LINE);
        u32 n = 0;
        while (n + 1U < desc_max && n < CAPSVC_DESC_MAX && a->attach[idx].description[n]) {
            desc_out[n] = a->attach[idx].description[n];
            n++;
        }
        desc_out[n] = 0;
    }
    return true;
}

bool capsvc_service_host(u32 idx, char *host_out, u32 host_max)
{
    if (idx >= g_svc_count || !g_svc[idx].active ||
        !host_out || host_max == 0U)
        return false;
    u32 n = 0;
    while (n + 1U < host_max && g_svc[idx].host[n]) {
        host_out[n] = g_svc[idx].host[n];
        n++;
    }
    host_out[n] = 0;
    return n != 0U;
}
