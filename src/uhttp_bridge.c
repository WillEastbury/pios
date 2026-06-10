/*
 * uhttp_bridge.c - core 0 side of the kernel<->userland HTTP bridge.
 *
 * The kernel owns the TCP listener on port 81 (so the network core is never
 * blocked). For each accepted connection it reads the request into the shared
 * bridge, bumps req_seq, and wakes the userland httpd process on its core via
 * proc_post_remote_wake (queued wake + SEV — no lost-wakeup window). When the
 * userland process publishes a response (resp_seq catches up), core 0 writes it
 * back and closes. One request is in flight at a time; further connections wait
 * in the TCP listen backlog.
 */
#include "uhttp_bridge.h"
#include "core.h"
#include "core_env.h"
#include "tcp.h"
#include "timer.h"
#include "proc.h"
#include "types.h"
#include "picowal_db.h"

_Static_assert(UHTTP_BRIDGE_ADDR == IPC_SHM_BASE,
               "uhttp bridge must live at IPC_SHM_BASE");
/* Zones must land on separate 64-byte cache lines, buffers line-aligned, so the
 * clean/invalidate discipline never write-back-clobbers the opposite direction. */
_Static_assert(__builtin_offsetof(struct uhttp_bridge, magic) == 0, "zoneB line");
_Static_assert(__builtin_offsetof(struct uhttp_bridge, req_seq) == 64, "zoneA line");
_Static_assert(__builtin_offsetof(struct uhttp_bridge, req) == 128, "req aligned");
_Static_assert((UHTTP_REQ_MAX % 64U) == 0U, "req multiple of line");
_Static_assert((__builtin_offsetof(struct uhttp_bridge, pico_prog_len) % 64U) == 0U, "pico metadata aligned");
_Static_assert((__builtin_offsetof(struct uhttp_bridge, pico_prog) % 64U) == 0U, "pico prog aligned");
_Static_assert((__builtin_offsetof(struct uhttp_bridge, pico_static) % 64U) == 0U, "pico static aligned");
_Static_assert((__builtin_offsetof(struct uhttp_bridge, pico_api) % 64U) == 0U, "pico api aligned");
_Static_assert((__builtin_offsetof(struct uhttp_bridge, resp) % 64U) == 0U, "resp aligned");
_Static_assert((UHTTP_RESP_MAX % 64U) == 0U, "resp multiple of line");

enum { U_IDLE = 0, U_READING, U_DISPATCHED, U_SENDING };

static tcp_conn_t listen_conn = -1;
static tcp_conn_t cur_conn = -1;
static u32 cur_state;
static u32 dispatch_seq;     /* req_seq value handed to the httpd */
static u32 send_len;         /* total response bytes to stream from b->resp */
static u32 send_off;         /* response bytes handed to tcp_write so far    */
static u64 phase_ms;         /* timestamp for per-phase timeouts */

void uhttp_bridge_init(void)
{
    struct uhttp_bridge *b = uhttp_bridge();
    b->magic = 0;
    b->httpd_pid = 0;
    b->req_seq = 0;
    b->resp_seq = 0;
    b->req_len = 0;
    b->resp_len = 0;
    b->reqs_total = 0;
    b->pico_prog_len = 0;
    b->pico_static_len = 0;
    b->pico_api_len = 0;
    b->pico_flags = 0;
    /* Push the zeroed control lines to PoC so no dirty core-0 copy can later be
     * evicted over the userland httpd's magic/pid/resp writes. */
    uhttp_clean(&b->magic, 128);
    dmb();
    listen_conn = tcp_listen((u16)UHTTP_PORT);
    cur_conn = -1;
    cur_state = U_IDLE;
}

static u32 uhttp_load_card(u32 record, u8 *dst)
{
    i32 n = picowal_db_get((u16)UHTTP_PICOWEB_CARD, record, dst, UHTTP_PICO_MAX);
    return n > 0 ? (u32)n : 0;
}

static bool uhttp_card_text_ok(const u8 *p, u32 n, u8 first)
{
    if (!p || n == 0 || p[0] != first)
        return false;
    for (u32 i = 0; i < n; i++) {
        u8 c = p[i];
        if (c == '\r' || c == '\n' || c == '\t')
            continue;
        if (c < 0x20U || c > 0x7EU)
            return false;
    }
    return true;
}

static void uhttp_preload_picoweb(struct uhttp_bridge *b)
{
    b->pico_prog_len = uhttp_load_card(UHTTP_PICO_PROGRAM_RECORD, b->pico_prog);
    b->pico_static_len = uhttp_load_card(UHTTP_PICO_STATIC_RECORD, b->pico_static);
    b->pico_api_len = uhttp_load_card(UHTTP_PICO_API_RECORD, b->pico_api);
    if ((b->pico_prog_len & 3U) != 0)
        b->pico_prog_len = 0;
    if (!uhttp_card_text_ok(b->pico_static, b->pico_static_len, '<'))
        b->pico_static_len = 0;
    if (!uhttp_card_text_ok(b->pico_api, b->pico_api_len, '{'))
        b->pico_api_len = 0;
    b->pico_flags = ((b->pico_prog_len ? 1U : 0U) |
                     (b->pico_static_len ? 2U : 0U) |
                     (b->pico_api_len ? 4U : 0U));
    if (b->pico_prog_len)
        uhttp_clean(b->pico_prog, b->pico_prog_len);
    if (b->pico_static_len)
        uhttp_clean(b->pico_static, b->pico_static_len);
    if (b->pico_api_len)
        uhttp_clean(b->pico_api, b->pico_api_len);
    uhttp_clean(&b->pico_prog_len, UHTTP_LINE);
}

bool uhttp_bridge_ready(void)
{
    struct uhttp_bridge *b = uhttp_bridge();
    uhttp_inval(&b->magic, UHTTP_LINE);
    return b->magic == UHTTP_BRIDGE_MAGIC;
}

u32 uhttp_bridge_requests(void)
{
    return uhttp_bridge()->reqs_total;
}

void uhttp_bridge_state(i32 *p_listen, u32 *p_state, u32 *p_req_seq, u32 *p_resp_seq,
                        u32 *p_reqs, u32 *p_magic, u32 *p_pid)
{
    struct uhttp_bridge *b = uhttp_bridge();
    /* Refresh core 0's view of both control lines so the snapshot reflects the
     * userland side's most recent cross-core writes (magic/pid/resp/dbg_loops). */
    uhttp_inval(&b->magic, 128);
    if (p_listen)   *p_listen   = (i32)listen_conn;
    if (p_state)    *p_state    = cur_state;
    if (p_req_seq)  *p_req_seq  = b->req_seq;
    if (p_resp_seq) *p_resp_seq = b->resp_seq;
    if (p_reqs)     *p_reqs     = b->reqs_total;
    if (p_magic)    *p_magic    = b->magic;
    if (p_pid)      *p_pid      = b->httpd_pid;
}

static bool req_headers_complete(const u8 *p, u32 n)
{
    for (u32 i = 3; i < n; i++) {
        if (p[i - 3] == '\r' && p[i - 2] == '\n' &&
            p[i - 1] == '\r' && p[i] == '\n')
            return true;
    }
    return false;
}

static void uhttp_finish(void)
{
    if (cur_conn >= 0)
        tcp_close(cur_conn);
    cur_conn = -1;
    cur_state = U_IDLE;
}

void uhttp_bridge_poll(void)
{
    if (listen_conn < 0)
        return;

    struct uhttp_bridge *b = uhttp_bridge();
    u64 now = timer_monotonic_ms();

    switch (cur_state) {
    case U_IDLE: {
        tcp_conn_t c = tcp_accept(listen_conn);
        if (c >= 0) {
            cur_conn = c;
            cur_state = U_READING;
            b->req_len = 0;
            phase_ms = now;
        }
        break;
    }

    case U_READING: {
        u32 st = tcp_state(cur_conn);
        if (st == TCP_CLOSED || st >= TCP_CLOSING) {
            uhttp_finish();
            break;
        }
        u32 avail = tcp_readable(cur_conn);
        if (avail > 0 && b->req_len < UHTTP_REQ_MAX) {
            u32 rd = tcp_read(cur_conn, b->req + b->req_len,
                              UHTTP_REQ_MAX - b->req_len);
            b->req_len += rd;
        }
        bool eoh = (b->req_len >= 4) && req_headers_complete(b->req, b->req_len);
        bool full = b->req_len >= UHTTP_REQ_MAX;
        bool stalled = (b->req_len > 0) && (now - phase_ms > 40);
        if (eoh || full || stalled) {
            /* Publish the request to the user core: request bytes + Zone A control
             * (req_len/req_seq) cleaned to PoC before the wake so the woken httpd
             * reads them coherently. */
            if (b->req_len)
                uhttp_clean(b->req, b->req_len);
            uhttp_preload_picoweb(b);
            b->req_seq = b->req_seq + 1U;
            dispatch_seq = b->req_seq;
            uhttp_clean(&b->req_seq, UHTTP_LINE);
            /* Read the userland attach state coherently: drop core 0's stale Zone B
             * copy so a magic/pid written once by the now-parked httpd is seen. */
            uhttp_inval(&b->magic, UHTTP_LINE);
            if (b->magic == UHTTP_BRIDGE_MAGIC && b->httpd_pid)
                proc_post_remote_wake(CORE_USER0, b->httpd_pid);
            cur_state = U_DISPATCHED;
            phase_ms = now;
        } else if (now - phase_ms > 2000) {
            uhttp_finish();   /* client opened but sent nothing */
        }
        break;
    }

    case U_DISPATCHED: {
        /* Refresh Zone B (resp_seq/resp_len) which the httpd writes on its core. */
        uhttp_inval(&b->magic, UHTTP_LINE);
        if (b->resp_seq == dispatch_seq) {
            u32 rl = b->resp_len;
            if (rl > UHTTP_RESP_MAX)
                rl = UHTTP_RESP_MAX;
            if (rl)
                uhttp_inval(b->resp, rl);   /* refresh the whole response from PoC once */
            /* Account the request now; reqs_total is published to httpd via Zone A.
             * httpd will not overwrite b->resp until req_seq advances again, so the
             * refreshed copy stays valid for the whole streamed send below. */
            b->reqs_total++;
            uhttp_clean(&b->req_seq, UHTTP_LINE);
            send_len = rl;
            send_off = 0;
            cur_state = U_SENDING;
            phase_ms = now;
        } else if (now - phase_ms > 3000) {
            static const char to[] =
                "HTTP/1.0 504 Gateway Timeout\r\n"
                "Content-Length: 0\r\nConnection: close\r\n\r\n";
            if (cur_conn >= 0)
                tcp_write(cur_conn, to, sizeof(to) - 1U);
            uhttp_finish();
        }
        break;
    }

    case U_SENDING: {
        /* Stream the response in small chunks gated on TX-ring space, one chunk
         * per poll so core 0's net loop processes the peer ACKs that grow cwnd and
         * drain the ring between writes. Close only once every byte has been sent
         * AND acknowledged (tcp_tx_pending == 0); a bare tcp_close() with data still
         * buffered would FIN at snd_nxt and truncate the unsent tail (the cause of
         * the short-body / premature-EOF responses on large pages). */
        u32 st = tcp_state(cur_conn);
        if (st == TCP_CLOSED || st >= TCP_CLOSING) {
            uhttp_finish();
            break;
        }
        if (send_off < send_len) {
            u32 writable = tcp_writable(cur_conn);
            if (writable > 0) {
                u32 remain = send_len - send_off;
                u32 chunk = remain < writable ? remain : writable;
                if (chunk > 512U)
                    chunk = 512U;
                u32 n = tcp_write(cur_conn, b->resp + send_off, chunk);
                send_off += n;
                if (n)
                    phase_ms = now;
            }
        }
        if (send_off >= send_len && tcp_tx_pending(cur_conn) == 0) {
            uhttp_finish();   /* fully sent + acked: graceful close, no truncation */
        } else if (now - phase_ms > 5000) {
            uhttp_finish();   /* stalled peer: drop the connection */
        }
        break;
    }

    default:
        cur_state = U_IDLE;
        break;
    }
}
