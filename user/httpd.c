/*
 * httpd.c - PIOS userland HTTP server (process on a user core).
 *
 * The kernel terminates TCP on port 81 and hands each request to this process
 * via the shared uhttp_bridge at IPC_SHM_BASE. This process SLEEPS (api->park)
 * between requests and is woken by a cross-core software interrupt when a
 * request arrives — no syscall spin, no blocking socket calls. It reads the
 * request, builds the response into the shared buffer, and signals the kernel,
 * which writes it back to the client. Live kernel data (pid, uptime, request
 * count) is read through the kernel_api table at request time.
 *
 * Constraint: the image is mapped read-execute (W^X) — no writable globals; all
 * mutable state is on the stack or in the shared bridge.
 */
#include "types.h"
#include "proc.h"          /* struct kernel_api */
#include "uhttp_bridge.h"  /* struct uhttp_bridge, UHTTP_BRIDGE_ADDR */

/* Minimal freestanding mem primitives (also satisfy implicit compiler calls). */
void *memcpy(void *d, const void *s, unsigned long n)
{
    u8 *dd = (u8 *)d; const u8 *ss = (const u8 *)s;
    for (unsigned long i = 0; i < n; i++) dd[i] = ss[i];
    return d;
}
void *memset(void *d, int c, unsigned long n)
{
    u8 *dd = (u8 *)d;
    for (unsigned long i = 0; i < n; i++) dd[i] = (u8)c;
    return d;
}

static u32 u_strlen(const char *s)
{
    u32 n = 0;
    while (s && s[n]) n++;
    return n;
}

static void u_append(char *dst, u32 *off, u32 cap, const char *src)
{
    u32 n = u_strlen(src);
    if (*off + n >= cap) n = (cap > *off) ? (cap - *off - 1) : 0;
    for (u32 i = 0; i < n; i++) dst[*off + i] = src[i];
    *off += n;
    dst[*off] = 0;
}

static void u_append_u32(char *dst, u32 *off, u32 cap, u32 v)
{
    char tmp[12];
    u32 i = 0;
    if (v == 0) tmp[i++] = '0';
    while (v) { tmp[i++] = (char)('0' + (v % 10)); v /= 10; }
    char rev[12];
    for (u32 j = 0; j < i; j++) rev[j] = tmp[i - 1 - j];
    rev[i] = 0;
    u_append(dst, off, cap, rev);
}

static u32 build_body(struct kernel_api *api, char *buf, u32 cap, u32 reqs)
{
    u32 off = 0;
    u32 pid = api->getpid();
    u32 up_s = (u32)(api->monotonic_ms() / 1000ULL);

    u_append(buf, &off, cap,
        "<!doctype html><html data-theme='dark'><head><meta charset='utf-8'>"
        "<title>PIOS userland httpd</title><style>"
        ":root{--bg:#3d3b3a;--surface:#292929;--border:#474747;--text:#dedede;"
        "--muted:#919191;--accent:#fd8ea1}"
        "body{margin:0;background:var(--bg);color:var(--text);"
        "font-family:'Segoe UI',Aptos,Calibri,-apple-system,sans-serif}"
        ".wrap{max-width:760px;margin:0 auto;padding:32px}"
        ".card{background:var(--surface);border:1px solid var(--border);"
        "border-radius:16px;padding:24px;margin:16px 0}"
        "h1{color:var(--accent);margin:0 0 4px}"
        ".muted{color:var(--muted)}.pill{display:inline-block;border:1px solid "
        "var(--border);border-radius:.625rem;padding:2px 10px;"
        "background:rgba(253,142,161,.14);color:var(--accent)}"
        "table{width:100%;border-collapse:collapse}td{border-bottom:1px solid "
        "var(--border);padding:8px;vertical-align:top}td:first-child{"
        "color:var(--muted);width:46%}</style></head><body><div class='wrap'>");

    u_append(buf, &off, cap, "<div class='card'><h1>PIOS userland httpd</h1>"
        "<p class='muted'>Kernel terminates TCP; this userland process owns "
        "HTTP. It sleeps between requests and is woken by a cross-core software "
        "interrupt &mdash; no syscall spin.</p><span class='pill'>port 81 "
        "&middot; userspace &middot; firehose</span></div>");

    u_append(buf, &off, cap, "<div class='card'><table>");
    u_append(buf, &off, cap, "<tr><td>Handler</td><td>userland process (core 2)"
        "</td></tr><tr><td>Process PID</td><td>");
    u_append_u32(buf, &off, cap, pid);
    u_append(buf, &off, cap, "</td></tr><tr><td>Kernel uptime</td><td>");
    u_append_u32(buf, &off, cap, up_s);
    u_append(buf, &off, cap, " s</td></tr><tr><td>Requests served</td><td>");
    u_append_u32(buf, &off, cap, reqs);
    u_append(buf, &off, cap, "</td></tr><tr><td>Transport</td><td>shared span "
        "bridge + SW-int wake</td></tr></table></div>");

    u_append(buf, &off, cap, "<p class='muted'>PIOS &middot; bare-metal "
        "Raspberry Pi 5</p></div></body></html>");
    return off;
}

void user_main(struct kernel_api *api)
{
    struct uhttp_bridge *b = (struct uhttp_bridge *)(usize)UHTTP_BRIDGE_ADDR;

    /* Prove process entry with a constant store that needs NO kernel API call,
     * before touching the api table or the UART. If magic appears but pid stays
     * 0, we entered but api->getpid() stalled; if magic stays 0, we never ran. */
    b->magic = UHTTP_BRIDGE_MAGIC;

    /* Now exercise the kernel API: publish our pid so the kernel can wake us. */
    b->resp_seq = 0;
    b->httpd_pid = api->getpid();
    /* Push the whole Zone B control line to PoC so core 0 sees magic/pid even
     * though this core is about to park and never evict the line on its own. */
    uhttp_clean(&b->magic, UHTTP_LINE);

    api->print("[httpd] attached, waiting on :81\n");

    uhttp_inval(&b->req_seq, UHTTP_LINE);
    u32 last = b->req_seq;
    for (;;) {
        b->dbg_loops++;
        b->dbg_phase = 10U;                     /* loop top: alive, about to poll */
        uhttp_clean(&b->magic, UHTTP_LINE);     /* publish dbg_loops/dbg_phase now */
        uhttp_inval(&b->req_seq, UHTTP_LINE);   /* refresh Zone A: req_seq/reqs_total */
        u32 cur = b->req_seq;
        if (cur == last) {
            b->dbg_phase = 12U;                 /* parking (no new request) */
            uhttp_clean(&b->magic, UHTTP_LINE); /* publish dbg_loops/resp_seq, then sleep */
            api->park();          /* sleep until the kernel wakes us */
            b->dbg_phase = 13U;                 /* resumed from park */
            uhttp_clean(&b->magic, UHTTP_LINE);
            continue;
        }

        b->dbg_phase = 20U;                     /* request seen: building body */
        uhttp_clean(&b->magic, UHTTP_LINE);
        u32 reqs = b->reqs_total + 1U;
        char body[3072];
        u32 blen = build_body(api, body, sizeof(body), reqs);

        b->dbg_phase = 21U;                     /* body built: writing response */
        uhttp_clean(&b->magic, UHTTP_LINE);
        char *r = (char *)b->resp;
        u32 off = 0;
        u_append(r, &off, UHTTP_RESP_MAX,
            "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nContent-Length: ");
        u_append_u32(r, &off, UHTTP_RESP_MAX, blen);
        u_append(r, &off, UHTTP_RESP_MAX, "\r\nConnection: close\r\n\r\n");
        for (u32 i = 0; i < blen && off < UHTTP_RESP_MAX - 1U; i++)
            r[off++] = body[i];

        b->resp_len = off;
        b->resp_seq = cur;        /* signal the kernel: response ready */
        b->dbg_phase = 24U;       /* response published */
        uhttp_clean(b->resp, off);          /* response bytes to PoC first */
        uhttp_clean(&b->magic, UHTTP_LINE); /* then resp_len/resp_seq (Zone B) */
        last = cur;
    }
}
