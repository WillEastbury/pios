/*
 * pixe_request.c - EL1 "Picowal/Picoweb protocol component".
 *
 * Decodes an HTTP request, validates framing, interns it into the offset-relative
 * spans of struct pixe_request_context, and attaches the kernel-established
 * principal binding. This is the EL1 half of the EL1<->EL0 web pipeline; the EL0
 * PicoScript endpoint consumes the sealed context (read-only) via Context.* hooks.
 */
#include "pixe_request.h"
#include "tls.h"
#include "principal.h"

static u32 g_pixe_request_id = 0;

static int pixe_ci_eq(const u8 *a, const char *b, u32 n)
{
    for (u32 i = 0; i < n; i++) {
        u8 ca = a[i], cb = (u8)b[i];
        if (ca >= 'A' && ca <= 'Z') ca = (u8)(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = (u8)(cb - 'A' + 'a');
        if (ca != cb) return 0;
    }
    return 1;
}

static u32 pixe_method_id(const u8 *v, u32 n)
{
    if (n == 3 && pixe_ci_eq(v, "get", 3)) return PIXE_M_GET;
    if (n == 4 && pixe_ci_eq(v, "post", 4)) return PIXE_M_POST;
    if (n == 3 && pixe_ci_eq(v, "put", 3)) return PIXE_M_PUT;
    if (n == 6 && pixe_ci_eq(v, "delete", 6)) return PIXE_M_DELETE;
    if (n == 4 && pixe_ci_eq(v, "head", 4)) return PIXE_M_HEAD;
    if (n == 7 && pixe_ci_eq(v, "options", 7)) return PIXE_M_OPTIONS;
    if (n == 5 && pixe_ci_eq(v, "patch", 5)) return PIXE_M_PATCH;
    return PIXE_M_UNKNOWN;
}

/* Scan a header-field region for "Host:" and return the value span (relative to
 * the request buffer base). Sets *out to a zero span if not found. */
static void pixe_find_host(const u8 *raw, u32 hdr_off, u32 hdr_len, struct pixe_span *out)
{
    out->off = 0;
    out->len = 0;
    u32 i = hdr_off;
    u32 end = hdr_off + hdr_len;
    while (i < end) {
        u32 ls = i;
        while (i + 1 < end && !(raw[i] == '\r' && raw[i + 1] == '\n')) i++;
        u32 le = i;                       /* [ls,le) is one header line */
        if (le > ls + 5 && pixe_ci_eq(raw + ls, "host:", 5)) {
            u32 v = ls + 5;
            while (v < le && (raw[v] == ' ' || raw[v] == '\t')) v++;
            out->off = v;
            out->len = (le >= v) ? (le - v) : 0;
            return;
        }
        i = (i + 1 < end) ? le + 2 : end; /* skip the CRLF */
    }
}

const u8 *pixe_span_ptr(const u8 *base, u32 base_len, struct pixe_span s, u32 *out_len)
{
    if (!base || s.len == 0 || s.off >= base_len || s.off + s.len > base_len) {
        if (out_len) *out_len = 0;
        return 0;
    }
    if (out_len) *out_len = s.len;
    return base + s.off;
}

i32 pixe_request_build(const u8 *raw, u32 len, u32 principal_id,
                       struct pixe_request_context *out)
{
    if (!raw || !out || len == 0) return PIXE_REQ_ERROR;

    /* Framing validation + body offset via the proven bridge parser. */
    struct tls_bridge_request br;
    i32 fr = tls_bridge_parse_request(raw, len, &br);
    if (fr != TLS_BRIDGE_OK) return fr; /* NEED_MORE(0) / ERROR(-1) map 1:1 */

    /* Locate the request-line structure to extract offset spans. */
    u32 line_end = 0;
    while (line_end + 1 < len && !(raw[line_end] == '\r' && raw[line_end + 1] == '\n'))
        line_end++;
    u32 sp1 = 0;
    while (sp1 < line_end && raw[sp1] != ' ') sp1++;
    u32 sp2 = sp1 + 1;
    while (sp2 < line_end && raw[sp2] != ' ') sp2++;
    if (sp1 == 0 || sp1 >= line_end || sp2 <= sp1 + 1 || sp2 >= line_end)
        return PIXE_REQ_ERROR;

    /* zero the context */
    u8 *z = (u8 *)out;
    for (u32 i = 0; i < sizeof(*out); i++) z[i] = 0;

    out->magic = PIXE_REQ_MAGIC;
    out->version = PIXE_REQ_VERSION;
    out->req_len = len;

    /* verb = [0, sp1) */
    out->verb.off = 0;
    out->verb.len = sp1;

    /* request-target = [sp1+1, sp2); split at '?' into path + query */
    u32 tstart = sp1 + 1, tend = sp2;
    u32 q = tstart;
    while (q < tend && raw[q] != '?') q++;
    out->path.off = tstart;
    out->path.len = q - tstart;
    if (q < tend) {
        out->query.off = q + 1;
        out->query.len = tend - (q + 1);
    }

    /* headers field block = [line_end+2, header_bytes-4) (excludes the request
     * line and the terminating blank-line CRLF, keeps each field's CRLF). */
    u32 hdr_off = line_end + 2;
    u32 hdr_len = (br.header_bytes > line_end + 4) ? (br.header_bytes - line_end - 4) : 0;
    out->headers.off = hdr_off;
    out->headers.len = hdr_len;

    pixe_find_host(raw, hdr_off, hdr_len, &out->host);

    /* body = [header_bytes, len) */
    if (len > br.header_bytes) {
        out->body.off = br.header_bytes;
        out->body.len = len - br.header_bytes;
        out->flags |= PIXE_REQ_F_HAS_BODY;
    }

    out->method_id = pixe_method_id(raw, sp1);

    /* Kernel-established bindings. The principal is supplied by the caller (the
     * kernel authenticates the request's auth header before calling us; the
     * endpoint can never set its own principal). Effective permissions are the
     * delegated-user capabilities; once a capsule is bound they union with the
     * capsule's grants (minus any deny) — that stacking happens at bind time. */
    out->principal_id = principal_id;
    out->capsule_id = 0;
    u32 perms = 0;
    if (principal_has_cap(principal_id, PRINCIPAL_ADMIN)) perms |= PRINCIPAL_ADMIN;
    if (principal_has_cap(principal_id, PRINCIPAL_NET))   perms |= PRINCIPAL_NET;
    if (principal_has_cap(principal_id, PRINCIPAL_DISK))  perms |= PRINCIPAL_DISK;
    if (principal_has_cap(principal_id, PRINCIPAL_EXEC))  perms |= PRINCIPAL_EXEC;
    if (principal_has_cap(principal_id, PRINCIPAL_IPC))   perms |= PRINCIPAL_IPC;
    out->permissions = perms;

    out->request_id = ++g_pixe_request_id;
    out->resp_status = -1;

    out->flags |= PIXE_REQ_F_SEALED;
    return PIXE_REQ_OK;
}

/* ---- selftest / dump (proves the EL1 component on hardware) ----------- */

static void pq_str(char *o, u32 *p, u32 max, const char *s)
{
    while (*s && *p + 1 < max) o[(*p)++] = *s++;
}
static void pq_u32(char *o, u32 *p, u32 max, u32 v)
{
    char t[12]; int n = 0;
    if (v == 0) t[n++] = '0';
    while (v) { t[n++] = (char)('0' + (v % 10)); v /= 10; }
    while (n > 0 && *p + 1 < max) o[(*p)++] = t[--n];
}
static void pq_i32(char *o, u32 *p, u32 max, i32 v)
{
    if (v < 0) { if (*p + 1 < max) o[(*p)++] = '-'; pq_u32(o, p, max, (u32)(-v)); }
    else pq_u32(o, p, max, (u32)v);
}
/* print a span both as off/len and as its resolved bytes (bounded). */
static void pq_span(char *o, u32 *p, u32 max, const char *label,
                    const u8 *base, u32 base_len, struct pixe_span s)
{
    pq_str(o, p, max, label);
    pq_str(o, p, max, " off=");
    pq_u32(o, p, max, s.off);
    pq_str(o, p, max, " len=");
    pq_u32(o, p, max, s.len);
    pq_str(o, p, max, " '");
    u32 rl = 0;
    const u8 *r = pixe_span_ptr(base, base_len, s, &rl);
    for (u32 i = 0; i < rl && *p + 1 < max; i++) {
        u8 c = r[i];
        o[(*p)++] = (c >= 32 && c < 127) ? (char)c : '.';
    }
    pq_str(o, p, max, "'\n");
}

u32 pixe_request_selftest(char *out, u32 max)
{
    static const u8 canned[] =
        "POST /api/sum?n=10 HTTP/1.1\r\n"
        "Host: pios.local\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: 5\r\n"
        "\r\n"
        "hello";
    u32 clen = (u32)sizeof(canned) - 1; /* drop the implicit NUL */

    struct pixe_request_context ctx;
    i32 r = pixe_request_build(canned, clen, principal_current(), &ctx);

    u32 p = 0;
    pq_str(out, &p, max, "pixe req build=");
    pq_i32(out, &p, max, r);
    pq_str(out, &p, max, " req_len=");
    pq_u32(out, &p, max, clen);
    pq_str(out, &p, max, "\n");
    if (r != PIXE_REQ_OK) {
        pq_str(out, &p, max, "(framing not OK)\n");
        return p;
    }
    pq_span(out, &p, max, "verb   ", canned, clen, ctx.verb);
    pq_span(out, &p, max, "path   ", canned, clen, ctx.path);
    pq_span(out, &p, max, "query  ", canned, clen, ctx.query);
    pq_span(out, &p, max, "host   ", canned, clen, ctx.host);
    pq_span(out, &p, max, "headers", canned, clen, ctx.headers);
    pq_span(out, &p, max, "body   ", canned, clen, ctx.body);
    pq_str(out, &p, max, "method_id=");
    pq_u32(out, &p, max, ctx.method_id);
    pq_str(out, &p, max, " principal=");
    pq_u32(out, &p, max, ctx.principal_id);
    pq_str(out, &p, max, " perms=0x");
    {
        const char *hx = "0123456789abcdef";
        for (int sh = 28; sh >= 0; sh -= 4)
            if (p + 1 < max) out[p++] = hx[(ctx.permissions >> sh) & 0xF];
    }
    pq_str(out, &p, max, " capsule=");
    pq_u32(out, &p, max, ctx.capsule_id);
    pq_str(out, &p, max, " req_id=");
    pq_u32(out, &p, max, ctx.request_id);
    pq_str(out, &p, max, " flags=0x");
    {
        const char *hx = "0123456789abcdef";
        for (int sh = 28; sh >= 0; sh -= 4)
            if (p + 1 < max) out[p++] = hx[(ctx.flags >> sh) & 0xF];
    }
    pq_str(out, &p, max, "\n");
    return p;
}
