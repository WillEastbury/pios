/*
 * pixe_host.c - EL0 "PIKEE / pix endpoint" host runtime.
 *
 * Implements the Context.* / Io.* / Span.* host hooks the EL0 endpoint needs to
 * read a sealed pixe_request_context and emit a response on the freestanding
 * picovm. Semantics mirror picoscript_vm.HostApi so register handles + output
 * bytes are byte-identical to the off-board reference VM.
 *
 * The host state embeds pv_ctx as its first member; the picovm host callback
 * recovers the host by casting the pv_ctx* back. This needs no edit to the
 * vendored picovm.c / picovm.h and is reentrant (no globals on the run path).
 */
#include "pixe_host.h"
#include "pico_hooks.h"
#include "principal.h"

/* ---- arena / span helpers (mirror picoscript_vm _str_span/spans) ------- */

static int pixe_alloc_span(struct pixe_host *h, const u8 *src, u32 n)
{
    if (h->span_count >= PIXE_MAX_SPANS) { h->oom = 1; return 0; }
    if (h->arena_top + n > PIXE_ARENA_SIZE) { h->oom = 1; return 0; }
    u32 ptr = h->arena_top;
    for (u32 i = 0; i < n; i++) h->mem[ptr + i] = src[i];
    h->arena_top += n;
    int handle = h->span_count++;
    h->spans[handle].ptr = ptr;
    h->spans[handle].len = n;
    return handle;
}

/* Materialize a request-context field span into the arena, returning a handle -
 * exactly what a reference host doing regs[rd] = _str_span(field) would produce. */
static int pixe_ctx_span(struct pixe_host *h, struct pixe_span s)
{
    u32 rl = 0;
    const u8 *r = pixe_span_ptr(h->req_bytes, h->req_bytes_len, s, &rl);
    if (!r) return pixe_alloc_span(h, (const u8 *)"", 0);
    return pixe_alloc_span(h, r, rl);
}

static void pixe_io_write(struct pixe_host *h, i32 handle)
{
    if (handle <= 0 || handle >= h->span_count) return;
    struct pixe_vm_span s = h->spans[handle];
    for (u32 i = 0; i < s.len; i++) {
        if (h->vm.out_len >= PV_MAX_OUT) break;
        h->vm.out[h->vm.out_len++] = h->mem[s.ptr + i];
    }
}

/* ---- the host-hook dispatcher (Context.* / Io.* / Span.*) -------------- */

static void pixe_host_hook(pv_ctx *ctx, int hook, int rd, int rs1, int rs2, int imm16)
{
    struct pixe_host *h = (struct pixe_host *)ctx; /* vm is the first member */
    (void)imm16;

    switch (hook) {
    /* Context.* -- string fields return span handles, scalars return ints. */
    case PV_HOOK_CONTEXT_GETVERB:        ctx->regs[rd] = pixe_ctx_span(h, h->req->verb);    return;
    case PV_HOOK_CONTEXT_GETPATH:        ctx->regs[rd] = pixe_ctx_span(h, h->req->path);    return;
    case PV_HOOK_CONTEXT_GETHOST:        ctx->regs[rd] = pixe_ctx_span(h, h->req->host);    return;
    case PV_HOOK_CONTEXT_GETHEADERS:     ctx->regs[rd] = pixe_ctx_span(h, h->req->headers); return;
    case PV_HOOK_CONTEXT_GETQUERYSTRING: ctx->regs[rd] = pixe_ctx_span(h, h->req->query);   return;
    case PV_HOOK_CONTEXT_GETBODY:        ctx->regs[rd] = pixe_ctx_span(h, h->req->body);    return;
    case PV_HOOK_CONTEXT_GETREMOTEADDR:  ctx->regs[rd] = pixe_alloc_span(h, (const u8 *)"", 0); return; /* TODO: format IPv4 */
    case PV_HOOK_CONTEXT_GETPORT:        ctx->regs[rd] = (i32)h->req->local_port;   return;
    case PV_HOOK_CONTEXT_GETUSER:        ctx->regs[rd] = (i32)h->req->principal_id; return;
    case PV_HOOK_CONTEXT_GETPERMISSIONS: ctx->regs[rd] = (i32)h->req->permissions;  return;
    case PV_HOOK_CONTEXT_GETREQUESTID:   ctx->regs[rd] = (i32)h->req->request_id;   return;

    /* Io.* -- write bytes to the response output. */
    case PV_HOOK_IO_WRITE:
        pixe_io_write(h, ctx->regs[rs1]);
        return;
    case PV_HOOK_IO_WRITEBYTE:
        if (h->vm.out_len < PV_MAX_OUT) h->vm.out[h->vm.out_len++] = (u8)(ctx->regs[rs1] & 0xFF);
        return;

    /* Span.* over handles produced above (read-only views + arena copies). */
    case PV_HOOK_SPAN_LEN: {
        i32 hd = ctx->regs[rs1];
        ctx->regs[rd] = (hd > 0 && hd < h->span_count) ? (i32)h->spans[hd].len : 0;
        return;
    }
    case PV_HOOK_SPAN_GET: {
        i32 hd = ctx->regs[rs1];
        i32 idx = ctx->regs[rs2];
        if (hd > 0 && hd < h->span_count && idx >= 0 && (u32)idx < h->spans[hd].len)
            ctx->regs[rd] = h->mem[h->spans[hd].ptr + (u32)idx];
        else
            ctx->regs[rd] = 0;
        return;
    }
    case PV_HOOK_SPAN_SLICE: {                       /* zero-copy sub-span VIEW */
        i32 hd = ctx->regs[rs1];
        i32 off = ctx->regs[rs2];
        struct pixe_vm_span s = (hd > 0 && hd < h->span_count)
                                    ? h->spans[hd] : (struct pixe_vm_span){0, 0};
        if (off < 0) off = 0;
        if ((u32)off > s.len) off = (i32)s.len;
        if (h->span_count >= PIXE_MAX_SPANS) { h->oom = 1; ctx->regs[rd] = 0; return; }
        int handle = h->span_count++;
        h->spans[handle].ptr = s.ptr + (u32)off;
        h->spans[handle].len = s.len - (u32)off;
        ctx->regs[rd] = handle;
        return;
    }
    case PV_HOOK_SPAN_MATERIALIZE: {                 /* copy to a fresh region */
        i32 hd = ctx->regs[rs1];
        struct pixe_vm_span s = (hd > 0 && hd < h->span_count)
                                    ? h->spans[hd] : (struct pixe_vm_span){0, 0};
        if (h->arena_top + s.len > PIXE_ARENA_SIZE || h->span_count >= PIXE_MAX_SPANS) {
            h->oom = 1; ctx->regs[rd] = 0; return;
        }
        u32 dst = h->arena_top;
        for (u32 i = 0; i < s.len; i++) h->mem[dst + i] = h->mem[s.ptr + i];
        h->arena_top += s.len;
        int handle = h->span_count++;
        h->spans[handle].ptr = dst;
        h->spans[handle].len = s.len;
        ctx->regs[rd] = handle;
        return;
    }

    /* everything else (Random.U32, Queue.*) keeps the default host behaviour. */
    default:
        pv_default_host(ctx, hook, rd, rs1, rs2, imm16);
        return;
    }
}

/* ---- lifecycle -------------------------------------------------------- */

void pixe_host_init(struct pixe_host *h, const struct pixe_request_context *req,
                    const u8 *req_bytes, u32 req_bytes_len)
{
    pv_init(&h->vm);
    h->vm.host = pixe_host_hook;
    h->vm.mem = h->mem;
    h->vm.mem_size = PIXE_ARENA_SIZE;
    h->arena_top = 0;
    h->span_count = 1;            /* spans[0] reserved/empty => first alloc is 1 */
    h->spans[0].ptr = 0;
    h->spans[0].len = 0;
    h->oom = 0;
    h->req = req;
    h->req_bytes = req_bytes;
    h->req_bytes_len = req_bytes_len;
}

long pixe_host_run(struct pixe_host *h, const u32 *program, int len)
{
    return pv_vm_run(&h->vm, program, len);
}

u32 pixe_host_seal(struct pixe_host *h, struct pixe_request_context *rc,
                   u8 *resp_buf, u32 resp_cap)
{
    rc->resp_status = h->vm.http_status;
    u32 n = (u32)h->vm.out_len;
    if (n > resp_cap) n = resp_cap;
    for (u32 i = 0; i < n; i++) resp_buf[i] = h->vm.out[i];
    rc->resp_body.off = 0;
    rc->resp_body.len = n;
    return n;
}

/* ---- selftest / report (proves the EL0 endpoint on hardware) ---------- */

/* The canned request mirrors pixe_request_selftest so the EL1->EL0 story is one
 * coherent example: EL1 builds this context, EL0 reads verb+body from it. */
static const u8 pixe_canned_req[] =
    "POST /api/sum?n=10 HTTP/1.1\r\n"
    "Host: pios.local\r\n"
    "Content-Type: text/plain\r\n"
    "Content-Length: 5\r\n"
    "\r\n"
    "hello";

/* Embedded echo endpoint (hand-assembled, byte-parity verifiable):
 *   Context.GetVerb -> R0 ; Io.Write(R0) ; Context.GetBody -> R1 ; Io.Write(R1)
 *   ; Net.Status(200) ; Flow.Return  =>  body "POSThello", status 200. */
static const u32 pixe_echo_endpoint[] = {
    0x000070E0u, /* NOOP host hook 0xE0  Context.GetVerb -> R0 */
    0x00007071u, /* NOOP host hook 0x71  Io.Write(R0)         */
    0x010070E9u, /* NOOP host hook 0xE9  Context.GetBody -> R1 */
    0x00107071u, /* NOOP host hook 0x71  Io.Write(R1)         */
    0x000080C8u, /* NOOP marker 0x80C8   Net.Status(200)      */
    0xC0000000u, /* RETURN               Flow.Return          */
};

static void ph_str(char *o, u32 *p, u32 max, const char *s)
{
    while (*s && *p + 1 < max) o[(*p)++] = *s++;
}
static void ph_u32(char *o, u32 *p, u32 max, u32 v)
{
    char t[12]; int n = 0;
    if (v == 0) t[n++] = '0';
    while (v) { t[n++] = (char)('0' + (v % 10)); v /= 10; }
    while (n > 0 && *p + 1 < max) o[(*p)++] = t[--n];
}
static void ph_i32(char *o, u32 *p, u32 max, i32 v)
{
    if (v < 0) { if (*p + 1 < max) o[(*p)++] = '-'; ph_u32(o, p, max, (u32)(-v)); }
    else ph_u32(o, p, max, (u32)v);
}
static void ph_hex(char *o, u32 *p, u32 max, u32 v)
{
    const char *hx = "0123456789abcdef";
    for (int sh = 28; sh >= 0; sh -= 4)
        if (*p + 1 < max) o[(*p)++] = hx[(v >> sh) & 0xF];
}
static void ph_hex8(char *o, u32 *p, u32 max, u8 b)
{
    const char *hx = "0123456789abcdef";
    if (*p + 1 < max) o[(*p)++] = hx[(b >> 4) & 0xF];
    if (*p + 1 < max) o[(*p)++] = hx[b & 0xF];
}
static void ph_ascii(char *o, u32 *p, u32 max, const u8 *b, u32 n)
{
    for (u32 i = 0; i < n && *p + 1 < max; i++)
        o[(*p)++] = (b[i] >= 32 && b[i] < 127) ? (char)b[i] : '.';
}

static u32 pixe_run_against_canned(char *out, u32 max, const u32 *program, int len)
{
    static struct pixe_host host;
    static struct pixe_request_context rc;
    static u8 respbuf[PV_MAX_OUT];
    u32 clen = (u32)sizeof(pixe_canned_req) - 1; /* drop the implicit NUL */

    u32 p = 0;
    i32 br = pixe_request_build(pixe_canned_req, clen, principal_current(), &rc);
    ph_str(out, &p, max, "pixe endpoint build=");
    ph_i32(out, &p, max, br);
    ph_str(out, &p, max, " req_len=");
    ph_u32(out, &p, max, clen);
    ph_str(out, &p, max, "\n");
    if (br != PIXE_REQ_OK) { ph_str(out, &p, max, "(framing not OK)\n"); return p; }

    ph_str(out, &p, max, "ctx method=");
    ph_u32(out, &p, max, rc.method_id);
    ph_str(out, &p, max, " user=");
    ph_u32(out, &p, max, rc.principal_id);
    ph_str(out, &p, max, " perms=0x");
    ph_hex(out, &p, max, rc.permissions);
    ph_str(out, &p, max, " req_id=");
    ph_u32(out, &p, max, rc.request_id);
    ph_str(out, &p, max, "\n");

    pixe_host_init(&host, &rc, pixe_canned_req, clen);
    long steps = pixe_host_run(&host, program, len);
    u32 rn = pixe_host_seal(&host, &rc, respbuf, sizeof(respbuf));

    ph_str(out, &p, max, "run words=");
    ph_u32(out, &p, max, (u32)len);
    ph_str(out, &p, max, " steps=");
    ph_u32(out, &p, max, (u32)steps);
    ph_str(out, &p, max, " status=");
    ph_i32(out, &p, max, host.vm.http_status);
    ph_str(out, &p, max, " oom=");
    ph_u32(out, &p, max, (u32)host.oom);
    ph_str(out, &p, max, "\n");

    ph_str(out, &p, max, "out(");
    ph_u32(out, &p, max, (u32)host.vm.out_len);
    ph_str(out, &p, max, ")='");
    ph_ascii(out, &p, max, host.vm.out, (u32)host.vm.out_len);
    ph_str(out, &p, max, "' hex=");
    for (int i = 0; i < host.vm.out_len; i++) ph_hex8(out, &p, max, host.vm.out[i]);
    ph_str(out, &p, max, "\n");

    ph_str(out, &p, max, "regs=");
    for (int i = 0; i < PV_NUM_REGS; i++) {
        if (i) ph_str(out, &p, max, ",");
        ph_i32(out, &p, max, host.vm.regs[i]);
    }
    ph_str(out, &p, max, "\n");

    ph_str(out, &p, max, "sealed resp_status=");
    ph_i32(out, &p, max, rc.resp_status);
    ph_str(out, &p, max, " resp_body off=");
    ph_u32(out, &p, max, rc.resp_body.off);
    ph_str(out, &p, max, " len=");
    ph_u32(out, &p, max, rc.resp_body.len);
    ph_str(out, &p, max, " '");
    ph_ascii(out, &p, max, respbuf, rn);
    ph_str(out, &p, max, "'\n");
    return p;
}

u32 pixe_host_selftest(char *out, u32 max)
{
    return pixe_run_against_canned(out, max, pixe_echo_endpoint,
                                   (int)(sizeof(pixe_echo_endpoint) / sizeof(pixe_echo_endpoint[0])));
}

u32 pixe_host_run_report(char *out, u32 max, const u32 *program, int len)
{
    return pixe_run_against_canned(out, max, program, len);
}
