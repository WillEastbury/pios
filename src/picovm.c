/* picovm.c -- portable C implementation of the PicoScript 16-opcode VM.
 *
 * Mirrors picoscript_vm.PicoVM._step exactly so the same bytecode yields the
 * same register file, output bytes and HTTP markers on host and on bare metal.
 */
#include "picovm.h"
#include "pico_hooks.h"
#include "picocompress.h"
#include "picobrotli.h"

#if defined(__ARM_FEATURE_DOTPROD) && defined(__aarch64__)
#include <arm_neon.h>        /* AArch64 NEON SDOT (Armv8.2 dotprod) */
#endif
#if defined(__ARM_FEATURE_SIMD32) && !defined(__aarch64__)
#include <arm_acle.h>        /* Cortex-M33 __smlad (DSP / SIMD32 extension) */
#endif

#define MASK32 0xFFFFFFFFu

/* Freestanding-friendly zero-fill (no <string.h> dependency for bare metal). */
static void pv_bzero(void *p, unsigned long len)
{
    unsigned char *b = (unsigned char *)p;
    for (unsigned long i = 0; i < len; i++) b[i] = 0;
}

static void pv_set_fault(pv_ctx *ctx, int code, int pc, int detail)
{
    /* Error.*: if a try/except handler is active (top of the handler stack
     * is truthy -- SetHandler(0) is a deliberate no-op push, matching
     * Python/JS's "0 = no handler" convention), redirect there instead of
     * halting -- exactly like Python's `except PicoFault` around _step().
     * Real VM faults and script-raised Error.Raise(code) share one
     * Error.Code() channel (see docs/EXCEPTION_ENGINE.md), same as Python/JS. */
    int32_t handler_pc = (ctx->err_sp > 0) ? ctx->err_stack[ctx->err_sp - 1] : 0;
    if (handler_pc) {
        ctx->err_code = code;
        ctx->err_detail = detail;
        ctx->err_resume_pc = pc + 1;
        /* Truncate the call stack back to its depth when this handler was
         * registered (Error.SetHandler) -- discards any return address left
         * by a subroutine called since then, so a later RETURN can't pop it
         * and resume skipped try-body code. See err_call_depth in picovm.h. */
        ctx->call_sp = ctx->err_call_depth[ctx->err_sp - 1];
        ctx->pending_jump = handler_pc;
        ctx->pending_jump_set = 1;
        return;
    }
    ctx->fault = code;
    ctx->fault_pc = pc;
    ctx->fault_detail = detail;
    ctx->halted = 1;
}

/* ---- card store: open addressing with linear probing ----------------- */

static int pv_card_slot(pv_ctx *ctx, int addr16, int create)
{
    unsigned h = (unsigned)(addr16 & 0xFFFF) & (PV_MAX_CARDS - 1);
    for (int i = 0; i < PV_MAX_CARDS; i++) {
        unsigned s = (h + (unsigned)i) & (PV_MAX_CARDS - 1);
        if (!ctx->card_used[s]) {
            if (!create) return -1;
            ctx->card_used[s] = 1;
            ctx->card_key[s] = (uint16_t)addr16;
            ctx->card_val[s] = 0;
            return (int)s;
        }
        if (ctx->card_key[s] == (uint16_t)addr16) return (int)s;
    }
    return -1;
}

int32_t pv_load(pv_ctx *ctx, int addr16)
{
    int s = pv_card_slot(ctx, addr16, 0);
    return s < 0 ? 0 : ctx->card_val[s];
}

void pv_save(pv_ctx *ctx, int addr16, int32_t val)
{
    int s = pv_card_slot(ctx, addr16, 1);
    if (s >= 0) ctx->card_val[s] = val;
}

static void pv_emit_word(pv_ctx *ctx, uint32_t v)
{
    if (ctx->out_len + 4 > PV_MAX_OUT) return;
    ctx->out[ctx->out_len++] = (uint8_t)((v >> 24) & 0xFF);
    ctx->out[ctx->out_len++] = (uint8_t)((v >> 16) & 0xFF);
    ctx->out[ctx->out_len++] = (uint8_t)((v >> 8) & 0xFF);
    ctx->out[ctx->out_len++] = (uint8_t)(v & 0xFF);
}

void pv_pipe(pv_ctx *ctx, int addr16, int32_t val)
{
    (void)addr16;
    /* PIPE emits the card value as 4 big-endian bytes to the output buffer. */
    if (ctx->out_len + 4 <= PV_MAX_OUT) {
        ctx->out[ctx->out_len++] = (uint8_t)((val >> 24) & 0xFF);
        ctx->out[ctx->out_len++] = (uint8_t)((val >> 16) & 0xFF);
        ctx->out[ctx->out_len++] = (uint8_t)((val >> 8) & 0xFF);
        ctx->out[ctx->out_len++] = (uint8_t)(val & 0xFF);
    }
}

void pv_net_status(pv_ctx *ctx, int code) { ctx->http_status = code & 0x0FFF; }
void pv_net_type(pv_ctx *ctx, const char *ct) { (void)ct; ctx->http_type = 0xA000; }
void pv_net_body(pv_ctx *ctx) { (void)ctx; }
void pv_net_header(pv_ctx *ctx) { (void)ctx; }
void pv_net_close(pv_ctx *ctx) { ctx->halted = 1; }
void pv_wait(pv_ctx *ctx) { ctx->waiting = 1; ctx->halted = 1; }
void pv_raise(pv_ctx *ctx, int channel) { (void)ctx; (void)channel; }
void pv_call(pv_ctx *ctx, const char *label) { (void)ctx; (void)label; }

int64_t pv_dsp(pv_ctx *ctx, int subop, int64_t a, int64_t b)
{
    (void)ctx;
    switch (subop) {
        case 0x4: return a < 0 ? 0 : a;          /* RELU */
        case 0x3: return a * b;                  /* SCALE */
        case 0x9: return a + b;                  /* VADD */
        default:  return 0;
    }
}

int pv_cond(pv_ctx *ctx, int mode) { (void)ctx; (void)mode; return 0; }

/* ---- Dot8: HW-accelerated signed int8 span dot product --------------- */

void pv_dot8_setlen(pv_ctx *ctx, int n) { ctx->dot_len = n; }

int32_t pv_dot8(pv_ctx *ctx, uint32_t wptr, uint32_t aptr)
{
    int n = ctx->dot_len;
    int32_t s = 0;
    int i = 0;
    if (!ctx->mem || ctx->mem_size <= 0 || n <= 0) return 0;
    {
        uint32_t sz = (uint32_t)ctx->mem_size;
        uint32_t wi = wptr % sz;
        uint32_t ai = aptr % sz;
        const int contiguous = ((uint64_t)wi + (uint32_t)n <= (uint64_t)sz) &&
                               ((uint64_t)ai + (uint32_t)n <= (uint64_t)sz);
        const int8_t *w = (const int8_t *)(ctx->mem + wi);
        const int8_t *a = (const int8_t *)(ctx->mem + ai);
        if (!contiguous) {
            for (; i < n; i++) {
                int8_t x = (int8_t)ctx->mem[wi];
                int8_t y = (int8_t)ctx->mem[ai];
                s += (int32_t)x * (int32_t)y;
                if (++wi == sz) wi = 0;
                if (++ai == sz) ai = 0;
            }
            return s;
        }
#if defined(__ARM_FEATURE_DOTPROD) && defined(__aarch64__)
        int32x4_t acc = vdupq_n_s32(0);
        for (; i + 16 <= n; i += 16)
            acc = vdotq_s32(acc, vld1q_s8(w + i), vld1q_s8(a + i));  /* 16 int8 MACs */
        s = vaddvq_s32(acc);
#elif defined(__ARM_FEATURE_SIMD32) && !defined(__aarch64__)
        for (; i + 2 <= n; i += 2) {
            uint32_t wp = (uint32_t)(uint16_t)(int16_t)w[i] | ((uint32_t)(uint16_t)(int16_t)w[i + 1] << 16);
            uint32_t ap = (uint32_t)(uint16_t)(int16_t)a[i] | ((uint32_t)(uint16_t)(int16_t)a[i + 1] << 16);
            s = (int32_t)__smlad(wp, ap, (uint32_t)s);   /* dual 16x16 MAC */
        }
#endif
        for (; i < n; i++) s += (int32_t)w[i] * (int32_t)a[i];
    }
    return s;
}

/* ---- span table + bump-arena helpers (mirror picoscript_vm.PicoVM) ----
 * A span handle is a 1-based index; handle 0 is the null/empty span. Result
 * bytes are written at arena_top and a new span registered, exactly as the
 * Python/JS interpreters' _new_span_bytes does. */
static uint32_t pv_span_p(pv_ctx *ctx, int h)
{
    return (h > 0 && h < ctx->span_count) ? ctx->span_ptr[h] : 0;
}
static int32_t pv_span_n(pv_ctx *ctx, int h)
{
    return (h > 0 && h < ctx->span_count) ? ctx->span_len[h] : 0;
}
static uint8_t pv_arena_get(pv_ctx *ctx, uint32_t a)
{
    return (ctx->mem && a < (uint32_t)ctx->mem_size) ? ctx->mem[a] : 0;
}
/* App-installable storage backend for Storage and Search card-pack hooks.
 * NULL by default; a native deployment sets this to compile in its own
 * pack/card store. Return non-zero if the hook was handled. */
pv_storage_fn pv_storage_hook = 0;
static int pv_span_make(pv_ctx *ctx, uint32_t ptr, int32_t len)
{
    if (len < 0) len = 0;
    if (ctx->span_count >= PV_MAX_SPANS) return 0;
    ctx->span_ptr[ctx->span_count] = ptr;
    ctx->span_len[ctx->span_count] = len;
    return ctx->span_count++;
}
static void pv_arena_put(pv_ctx *ctx, uint32_t *k, uint8_t b)
{
    uint32_t a = ctx->arena_top + *k;
    if (ctx->mem && a < (uint32_t)ctx->mem_size) ctx->mem[a] = b;
    (*k)++;
}
static int pv_arena_finish(pv_ctx *ctx, uint32_t k)
{
    if (ctx->no_alloc) { ctx->fault = PV_FAULT_ALLOC; ctx->halted = 1; return 0; }  /* INV-5 */
    /* Ceiling: never let the bump pointer run past the arena, so later span
       reads can't be handed an out-of-bounds [ptr,ptr+len) range. */
    if (ctx->mem_size > 0 && (uint64_t)ctx->arena_top + k > (uint64_t)ctx->mem_size) {
        ctx->fault = PV_FAULT_ALLOC; ctx->halted = 1; return 0;
    }
    int h = pv_span_make(ctx, ctx->arena_top, (int32_t)k);
    ctx->arena_top += k;
    return h;
}
static void pv_arena_puts(pv_ctx *ctx, uint32_t *k, const char *s)
{
    while (*s) pv_arena_put(ctx, k, (uint8_t)*s++);
}
/* Copy raw C bytes into the bump arena and return a span handle (0 on empty).
 * Used to surface the HTTP request context (ctx->req_*) to Req.* hooks. */
static int pv_span_from_cbytes(pv_ctx *ctx, const char *s, int n)
{
    uint32_t k = 0;
    if (!s || n <= 0) return 0;
    for (int i = 0; i < n; i++) pv_arena_put(ctx, &k, (uint8_t)s[i]);
    return pv_arena_finish(ctx, k);
}
/* Case-insensitive lookup of a header value in ctx->req_headers (name is a C
 * string of length nn). Returns a span of the trimmed value, or 0 if absent.
 * Used by Req.Header and Req.Principal (the trusted X-Forge-Principal header
 * the auth proxy/kernel injects). */
static int pv_req_header_value(pv_ctx *ctx, const char *name, int nn)
{
    const char *hdr = ctx->req_headers;
    int hl = ctx->req_headers_len;
    if (!hdr || hl <= 0 || nn <= 0) return 0;
    int i = 0;
    while (i < hl) {
        int ls = i;
        while (i < hl && hdr[i] != '\n') i++;
        int le = i;
        if (le > ls && hdr[le - 1] == '\r') le--;
        int j = 0, match = 1;
        while (j < nn) {
            if (ls + j >= le) { match = 0; break; }
            uint8_t a = (uint8_t)hdr[ls + j], b = (uint8_t)name[j];
            if (a >= 'A' && a <= 'Z') a = (uint8_t)(a - 'A' + 'a');
            if (b >= 'A' && b <= 'Z') b = (uint8_t)(b - 'A' + 'a');
            if (a != b) { match = 0; break; }
            j++;
        }
        if (match && ls + nn < le && hdr[ls + nn] == ':') {
            int vs = ls + nn + 1;
            while (vs < le && (hdr[vs] == ' ' || hdr[vs] == '\t')) vs++;
            return pv_span_from_cbytes(ctx, hdr + vs, le - vs);
        }
        i++;
    }
    return 0;
}
static int pv_arena_match(pv_ctx *ctx, uint32_t at, int32_t avail, const char *s)
{
    int32_t n = 0;
    while (s[n]) n++;
    if (avail < n) return 0;
    for (int32_t i = 0; i < n; i++)
        if (pv_arena_get(ctx, at + (uint32_t)i) != (uint8_t)s[i]) return 0;
    return 1;
}

/* ---- Html.* helpers: a real, pure, deterministic DOM node table (no host
 * state needed -- mirrors picoscript_vm.py's _htmllib exactly). Fixed-size
 * (PV_MAX_HTML_NODES/PV_HTML_MAX_ATTRS/PV_HTML_MAX_CHILDREN), consistent
 * with this embedded runtime's other handle tables (Map/Descriptor/Lease/
 * Fifo/Log) -- a bounded, deterministic difference from Python/JS's
 * unbounded dict-backed version, not a behavioral divergence at any
 * realistic scale (a genuine HTML document over ~64 nodes/16 children-per-
 * node/8 attrs-per-node would need a raised constant, same tradeoff already
 * accepted for every other table here). ------------------------------- */
static int pv_span_eq(pv_ctx *ctx, int ha, int hb)
{
    int32_t la = pv_span_n(ctx, ha), lb = pv_span_n(ctx, hb);
    if (la != lb) return 0;
    uint32_t pa = pv_span_p(ctx, ha), pb = pv_span_p(ctx, hb);
    for (int32_t i = 0; i < la; i++)
        if (pv_arena_get(ctx, pa + (uint32_t)i) != pv_arena_get(ctx, pb + (uint32_t)i)) return 0;
    return 1;
}
static int pv_span_eq_lit(pv_ctx *ctx, int h, const char *s)
{
    int32_t n = 0; while (s[n]) n++;
    if (pv_span_n(ctx, h) != n) return 0;
    uint32_t p = pv_span_p(ctx, h);
    for (int32_t i = 0; i < n; i++)
        if (pv_arena_get(ctx, p + (uint32_t)i) != (uint8_t)s[i]) return 0;
    return 1;
}
static int pv_html_valid(pv_ctx *ctx, int h)
{
    return h > 0 && h < ctx->html_count && ctx->html_used[h];
}
static int pv_html_new_node(pv_ctx *ctx, int32_t tag_span)
{
    if (ctx->html_count >= PV_MAX_HTML_NODES) return 0;   /* table full: null handle (bounded, see above) */
    int h = ctx->html_count++;
    ctx->html_tag[h] = tag_span;
    ctx->html_attr_count[h] = 0;
    ctx->html_child_count[h] = 0;
    ctx->html_used[h] = 1;
    return h;
}
static int pv_html_attr_get(pv_ctx *ctx, int h, int key_span, int *out_val)
{
    if (!pv_html_valid(ctx, h)) return 0;
    for (int i = 0; i < ctx->html_attr_count[h]; i++)
        if (pv_span_eq(ctx, ctx->html_attr_key[h][i], key_span)) { *out_val = ctx->html_attr_val[h][i]; return 1; }
    return 0;
}
static int pv_html_attr_get_lit(pv_ctx *ctx, int h, const char *key, int *out_val)
{
    if (!pv_html_valid(ctx, h)) return 0;
    for (int i = 0; i < ctx->html_attr_count[h]; i++)
        if (pv_span_eq_lit(ctx, ctx->html_attr_key[h][i], key)) { *out_val = ctx->html_attr_val[h][i]; return 1; }
    return 0;
}
static void pv_html_attr_set(pv_ctx *ctx, int h, int key_span, int val_span)
{
    if (!pv_html_valid(ctx, h)) return;
    for (int i = 0; i < ctx->html_attr_count[h]; i++) {
        if (pv_span_eq(ctx, ctx->html_attr_key[h][i], key_span)) { ctx->html_attr_val[h][i] = val_span; return; }
    }
    if (ctx->html_attr_count[h] < PV_HTML_MAX_ATTRS) {
        int i = ctx->html_attr_count[h]++;
        ctx->html_attr_key[h][i] = key_span;
        ctx->html_attr_val[h][i] = val_span;
    }
}
/* full=1: escape &<>"' (text content, matches Html.Encode). full=0: escape
 * &" only (attribute-value quoting -- matches the interpreters' Serialize). */
static void pv_html_put_escaped(pv_ctx *ctx, uint32_t *k, uint32_t p, int32_t l, int full)
{
    for (int32_t i = 0; i < l; i++) {
        uint8_t c = pv_arena_get(ctx, p + (uint32_t)i);
        if (c == '&') pv_arena_puts(ctx, k, "&amp;");
        else if (full && c == '<') pv_arena_puts(ctx, k, "&lt;");
        else if (full && c == '>') pv_arena_puts(ctx, k, "&gt;");
        else if (c == '"') pv_arena_puts(ctx, k, "&quot;");
        else if (full && c == 0x27) pv_arena_puts(ctx, k, "&#39;");
        else pv_arena_put(ctx, k, c);
    }
}
static void pv_html_serialize_rec(pv_ctx *ctx, int h, int depth, uint32_t *k)
{
    if (!pv_html_valid(ctx, h) || depth >= PV_HTML_MAX_DEPTH) return;
    int text_val;
    if (pv_html_attr_get_lit(ctx, h, "#text", &text_val)) {
        pv_html_put_escaped(ctx, k, pv_span_p(ctx, text_val), pv_span_n(ctx, text_val), 1);
        return;
    }
    int32_t tag_h = ctx->html_tag[h];
    int32_t tl = pv_span_n(ctx, tag_h);
    if (tl == 0) {                                        /* transparent fragment wrapper */
        for (int i = 0; i < ctx->html_child_count[h]; i++)
            pv_html_serialize_rec(ctx, ctx->html_child[h][i], depth + 1, k);
        return;
    }
    uint32_t tp = pv_span_p(ctx, tag_h);
    pv_arena_put(ctx, k, '<');
    for (int32_t i = 0; i < tl; i++) pv_arena_put(ctx, k, pv_arena_get(ctx, tp + (uint32_t)i));
    for (int a = 0; a < ctx->html_attr_count[h]; a++) {
        int kh = ctx->html_attr_key[h][a], vh = ctx->html_attr_val[h][a];
        uint32_t kp = pv_span_p(ctx, kh); int32_t kl = pv_span_n(ctx, kh);
        pv_arena_put(ctx, k, ' ');
        for (int32_t i = 0; i < kl; i++) pv_arena_put(ctx, k, pv_arena_get(ctx, kp + (uint32_t)i));
        pv_arena_puts(ctx, k, "=\"");
        pv_html_put_escaped(ctx, k, pv_span_p(ctx, vh), pv_span_n(ctx, vh), 0);
        pv_arena_put(ctx, k, '"');
    }
    pv_arena_put(ctx, k, '>');
    for (int i = 0; i < ctx->html_child_count[h]; i++)
        pv_html_serialize_rec(ctx, ctx->html_child[h][i], depth + 1, k);
    pv_arena_puts(ctx, k, "</");
    for (int32_t i = 0; i < tl; i++) pv_arena_put(ctx, k, pv_arena_get(ctx, tp + (uint32_t)i));
    pv_arena_put(ctx, k, '>');
}
static int pv_html_matches(pv_ctx *ctx, int h, uint32_t sel_p, int32_t sel_l)
{
    if (sel_l <= 0) return 0;
    uint8_t c0 = pv_arena_get(ctx, sel_p);
    if (c0 == '#') {
        int idv;
        if (!pv_html_attr_get_lit(ctx, h, "id", &idv)) return 0;
        int32_t il = pv_span_n(ctx, idv);
        if (il != sel_l - 1) return 0;
        uint32_t ip = pv_span_p(ctx, idv);
        for (int32_t i = 0; i < il; i++)
            if (pv_arena_get(ctx, ip + (uint32_t)i) != pv_arena_get(ctx, sel_p + 1 + (uint32_t)i)) return 0;
        return 1;
    }
    if (c0 == '.') {
        int clv;
        if (!pv_html_attr_get_lit(ctx, h, "class", &clv)) return 0;
        uint32_t cp = pv_span_p(ctx, clv); int32_t cl = pv_span_n(ctx, clv);
        int32_t want_l = sel_l - 1; uint32_t want_p = sel_p + 1;
        int32_t i = 0;
        while (i < cl) {
            while (i < cl && pv_arena_get(ctx, cp + (uint32_t)i) <= ' ') i++;
            int32_t start = i;
            while (i < cl && pv_arena_get(ctx, cp + (uint32_t)i) > ' ') i++;
            int32_t tok_l = i - start;
            if (tok_l == want_l) {
                int ok = 1;
                for (int32_t j = 0; j < tok_l; j++)
                    if (pv_arena_get(ctx, cp + (uint32_t)(start + j)) != pv_arena_get(ctx, want_p + (uint32_t)j)) { ok = 0; break; }
                if (ok) return 1;
            }
        }
        return 0;
    }
    int32_t tag_h = ctx->html_tag[h];
    int32_t tl = pv_span_n(ctx, tag_h);
    if (tl != sel_l) return 0;
    uint32_t tp = pv_span_p(ctx, tag_h);
    for (int32_t i = 0; i < tl; i++)
        if (pv_arena_get(ctx, tp + (uint32_t)i) != pv_arena_get(ctx, sel_p + (uint32_t)i)) return 0;
    return 1;
}
static int pv_html_query_rec(pv_ctx *ctx, int h, uint32_t sel_p, int32_t sel_l, int depth)
{
    if (!pv_html_valid(ctx, h) || depth >= PV_HTML_MAX_DEPTH) return 0;
    if (pv_html_matches(ctx, h, sel_p, sel_l)) return h;
    for (int i = 0; i < ctx->html_child_count[h]; i++) {
        int m = pv_html_query_rec(ctx, ctx->html_child[h][i], sel_p, sel_l, depth + 1);
        if (m) return m;
    }
    return 0;
}
static void pv_html_parse_attrs(pv_ctx *ctx, int node_h, uint32_t p, int32_t l)
{
    int32_t i = 0;
    while (i < l) {
        while (i < l && pv_arena_get(ctx, p + (uint32_t)i) <= ' ') i++;
        int32_t start = i;
        while (i < l) {
            uint8_t c = pv_arena_get(ctx, p + (uint32_t)i);
            if (c <= ' ' || c == '=') break;
            i++;
        }
        if (i == start) break;
        int32_t key_l = i - start;
        uint32_t key_p = p + (uint32_t)start;
        while (i < l && pv_arena_get(ctx, p + (uint32_t)i) <= ' ') i++;
        uint32_t kk = 0;
        for (int32_t j = 0; j < key_l; j++) pv_arena_put(ctx, &kk, pv_arena_get(ctx, key_p + (uint32_t)j));
        int key_span = pv_arena_finish(ctx, kk);
        int val_span;
        if (i < l && pv_arena_get(ctx, p + (uint32_t)i) == '=') {
            i++;
            while (i < l && pv_arena_get(ctx, p + (uint32_t)i) <= ' ') i++;
            uint8_t q = (i < l) ? pv_arena_get(ctx, p + (uint32_t)i) : 0;
            uint32_t vk = 0;
            if (q == '"' || q == 0x27) {
                i++;
                int32_t vs = i;
                while (i < l && pv_arena_get(ctx, p + (uint32_t)i) != q) i++;
                for (int32_t j = vs; j < i; j++) pv_arena_put(ctx, &vk, pv_arena_get(ctx, p + (uint32_t)j));
                if (i < l) i++;                            /* skip closing quote */
            } else {
                int32_t vs = i;
                while (i < l && pv_arena_get(ctx, p + (uint32_t)i) > ' ') i++;
                for (int32_t j = vs; j < i; j++) pv_arena_put(ctx, &vk, pv_arena_get(ctx, p + (uint32_t)j));
            }
            val_span = pv_arena_finish(ctx, vk);
        } else {
            val_span = pv_arena_finish(ctx, 0);
        }
        pv_html_attr_set(ctx, node_h, key_span, val_span);
    }
}
static int pv_html_is_void(pv_ctx *ctx, uint32_t p, int32_t l)
{
    static const char *VOID_TAGS[] = {"br", "img", "hr", "input", "meta", "link", "area",
                                       "base", "col", "embed", "source", "track", "wbr", 0};
    for (int vi = 0; VOID_TAGS[vi]; vi++) {
        const char *s = VOID_TAGS[vi];
        int32_t n = 0; while (s[n]) n++;
        if (n != l) continue;
        int ok = 1;
        for (int32_t i = 0; i < l; i++) {
            uint8_t c = pv_arena_get(ctx, p + (uint32_t)i);
            if (c >= 'A' && c <= 'Z') c = (uint8_t)(c - 'A' + 'a');
            if (c != (uint8_t)s[i]) { ok = 0; break; }
        }
        if (ok) return 1;
    }
    return 0;
}
static int pv_html_find_char(pv_ctx *ctx, uint32_t p, int32_t l, int32_t from, uint8_t ch)
{
    for (int32_t i = from; i < l; i++)
        if (pv_arena_get(ctx, p + (uint32_t)i) == ch) return i;
    return -1;
}
/* Minimal, permissive HTML parser (not full HTML5 conformance -- see
 * docs/NAMESPACE_STATUS.md), mirroring picoscript_vm.py's _html_parse
 * exactly: tokenizes `<tag k="v" k2='v2'>`, `</tag>` (closes the innermost
 * open element regardless of name match -- permissive), self-closing
 * `<tag/>`, and a fixed void-element list. Everything else is a text run.
 * Always returns a synthetic fragment-root handle (empty tag, no "#text"
 * attr) whose children are the top-level parsed nodes. */
static int pv_html_parse(pv_ctx *ctx, uint32_t p, int32_t l)
{
    int empty_tag = pv_arena_finish(ctx, 0);
    int root = pv_html_new_node(ctx, empty_tag);
    int stack[PV_HTML_MAX_DEPTH];
    int sp = 0; stack[sp++] = root;
    int32_t i = 0;
    while (i < l) {
        int32_t lt = pv_html_find_char(ctx, p, l, i, '<');
        if (lt < 0) {
            if (i < l && sp < PV_HTML_MAX_DEPTH) {
                uint32_t k = 0;
                for (int32_t j = i; j < l; j++) pv_arena_put(ctx, &k, pv_arena_get(ctx, p + (uint32_t)j));
                int txt_span = pv_arena_finish(ctx, k);
                int txt = pv_html_new_node(ctx, empty_tag);
                pv_html_attr_set(ctx, txt, pv_span_from_cbytes(ctx, "#text", 5), txt_span);
                int par = stack[sp - 1];
                if (ctx->html_child_count[par] < PV_HTML_MAX_CHILDREN)
                    ctx->html_child[par][ctx->html_child_count[par]++] = txt;
            }
            break;
        }
        if (lt > i && sp < PV_HTML_MAX_DEPTH) {
            uint32_t k = 0;
            for (int32_t j = i; j < lt; j++) pv_arena_put(ctx, &k, pv_arena_get(ctx, p + (uint32_t)j));
            int txt_span = pv_arena_finish(ctx, k);
            int txt = pv_html_new_node(ctx, empty_tag);
            pv_html_attr_set(ctx, txt, pv_span_from_cbytes(ctx, "#text", 5), txt_span);
            int par = stack[sp - 1];
            if (ctx->html_child_count[par] < PV_HTML_MAX_CHILDREN)
                ctx->html_child[par][ctx->html_child_count[par]++] = txt;
        }
        int32_t gt = pv_html_find_char(ctx, p, l, lt + 1, '>');
        if (gt < 0) break;                                /* unterminated tag: stop (permissive) */
        int32_t ts = lt + 1, te = gt;
        i = gt + 1;
        if (ts < te && pv_arena_get(ctx, p + (uint32_t)ts) == '/') {   /* closing tag */
            if (sp > 1) sp--;
            continue;
        }
        int self_close = (te > ts && pv_arena_get(ctx, p + (uint32_t)(te - 1)) == '/');
        if (self_close) te--;
        int32_t ns = ts;
        while (ns < te && pv_arena_get(ctx, p + (uint32_t)ns) > ' ') ns++;
        int32_t name_l = ns - ts;
        if (name_l <= 0) continue;
        uint32_t name_p = p + (uint32_t)ts;
        uint32_t nk = 0;
        for (int32_t j = 0; j < name_l; j++) pv_arena_put(ctx, &nk, pv_arena_get(ctx, name_p + (uint32_t)j));
        int tag_span = pv_arena_finish(ctx, nk);
        int elem = pv_html_new_node(ctx, tag_span);
        if (sp < PV_HTML_MAX_DEPTH) {
            int par = stack[sp - 1];
            if (ctx->html_child_count[par] < PV_HTML_MAX_CHILDREN)
                ctx->html_child[par][ctx->html_child_count[par]++] = elem;
        }
        if (ns < te) pv_html_parse_attrs(ctx, elem, p + (uint32_t)ns, te - ns);
        int is_void = pv_html_is_void(ctx, pv_span_p(ctx, tag_span), pv_span_n(ctx, tag_span));
        if (!self_close && !is_void && sp < PV_HTML_MAX_DEPTH) stack[sp++] = elem;
    }
    return root;
}

/* ---- Http.* helpers (pure string parsing, byte-exact with the interpreters) - */
static int pv_ishex(uint8_t c)
{
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}
static uint8_t pv_hexv(uint8_t c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return (uint8_t)(c - 'A' + 10);
}
static uint8_t pv_hexd(uint32_t d)
{
    return (uint8_t)(d < 10 ? '0' + d : 'a' + (d - 10));
}
/* URL-decode arena bytes [from,to) into the result span (k = running length). */
static void pv_urldecode_into(pv_ctx *ctx, uint32_t *k, uint32_t p, int32_t from, int32_t to)
{
    int32_t i = from;
    while (i < to) {
        uint8_t c = pv_arena_get(ctx, p + (uint32_t)i);
        if (c == '+') { pv_arena_put(ctx, k, ' '); i++; }
        else if (c == '%' && i + 2 < to &&
                 pv_ishex(pv_arena_get(ctx, p + (uint32_t)(i + 1))) &&
                 pv_ishex(pv_arena_get(ctx, p + (uint32_t)(i + 2)))) {
            uint8_t hi = pv_hexv(pv_arena_get(ctx, p + (uint32_t)(i + 1)));
            uint8_t lo = pv_hexv(pv_arena_get(ctx, p + (uint32_t)(i + 2)));
            pv_arena_put(ctx, k, (uint8_t)((hi << 4) | lo)); i += 3;
        } else { pv_arena_put(ctx, k, c); i++; }
    }
}
/* JSON-escape arena bytes [from,to) into the result span. */
static void pv_jsonesc_into(pv_ctx *ctx, uint32_t *k, uint32_t p, int32_t from, int32_t to)
{
    for (int32_t i = from; i < to; i++) {
        uint8_t c = pv_arena_get(ctx, p + (uint32_t)i);
        if (c == '"') { pv_arena_put(ctx, k, '\\'); pv_arena_put(ctx, k, '"'); }
        else if (c == '\\') { pv_arena_put(ctx, k, '\\'); pv_arena_put(ctx, k, '\\'); }
        else if (c == '\n') { pv_arena_put(ctx, k, '\\'); pv_arena_put(ctx, k, 'n'); }
        else if (c == '\r') { pv_arena_put(ctx, k, '\\'); pv_arena_put(ctx, k, 'r'); }
        else if (c == '\t') { pv_arena_put(ctx, k, '\\'); pv_arena_put(ctx, k, 't'); }
        else if (c < 0x20) {
            pv_arena_puts(ctx, k, "\\u00");
            pv_arena_put(ctx, k, pv_hexd((c >> 4) & 0xF));
            pv_arena_put(ctx, k, pv_hexd(c & 0xF));
        } else pv_arena_put(ctx, k, c);
    }
}

/* Recursive JSON -> dotted-path key=value model (the Template {{#each}} model). */
typedef struct { pv_ctx *ctx; uint32_t p; int32_t n; int32_t pos; uint32_t k; } pv_pjs;

static uint8_t pjs_g(pv_pjs *s, int32_t i) { return pv_arena_get(s->ctx, s->p + (uint32_t)i); }
static void pjs_skipws(pv_pjs *s)
{
    while (s->pos < s->n) {
        uint8_t c = pjs_g(s, s->pos);
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') s->pos++; else break;
    }
}
static int32_t pjs_string(pv_pjs *s, uint8_t *buf, int32_t cap)
{
    int32_t bl = 0;
    s->pos++;   /* opening quote */
    while (s->pos < s->n) {
        uint8_t c = pjs_g(s, s->pos); s->pos++;
        if (c == '"') break;
        if (c == '\\' && s->pos < s->n) {
            uint8_t e = pjs_g(s, s->pos); s->pos++;
            if (e == 'n') { if (bl < cap) buf[bl++] = 0x0a; }
            else if (e == 't') { if (bl < cap) buf[bl++] = 0x09; }
            else if (e == 'r') { if (bl < cap) buf[bl++] = 0x0d; }
            else if (e == 'b') { if (bl < cap) buf[bl++] = 0x08; }
            else if (e == 'f') { if (bl < cap) buf[bl++] = 0x0c; }
            else if (e == 'u' && s->pos + 4 <= s->n &&
                     pv_ishex(pjs_g(s, s->pos)) && pv_ishex(pjs_g(s, s->pos + 1)) &&
                     pv_ishex(pjs_g(s, s->pos + 2)) && pv_ishex(pjs_g(s, s->pos + 3))) {
                uint32_t cp = ((uint32_t)pv_hexv(pjs_g(s, s->pos)) << 12) |
                              ((uint32_t)pv_hexv(pjs_g(s, s->pos + 1)) << 8) |
                              ((uint32_t)pv_hexv(pjs_g(s, s->pos + 2)) << 4) |
                              (uint32_t)pv_hexv(pjs_g(s, s->pos + 3));
                s->pos += 4;
                if (cp < 0x80) { if (bl < cap) buf[bl++] = (uint8_t)cp; }
                else if (cp < 0x800) {
                    if (bl < cap) buf[bl++] = (uint8_t)(0xC0 | (cp >> 6));
                    if (bl < cap) buf[bl++] = (uint8_t)(0x80 | (cp & 0x3F));
                } else {
                    if (bl < cap) buf[bl++] = (uint8_t)(0xE0 | (cp >> 12));
                    if (bl < cap) buf[bl++] = (uint8_t)(0x80 | ((cp >> 6) & 0x3F));
                    if (bl < cap) buf[bl++] = (uint8_t)(0x80 | (cp & 0x3F));
                }
            } else { if (bl < cap) buf[bl++] = e; }
        } else { if (bl < cap) buf[bl++] = c; }
    }
    return bl;
}
static int32_t pjs_dec(int32_t v, uint8_t *buf)
{
    uint8_t tmp[12]; int t = 0;
    uint32_t u = (uint32_t)v;
    if (u == 0) tmp[t++] = '0';
    while (u) { tmp[t++] = (uint8_t)('0' + (u % 10u)); u /= 10u; }
    int32_t bl = 0;
    while (t > 0) buf[bl++] = tmp[--t];
    return bl;
}
static int32_t pjs_childkey(uint8_t *prefix, int32_t plen, uint8_t *key, int32_t klen, uint8_t *out)
{
    int32_t o = 0;
    if (plen > 0) {
        for (int32_t i = 0; i < plen && o < 255; i++) out[o++] = prefix[i];
        if (o < 255) out[o++] = '.';
    }
    for (int32_t i = 0; i < klen && o < 255; i++) out[o++] = key[i];
    return o;
}
static void pjs_leaf(pv_pjs *s, uint8_t *prefix, int32_t plen, int32_t vstart, int32_t vend, uint8_t *vbuf, int32_t vblen)
{
    for (int32_t i = 0; i < plen; i++) pv_arena_put(s->ctx, &s->k, prefix[i]);
    pv_arena_put(s->ctx, &s->k, '=');
    if (vbuf) { for (int32_t i = 0; i < vblen; i++) pv_arena_put(s->ctx, &s->k, vbuf[i]); }
    else { for (int32_t i = vstart; i < vend; i++) pv_arena_put(s->ctx, &s->k, pjs_g(s, i)); }
    pv_arena_put(s->ctx, &s->k, '\n');
}
static void pjs_emit(pv_pjs *s, uint8_t *prefix, int32_t plen, int depth)
{
    if (depth > 64) return;
    pjs_skipws(s);
    if (s->pos >= s->n) return;
    uint8_t c = pjs_g(s, s->pos);
    if (c == '{') {
        s->pos++; pjs_skipws(s);
        if (s->pos < s->n && pjs_g(s, s->pos) == '}') { s->pos++; return; }
        while (s->pos < s->n) {
            pjs_skipws(s);
            if (s->pos >= s->n || pjs_g(s, s->pos) != '"') break;
            uint8_t key[256]; int32_t klen = pjs_string(s, key, 256);
            pjs_skipws(s);
            if (s->pos < s->n && pjs_g(s, s->pos) == ':') s->pos++;
            uint8_t np[256]; int32_t npl = pjs_childkey(prefix, plen, key, klen, np);
            pjs_emit(s, np, npl, depth + 1);
            pjs_skipws(s);
            if (s->pos < s->n && pjs_g(s, s->pos) == ',') { s->pos++; continue; }
            if (s->pos < s->n && pjs_g(s, s->pos) == '}') s->pos++;
            break;
        }
    } else if (c == '[') {
        s->pos++; pjs_skipws(s);
        if (s->pos < s->n && pjs_g(s, s->pos) == ']') { s->pos++; return; }
        int32_t idx = 0;
        while (s->pos < s->n) {
            uint8_t ib[12]; int32_t ibl = pjs_dec(idx, ib);
            uint8_t np[256]; int32_t npl = pjs_childkey(prefix, plen, ib, ibl, np);
            pjs_emit(s, np, npl, depth + 1);
            idx++;
            pjs_skipws(s);
            if (s->pos < s->n && pjs_g(s, s->pos) == ',') { s->pos++; continue; }
            if (s->pos < s->n && pjs_g(s, s->pos) == ']') s->pos++;
            break;
        }
    } else if (c == '"') {
        uint8_t val[1024]; int32_t vl = pjs_string(s, val, 1024);
        pjs_leaf(s, prefix, plen, 0, 0, val, vl);
    } else {
        int32_t start = s->pos;
        while (s->pos < s->n) {
            uint8_t cc = pjs_g(s, s->pos);
            if (cc == ',' || cc == '}' || cc == ']' || cc == ' ' || cc == '\t' || cc == '\n' || cc == '\r') break;
            s->pos++;
        }
        pjs_leaf(s, prefix, plen, start, s->pos, 0, 0);
    }
}

/* ---- Crypto.Sha256: scalar FIPS-180-4 (canonical, so == hashlib == JS) ---- */
static const uint32_t PV_SHA256_K[64] = {
    0x428a2f98u,0x71374491u,0xb5c0fbcfu,0xe9b5dba5u,0x3956c25bu,0x59f111f1u,0x923f82a4u,0xab1c5ed5u,
    0xd807aa98u,0x12835b01u,0x243185beu,0x550c7dc3u,0x72be5d74u,0x80deb1feu,0x9bdc06a7u,0xc19bf174u,
    0xe49b69c1u,0xefbe4786u,0x0fc19dc6u,0x240ca1ccu,0x2de92c6fu,0x4a7484aau,0x5cb0a9dcu,0x76f988dau,
    0x983e5152u,0xa831c66du,0xb00327c8u,0xbf597fc7u,0xc6e00bf3u,0xd5a79147u,0x06ca6351u,0x14292967u,
    0x27b70a85u,0x2e1b2138u,0x4d2c6dfcu,0x53380d13u,0x650a7354u,0x766a0abbu,0x81c2c92eu,0x92722c85u,
    0xa2bfe8a1u,0xa81a664bu,0xc24b8b70u,0xc76c51a3u,0xd192e819u,0xd6990624u,0xf40e3585u,0x106aa070u,
    0x19a4c116u,0x1e376c08u,0x2748774cu,0x34b0bcb5u,0x391c0cb3u,0x4ed8aa4au,0x5b9cca4fu,0x682e6ff3u,
    0x748f82eeu,0x78a5636fu,0x84c87814u,0x8cc70208u,0x90befffau,0xa4506cebu,0xbef9a3f7u,0xc67178f2u
};
static uint32_t pv_rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
static void pv_sha256_block(uint32_t H[8], const uint8_t block[64])
{
    uint32_t w[64];
    for (int t = 0; t < 16; t++)
        w[t] = ((uint32_t)block[t * 4] << 24) | ((uint32_t)block[t * 4 + 1] << 16) |
               ((uint32_t)block[t * 4 + 2] << 8) | (uint32_t)block[t * 4 + 3];
    for (int t = 16; t < 64; t++) {
        uint32_t s0 = pv_rotr(w[t - 15], 7) ^ pv_rotr(w[t - 15], 18) ^ (w[t - 15] >> 3);
        uint32_t s1 = pv_rotr(w[t - 2], 17) ^ pv_rotr(w[t - 2], 19) ^ (w[t - 2] >> 10);
        w[t] = w[t - 16] + s0 + w[t - 7] + s1;
    }
    uint32_t a = H[0], b = H[1], c = H[2], d = H[3], e = H[4], f = H[5], g = H[6], h = H[7];
    for (int t = 0; t < 64; t++) {
        uint32_t S1 = pv_rotr(e, 6) ^ pv_rotr(e, 11) ^ pv_rotr(e, 25);
        uint32_t ch = (e & f) ^ ((~e) & g);
        uint32_t t1 = h + S1 + ch + PV_SHA256_K[t] + w[t];
        uint32_t S0 = pv_rotr(a, 2) ^ pv_rotr(a, 13) ^ pv_rotr(a, 22);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t t2 = S0 + maj;
        h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    H[0] += a; H[1] += b; H[2] += c; H[3] += d; H[4] += e; H[5] += f; H[6] += g; H[7] += h;
}
static void pv_sha256(pv_ctx *ctx, uint32_t p, int32_t len, uint8_t out[32])
{
    uint32_t H[8] = { 0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                      0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u };
    if (len < 0) len = 0;
    uint64_t bitlen = (uint64_t)(uint32_t)len * 8u;
    int32_t total = len + 1 + 8;
    int32_t nblocks = (total + 63) / 64;
    int32_t padded = nblocks * 64;
    for (int32_t blk = 0; blk < nblocks; blk++) {
        uint8_t block[64];
        for (int j = 0; j < 64; j++) {
            int32_t idx = blk * 64 + j;
            uint8_t bb;
            if (idx < len) bb = pv_arena_get(ctx, p + (uint32_t)idx);
            else if (idx == len) bb = 0x80;
            else if (idx < padded - 8) bb = 0x00;
            else { int bp = idx - (padded - 8); bb = (uint8_t)((bitlen >> (56 - 8 * bp)) & 0xFF); }
            block[j] = bb;
        }
        pv_sha256_block(H, block);
    }
    for (int i = 0; i < 8; i++) {
        out[i * 4]     = (uint8_t)(H[i] >> 24);
        out[i * 4 + 1] = (uint8_t)(H[i] >> 16);
        out[i * 4 + 2] = (uint8_t)(H[i] >> 8);
        out[i * 4 + 3] = (uint8_t)(H[i]);
    }
}

/* ---- Crypto.HmacSha256: RFC 2104 over the canonical SHA-256 (== Python hmac == JS).
   Streaming so the inner hash is ipad-block || message (message read from the arena)
   without materializing a concatenation buffer. Key/message are two input spans. ---- */
typedef struct { uint32_t H[8]; uint8_t buf[64]; int fill; uint64_t total; } pv_sha256_stream;
static void pv_sha256_s_init(pv_sha256_stream *s)
{
    static const uint32_t IV[8] = { 0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                                     0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u };
    for (int i = 0; i < 8; i++) s->H[i] = IV[i];
    s->fill = 0; s->total = 0;
}
static void pv_sha256_s_push(pv_sha256_stream *s, uint8_t b)
{
    s->buf[s->fill++] = b; s->total++;
    if (s->fill == 64) { pv_sha256_block(s->H, s->buf); s->fill = 0; }
}
static void pv_sha256_s_final(pv_sha256_stream *s, uint8_t out[32])
{
    uint64_t bitlen = s->total * 8u;
    pv_sha256_s_push(s, 0x80);
    while (s->fill != 56) pv_sha256_s_push(s, 0x00);
    for (int i = 0; i < 8; i++) pv_sha256_s_push(s, (uint8_t)((bitlen >> (56 - 8 * i)) & 0xFF));
    for (int i = 0; i < 8; i++) {
        out[i * 4]     = (uint8_t)(s->H[i] >> 24);
        out[i * 4 + 1] = (uint8_t)(s->H[i] >> 16);
        out[i * 4 + 2] = (uint8_t)(s->H[i] >> 8);
        out[i * 4 + 3] = (uint8_t)(s->H[i]);
    }
}
static void pv_hmac_sha256(pv_ctx *ctx, uint32_t key_p, int32_t key_len,
                           uint32_t msg_p, int32_t msg_len, uint8_t out[32])
{
    if (key_len < 0) key_len = 0;
    if (msg_len < 0) msg_len = 0;
    uint8_t k[64];
    for (int i = 0; i < 64; i++) k[i] = 0;
    if (key_len > 64) {
        pv_sha256(ctx, key_p, key_len, k);          /* k[0..31] = H(key), k[32..63] = 0 */
    } else {
        for (int32_t i = 0; i < key_len; i++) k[i] = pv_arena_get(ctx, key_p + (uint32_t)i);
    }
    uint8_t ipad[64], opad[64];
    for (int i = 0; i < 64; i++) { ipad[i] = (uint8_t)(k[i] ^ 0x36); opad[i] = (uint8_t)(k[i] ^ 0x5c); }
    uint8_t inner[32];
    pv_sha256_stream s; pv_sha256_s_init(&s);
    for (int i = 0; i < 64; i++) pv_sha256_s_push(&s, ipad[i]);
    for (int32_t i = 0; i < msg_len; i++) pv_sha256_s_push(&s, pv_arena_get(ctx, msg_p + (uint32_t)i));
    pv_sha256_s_final(&s, inner);
    pv_sha256_stream s2; pv_sha256_s_init(&s2);
    for (int i = 0; i < 64; i++) pv_sha256_s_push(&s2, opad[i]);
    for (int i = 0; i < 32; i++) pv_sha256_s_push(&s2, inner[i]);
    pv_sha256_s_final(&s2, out);
}

/* ---- Template.* (AOT plan + renderer; mirrors picoscript_vm._templatelib) --
 * Plan ops: 0x01 LEN_HI LEN_LO bytes=literal, 0x02 KEYLEN key=hole, 0x03/0x04
 * KEYLEN key=(inverted) section, 0x05=end, 0x06 KEYLEN list=each. */
#define TPL_KEYMAX   512
#define TPL_MAXMODEL 512
#define TPL_MAXDEPTH 32
#define TPL_MAXOUTPUT (256 * 1024)   /* INV-19: bound total rendered output */
#define TPL_MAXEACH   100000         /* INV-19: bound {{#each}} iteration count */

static int32_t tpl_find2(pv_ctx *ctx, uint32_t p, int32_t n, int32_t from, uint8_t c0, uint8_t c1)
{
    for (int32_t i = from; i + 1 < n; i++)
        if (pv_arena_get(ctx, p + (uint32_t)i) == c0 && pv_arena_get(ctx, p + (uint32_t)(i + 1)) == c1) return i;
    return -1;
}
static void tpl_trim(pv_ctx *ctx, uint32_t p, int32_t *s, int32_t *e)
{
    while (*s < *e) { uint8_t c = pv_arena_get(ctx, p + (uint32_t)*s);       if (c==' '||c=='\t'||c=='\r'||c=='\n') (*s)++; else break; }
    while (*e > *s) { uint8_t c = pv_arena_get(ctx, p + (uint32_t)(*e - 1)); if (c==' '||c=='\t'||c=='\r'||c=='\n') (*e)--; else break; }
}
static void tpl_lit(pv_ctx *ctx, uint32_t *k, uint32_t p, int32_t from, int32_t to)
{
    int32_t len = to - from;
    if (len <= 0) return;
    pv_arena_put(ctx, k, 0x01);
    pv_arena_put(ctx, k, (uint8_t)((len >> 8) & 0xFF));
    pv_arena_put(ctx, k, (uint8_t)(len & 0xFF));
    for (int32_t i = from; i < to; i++) pv_arena_put(ctx, k, pv_arena_get(ctx, p + (uint32_t)i));
}
static void tpl_key(pv_ctx *ctx, uint32_t *k, uint8_t op, uint32_t p, int32_t s, int32_t e)
{
    int32_t len = e - s;
    if (len < 0) len = 0;
    if (len > 255) len = 255;
    pv_arena_put(ctx, k, op);
    pv_arena_put(ctx, k, (uint8_t)len);
    for (int32_t i = 0; i < len; i++) pv_arena_put(ctx, k, pv_arena_get(ctx, p + (uint32_t)(s + i)));
}
static int pv_template_compile(pv_ctx *ctx, uint32_t p, int32_t n)
{
    uint32_t k = 0;
    int32_t i = 0;
    while (i < n) {
        int32_t j = tpl_find2(ctx, p, n, i, '{', '{');
        if (j < 0) { tpl_lit(ctx, &k, p, i, n); break; }
        tpl_lit(ctx, &k, p, i, j);
        int32_t kk = tpl_find2(ctx, p, n, j + 2, '}', '}');
        if (kk < 0) { tpl_lit(ctx, &k, p, j, n); break; }
        int32_t s0 = j + 2, e0 = kk;
        tpl_trim(ctx, p, &s0, &e0);
        uint8_t f0 = (s0 < e0) ? pv_arena_get(ctx, p + (uint32_t)s0) : 0;
        if (f0 == '#') {
            int32_t rs = s0 + 1, re = e0;
            tpl_trim(ctx, p, &rs, &re);
            int is_each = (re - rs >= 4) &&
                pv_arena_get(ctx, p + (uint32_t)rs) == 'e' && pv_arena_get(ctx, p + (uint32_t)(rs + 1)) == 'a' &&
                pv_arena_get(ctx, p + (uint32_t)(rs + 2)) == 'c' && pv_arena_get(ctx, p + (uint32_t)(rs + 3)) == 'h' &&
                (rs + 4 == re || pv_arena_get(ctx, p + (uint32_t)(rs + 4)) == ' ' || pv_arena_get(ctx, p + (uint32_t)(rs + 4)) == '\t');
            if (is_each) {
                int32_t ls = rs + 4, le = re;
                tpl_trim(ctx, p, &ls, &le);
                tpl_key(ctx, &k, 0x06, p, ls, le);
            } else {
                tpl_key(ctx, &k, 0x03, p, rs, re);
            }
        } else if (f0 == '^') {
            int32_t rs = s0 + 1, re = e0;
            tpl_trim(ctx, p, &rs, &re);
            tpl_key(ctx, &k, 0x04, p, rs, re);
        } else if (f0 == '/') {
            pv_arena_put(ctx, &k, 0x05);
        } else {
            tpl_key(ctx, &k, 0x02, p, s0, e0);
        }
        i = kk + 2;
    }
    return pv_arena_finish(ctx, k);
}

typedef struct {
    pv_ctx  *ctx;
    uint32_t mp;
    int32_t  mk_off[TPL_MAXMODEL], mk_len[TPL_MAXMODEL];
    int32_t  mv_off[TPL_MAXMODEL], mv_len[TPL_MAXMODEL];
    int32_t  mcount;
} tpl_model;

static int tpl_parse_model(tpl_model *M, pv_ctx *ctx, uint32_t mp, int32_t mn)
{
    M->ctx = ctx; M->mp = mp; M->mcount = 0;
    int overflow = 0;                       /* INV-19: model larger than TPL_MAXMODEL */
    int32_t i = 0;
    while (i < mn) {
        int32_t start = i;
        while (i < mn && pv_arena_get(ctx, mp + (uint32_t)i) != '\n') i++;
        int32_t end = i;
        if (i < mn) i++;
        int32_t eq = start;
        while (eq < end && pv_arena_get(ctx, mp + (uint32_t)eq) != '=') eq++;
        if (eq < end) {
            if (M->mcount < TPL_MAXMODEL) {
                M->mk_off[M->mcount] = start;     M->mk_len[M->mcount] = eq - start;
                M->mv_off[M->mcount] = eq + 1;    M->mv_len[M->mcount] = end - (eq + 1);
                M->mcount++;
            } else {
                overflow = 1;
            }
        }
    }
    return overflow;
}
/* last match wins, mirroring a Python dict built by iterating lines. */
static int32_t tpl_find_key(tpl_model *M, const uint8_t *buf, int32_t buflen)
{
    int32_t found = -1;
    for (int32_t e = 0; e < M->mcount; e++) {
        if (M->mk_len[e] != buflen) continue;
        int32_t j = 0;
        for (; j < buflen; j++)
            if (pv_arena_get(M->ctx, M->mp + (uint32_t)(M->mk_off[e] + j)) != buf[j]) break;
        if (j == buflen) found = e;
    }
    return found;
}
static int tpl_startswith(tpl_model *M, const uint8_t *buf, int32_t buflen)
{
    for (int32_t e = 0; e < M->mcount; e++) {
        if (M->mk_len[e] < buflen) continue;
        int32_t j = 0;
        for (; j < buflen; j++)
            if (pv_arena_get(M->ctx, M->mp + (uint32_t)(M->mk_off[e] + j)) != buf[j]) break;
        if (j == buflen) return 1;
    }
    return 0;
}
static void tpl_resolve(tpl_model *M, pv_ctx *ctx, uint32_t pp, int32_t koff, int32_t klen,
                        const uint8_t *prefix, int32_t plen, int32_t *voff, int32_t *vlen)
{
    uint8_t lk[TPL_KEYMAX];
    int32_t l;
    *voff = 0; *vlen = 0;
    if (klen == 1 && pv_arena_get(ctx, pp + (uint32_t)koff) == '.') {
        int32_t idx = tpl_find_key(M, prefix, plen);
        if (idx >= 0) { *voff = M->mv_off[idx]; *vlen = M->mv_len[idx]; }
        return;
    }
    if (plen > 0) {
        l = 0;
        for (int32_t i = 0; i < plen && l < TPL_KEYMAX; i++) lk[l++] = prefix[i];
        if (l < TPL_KEYMAX) lk[l++] = '.';
        for (int32_t i = 0; i < klen && l < TPL_KEYMAX; i++) lk[l++] = pv_arena_get(ctx, pp + (uint32_t)(koff + i));
        int32_t idx = tpl_find_key(M, lk, l);
        if (idx >= 0) { *voff = M->mv_off[idx]; *vlen = M->mv_len[idx]; return; }
    }
    l = 0;
    for (int32_t i = 0; i < klen && l < TPL_KEYMAX; i++) lk[l++] = pv_arena_get(ctx, pp + (uint32_t)(koff + i));
    int32_t idx2 = tpl_find_key(M, lk, l);
    if (idx2 >= 0) { *voff = M->mv_off[idx2]; *vlen = M->mv_len[idx2]; }
}
static int32_t tpl_count_list(tpl_model *M, const uint8_t *full, int32_t full_len)
{
    int32_t c = 0;
    while (1) {
        uint8_t base[TPL_KEYMAX];
        int32_t bl = 0;
        for (int32_t i = 0; i < full_len && bl < TPL_KEYMAX; i++) base[bl++] = full[i];
        if (bl < TPL_KEYMAX) base[bl++] = '.';
        uint8_t dec[12];
        int32_t dl = pjs_dec(c, dec);
        for (int32_t i = 0; i < dl && bl < TPL_KEYMAX; i++) base[bl++] = dec[i];
        if (tpl_find_key(M, base, bl) >= 0) { c++; continue; }
        if (bl < TPL_KEYMAX) { base[bl++] = '.'; if (tpl_startswith(M, base, bl)) { c++; continue; } }
        return c;
    }
}
static int32_t tpl_skip(pv_ctx *ctx, uint32_t pp, int32_t pn, int32_t i)
{
    int depth = 1;
    while (i < pn && depth > 0) {
        uint8_t op = pv_arena_get(ctx, pp + (uint32_t)i); i++;
        if (op == 0x01) {
            int32_t ln = (pv_arena_get(ctx, pp + (uint32_t)i) << 8) | pv_arena_get(ctx, pp + (uint32_t)(i + 1));
            i += 2 + ln;
        } else if (op == 0x02) {
            i += 1 + pv_arena_get(ctx, pp + (uint32_t)i);
        } else if (op == 0x03 || op == 0x04 || op == 0x06) {
            i += 1 + pv_arena_get(ctx, pp + (uint32_t)i); depth++;
        } else if (op == 0x05) {
            depth--;
        }
    }
    return i;
}
static int pv_template_render(pv_ctx *ctx, uint32_t pp, int32_t pn, uint32_t mp, int32_t mn)
{
    tpl_model M;
    if (tpl_parse_model(&M, ctx, mp, mn)) { pv_set_fault(ctx, PV_FAULT_TEMPLATE, ctx->cur_pc, M.mcount); return 0; }  /* INV-19 model cap */
    struct { int kind; uint8_t sp[TPL_KEYMAX]; int32_t splen; int32_t body; int32_t count;
             uint8_t full[TPL_KEYMAX]; int32_t fulllen; int32_t idx; } fr[TPL_MAXDEPTH];
    int sp = 0;
    uint8_t prefix[TPL_KEYMAX];
    int32_t prefixlen = 0;
    uint32_t k = 0;
    int32_t i = 0;
    while (i < pn) {
        if (k > (uint32_t)TPL_MAXOUTPUT) { pv_set_fault(ctx, PV_FAULT_TEMPLATE, ctx->cur_pc, 0); break; }  /* INV-19 */
        uint8_t op = pv_arena_get(ctx, pp + (uint32_t)i); i++;
        if (op == 0x01) {
            int32_t ln = (pv_arena_get(ctx, pp + (uint32_t)i) << 8) | pv_arena_get(ctx, pp + (uint32_t)(i + 1));
            i += 2;
            for (int32_t t = 0; t < ln; t++) pv_arena_put(ctx, &k, pv_arena_get(ctx, pp + (uint32_t)(i + t)));
            i += ln;
        } else if (op == 0x02) {
            int32_t kl = pv_arena_get(ctx, pp + (uint32_t)i); i++;
            int32_t voff, vlen;
            tpl_resolve(&M, ctx, pp, i, kl, prefix, prefixlen, &voff, &vlen);
            for (int32_t t = 0; t < vlen; t++) pv_arena_put(ctx, &k, pv_arena_get(ctx, mp + (uint32_t)(voff + t)));
            i += kl;
        } else if (op == 0x03 || op == 0x04) {
            int32_t kl = pv_arena_get(ctx, pp + (uint32_t)i); i++;
            int32_t koff = i; i += kl;
            int32_t voff, vlen;
            tpl_resolve(&M, ctx, pp, koff, kl, prefix, prefixlen, &voff, &vlen);
            int truthy = (vlen > 0);
            int take = (op == 0x03) ? truthy : (!truthy);
            if (take) {
                if (sp >= TPL_MAXDEPTH) { pv_set_fault(ctx, PV_FAULT_TEMPLATE, ctx->cur_pc, 0); break; }
                fr[sp].kind = 0;
                for (int32_t t = 0; t < prefixlen; t++) fr[sp].sp[t] = prefix[t];
                fr[sp].splen = prefixlen;
                sp++;
            } else {
                i = tpl_skip(ctx, pp, pn, i);
            }
        } else if (op == 0x06) {
            int32_t kl = pv_arena_get(ctx, pp + (uint32_t)i); i++;
            int32_t koff = i; i += kl;
            uint8_t full[TPL_KEYMAX];
            int32_t fl = 0;
            if (prefixlen > 0) {
                for (int32_t t = 0; t < prefixlen && fl < TPL_KEYMAX; t++) full[fl++] = prefix[t];
                if (fl < TPL_KEYMAX) full[fl++] = '.';
            }
            for (int32_t t = 0; t < kl && fl < TPL_KEYMAX; t++) full[fl++] = pv_arena_get(ctx, pp + (uint32_t)(koff + t));
            int32_t cnt = tpl_count_list(&M, full, fl);
            if (cnt > TPL_MAXEACH) { pv_set_fault(ctx, PV_FAULT_TEMPLATE, ctx->cur_pc, cnt); break; }  /* INV-19 */
            if (cnt == 0) {
                i = tpl_skip(ctx, pp, pn, i);
            } else {
                if (sp >= TPL_MAXDEPTH) { pv_set_fault(ctx, PV_FAULT_TEMPLATE, ctx->cur_pc, 0); break; }
                fr[sp].kind = 1;
                for (int32_t t = 0; t < prefixlen; t++) fr[sp].sp[t] = prefix[t];
                fr[sp].splen = prefixlen;
                fr[sp].body = i; fr[sp].count = cnt;
                for (int32_t t = 0; t < fl; t++) fr[sp].full[t] = full[t];
                fr[sp].fulllen = fl; fr[sp].idx = 0;
                sp++;
                prefixlen = 0;
                for (int32_t t = 0; t < fl && prefixlen < TPL_KEYMAX; t++) prefix[prefixlen++] = full[t];
                if (prefixlen < TPL_KEYMAX) prefix[prefixlen++] = '.';
                if (prefixlen < TPL_KEYMAX) prefix[prefixlen++] = '0';
            }
        } else if (op == 0x05) {
            if (sp > 0) {
                int f = sp - 1;
                if (fr[f].kind == 1) {
                    fr[f].idx++;
                    if (fr[f].idx < fr[f].count) {
                        prefixlen = 0;
                        for (int32_t t = 0; t < fr[f].fulllen && prefixlen < TPL_KEYMAX; t++) prefix[prefixlen++] = fr[f].full[t];
                        if (prefixlen < TPL_KEYMAX) prefix[prefixlen++] = '.';
                        uint8_t dec[12];
                        int32_t dl = pjs_dec(fr[f].idx, dec);
                        for (int32_t t = 0; t < dl && prefixlen < TPL_KEYMAX; t++) prefix[prefixlen++] = dec[t];
                        i = fr[f].body;
                    } else {
                        prefixlen = fr[f].splen;
                        for (int32_t t = 0; t < prefixlen; t++) prefix[t] = fr[f].sp[t];
                        sp--;
                    }
                } else {
                    prefixlen = fr[f].splen;
                    for (int32_t t = 0; t < prefixlen; t++) prefix[t] = fr[f].sp[t];
                    sp--;
                }
            }
        } else {
            break;
        }
    }
    return pv_arena_finish(ctx, k);
}

/* ---- Utf8Writer / Utf8Reader / Json / Xml (arena-backed; mirror _textio) ---- */
static void pv_w_byte(pv_ctx *ctx, int w, uint8_t b)
{
    if (ctx->w_pos[w] < ctx->w_cap[w]) {
        uint32_t a = ctx->w_ptr[w] + ctx->w_pos[w];
        if (ctx->mem && a < (uint32_t)ctx->mem_size) ctx->mem[a] = b;
        ctx->w_pos[w]++;
    }
}
static void pv_w_cstr(pv_ctx *ctx, int w, const char *s) { while (*s) pv_w_byte(ctx, w, (uint8_t)*s++); }
static void pv_w_span(pv_ctx *ctx, int w, int h)
{
    uint32_t p = pv_span_p(ctx, h);
    int32_t l = pv_span_n(ctx, h);
    for (int32_t i = 0; i < l; i++) pv_w_byte(ctx, w, pv_arena_get(ctx, p + (uint32_t)i));
}
static void pv_w_int(pv_ctx *ctx, int w, int32_t v)
{
    uint8_t tmp[16]; int t = 0, neg = 0; uint32_t u;
    if (v < 0) { neg = 1; u = 0u - (uint32_t)v; } else u = (uint32_t)v;
    if (u == 0) tmp[t++] = '0';
    while (u) { tmp[t++] = (uint8_t)('0' + (u % 10u)); u /= 10u; }
    if (neg) pv_w_byte(ctx, w, '-');
    while (t > 0) pv_w_byte(ctx, w, tmp[--t]);
}
static void pv_w_json_esc(pv_ctx *ctx, int w, int h)
{
    uint32_t p = pv_span_p(ctx, h);
    int32_t l = pv_span_n(ctx, h);
    for (int32_t i = 0; i < l; i++) {
        uint8_t c = pv_arena_get(ctx, p + (uint32_t)i);
        if (c == '"') { pv_w_byte(ctx, w, '\\'); pv_w_byte(ctx, w, '"'); }
        else if (c == '\\') { pv_w_byte(ctx, w, '\\'); pv_w_byte(ctx, w, '\\'); }
        else if (c == '\n') { pv_w_byte(ctx, w, '\\'); pv_w_byte(ctx, w, 'n'); }
        else if (c == '\r') { pv_w_byte(ctx, w, '\\'); pv_w_byte(ctx, w, 'r'); }
        else if (c == '\t') { pv_w_byte(ctx, w, '\\'); pv_w_byte(ctx, w, 't'); }
        else if (c < 0x20) { pv_w_cstr(ctx, w, "\\u00"); pv_w_byte(ctx, w, pv_hexd((c >> 4) & 0xF)); pv_w_byte(ctx, w, pv_hexd(c & 0xF)); }
        else pv_w_byte(ctx, w, c);
    }
}
static void pv_w_xml_esc(pv_ctx *ctx, int w, int h)
{
    uint32_t p = pv_span_p(ctx, h);
    int32_t l = pv_span_n(ctx, h);
    for (int32_t i = 0; i < l; i++) {
        uint8_t c = pv_arena_get(ctx, p + (uint32_t)i);
        if (c == '&') pv_w_cstr(ctx, w, "&amp;");
        else if (c == '<') pv_w_cstr(ctx, w, "&lt;");
        else if (c == '>') pv_w_cstr(ctx, w, "&gt;");
        else pv_w_byte(ctx, w, c);
    }
}
static void pv_json_pre(pv_ctx *ctx, int w)
{
    int sp = ctx->w_sp[w];
    if (sp == 0) return;
    int top = w * PV_JSON_DEPTH + (sp - 1);
    if (ctx->w_safter[top]) ctx->w_safter[top] = 0;
    else if (ctx->w_scount[top] > 0) pv_w_byte(ctx, w, ',');
}
static void pv_json_post(pv_ctx *ctx, int w)
{
    int sp = ctx->w_sp[w];
    if (sp > 0) ctx->w_scount[w * PV_JSON_DEPTH + (sp - 1)]++;
}
/* Returns 1 if the hook was a Utf8Writer/Utf8Reader/Json/Xml op. */
static int pv_textio(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    if (hook == PV_HOOK_UTF8WRITER_NEW) {
        if (ctx->w_count >= PV_MAX_WRITERS) { ctx->regs[rd] = 0; return 1; }
        int w = ctx->w_count++;
        ctx->w_ptr[w] = (uint32_t)(ctx->regs[rs1] & 0xFFFF);
        ctx->w_cap[w] = (uint32_t)(ctx->regs[rs2] & 0xFFFF);
        ctx->w_pos[w] = 0; ctx->w_sp[w] = 0;
        ctx->regs[rd] = w; return 1;
    }
    if (hook >= PV_HOOK_UTF8WRITER_BYTE && hook <= PV_HOOK_UTF8WRITER_RESET) {
        int w = ctx->regs[rs1];
        if (w <= 0 || w >= ctx->w_count) { ctx->regs[rd] = 0; return 1; }
        if (hook == PV_HOOK_UTF8WRITER_BYTE) { pv_w_byte(ctx, w, (uint8_t)ctx->regs[rs2]); return 1; }
        if (hook == PV_HOOK_UTF8WRITER_INT) { pv_w_int(ctx, w, ctx->regs[rs2]); return 1; }
        if (hook == PV_HOOK_UTF8WRITER_SPAN) { pv_w_span(ctx, w, ctx->regs[rs2]); return 1; }
        if (hook == PV_HOOK_UTF8WRITER_TOSPAN) { ctx->regs[rd] = pv_span_make(ctx, ctx->w_ptr[w], (int32_t)ctx->w_pos[w]); return 1; }
        if (hook == PV_HOOK_UTF8WRITER_LEN) { ctx->regs[rd] = (int32_t)ctx->w_pos[w]; return 1; }
        if (hook == PV_HOOK_UTF8WRITER_RESET) { ctx->w_pos[w] = 0; ctx->w_sp[w] = 0; return 1; }
    }
    if (hook == PV_HOOK_UTF8READER_NEW) {
        if (ctx->r_count >= PV_MAX_READERS) { ctx->regs[rd] = 0; return 1; }
        int r = ctx->r_count++;
        int sh = ctx->regs[rs1];
        ctx->r_ptr[r] = pv_span_p(ctx, sh); ctx->r_len[r] = (uint32_t)pv_span_n(ctx, sh); ctx->r_pos[r] = 0;
        ctx->regs[rd] = r; return 1;
    }
    if (hook >= PV_HOOK_UTF8READER_PEEK && hook <= PV_HOOK_UTF8READER_MATCH) {
        int r = ctx->regs[rs1];
        if (r <= 0 || r >= ctx->r_count) { ctx->regs[rd] = 0; return 1; }
        uint32_t p = ctx->r_ptr[r]; uint32_t len = ctx->r_len[r];
        if (hook == PV_HOOK_UTF8READER_PEEK) { ctx->regs[rd] = (ctx->r_pos[r] < len) ? pv_arena_get(ctx, p + ctx->r_pos[r]) : 0; return 1; }
        if (hook == PV_HOOK_UTF8READER_NEXT) { ctx->regs[rd] = (ctx->r_pos[r] < len) ? pv_arena_get(ctx, p + ctx->r_pos[r]) : 0; if (ctx->r_pos[r] < len) ctx->r_pos[r]++; return 1; }
        if (hook == PV_HOOK_UTF8READER_SKIPWS) { while (ctx->r_pos[r] < len) { uint8_t c = pv_arena_get(ctx, p + ctx->r_pos[r]); if (c == 32 || c == 9 || c == 10 || c == 13) ctx->r_pos[r]++; else break; } return 1; }
        if (hook == PV_HOOK_UTF8READER_EOF) { ctx->regs[rd] = (ctx->r_pos[r] >= len) ? 1 : 0; return 1; }
        if (hook == PV_HOOK_UTF8READER_POS) { ctx->regs[rd] = (int32_t)ctx->r_pos[r]; return 1; }
        if (hook == PV_HOOK_UTF8READER_MATCH) {
            if (ctx->r_pos[r] < len && pv_arena_get(ctx, p + ctx->r_pos[r]) == (uint8_t)(ctx->regs[rs2] & 0xFF)) { ctx->r_pos[r]++; ctx->regs[rd] = 1; } else ctx->regs[rd] = 0;
            return 1;
        }
        if (hook == PV_HOOK_UTF8READER_INT) {
            while (ctx->r_pos[r] < len) { uint8_t c = pv_arena_get(ctx, p + ctx->r_pos[r]); if (c == 32 || c == 9 || c == 10 || c == 13) ctx->r_pos[r]++; else break; }
            int neg = 0;
            if (ctx->r_pos[r] < len && pv_arena_get(ctx, p + ctx->r_pos[r]) == 0x2D) { neg = 1; ctx->r_pos[r]++; }
            uint32_t v = 0;
            while (ctx->r_pos[r] < len) { uint8_t c = pv_arena_get(ctx, p + ctx->r_pos[r]); if (c >= 0x30 && c <= 0x39) { v = v * 10u + (uint32_t)(c - 0x30); ctx->r_pos[r]++; } else break; }
            ctx->regs[rd] = neg ? (int32_t)(0u - v) : (int32_t)v; return 1;
        }
    }
    if (hook >= PV_HOOK_JSON_BEGINOBJECT && hook <= PV_HOOK_JSON_RAW) {
        int w = ctx->regs[rs1];
        if (w <= 0 || w >= ctx->w_count) { ctx->regs[rd] = 0; return 1; }
        if (hook == PV_HOOK_JSON_BEGINOBJECT || hook == PV_HOOK_JSON_BEGINARRAY) {
            pv_json_pre(ctx, w);
            pv_w_byte(ctx, w, hook == PV_HOOK_JSON_BEGINOBJECT ? '{' : '[');
            if (ctx->w_sp[w] > 0) ctx->w_scount[w * PV_JSON_DEPTH + (ctx->w_sp[w] - 1)]++;
            if (ctx->w_sp[w] < PV_JSON_DEPTH) { int nf = w * PV_JSON_DEPTH + ctx->w_sp[w]; ctx->w_scount[nf] = 0; ctx->w_safter[nf] = 0; ctx->w_sp[w]++; }
            return 1;
        }
        if (hook == PV_HOOK_JSON_ENDOBJECT || hook == PV_HOOK_JSON_ENDARRAY) {
            if (ctx->w_sp[w] > 0) ctx->w_sp[w]--;
            pv_w_byte(ctx, w, hook == PV_HOOK_JSON_ENDOBJECT ? '}' : ']');
            return 1;
        }
        if (hook == PV_HOOK_JSON_KEY) {
            int sp = ctx->w_sp[w];
            if (sp > 0 && ctx->w_scount[w * PV_JSON_DEPTH + (sp - 1)] > 0) pv_w_byte(ctx, w, ',');
            pv_w_byte(ctx, w, '"'); pv_w_json_esc(ctx, w, ctx->regs[rs2]); pv_w_byte(ctx, w, '"'); pv_w_byte(ctx, w, ':');
            if (sp > 0) ctx->w_safter[w * PV_JSON_DEPTH + (sp - 1)] = 1;
            return 1;
        }
        if (hook == PV_HOOK_JSON_STR) { pv_json_pre(ctx, w); pv_w_byte(ctx, w, '"'); pv_w_json_esc(ctx, w, ctx->regs[rs2]); pv_w_byte(ctx, w, '"'); pv_json_post(ctx, w); return 1; }
        if (hook == PV_HOOK_JSON_INT) { pv_json_pre(ctx, w); pv_w_int(ctx, w, ctx->regs[rs2]); pv_json_post(ctx, w); return 1; }
        if (hook == PV_HOOK_JSON_BOOL) { pv_json_pre(ctx, w); pv_w_cstr(ctx, w, ctx->regs[rs2] ? "true" : "false"); pv_json_post(ctx, w); return 1; }
        if (hook == PV_HOOK_JSON_NULL) { pv_json_pre(ctx, w); pv_w_cstr(ctx, w, "null"); pv_json_post(ctx, w); return 1; }
        if (hook == PV_HOOK_JSON_RAW) { pv_json_pre(ctx, w); pv_w_span(ctx, w, ctx->regs[rs2]); pv_json_post(ctx, w); return 1; }
    }
    if (hook >= PV_HOOK_XML_OPEN && hook <= PV_HOOK_XML_EMPTY) {
        int w = ctx->regs[rs1];
        if (w <= 0 || w >= ctx->w_count) { ctx->regs[rd] = 0; return 1; }
        if (hook == PV_HOOK_XML_OPEN) { pv_w_byte(ctx, w, '<'); pv_w_span(ctx, w, ctx->regs[rs2]); return 1; }
        if (hook == PV_HOOK_XML_ATTRNAME) { pv_w_byte(ctx, w, ' '); pv_w_span(ctx, w, ctx->regs[rs2]); pv_w_byte(ctx, w, '='); pv_w_byte(ctx, w, '"'); return 1; }
        if (hook == PV_HOOK_XML_ATTRVALUE) { pv_w_xml_esc(ctx, w, ctx->regs[rs2]); pv_w_byte(ctx, w, '"'); return 1; }
        if (hook == PV_HOOK_XML_OPENEND) { pv_w_byte(ctx, w, '>'); return 1; }
        if (hook == PV_HOOK_XML_TEXT) { pv_w_xml_esc(ctx, w, ctx->regs[rs2]); return 1; }
        if (hook == PV_HOOK_XML_CLOSE) { pv_w_byte(ctx, w, '<'); pv_w_byte(ctx, w, '/'); pv_w_span(ctx, w, ctx->regs[rs2]); pv_w_byte(ctx, w, '>'); return 1; }
        if (hook == PV_HOOK_XML_EMPTY) { pv_w_byte(ctx, w, '/'); pv_w_byte(ctx, w, '>'); return 1; }
    }
    return 0;
}

/* ---- default host: Random.U32 + Queue.* (mirrors HostApi) ------------- */

/* Binding capability class required by a hook (0 = pure computation, always allowed).
 * Classified by hook code (ranges + the few mixed-namespace exceptions), kept in lockstep
 * with the Python/JS classifiers so a denied hook faults identically on every path. */
uint32_t pv_hook_cap(int hook)
{
    /* mixed-namespace exceptions first */
    if (hook == PV_HOOK_MATHS_RANDOM || hook == PV_HOOK_MATHS_RANDOMRANGE) return PV_CAP_RANDOM;
    if (hook == PV_HOOK_CRYPTO_RANDOMBYTES) return PV_CAP_RANDOM;
    if (hook == PV_HOOK_CRYPTO_ENCRYPT || hook == PV_HOOK_CRYPTO_DECRYPT) return PV_CAP_CRYPTO;
    if (hook >= 0x130 && hook <= 0x133) return PV_CAP_NET;   /* Http Read/Generate (Parse/Encode are pure) */
    if (hook >= 0x01 && hook <= 0x06) return PV_CAP_KERNEL;  /* Kernel.* */
    if ((hook >= 0x07 && hook <= 0x0E) || (hook >= 0x1B0 && hook <= 0x1B2)) return PV_CAP_NET;     /* Req.* */
    if (hook >= 0x10 && hook <= 0x14) return PV_CAP_QUEUE;   /* Queue.* */
    if ((hook >= 0x15 && hook <= 0x1F) || hook == 0x38 || hook == 0x39) return PV_CAP_NET;  /* Resp.* */
    if (hook == PV_HOOK_RANDOM_U32) return PV_CAP_RANDOM;    /* Random.U32 */
    if ((hook >= 0x60 && hook <= 0x6F) || (hook >= 0x1A0 && hook <= 0x1A4)) return PV_CAP_STORAGE; /* Storage.* */
    if (hook >= 0xB0 && hook <= 0xBA) return PV_CAP_TIME;    /* DateTime.* */
    if (hook >= 0xC0 && hook <= 0xC6) return PV_CAP_ENV;     /* Locale.* */
    if (hook >= 0xD0 && hook <= 0xD8) return PV_CAP_ENV;     /* Environment.* */
    if (hook >= 0xE0 && hook <= 0xEE) return PV_CAP_CONTEXT; /* Context.* */
    if (hook >= 0x110 && hook <= 0x117) return PV_CAP_AUTH;  /* X509.* */
    if (hook >= 0x120 && hook <= 0x129) return PV_CAP_AUTH;  /* Auth.* */
    if (hook >= 0x150 && hook <= 0x156) return PV_CAP_GPIO;  /* Gpio.* */
    if (hook >= 0x160 && hook <= 0x167) return PV_CAP_CAPSULE; /* Pack/Card/Fifo */
    if (hook >= 0x168 && hook <= 0x16B) return PV_CAP_DEVICE;  /* Device.* */
    if (hook >= 0x170 && hook <= 0x177) return PV_CAP_DMA;     /* Stream.* */
    if ((hook >= 0x180 && hook <= 0x186) || (hook >= 0x1B3 && hook <= 0x1B5)) return PV_CAP_EVENT;   /* Event.* */
    if (hook >= 0x188 && hook <= 0x193) return PV_CAP_UI;      /* Ui.* */
    if ((hook >= 0x1D0 && hook <= 0x1DF) || (hook >= 0x200 && hook <= 0x20B)) return PV_CAP_STORAGE; /* Search.* over card packs */
    return 0;                                                /* pure: String/Number/Maths/Span/... */
}

/* ── Q16.16 fixed-point CORDIC (Maths.Sin/Cos/Tan, ...) ──────────────────────
 * All-integer; constants/iteration count are shared verbatim with picoscript_vm.py
 * (_q16_*) and vm/picovm.js so results are byte-identical on every path. */
#define PV_Q16_ONE      65536
#define PV_Q16_HALF_PI  102944
#define PV_Q16_PI       205887
#define PV_Q16_TWO_PI   411775
#define PV_Q16_GAIN_INV 39797
static const int32_t PV_Q16_ATAN[16] = {
    51472, 30386, 16055, 8150, 4091, 2047, 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2
};

static void pv_q16_sincos(int32_t angle, int32_t *out_sin, int32_t *out_cos)
{
    int32_t a = angle % PV_Q16_TWO_PI;
    if (a < 0) a += PV_Q16_TWO_PI;
    int32_t q = a / PV_Q16_HALF_PI;
    int32_t r = a - q * PV_Q16_HALF_PI;
    int32_t x = PV_Q16_GAIN_INV, y = 0, z = r, i, s, c;
    for (i = 0; i < 16; i++) {
        int32_t dx = x >> i, dy = y >> i;
        if (z >= 0) { x -= dy; y += dx; z -= PV_Q16_ATAN[i]; }
        else        { x += dy; y -= dx; z += PV_Q16_ATAN[i]; }
    }
    switch (q) {
        case 0:  s = y;  c = x;  break;
        case 1:  s = x;  c = -y; break;
        case 2:  s = -y; c = -x; break;
        default: s = -x; c = y;  break;
    }
    *out_sin = s; *out_cos = c;
}

static int32_t pv_q16_tan(int32_t angle)
{
    int32_t s, c;
    pv_q16_sincos(angle, &s, &c);
    if (c == 0) return (s >= 0) ? 0x7FFFFFFF : (int32_t)0x80000000;
    return (int32_t)(((int64_t)s * PV_Q16_ONE) / c);   /* trunc toward zero (C99 /) */
}

/* Q16.16 exp/log. fixmul uses arithmetic >>16; series divides trunc-toward-zero. */
#define PV_Q16_LN2       45426
#define PV_Q16_INV_LN2   94548
#define PV_Q16_INV_LN10  28462
#define PV_Q16_EXP_MAX_Z 681300

static int32_t pv_q16_fixmul(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * b) >> 16);
}
static int32_t pv_q16_idiv(int32_t a, int32_t n)   /* trunc toward zero */
{
    int64_t aa = (a < 0) ? -(int64_t)a : a;
    int64_t nn = (n < 0) ? -(int64_t)n : n;
    int64_t q = aa / nn;
    return (int32_t)(((a < 0) != (n < 0)) ? -q : q);
}
static int32_t pv_q16_fixdiv(int32_t a, int32_t b)
{
    int64_t num = (int64_t)a * PV_Q16_ONE;
    int64_t nn = (num < 0) ? -num : num;
    int64_t bb = (b < 0) ? -(int64_t)b : b;
    int64_t q = nn / bb;
    return (int32_t)(((num < 0) != (b < 0)) ? -q : q);
}
static int32_t pv_q16_exp(int32_t z)
{
    int32_t k, r, term, acc, n, i;
    if (z >= PV_Q16_EXP_MAX_Z) return 0x7FFFFFFF;
    if (z <= -PV_Q16_EXP_MAX_Z) return 0;
    k = (pv_q16_fixmul(z, PV_Q16_INV_LN2) + (PV_Q16_ONE >> 1)) >> 16;
    r = z - k * PV_Q16_LN2;
    term = PV_Q16_ONE; acc = PV_Q16_ONE;
    for (n = 1; n < 8; n++) {
        term = pv_q16_idiv(pv_q16_fixmul(term, r), n);
        acc += term;
    }
    if (k >= 0) {
        int64_t a64 = acc;
        for (i = 0; i < k; i++) { a64 *= 2; if (a64 > 0x7FFFFFFF) return 0x7FFFFFFF; }
        return (int32_t)a64;
    }
    for (i = 0; i < -k; i++) acc >>= 1;
    return acc;
}
static int32_t pv_q16_log(int32_t x)
{
    int32_t e = 0, m = x, u, u2, term, acc = 0, n;
    if (x <= 0) return (int32_t)0x80000000;
    while (m >= 2 * PV_Q16_ONE) { m >>= 1; e++; }
    while (m < PV_Q16_ONE) { m <<= 1; e--; }
    u = pv_q16_fixdiv(m - PV_Q16_ONE, m + PV_Q16_ONE);
    u2 = pv_q16_fixmul(u, u); term = u;
    for (n = 0; n < 6; n++) {
        acc += pv_q16_idiv(term, 2 * n + 1);
        term = pv_q16_fixmul(term, u2);
    }
    return (2 * acc) + e * PV_Q16_LN2;
}

/* ── Decimal.*: Q16.16 fixed-point fractional numeric library ────────────
 * Mirrors picoscript_vm.py's HostApi._decimallib / vm/picovm.js's
 * _decimallib exactly. Unlike Number.Parse (32-bit-integer only, truncates
 * any fraction), this preserves it as a Q16.16 fixed-point value in a plain
 * 32-bit register -- the same encoding pv_q16_sincos/pv_q16_exp/pv_q16_log
 * already use above. */
#define PV_Q16_FRAC_DIGITS 5

static int pv_q16_parse_bytes(const uint8_t *b, int32_t len, int32_t *out)
{
    int32_t i = 0, e = len;
    while (i < e && (b[i] == 0x20 || b[i] == 0x09 || b[i] == 0x0d || b[i] == 0x0a)) i++;
    while (e > i && (b[e - 1] == 0x20 || b[e - 1] == 0x09 || b[e - 1] == 0x0d || b[e - 1] == 0x0a)) e--;
    if (i >= e) return 0;
    int neg = 0;
    if (b[i] == '+' || b[i] == '-') { neg = (b[i] == '-'); i++; }
    int32_t int_start = i, j = i;
    uint32_t ipart = 0;
    for (; j < e; j++) {
        if (b[j] < '0' || b[j] > '9') break;
        ipart = ipart * 10u + (uint32_t)(b[j] - '0');
    }
    if (j == int_start) return 0;   /* at least one integer digit required */
    uint32_t fscaled = 0;
    if (j < e) {
        if (b[j] != '.') return 0;
        int32_t k = j + 1, flen = 0;
        uint32_t fnum = 0, fden = 1;
        for (; k < e; k++) {
            if (b[k] < '0' || b[k] > '9') return 0;
            /* Cap accumulated fractional digits well under 32 bits; extra
             * digits beyond 9 add no real Q16.16 precision (2**-16 ~= 1.5e-5)
             * anyway. */
            if (flen < 9) { fnum = fnum * 10u + (uint32_t)(b[k] - '0'); fden *= 10u; flen++; }
        }
        if (flen > 0) fscaled = (uint32_t)(((uint64_t)fnum * PV_Q16_ONE + fden / 2) / fden);
    }
    {
        uint32_t v = (ipart << 16) + fscaled;
        *out = neg ? (int32_t)(0u - v) : (int32_t)v;
    }
    return 1;
}

static int32_t pv_q16_format_fixed(int32_t v, int digits, uint8_t *buf)
{
    int neg = v < 0;
    uint32_t uv = neg ? (uint32_t)(0 - (int64_t)v) : (uint32_t)v;
    uint32_t ip = uv >> 16, frac = uv & 0xFFFFu;
    uint32_t scale = 1; int di;
    for (di = 0; di < digits; di++) scale *= 10u;
    uint32_t fdigits = (uint32_t)(((uint64_t)frac * scale + PV_Q16_ONE / 2) / PV_Q16_ONE);
    if (fdigits >= scale) { ip += 1; fdigits -= scale; }
    uint8_t tmp[16]; int t = 0;
    uint32_t x = ip;
    if (x == 0) tmp[t++] = '0';
    while (x) { tmp[t++] = (uint8_t)('0' + (x % 10u)); x /= 10u; }
    int32_t n = 0;
    if (neg && (ip != 0 || fdigits != 0)) buf[n++] = '-';
    while (t > 0) buf[n++] = tmp[--t];
    if (digits > 0) {
        uint8_t ftmp[16]; int ft = 0;
        uint32_t fx = fdigits;
        buf[n++] = '.';
        for (di = 0; di < digits; di++) { ftmp[ft++] = (uint8_t)('0' + (fx % 10u)); fx /= 10u; }
        while (ft > 0) buf[n++] = ftmp[--ft];
    }
    return n;
}

/* Shortest decimal string that parses back to the exact same Q16.16 value
 * (mirrors picoscript_vm.py's _q16_to_str / vm/picovm.js's q16ToStr -- same
 * "shortest round-trip" technique as repr(float)/Number.toString() for
 * IEEE754, avoiding binary-fraction noise on common values like "19.99"). */
static int32_t pv_q16_to_str(int32_t v, uint8_t *buf)
{
    int digits;
    for (digits = 0; digits <= PV_Q16_FRAC_DIGITS; digits++) {
        uint8_t tmp[24];
        int32_t n = pv_q16_format_fixed(v, digits, tmp);
        int32_t rt;
        if (pv_q16_parse_bytes(tmp, n, &rt) && rt == v) {
            int32_t bi; for (bi = 0; bi < n; bi++) buf[bi] = tmp[bi];
            return n;
        }
    }
    return pv_q16_format_fixed(v, PV_Q16_FRAC_DIGITS, buf);
}

/* ── AES-256-CTR (Crypto.Encrypt/Decrypt). Tables + algorithm byte-identical with
 * picoscript_vm.py and vm/picovm.js; CTR is symmetric so encrypt == decrypt. ── */
static const uint8_t PV_AES_SBOX[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16
};
static const uint8_t PV_AES_RCON[14] = {
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36, 0x6c, 0xd8, 0xab, 0x4d
};
static uint8_t pv_aes_xtime(uint8_t a) { return (a & 0x80) ? (uint8_t)((a << 1) ^ 0x1B) : (uint8_t)(a << 1); }
static uint8_t pv_aes_gmul(uint8_t a, uint8_t b)
{
    uint8_t r = 0, i;
    for (i = 0; i < 8; i++) { if (b & 1) r ^= a; a = pv_aes_xtime(a); b >>= 1; }
    return r;
}
static void pv_aes256_key_expand(const uint8_t key[32], uint8_t rk[240])
{
    int i, j;
    uint8_t t[4], tmp;
    for (i = 0; i < 32; i++) rk[i] = key[i];
    for (i = 8; i < 60; i++) {
        for (j = 0; j < 4; j++) t[j] = rk[(i - 1) * 4 + j];
        if (i % 8 == 0) {
            tmp = t[0]; t[0] = t[1]; t[1] = t[2]; t[2] = t[3]; t[3] = tmp;
            for (j = 0; j < 4; j++) t[j] = PV_AES_SBOX[t[j]];
            t[0] ^= PV_AES_RCON[i / 8 - 1];
        } else if (i % 8 == 4) {
            for (j = 0; j < 4; j++) t[j] = PV_AES_SBOX[t[j]];
        }
        for (j = 0; j < 4; j++) rk[i * 4 + j] = rk[(i - 8) * 4 + j] ^ t[j];
    }
}
static void pv_aes256_encrypt_block(const uint8_t in[16], const uint8_t rk[240], uint8_t out[16])
{
    uint8_t s[16], t[16], a0, a1, a2, a3;
    int i, c, r, rnd;
    for (i = 0; i < 16; i++) s[i] = in[i] ^ rk[i];
    for (rnd = 1; rnd < 14; rnd++) {
        for (i = 0; i < 16; i++) s[i] = PV_AES_SBOX[s[i]];
        for (r = 0; r < 4; r++) for (c = 0; c < 4; c++) t[r + 4 * c] = s[r + 4 * ((c + r) & 3)];
        for (c = 0; c < 4; c++) {
            a0 = t[4 * c]; a1 = t[4 * c + 1]; a2 = t[4 * c + 2]; a3 = t[4 * c + 3];
            s[4 * c]     = pv_aes_gmul(a0, 2) ^ pv_aes_gmul(a1, 3) ^ a2 ^ a3;
            s[4 * c + 1] = a0 ^ pv_aes_gmul(a1, 2) ^ pv_aes_gmul(a2, 3) ^ a3;
            s[4 * c + 2] = a0 ^ a1 ^ pv_aes_gmul(a2, 2) ^ pv_aes_gmul(a3, 3);
            s[4 * c + 3] = pv_aes_gmul(a0, 3) ^ a1 ^ a2 ^ pv_aes_gmul(a3, 2);
        }
        for (i = 0; i < 16; i++) s[i] ^= rk[rnd * 16 + i];
    }
    for (i = 0; i < 16; i++) s[i] = PV_AES_SBOX[s[i]];
    for (r = 0; r < 4; r++) for (c = 0; c < 4; c++) t[r + 4 * c] = s[r + 4 * ((c + r) & 3)];
    for (i = 0; i < 16; i++) out[i] = t[i] ^ rk[14 * 16 + i];
}

/* ---- DEFLATE inflate (RFC 1951) + gunzip: decompression is canonical, so the
 *      output is byte-identical to the Python/JS runtime. (Compression stays in
 *      the reference runtime + host -- see docs/COMPRESS.md.) Adapted from Mark
 *      Adler's public-domain puff.c; reads the input span, writes the output via
 *      the bump arena (back-references read already-written bytes). --------- */
#define PV_MAXBITS   15
#define PV_MAXLCODES 286
#define PV_MAXDCODES 30
#define PV_MAXCODES  (PV_MAXLCODES + PV_MAXDCODES)
#define PV_FIXLCODES 288

typedef struct {
    pv_ctx  *ctx;
    uint32_t in;        /* input arena pointer */
    int32_t  inlen;     /* input length (excludes any gzip trailer) */
    int32_t  incnt;     /* input bytes consumed */
    int      bitbuf;
    int      bitcnt;
    uint32_t *outk;     /* offset into the output span being built */
    int      err;       /* set when input is exhausted (truncated) */
} pv_puff;

typedef struct { short *count; short *symbol; } pv_huff;

static int pv_pbits(pv_puff *s, int need)
{
    long val = s->bitbuf;
    while (s->bitcnt < need) {
        if (s->incnt >= s->inlen) { s->err = 1; return 0; }
        val |= (long)pv_arena_get(s->ctx, s->in + (uint32_t)s->incnt++) << s->bitcnt;
        s->bitcnt += 8;
    }
    s->bitbuf = (int)(val >> need);
    s->bitcnt -= need;
    return (int)(val & ((1L << need) - 1));
}

static int pv_construct(pv_huff *h, const short *length, int n)
{
    int symbol, len, left;
    short offs[PV_MAXBITS + 1];
    for (len = 0; len <= PV_MAXBITS; len++) h->count[len] = 0;
    for (symbol = 0; symbol < n; symbol++) h->count[length[symbol]]++;
    if (h->count[0] == n) return 0;
    left = 1;
    for (len = 1; len <= PV_MAXBITS; len++) {
        left <<= 1;
        left -= h->count[len];
        if (left < 0) return left;          /* over-subscribed */
    }
    offs[1] = 0;
    for (len = 1; len < PV_MAXBITS; len++) offs[len + 1] = offs[len] + h->count[len];
    for (symbol = 0; symbol < n; symbol++)
        if (length[symbol] != 0) h->symbol[offs[length[symbol]]++] = (short)symbol;
    return left;
}

static int pv_decode(pv_puff *s, const pv_huff *h)
{
    int len, code = 0, first = 0, count, index = 0;
    for (len = 1; len <= PV_MAXBITS; len++) {
        code |= pv_pbits(s, 1);
        if (s->err) return -99;
        count = h->count[len];
        if (code - first < count) return h->symbol[index + (code - first)];
        index += count;
        first += count;
        first <<= 1;
        code <<= 1;
    }
    return -10;
}

static const short PV_LENS[29] = {3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,35,43,51,59,67,83,99,115,131,163,195,227,258};
static const short PV_LEXT[29] = {0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0};
static const short PV_DISTS[30] = {1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,257,385,513,769,1025,1537,2049,3073,4097,6145,8193,12289,16385,24577};
static const short PV_DEXT[30] = {0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13};
static const short PV_CLCIDX[19] = {16,17,18,0,8,7,9,6,10,5,11,4,12,3,13,2,14,1,15};

static int pv_pcodes(pv_puff *s, const pv_huff *lencode, const pv_huff *distcode)
{
    int symbol, len, i;
    unsigned dist;
    do {
        symbol = pv_decode(s, lencode);
        if (symbol < 0) return symbol;
        if (symbol < 256) {
            pv_arena_put(s->ctx, s->outk, (uint8_t)symbol);
        } else if (symbol > 256) {
            symbol -= 257;
            if (symbol >= 29) return -10;
            len = PV_LENS[symbol] + pv_pbits(s, PV_LEXT[symbol]);
            if (s->err) return -99;
            symbol = pv_decode(s, distcode);
            if (symbol < 0) return symbol;
            dist = (unsigned)(PV_DISTS[symbol] + pv_pbits(s, PV_DEXT[symbol]));
            if (s->err) return -99;
            for (i = 0; i < len; i++) {
                uint8_t b = pv_arena_get(s->ctx, s->ctx->arena_top + (*s->outk - dist));
                pv_arena_put(s->ctx, s->outk, b);
            }
        }
    } while (symbol != 256);
    return 0;
}

static int pv_pstored(pv_puff *s)
{
    unsigned len;
    s->bitbuf = 0; s->bitcnt = 0;            /* discard to byte boundary */
    if (s->incnt + 4 > s->inlen) return -2;
    len = (unsigned)pv_arena_get(s->ctx, s->in + (uint32_t)s->incnt)
        | ((unsigned)pv_arena_get(s->ctx, s->in + (uint32_t)s->incnt + 1) << 8);
    s->incnt += 4;
    if (s->incnt + (int32_t)len > s->inlen) return -2;
    while (len--) pv_arena_put(s->ctx, s->outk, pv_arena_get(s->ctx, s->in + (uint32_t)s->incnt++));
    return 0;
}

static int pv_pfixed(pv_puff *s)
{
    static short lcnt[PV_MAXBITS + 1], lsym[PV_FIXLCODES];
    static short dcnt[PV_MAXBITS + 1], dsym[PV_MAXDCODES];
    static pv_huff lencode = {lcnt, lsym};
    static pv_huff distcode = {dcnt, dsym};
    static int built = 0;
    if (!built) {
        short lengths[PV_FIXLCODES];
        int i;
        for (i = 0; i < 144; i++) lengths[i] = 8;
        for (; i < 256; i++) lengths[i] = 9;
        for (; i < 280; i++) lengths[i] = 7;
        for (; i < 288; i++) lengths[i] = 8;
        pv_construct(&lencode, lengths, PV_FIXLCODES);
        for (i = 0; i < PV_MAXDCODES; i++) lengths[i] = 5;
        pv_construct(&distcode, lengths, PV_MAXDCODES);
        built = 1;
    }
    return pv_pcodes(s, &lencode, &distcode);
}

static int pv_pdynamic(pv_puff *s)
{
    int nlen, ndist, ncode, index, symbol, len_rep;
    short lengths[PV_MAXCODES];
    short lcnt[PV_MAXBITS + 1], lsym[PV_MAXLCODES];
    short dcnt[PV_MAXBITS + 1], dsym[PV_MAXDCODES];
    short ccnt[PV_MAXBITS + 1], csym[19];
    pv_huff lencode = {lcnt, lsym};
    pv_huff distcode = {dcnt, dsym};
    pv_huff clcode = {ccnt, csym};
    nlen = pv_pbits(s, 5) + 257;
    ndist = pv_pbits(s, 5) + 1;
    ncode = pv_pbits(s, 4) + 4;
    if (s->err) return -99;
    if (nlen > PV_MAXLCODES || ndist > PV_MAXDCODES) return -3;
    for (index = 0; index < ncode; index++) lengths[PV_CLCIDX[index]] = (short)pv_pbits(s, 3);
    for (; index < 19; index++) lengths[PV_CLCIDX[index]] = 0;
    if (s->err) return -99;
    if (pv_construct(&clcode, lengths, 19) != 0) return -4;
    index = 0;
    while (index < nlen + ndist) {
        symbol = pv_decode(s, &clcode);
        if (symbol < 0) return symbol;
        if (symbol < 16) {
            lengths[index++] = (short)symbol;
        } else {
            len_rep = 0;
            if (symbol == 16) { if (index == 0) return -5; len_rep = lengths[index - 1]; symbol = 3 + pv_pbits(s, 2); }
            else if (symbol == 17) { symbol = 3 + pv_pbits(s, 3); }
            else { symbol = 11 + pv_pbits(s, 7); }
            if (s->err) return -99;
            if (index + symbol > nlen + ndist) return -6;
            while (symbol--) lengths[index++] = (short)len_rep;
        }
    }
    pv_construct(&lencode, lengths, nlen);
    pv_construct(&distcode, lengths + nlen, ndist);
    return pv_pcodes(s, &lencode, &distcode);
}

static int pv_inflate(pv_puff *s)
{
    int last, type, err;
    do {
        last = pv_pbits(s, 1);
        type = pv_pbits(s, 2);
        if (s->err) return -99;
        err = (type == 0) ? pv_pstored(s) : (type == 1) ? pv_pfixed(s) : (type == 2) ? pv_pdynamic(s) : -1;
        if (err != 0) return err;
    } while (!last);
    return 0;
}

/* ---- Map.* first-class dictionary (active-handle model; docs/MAP.md) -------
 * Entries live in a shared pool, singly linked per map for insertion-order
 * enumeration; key/value bytes intern into map_pool. FNV-1a (offset 0x811c9dc5,
 * prime 0x01000193) is identical to the JS/Python/C# implementations. */
static uint32_t pv_fnv1a_span(pv_ctx *ctx, int span_h)
{
    uint32_t p = pv_span_p(ctx, span_h); int32_t n = pv_span_n(ctx, span_h), i;
    uint32_t h = 0x811c9dc5u;
    for (i = 0; i < n; i++) { h ^= (uint32_t)pv_arena_get(ctx, p + (uint32_t)i); h *= 0x01000193u; }
    return h;
}
static int pv_map_intern(pv_ctx *ctx, int span_h, uint32_t *off, int32_t *len)
{
    uint32_t p = pv_span_p(ctx, span_h); int32_t n = pv_span_n(ctx, span_h), i;
    if (ctx->map_pool_top + (uint32_t)n > PV_MAP_POOL) return 0;
    *off = ctx->map_pool_top; *len = n;
    for (i = 0; i < n; i++) ctx->map_pool[ctx->map_pool_top + (uint32_t)i] = pv_arena_get(ctx, p + (uint32_t)i);
    ctx->map_pool_top += (uint32_t)n;
    return 1;
}
static int pv_map_span_from_pool(pv_ctx *ctx, uint32_t off, int32_t len)
{
    uint32_t k = 0; int32_t i;
    for (i = 0; i < len; i++) pv_arena_put(ctx, &k, ctx->map_pool[off + (uint32_t)i]);
    return pv_arena_finish(ctx, k);
}
static int pv_map_key_eq_span(pv_ctx *ctx, int e, int span_h)
{
    int32_t n = pv_span_n(ctx, span_h), i; uint32_t p = pv_span_p(ctx, span_h);
    if (ctx->me_kk[e] != 1 || ctx->me_klen[e] != n) return 0;
    for (i = 0; i < n; i++) if (ctx->map_pool[ctx->me_koff[e] + (uint32_t)i] != pv_arena_get(ctx, p + (uint32_t)i)) return 0;
    return 1;
}
static int pv_map_find_i(pv_ctx *ctx, int mi, int32_t k)
{
    int e = ctx->map_head[mi];
    while (e >= 0) { if (ctx->me_kk[e] == 0 && ctx->me_ki[e] == k) return e; e = ctx->me_next[e]; }
    return -1;
}
static int pv_map_find_s(pv_ctx *ctx, int mi, int span_h)
{
    int e = ctx->map_head[mi];
    while (e >= 0) { if (pv_map_key_eq_span(ctx, e, span_h)) return e; e = ctx->me_next[e]; }
    return -1;
}
static int pv_map_new_entry(pv_ctx *ctx, int mi)
{
    int e;
    if (ctx->me_count >= PV_MAX_MAP_ENTRIES) return -1;
    e = ctx->me_count++;
    ctx->me_next[e] = -1;
    if (ctx->map_tail[mi] >= 0) ctx->me_next[ctx->map_tail[mi]] = e;
    else ctx->map_head[mi] = e;
    ctx->map_tail[mi] = e;
    ctx->map_count[mi]++;
    return e;
}
static int pv_map_entry_at(pv_ctx *ctx, int mi, int32_t idx)
{
    int e = ctx->map_head[mi]; int32_t i = 0;
    while (e >= 0 && i < idx) { e = ctx->me_next[e]; i++; }
    return (i == idx) ? e : -1;
}
static void pv_map_unlink(pv_ctx *ctx, int mi, int match_e)
{
    int prev = -1, e = ctx->map_head[mi];
    while (e >= 0) {
        if (e == match_e) {
            int nx = ctx->me_next[e];
            if (prev < 0) ctx->map_head[mi] = nx; else ctx->me_next[prev] = nx;
            if (ctx->map_tail[mi] == e) ctx->map_tail[mi] = prev;
            ctx->map_count[mi]--; return;
        }
        prev = e; e = ctx->me_next[e];
    }
}
static int pv_map_hook(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    int mi = ctx->map_active;
    int active_ok = (mi > 0 && mi < ctx->map_nmaps && ctx->map_used[mi]);
    switch (hook) {
    case PV_HOOK_MAP_NEW: {
        int h;
        if (ctx->map_nmaps >= PV_MAX_MAPS) { ctx->regs[rd] = 0; return 1; }
        h = ctx->map_nmaps++;
        ctx->map_used[h] = 1; ctx->map_head[h] = -1; ctx->map_tail[h] = -1; ctx->map_count[h] = 0;
        ctx->map_active = h; ctx->regs[rd] = h; return 1;
    }
    case PV_HOOK_MAP_USE: {
        int h = ctx->regs[rs1];
        ctx->map_active = (h > 0 && h < ctx->map_nmaps && ctx->map_used[h]) ? h : 0; return 1;
    }
    case PV_HOOK_MAP_FREE: {
        int h = ctx->regs[rs1];
        if (h > 0 && h < ctx->map_nmaps) { ctx->map_used[h] = 0; ctx->map_head[h] = -1; ctx->map_tail[h] = -1; ctx->map_count[h] = 0; if (ctx->map_active == h) ctx->map_active = 0; }
        return 1;
    }
    case PV_HOOK_MAP_HASH: ctx->regs[rd] = (int32_t)pv_fnv1a_span(ctx, ctx->regs[rs1]); return 1;
    default: break;
    }
    if (!active_ok) {
        switch (hook) {
        case PV_HOOK_MAP_GETIS: case PV_HOOK_MAP_GETSS: case PV_HOOK_MAP_KEYSPANAT:
        case PV_HOOK_MAP_VALSPANAT: ctx->regs[rd] = pv_map_span_from_pool(ctx, 0, 0); return 1;
        default: ctx->regs[rd] = 0; return 1;   /* reads 0, writes no-op */
        }
    }
    switch (hook) {
    case PV_HOOK_MAP_CLEAR: ctx->map_head[mi] = -1; ctx->map_tail[mi] = -1; ctx->map_count[mi] = 0; return 1;
    case PV_HOOK_MAP_COUNT: ctx->regs[rd] = ctx->map_count[mi]; return 1;
    case PV_HOOK_MAP_PUTII: {
        int32_t k = ctx->regs[rs1]; int e = pv_map_find_i(ctx, mi, k);
        if (e < 0) { e = pv_map_new_entry(ctx, mi); if (e < 0) return 1; ctx->me_kk[e] = 0; ctx->me_ki[e] = k; }
        ctx->me_vk[e] = 0; ctx->me_vi[e] = ctx->regs[rs2]; return 1;
    }
    case PV_HOOK_MAP_GETII: {
        int e = pv_map_find_i(ctx, mi, ctx->regs[rs1]);
        ctx->regs[rd] = (e >= 0 && ctx->me_vk[e] == 0) ? ctx->me_vi[e] : 0;
        ctx->host_status = (e >= 0) ? 0 : 1; return 1;
    }
    case PV_HOOK_MAP_HASI: ctx->regs[rd] = (pv_map_find_i(ctx, mi, ctx->regs[rs1]) >= 0) ? 1 : 0; return 1;
    case PV_HOOK_MAP_DELI: { int e = pv_map_find_i(ctx, mi, ctx->regs[rs1]); if (e >= 0) pv_map_unlink(ctx, mi, e); return 1; }
    case PV_HOOK_MAP_PUTIS: {
        int32_t k = ctx->regs[rs1]; uint32_t off; int32_t len;
        if (!pv_map_intern(ctx, ctx->regs[rs2], &off, &len)) return 1;
        int e = pv_map_find_i(ctx, mi, k);
        if (e < 0) { e = pv_map_new_entry(ctx, mi); if (e < 0) return 1; ctx->me_kk[e] = 0; ctx->me_ki[e] = k; }
        ctx->me_vk[e] = 1; ctx->me_voff[e] = off; ctx->me_vlen[e] = len; return 1;
    }
    case PV_HOOK_MAP_GETIS: {
        int e = pv_map_find_i(ctx, mi, ctx->regs[rs1]);
        if (e >= 0 && ctx->me_vk[e] == 1) { ctx->regs[rd] = pv_map_span_from_pool(ctx, ctx->me_voff[e], ctx->me_vlen[e]); ctx->host_status = 0; }
        else { ctx->regs[rd] = pv_map_span_from_pool(ctx, 0, 0); ctx->host_status = 1; }
        return 1;
    }
    case PV_HOOK_MAP_PUTNULLI: {
        int32_t k = ctx->regs[rs1]; int e = pv_map_find_i(ctx, mi, k);
        if (e < 0) { e = pv_map_new_entry(ctx, mi); if (e < 0) return 1; ctx->me_kk[e] = 0; ctx->me_ki[e] = k; }
        ctx->me_vk[e] = 2; return 1;
    }
    case PV_HOOK_MAP_ISNULLI: { int e = pv_map_find_i(ctx, mi, ctx->regs[rs1]); ctx->regs[rd] = (e >= 0 && ctx->me_vk[e] == 2) ? 1 : 0; return 1; }
    case PV_HOOK_MAP_PUTSI: {
        int e = pv_map_find_s(ctx, mi, ctx->regs[rs1]);
        if (e < 0) { uint32_t ko; int32_t kl; if (!pv_map_intern(ctx, ctx->regs[rs1], &ko, &kl)) return 1; e = pv_map_new_entry(ctx, mi); if (e < 0) return 1; ctx->me_kk[e] = 1; ctx->me_koff[e] = ko; ctx->me_klen[e] = kl; }
        ctx->me_vk[e] = 0; ctx->me_vi[e] = ctx->regs[rs2]; return 1;
    }
    case PV_HOOK_MAP_GETSI: {
        int e = pv_map_find_s(ctx, mi, ctx->regs[rs1]);
        ctx->regs[rd] = (e >= 0 && ctx->me_vk[e] == 0) ? ctx->me_vi[e] : 0;
        ctx->host_status = (e >= 0) ? 0 : 1; return 1;
    }
    case PV_HOOK_MAP_HASS: ctx->regs[rd] = (pv_map_find_s(ctx, mi, ctx->regs[rs1]) >= 0) ? 1 : 0; return 1;
    case PV_HOOK_MAP_DELS: { int e = pv_map_find_s(ctx, mi, ctx->regs[rs1]); if (e >= 0) pv_map_unlink(ctx, mi, e); return 1; }
    case PV_HOOK_MAP_PUTSS: {
        int e = pv_map_find_s(ctx, mi, ctx->regs[rs1]);
        if (e < 0) { uint32_t ko; int32_t kl; if (!pv_map_intern(ctx, ctx->regs[rs1], &ko, &kl)) return 1; e = pv_map_new_entry(ctx, mi); if (e < 0) return 1; ctx->me_kk[e] = 1; ctx->me_koff[e] = ko; ctx->me_klen[e] = kl; }
        { uint32_t vo; int32_t vl; if (!pv_map_intern(ctx, ctx->regs[rs2], &vo, &vl)) return 1; ctx->me_vk[e] = 1; ctx->me_voff[e] = vo; ctx->me_vlen[e] = vl; }
        return 1;
    }
    case PV_HOOK_MAP_GETSS: {
        int e = pv_map_find_s(ctx, mi, ctx->regs[rs1]);
        if (e >= 0 && ctx->me_vk[e] == 1) { ctx->regs[rd] = pv_map_span_from_pool(ctx, ctx->me_voff[e], ctx->me_vlen[e]); ctx->host_status = 0; }
        else { ctx->regs[rd] = pv_map_span_from_pool(ctx, 0, 0); ctx->host_status = 1; }
        return 1;
    }
    case PV_HOOK_MAP_PUTNULLS: {
        int e = pv_map_find_s(ctx, mi, ctx->regs[rs1]);
        if (e < 0) { uint32_t ko; int32_t kl; if (!pv_map_intern(ctx, ctx->regs[rs1], &ko, &kl)) return 1; e = pv_map_new_entry(ctx, mi); if (e < 0) return 1; ctx->me_kk[e] = 1; ctx->me_koff[e] = ko; ctx->me_klen[e] = kl; }
        ctx->me_vk[e] = 2; return 1;
    }
    case PV_HOOK_MAP_ISNULLS: { int e = pv_map_find_s(ctx, mi, ctx->regs[rs1]); ctx->regs[rd] = (e >= 0 && ctx->me_vk[e] == 2) ? 1 : 0; return 1; }
    case PV_HOOK_MAP_KEYAT: { int e = pv_map_entry_at(ctx, mi, ctx->regs[rs1]); ctx->regs[rd] = (e >= 0 && ctx->me_kk[e] == 0) ? ctx->me_ki[e] : 0; return 1; }
    case PV_HOOK_MAP_KEYSPANAT: { int e = pv_map_entry_at(ctx, mi, ctx->regs[rs1]); ctx->regs[rd] = (e >= 0 && ctx->me_kk[e] == 1) ? pv_map_span_from_pool(ctx, ctx->me_koff[e], ctx->me_klen[e]) : pv_map_span_from_pool(ctx, 0, 0); return 1; }
    case PV_HOOK_MAP_VALAT: { int e = pv_map_entry_at(ctx, mi, ctx->regs[rs1]); ctx->regs[rd] = (e >= 0 && ctx->me_vk[e] == 0) ? ctx->me_vi[e] : 0; return 1; }
    case PV_HOOK_MAP_VALSPANAT: { int e = pv_map_entry_at(ctx, mi, ctx->regs[rs1]); ctx->regs[rd] = (e >= 0 && ctx->me_vk[e] == 1) ? pv_map_span_from_pool(ctx, ctx->me_voff[e], ctx->me_vlen[e]) : pv_map_span_from_pool(ctx, 0, 0); return 1; }
    case PV_HOOK_MAP_VALISSPAN: { int e = pv_map_entry_at(ctx, mi, ctx->regs[rs1]); ctx->regs[rd] = (e >= 0 && ctx->me_vk[e] == 1) ? 1 : 0; return 1; }
    default: return 0;
    }
}

/* ---- parsers: string/bytes -> Map (mirror picovm.js/.py; docs/MAP.md) ------
 * Byte scanners identical to the JS/Python reference VMs so a parsed Map is
 * bit-identical. Keys/values intern into map_pool; nested JSON is captured raw. */
static int pv_pool_put(pv_ctx *ctx, uint8_t b)
{
    if (ctx->map_pool_top >= PV_MAP_POOL) return 0;
    ctx->map_pool[ctx->map_pool_top++] = b; return 1;
}
static int pv_hexv_c(uint8_t c) { if (c >= 48 && c <= 57) return c - 48; if (c >= 97 && c <= 102) return c - 87; if (c >= 65 && c <= 70) return c - 55; return 0; }
static void pv_pool_utf8(pv_ctx *ctx, int cp)
{
    if (cp < 0x80) pv_pool_put(ctx, (uint8_t)cp);
    else if (cp < 0x800) { pv_pool_put(ctx, (uint8_t)(0xC0 | (cp >> 6))); pv_pool_put(ctx, (uint8_t)(0x80 | (cp & 0x3F))); }
    else { pv_pool_put(ctx, (uint8_t)(0xE0 | (cp >> 12))); pv_pool_put(ctx, (uint8_t)(0x80 | ((cp >> 6) & 0x3F))); pv_pool_put(ctx, (uint8_t)(0x80 | (cp & 0x3F))); }
}
static int pv_map_find_s_bytes(pv_ctx *ctx, int mi, uint32_t off, int32_t len)
{
    int e = ctx->map_head[mi], i;
    while (e >= 0) {
        if (ctx->me_kk[e] == 1 && ctx->me_klen[e] == len) {
            int eq = 1;
            for (i = 0; i < len; i++) if (ctx->map_pool[ctx->me_koff[e] + (uint32_t)i] != ctx->map_pool[off + (uint32_t)i]) { eq = 0; break; }
            if (eq) return e;
        }
        e = ctx->me_next[e];
    }
    return -1;
}
static void pv_map_put_s_bytes(pv_ctx *ctx, int mi, uint32_t koff, int32_t klen, uint8_t vk, int32_t vi, uint32_t voff, int32_t vlen)
{
    int e = pv_map_find_s_bytes(ctx, mi, koff, klen);
    if (e < 0) { e = pv_map_new_entry(ctx, mi); if (e < 0) return; ctx->me_kk[e] = 1; ctx->me_koff[e] = koff; ctx->me_klen[e] = klen; }
    ctx->me_vk[e] = vk; ctx->me_vi[e] = vi; ctx->me_voff[e] = voff; ctx->me_vlen[e] = vlen;
}
static int pv_new_active_map(pv_ctx *ctx)
{
    int h;
    if (ctx->map_nmaps >= PV_MAX_MAPS) return 0;
    h = ctx->map_nmaps++;
    ctx->map_used[h] = 1; ctx->map_head[h] = -1; ctx->map_tail[h] = -1; ctx->map_count[h] = 0;
    ctx->map_active = h; return h;
}
/* decode a JSON string at *i (opening quote) into the pool; set *off/*len. */
static void pv_json_str(pv_ctx *ctx, uint32_t sp, int32_t n, int *i, uint32_t *off, int32_t *len)
{
    int p = *i + 1; uint32_t start = ctx->map_pool_top;
    while (p < n) {
        uint8_t c = pv_arena_get(ctx, sp + (uint32_t)p); p++;
        if (c == 34) break;
        if (c == 92) {
            uint8_t e = pv_arena_get(ctx, sp + (uint32_t)p); p++;
            if (e == 34) pv_pool_put(ctx, 34); else if (e == 92) pv_pool_put(ctx, 92); else if (e == 47) pv_pool_put(ctx, 47);
            else if (e == 110) pv_pool_put(ctx, 10); else if (e == 116) pv_pool_put(ctx, 9); else if (e == 114) pv_pool_put(ctx, 13);
            else if (e == 98) pv_pool_put(ctx, 8); else if (e == 102) pv_pool_put(ctx, 12);
            else if (e == 117) { int cp = 0, k; for (k = 0; k < 4; k++) { cp = (cp << 4) | pv_hexv_c(pv_arena_get(ctx, sp + (uint32_t)p)); p++; } pv_pool_utf8(ctx, cp); }
            else pv_pool_put(ctx, e);
        } else pv_pool_put(ctx, c);
    }
    *off = start; *len = (int32_t)(ctx->map_pool_top - start); *i = p;
}
static void pv_json_parse(pv_ctx *ctx, uint32_t sp, int32_t n, int mi)
{
    int i = 0;
#define JIN(k) pv_arena_get(ctx, sp + (uint32_t)(k))
#define JWS(c) ((c) == 32 || (c) == 9 || (c) == 10 || (c) == 13)
    while (i < n && JWS(JIN(i))) i++;
    if (i >= n || JIN(i) != 123) return; i++;
    while (i < n) {
        uint32_t koff; int32_t klen; uint8_t c;
        while (i < n && JWS(JIN(i))) i++;
        if (i < n && JIN(i) == 125) { i++; break; }
        if (i >= n || JIN(i) != 34) break;
        pv_json_str(ctx, sp, n, &i, &koff, &klen);
        while (i < n && JWS(JIN(i))) i++;
        if (i >= n || JIN(i) != 58) break; i++;
        while (i < n && JWS(JIN(i))) i++;
        if (i >= n) break;
        c = JIN(i);
        if (c == 34) { uint32_t vo; int32_t vl; pv_json_str(ctx, sp, n, &i, &vo, &vl); pv_map_put_s_bytes(ctx, mi, koff, klen, 1, 0, vo, vl); }
        else if (c == 123 || c == 91) {
            int start = i, depth = 0, instr = 0, k; uint32_t vo; int32_t vl;
            while (i < n) { uint8_t d = JIN(i);
                if (instr) { if (d == 92) { i += 2; continue; } if (d == 34) instr = 0; i++; continue; }
                if (d == 34) { instr = 1; i++; continue; }
                if (d == 123 || d == 91) depth++;
                else if (d == 125 || d == 93) { depth--; if (depth == 0) { i++; break; } }
                i++;
            }
            vo = ctx->map_pool_top; for (k = start; k < i; k++) pv_pool_put(ctx, JIN(k));
            vl = (int32_t)(ctx->map_pool_top - vo);
            pv_map_put_s_bytes(ctx, mi, koff, klen, 1, 0, vo, vl);
        }
        else if (c == 116) { i += 4; pv_map_put_s_bytes(ctx, mi, koff, klen, 0, 1, 0, 0); }
        else if (c == 102) { i += 5; pv_map_put_s_bytes(ctx, mi, koff, klen, 0, 0, 0, 0); }
        else if (c == 110) { i += 4; pv_map_put_s_bytes(ctx, mi, koff, klen, 2, 0, 0, 0); }
        else {
            int neg = (JIN(i) == 45); int32_t val = 0;
            if (JIN(i) == 45 || JIN(i) == 43) i++;
            while (i < n && JIN(i) >= 48 && JIN(i) <= 57) { val = val * 10 + (JIN(i) - 48); i++; }
            if (i < n && (JIN(i) == 46 || JIN(i) == 101 || JIN(i) == 69)) { i++; while (i < n) { uint8_t d = JIN(i); if ((d >= 48 && d <= 57) || d == 46 || d == 101 || d == 69 || d == 45 || d == 43) i++; else break; } }
            pv_map_put_s_bytes(ctx, mi, koff, klen, 0, neg ? -val : val, 0, 0);
        }
        while (i < n && JWS(JIN(i))) i++;
        if (i < n && JIN(i) == 44) { i++; continue; }
        if (i < n && JIN(i) == 125) { i++; break; }
        break;
    }
#undef JIN
#undef JWS
}
static void pv_psc1_parse(pv_ctx *ctx, uint32_t sp, int32_t n, int mi)
{
    uint32_t magic; int count, pos, c;
    if (n < 6) return;
    magic = ((uint32_t)pv_arena_get(ctx, sp) << 24) | ((uint32_t)pv_arena_get(ctx, sp + 1) << 16) | ((uint32_t)pv_arena_get(ctx, sp + 2) << 8) | pv_arena_get(ctx, sp + 3);
    if (magic != 0x50534331u) return;
    count = (pv_arena_get(ctx, sp + 4) << 8) | pv_arena_get(ctx, sp + 5); pos = 6;
    for (c = 0; c < count; c++) {
        int nlen = pv_arena_get(ctx, sp + (uint32_t)pos), k, t; uint32_t koff;
        pos++;
        koff = ctx->map_pool_top;
        for (k = 0; k < nlen; k++) pv_pool_put(ctx, pv_arena_get(ctx, sp + (uint32_t)(pos + k)));
        pos += nlen;
        t = pv_arena_get(ctx, sp + (uint32_t)pos); pos++;
        if (t == 1) {
            int32_t x = (int32_t)(((uint32_t)pv_arena_get(ctx, sp + (uint32_t)pos) << 24) | ((uint32_t)pv_arena_get(ctx, sp + (uint32_t)(pos + 1)) << 16) | ((uint32_t)pv_arena_get(ctx, sp + (uint32_t)(pos + 2)) << 8) | pv_arena_get(ctx, sp + (uint32_t)(pos + 3)));
            pos += 4; pv_map_put_s_bytes(ctx, mi, koff, (int32_t)nlen, 0, x, 0, 0);
        } else if (t == 2) {
            int vlen = (pv_arena_get(ctx, sp + (uint32_t)pos) << 8) | pv_arena_get(ctx, sp + (uint32_t)(pos + 1)), j; uint32_t voff;
            pos += 2; voff = ctx->map_pool_top;
            for (j = 0; j < vlen; j++) pv_pool_put(ctx, pv_arena_get(ctx, sp + (uint32_t)(pos + j)));
            pos += vlen; pv_map_put_s_bytes(ctx, mi, koff, (int32_t)nlen, 1, 0, voff, (int32_t)vlen);
        } else break;
    }
}
static int pv_key_cmp(pv_ctx *ctx, int a, int b)
{
    int32_t la = ctx->me_klen[a], lb = ctx->me_klen[b], m = la < lb ? la : lb, i;
    for (i = 0; i < m; i++) { int d = ctx->map_pool[ctx->me_koff[a] + (uint32_t)i] - ctx->map_pool[ctx->me_koff[b] + (uint32_t)i]; if (d) return d; }
    return la - lb;
}
static int pv_psc1_serialize(pv_ctx *ctx, int mi)
{
    int idx[PV_MAX_MAP_ENTRIES], cnt = 0, e, i, j; uint32_t k = 0;
    for (e = ctx->map_head[mi]; e >= 0; e = ctx->me_next[e]) if (ctx->me_kk[e] == 1) idx[cnt++] = e;
    for (i = 1; i < cnt; i++) { int v = idx[i]; j = i - 1; while (j >= 0 && pv_key_cmp(ctx, idx[j], v) > 0) { idx[j + 1] = idx[j]; j--; } idx[j + 1] = v; }
    pv_arena_put(ctx, &k, 0x50); pv_arena_put(ctx, &k, 0x53); pv_arena_put(ctx, &k, 0x43); pv_arena_put(ctx, &k, 0x31);
    pv_arena_put(ctx, &k, (uint8_t)((cnt >> 8) & 255)); pv_arena_put(ctx, &k, (uint8_t)(cnt & 255));
    for (i = 0; i < cnt; i++) {
        e = idx[i];
        pv_arena_put(ctx, &k, (uint8_t)(ctx->me_klen[e] & 255));
        for (j = 0; j < ctx->me_klen[e]; j++) pv_arena_put(ctx, &k, ctx->map_pool[ctx->me_koff[e] + (uint32_t)j]);
        if (ctx->me_vk[e] == 1) {
            int32_t vl = ctx->me_vlen[e];
            pv_arena_put(ctx, &k, 2); pv_arena_put(ctx, &k, (uint8_t)((vl >> 8) & 255)); pv_arena_put(ctx, &k, (uint8_t)(vl & 255));
            for (j = 0; j < vl; j++) pv_arena_put(ctx, &k, ctx->map_pool[ctx->me_voff[e] + (uint32_t)j]);
        } else {
            int32_t x = (ctx->me_vk[e] == 0 ? ctx->me_vi[e] : 0);
            pv_arena_put(ctx, &k, 1); pv_arena_put(ctx, &k, (uint8_t)((x >> 24) & 255)); pv_arena_put(ctx, &k, (uint8_t)((x >> 16) & 255)); pv_arena_put(ctx, &k, (uint8_t)((x >> 8) & 255)); pv_arena_put(ctx, &k, (uint8_t)(x & 255));
        }
    }
    return pv_arena_finish(ctx, k);
}
/* ---- BSO1 (BareMetal.Binary): schema-driven, LE, HMAC-SHA256 signed ---------
 * Wire-compatible with BareMetalJsTools/src/BareMetal.Binary.js. 64-bit/float/
 * temporal values stored as raw LE bytes (string) in the Map. */
static const int PV_BSO1_SZ[23] = { 0, 1, 1, 1, 2, 2, 4, 4, 8, 8, 4, 8, 16, 2, 0, 16, 9, 4, 8, 10, 8, 16, 4 };
static int pv_bso1_intkind(int wt) {  /* 0=raw/string, 1=uint, 2=signed */
    switch (wt) { case 1: case 2: case 5: case 7: case 13: return 1;
                  case 3: case 4: case 6: case 17: case 22: return 2; }
    return 0;
}
static int32_t pv_le_read(pv_ctx *ctx, uint32_t sp, int pos, int size, int signd) {
    uint32_t v = 0; int i;
    for (i = 0; i < size; i++) v |= (uint32_t)pv_arena_get(ctx, sp + (uint32_t)(pos + i)) << (8 * i);
    if (signd && size < 4) { uint32_t lim = 1u << (size * 8 - 1); if (v >= lim) v -= (lim << 1); }
    return (int32_t)v;
}
static void pv_hmac_arena2(pv_ctx *ctx, const uint8_t *key, int keylen, uint32_t sp, int o1, int l1, int o2, int l2, uint8_t out[32]) {
    uint8_t k[64], ipad[64], opad[64], inner[32]; int i;
    for (i = 0; i < 64; i++) k[i] = 0;
    for (i = 0; i < keylen && i < 64; i++) k[i] = key[i];
    for (i = 0; i < 64; i++) { ipad[i] = (uint8_t)(k[i] ^ 0x36); opad[i] = (uint8_t)(k[i] ^ 0x5c); }
    { pv_sha256_stream s; pv_sha256_s_init(&s);
      for (i = 0; i < 64; i++) pv_sha256_s_push(&s, ipad[i]);
      for (i = 0; i < l1; i++) pv_sha256_s_push(&s, pv_arena_get(ctx, sp + (uint32_t)(o1 + i)));
      for (i = 0; i < l2; i++) pv_sha256_s_push(&s, pv_arena_get(ctx, sp + (uint32_t)(o2 + i)));
      pv_sha256_s_final(&s, inner); }
    { pv_sha256_stream s2; pv_sha256_s_init(&s2);
      for (i = 0; i < 64; i++) pv_sha256_s_push(&s2, opad[i]);
      for (i = 0; i < 32; i++) pv_sha256_s_push(&s2, inner[i]);
      pv_sha256_s_final(&s2, out); }
}
static int pv_bso1_is_pseudo(pv_ctx *ctx, int e) { return (ctx->me_klen[e] > 0 && ctx->map_pool[ctx->me_koff[e]] == ':'); }
static int pv_bso1_version(pv_ctx *ctx, int smi) {
    static const char VER[8] = { ':', 'v', 'e', 'r', 's', 'i', 'o', 'n' };
    int e, i, ok;
    for (e = ctx->map_head[smi]; e >= 0; e = ctx->me_next[e]) {
        if (ctx->me_klen[e] != 8 || ctx->me_kk[e] != 1) continue;
        ok = 1; for (i = 0; i < 8; i++) if (ctx->map_pool[ctx->me_koff[e] + (uint32_t)i] != (uint8_t)VER[i]) { ok = 0; break; }
        if (ok && ctx->me_vk[e] == 0) return ctx->me_vi[e];
    }
    return 1;
}
static void pv_bso1_read(pv_ctx *ctx, uint32_t sp, int32_t n, int smi, int rmi) {
    int pos = 45, e, ik;
    uint8_t present;
    if (pos >= n) return;
    present = pv_arena_get(ctx, sp + (uint32_t)pos); pos++;
    if (present == 0) return;
    for (e = ctx->map_head[smi]; e >= 0; e = ctx->me_next[e]) {
        int code, wt, nullable; uint32_t koff; int32_t klen;
        if (pv_bso1_is_pseudo(ctx, e)) continue;
        code = (ctx->me_vk[e] == 0) ? ctx->me_vi[e] : 0;
        wt = code & 0xFF; nullable = (code & 0x100) != 0;
        koff = ctx->me_koff[e]; klen = ctx->me_klen[e];
        if (nullable) { if (pv_arena_get(ctx, sp + (uint32_t)pos) == 0) { pos++; pv_map_put_s_bytes(ctx, rmi, koff, klen, 2, 0, 0, 0); continue; } pos++; }
        if (wt == 14) {
            int32_t len = pv_le_read(ctx, sp, pos, 4, 1); pos += 4;
            if (len < 0) pv_map_put_s_bytes(ctx, rmi, koff, klen, 2, 0, 0, 0);
            else { uint32_t vo = ctx->map_pool_top; int k; for (k = 0; k < len; k++) pv_pool_put(ctx, pv_arena_get(ctx, sp + (uint32_t)(pos + k))); pos += len; pv_map_put_s_bytes(ctx, rmi, koff, klen, 1, 0, vo, len); }
        } else if ((ik = pv_bso1_intkind(wt)) != 0) {
            int sz = PV_BSO1_SZ[wt]; int32_t val = pv_le_read(ctx, sp, pos, sz, ik == 2); pos += sz;
            pv_map_put_s_bytes(ctx, rmi, koff, klen, 0, val, 0, 0);
        } else {
            int sz = PV_BSO1_SZ[wt], k; uint32_t vo = ctx->map_pool_top;
            for (k = 0; k < sz; k++) pv_pool_put(ctx, pv_arena_get(ctx, sp + (uint32_t)(pos + k))); pos += sz;
            pv_map_put_s_bytes(ctx, rmi, koff, klen, 1, 0, vo, sz);
        }
    }
}
static int pv_bso1_write(pv_ctx *ctx, int dmi, int smi) {
    uint32_t kk = 0, magic = 0x314F5342u; int e, ver = pv_bso1_version(ctx, smi), i, h; uint32_t sp; int32_t n;
    pv_arena_put(ctx, &kk, (uint8_t)(magic & 255)); pv_arena_put(ctx, &kk, (uint8_t)((magic >> 8) & 255)); pv_arena_put(ctx, &kk, (uint8_t)((magic >> 16) & 255)); pv_arena_put(ctx, &kk, (uint8_t)((magic >> 24) & 255));
    pv_arena_put(ctx, &kk, 3); pv_arena_put(ctx, &kk, 0); pv_arena_put(ctx, &kk, 0); pv_arena_put(ctx, &kk, 0);
    pv_arena_put(ctx, &kk, (uint8_t)(ver & 255)); pv_arena_put(ctx, &kk, (uint8_t)((ver >> 8) & 255)); pv_arena_put(ctx, &kk, (uint8_t)((ver >> 16) & 255)); pv_arena_put(ctx, &kk, (uint8_t)((ver >> 24) & 255));
    pv_arena_put(ctx, &kk, 0);
    for (i = 0; i < 32; i++) pv_arena_put(ctx, &kk, 0);
    pv_arena_put(ctx, &kk, 1);
    for (e = ctx->map_head[smi]; e >= 0; e = ctx->me_next[e]) {
        int code, wt, nullable, ik, de, is_null;
        if (pv_bso1_is_pseudo(ctx, e)) continue;
        code = (ctx->me_vk[e] == 0) ? ctx->me_vi[e] : 0; wt = code & 0xFF; nullable = (code & 0x100) != 0;
        de = pv_map_find_s_bytes(ctx, dmi, ctx->me_koff[e], ctx->me_klen[e]);
        is_null = (de < 0) || (ctx->me_vk[de] == 2);
        if (nullable) { if (is_null) { pv_arena_put(ctx, &kk, 0); continue; } pv_arena_put(ctx, &kk, 1); }
        if (wt == 14) {
            if (is_null) { pv_arena_put(ctx, &kk, 255); pv_arena_put(ctx, &kk, 255); pv_arena_put(ctx, &kk, 255); pv_arena_put(ctx, &kk, 255); }
            else { int32_t len = (ctx->me_vk[de] == 1) ? ctx->me_vlen[de] : 0, j;
                pv_arena_put(ctx, &kk, (uint8_t)(len & 255)); pv_arena_put(ctx, &kk, (uint8_t)((len >> 8) & 255)); pv_arena_put(ctx, &kk, (uint8_t)((len >> 16) & 255)); pv_arena_put(ctx, &kk, (uint8_t)((len >> 24) & 255));
                for (j = 0; j < len; j++) pv_arena_put(ctx, &kk, ctx->map_pool[ctx->me_voff[de] + (uint32_t)j]); }
        } else if ((ik = pv_bso1_intkind(wt)) != 0) {
            int sz = PV_BSO1_SZ[wt], j; int32_t val = (de >= 0 && ctx->me_vk[de] == 0) ? ctx->me_vi[de] : 0;
            for (j = 0; j < sz; j++) pv_arena_put(ctx, &kk, (uint8_t)((val >> (8 * j)) & 255));
        } else {
            int sz = PV_BSO1_SZ[wt], j;
            for (j = 0; j < sz; j++) { uint8_t bb = (de >= 0 && ctx->me_vk[de] == 1 && j < ctx->me_vlen[de]) ? ctx->map_pool[ctx->me_voff[de] + (uint32_t)j] : 0; pv_arena_put(ctx, &kk, bb); }
        }
    }
    h = pv_arena_finish(ctx, kk); sp = pv_span_p(ctx, h); n = pv_span_n(ctx, h);
    if (ctx->bso1_key_len > 0) { uint8_t sig[32]; pv_hmac_arena2(ctx, ctx->bso1_key, ctx->bso1_key_len, sp, 0, 13, 45, n - 45, sig); for (i = 0; i < 32; i++) ctx->mem[sp + 13 + (uint32_t)i] = sig[i]; }
    return h;
}
static int pv_bso1_verify(pv_ctx *ctx, uint32_t sp, int32_t n) {
    uint8_t sig[32]; int i;
    if (ctx->bso1_key_len <= 0 || n < 45) return 0;
    pv_hmac_arena2(ctx, ctx->bso1_key, ctx->bso1_key_len, sp, 0, 13, 45, n - 45, sig);
    for (i = 0; i < 32; i++) if (pv_arena_get(ctx, sp + (uint32_t)(13 + i)) != sig[i]) return 0;
    return 1;
}
static int pv_parse_hook(pv_ctx *ctx, int hook, int rd, int rs1, int rs2)
{
    if (hook == PV_HOOK_JSON_PARSE || hook == PV_HOOK_BINARY_PARSECARD) {
        int sh = ctx->regs[rs1]; uint32_t sp = pv_span_p(ctx, sh); int32_t n = pv_span_n(ctx, sh);
        int mi = pv_new_active_map(ctx);
        if (mi == 0) { ctx->regs[rd] = 0; return 1; }
        if (hook == PV_HOOK_JSON_PARSE) pv_json_parse(ctx, sp, n, mi);
        else pv_psc1_parse(ctx, sp, n, mi);
        ctx->regs[rd] = mi; return 1;
    }
    if (hook == PV_HOOK_BINARY_SERIALIZECARD) {
        int mi = ctx->map_active;
        if (mi > 0 && mi < ctx->map_nmaps && ctx->map_used[mi]) ctx->regs[rd] = pv_psc1_serialize(ctx, mi);
        else ctx->regs[rd] = pv_map_span_from_pool(ctx, 0, 0);
        return 1;
    }
    if (hook == PV_HOOK_BINARY_SETKEY) {
        int sh = ctx->regs[rs1]; uint32_t sp = pv_span_p(ctx, sh); int32_t n = pv_span_n(ctx, sh), i;
        if (n > 64) n = 64; if (n < 0) n = 0; ctx->bso1_key_len = n;
        for (i = 0; i < n; i++) ctx->bso1_key[i] = pv_arena_get(ctx, sp + (uint32_t)i);
        return 1;
    }
    if (hook == PV_HOOK_BINARY_PARSEENTITY) {
        int sh = ctx->regs[rs1]; uint32_t sp = pv_span_p(ctx, sh); int32_t n = pv_span_n(ctx, sh);
        int smi = ctx->regs[rs2]; int rmi = pv_new_active_map(ctx);
        if (rmi == 0) { ctx->regs[rd] = 0; return 1; }
        if (smi > 0 && smi < ctx->map_nmaps && ctx->map_used[smi]) pv_bso1_read(ctx, sp, n, smi, rmi);
        ctx->regs[rd] = rmi; return 1;
    }
    if (hook == PV_HOOK_BINARY_SERIALIZEENTITY) {
        int dmi = ctx->regs[rs1], smi = ctx->regs[rs2];
        if (smi > 0 && smi < ctx->map_nmaps && ctx->map_used[smi]) ctx->regs[rd] = pv_bso1_write(ctx, dmi, smi);
        else ctx->regs[rd] = pv_map_span_from_pool(ctx, 0, 0);
        return 1;
    }
    if (hook == PV_HOOK_BINARY_VERIFY) {
        int sh = ctx->regs[rs1]; uint32_t sp = pv_span_p(ctx, sh); int32_t n = pv_span_n(ctx, sh);
        ctx->regs[rd] = pv_bso1_verify(ctx, sp, n); return 1;
    }
    return 0;
}

void pv_default_host(pv_ctx *ctx, int hook, int rd, int rs1, int rs2, int imm16)
{
    (void)imm16;
    /* INV-17: bindings are not ambient -- deny the hook unless its class is granted. */
    uint32_t need = pv_hook_cap(hook);
    if (need && !(ctx->caps & need)) { pv_set_fault(ctx, PV_FAULT_CAPABILITY, ctx->cur_pc, hook); return; }
    /* Map.* first-class dictionary (active-handle model; see docs/MAP.md). */
    if (hook >= PV_HOOK_MAP_NEW && hook <= PV_HOOK_MAP_USE) {
        if (pv_map_hook(ctx, hook, rd, rs1, rs2)) return;
    }
    /* Parsers: Json.Parse / Binary.* (PSC1 card + BSO1 entity) -> Map. */
    if (hook >= PV_HOOK_JSON_PARSE && hook <= PV_HOOK_BINARY_VERIFY) {
        if (pv_parse_hook(ctx, hook, rd, rs1, rs2)) return;
    }
    /* Storage.* (0x60-0x6F) / Search.* card packs (0x1A0-0x1A4): delegate to the
     * app-provided storage backend if one is installed (pv_storage_hook). Lets a
     * native binary compile in its own pack/card store (e.g. file-backed or
     * PicoWAL) without coupling it to the runtime. */
    if (pv_storage_hook &&
        ((hook >= 0x60 && hook <= 0x6F) || (hook >= 0x1A0 && hook <= 0x1A4))) {
        if (pv_storage_hook(ctx, hook, rd, rs1, rs2)) return;
    }
    /* Data.* host-bound read: no active server/data context in the reference
     * runtime, so return a defined empty/0 default (mirrors vm/picovm.js
     * exactly) and let the authoritative host enforce data-dependent rules. */
    if (hook >= PV_HOOK_DATA_LOOKUP && hook <= PV_HOOK_DATA_FIELDSTR) {
        ctx->regs[rd] = (hook == PV_HOOK_DATA_FIELDSTR) ? pv_arena_finish(ctx, 0) : 0;
        return;
    }
    /* Descriptor.*: a pure buffer descriptor (ptr/len/flags handle table),
     * deliberately kept separate from Span.* (Span is the arena-string-
     * library view type; Descriptor adds a host/driver-facing `flags` word).
     * No host state, so real + fully deterministic. Mirrors
     * picoscript_vm.py's _descriptor / vm/picovm.js's _descriptor exactly. */
    if (hook == PV_HOOK_DESCRIPTOR_MAKE) {
        int h;
        if (ctx->desc_count >= PV_MAX_DESCRIPTORS) { ctx->regs[rd] = 0; return; }
        h = ctx->desc_count++;
        ctx->desc_ptr[h] = (uint32_t)ctx->regs[rs1];
        ctx->desc_len[h] = ctx->regs[rs2] < 0 ? 0 : ctx->regs[rs2];
        ctx->desc_flags[h] = 0;
        ctx->desc_used[h] = 1;
        ctx->regs[rd] = h;
        return;
    }
    if (hook >= PV_HOOK_DESCRIPTOR_SETFLAGS && hook <= PV_HOOK_DESCRIPTOR_COPYBATCH) {
        int dh = ctx->regs[rs1];
        int d_ok = (dh > 0 && dh < PV_MAX_DESCRIPTORS && ctx->desc_used[dh]);
        if (hook == PV_HOOK_DESCRIPTOR_SETFLAGS) {
            if (d_ok) ctx->desc_flags[dh] = (uint32_t)ctx->regs[rs2];
            ctx->regs[rd] = d_ok ? 1 : 0;
            return;
        }
        if (hook == PV_HOOK_DESCRIPTOR_GETPTR) { ctx->regs[rd] = d_ok ? (int32_t)ctx->desc_ptr[dh] : 0; return; }
        if (hook == PV_HOOK_DESCRIPTOR_GETLEN) { ctx->regs[rd] = d_ok ? ctx->desc_len[dh] : 0; return; }
        if (hook == PV_HOOK_DESCRIPTOR_GETFLAGS) { ctx->regs[rd] = d_ok ? (int32_t)ctx->desc_flags[dh] : 0; return; }
        if (hook == PV_HOOK_DESCRIPTOR_COPYBATCH) {
            int dh2 = ctx->regs[rs2];
            int d2_ok = (dh2 > 0 && dh2 < PV_MAX_DESCRIPTORS && ctx->desc_used[dh2]);
            int32_t n, i;
            if (!d_ok || !d2_ok) { ctx->regs[rd] = 0; return; }
            n = ctx->desc_len[dh] < ctx->desc_len[dh2] ? ctx->desc_len[dh] : ctx->desc_len[dh2];
            for (i = 0; i < n; i++) {
                uint32_t sp = ctx->desc_ptr[dh] + (uint32_t)i, dp = ctx->desc_ptr[dh2] + (uint32_t)i;
                if (ctx->mem && sp < (uint32_t)ctx->mem_size && dp < (uint32_t)ctx->mem_size) ctx->mem[dp] = ctx->mem[sp];
            }
            ctx->regs[rd] = n;
            return;
        }
    }
    /* Lease.*: a generic capability/ownership token over a span + type hint.
     * Pure in-VM bookkeeping, distinct from Stream.Next's own unrelated
     * internal per-frame lease concept. Mirrors picoscript_vm.py's
     * _lease_ns / vm/picovm.js's _leaseNs exactly. */
    if (hook == PV_HOOK_LEASE_ACQUIRE) {
        int h;
        if (ctx->lease_count >= PV_MAX_LEASES) { ctx->regs[rd] = 0; return; }
        h = ctx->lease_count++;
        ctx->lease_span[h] = ctx->regs[rs1];
        ctx->lease_type[h] = ctx->regs[rs2];
        ctx->lease_valid[h] = 1;
        ctx->lease_used[h] = 1;
        ctx->regs[rd] = h;
        return;
    }
    if (hook >= PV_HOOK_LEASE_RELEASE && hook <= PV_HOOK_LEASE_GETTYPEHINT) {
        int lh = ctx->regs[rs1];
        int l_ok = (lh > 0 && lh < PV_MAX_LEASES && ctx->lease_used[lh] && ctx->lease_valid[lh]);
        int l_exists = (lh > 0 && lh < PV_MAX_LEASES && ctx->lease_used[lh]);
        if (hook == PV_HOOK_LEASE_RELEASE) {
            if (l_exists) ctx->lease_valid[lh] = 0;
            ctx->regs[rd] = l_exists ? 1 : 0;
            return;
        }
        if (hook == PV_HOOK_LEASE_VALIDATE || hook == PV_HOOK_LEASE_CACHEDVALIDATE) {
            /* CachedValidate is a host-optimization hint; the reference VM
             * has no cache to distinguish, so both give the same answer. */
            ctx->regs[rd] = l_ok ? 1 : 0;
            ctx->host_status = l_ok ? 0 : 1;
            return;
        }
        if (hook == PV_HOOK_LEASE_GETSPAN) { ctx->regs[rd] = l_ok ? ctx->lease_span[lh] : pv_arena_finish(ctx, 0); return; }
        if (hook == PV_HOOK_LEASE_GETTYPEHINT) { ctx->regs[rd] = l_ok ? ctx->lease_type[lh] : 0; return; }
    }
    /* Fifo.*: independent named byte-channel FIFOs (Open returns a fresh
     * channel handle). Distinct from Queue.* (fixed 8-channel int FIFO).
     * Mirrors picoscript_vm.py's _fifo / vm/picovm.js's _fifo exactly. */
    if (hook == PV_HOOK_FIFO_OPEN) {
        int h;
        if (ctx->fifo_count >= PV_MAX_FIFOS) { ctx->regs[rd] = 0; return; }
        h = ctx->fifo_count++;
        ctx->fifo_head[h] = 0; ctx->fifo_tail[h] = 0; ctx->fifo_depth[h] = 0;
        ctx->fifo_used[h] = 1;
        ctx->regs[rd] = h;
        return;
    }
    if (hook >= PV_HOOK_FIFO_SEND && hook <= PV_HOOK_FIFO_POLL) {
        int fh = ctx->regs[rs1];
        int f_ok = (fh > 0 && fh < PV_MAX_FIFOS && ctx->fifo_used[fh]);
        if (hook == PV_HOOK_FIFO_SEND) {
            if (f_ok && ctx->fifo_depth[fh] < PV_FIFO_DEPTH) {
                ctx->fifo_msg[fh][ctx->fifo_tail[fh]] = ctx->regs[rs2];
                ctx->fifo_tail[fh] = (ctx->fifo_tail[fh] + 1) % PV_FIFO_DEPTH;
                ctx->fifo_depth[fh]++;
            }
            ctx->regs[rd] = f_ok ? 1 : 0;
            return;
        }
        if (hook == PV_HOOK_FIFO_RECV) {
            if (f_ok && ctx->fifo_depth[fh] > 0) {
                ctx->regs[rd] = ctx->fifo_msg[fh][ctx->fifo_head[fh]];
                ctx->fifo_head[fh] = (ctx->fifo_head[fh] + 1) % PV_FIFO_DEPTH;
                ctx->fifo_depth[fh]--;
                ctx->host_status = 0;
            } else {
                ctx->regs[rd] = pv_arena_finish(ctx, 0);
                ctx->host_status = 1;
            }
            return;
        }
        if (hook == PV_HOOK_FIFO_POLL) { ctx->regs[rd] = f_ok ? ctx->fifo_depth[fh] : 0; return; }
    }
    /* Error.*: real try/except (handler stack) -- see docs/EXCEPTION_ENGINE.md.
     * Mirrors picoscript_vm.py's _error_hook / vm/picovm.js's _errorHook
     * exactly. `laddr` (the label-address IL construct lower_try/Raise use
     * to pass a handler PC into Error.SetHandler) needs NO new C opcode --
     * it lowers to plain SUB/ADD/MUL bytecode (the same "wide constant load"
     * form used for any large integer literal, see picoscript_il.py's
     * _emit_const), which this interpreter already executes; the only real
     * gap was this dispatch + the handler-stack/pending-jump mechanism. */
    if (hook == PV_HOOK_ERROR_SETHANDLER) {
        if (ctx->err_sp < PV_MAX_ERR_HANDLERS) {
            ctx->err_stack[ctx->err_sp] = ctx->regs[rs1];
            ctx->err_call_depth[ctx->err_sp] = ctx->call_sp;
            ctx->err_sp++;
        }
        ctx->regs[rd] = 1;
        return;
    }
    if (hook == PV_HOOK_ERROR_POPHANDLER) {
        if (ctx->err_sp > 0) { ctx->err_sp--; ctx->regs[rd] = 1; }
        else ctx->regs[rd] = 0;
        return;
    }
    if (hook == PV_HOOK_ERROR_HASHANDLER) {
        int32_t top = (ctx->err_sp > 0) ? ctx->err_stack[ctx->err_sp - 1] : 0;
        ctx->regs[rd] = top ? 1 : 0;
        return;
    }
    if (hook == PV_HOOK_ERROR_CODE) { ctx->regs[rd] = ctx->err_code; return; }
    if (hook == PV_HOOK_ERROR_DETAIL) { ctx->regs[rd] = ctx->err_detail; return; }
    if (hook == PV_HOOK_ERROR_RESUME) {
        ctx->err_code = 0;
        ctx->err_detail = 0;
        if (ctx->err_resume_pc) {
            ctx->pending_jump = ctx->err_resume_pc;
            ctx->pending_jump_set = 1;
            ctx->err_resume_pc = 0;
        }
        ctx->regs[rd] = 1;
        return;
    }
    if (hook == PV_HOOK_ERROR_CLEAR) {
        ctx->err_code = 0;
        ctx->err_detail = 0;
        ctx->regs[rd] = 1;
        return;
    }
    if (hook == PV_HOOK_ERROR_RAISE) {
        /* Script-level "throw a value": jump to the active handler if one is
         * registered (same channel as a genuine VM fault -- Error.Code()
         * reads back exactly the raised value either way); otherwise this
         * must NOT be silently swallowed -- propagate as a real, uncaught
         * fault (halts the VM), exactly like an unhandled exception would. */
        int32_t code = ctx->regs[rs1];
        int32_t handler_pc = (ctx->err_sp > 0) ? ctx->err_stack[ctx->err_sp - 1] : 0;
        if (handler_pc) {
            ctx->err_code = code;
            ctx->err_detail = 0;
            ctx->err_resume_pc = ctx->cur_pc + 1;
            /* See pv_set_fault's identical comment: discard any return
             * addresses pushed since this handler was registered. */
            ctx->call_sp = ctx->err_call_depth[ctx->err_sp - 1];
            ctx->pending_jump = handler_pc;
            ctx->pending_jump_set = 1;
            ctx->regs[rd] = 1;
        } else {
            ctx->fault = code;
            ctx->fault_pc = ctx->cur_pc;
            ctx->fault_detail = 0;
            ctx->halted = 1;
            ctx->regs[rd] = 0;
        }
        return;
    }
    /* Log.*: deterministic, script-visible tracing/audit log (see
     * docs/LOGGING.md). Real, deterministic, no host state -- fixed-size
     * (PV_MAX_LOGS), matching this embedded runtime's other handle tables.
     * Mirrors picoscript_vm.py's _log_hook / vm/picovm.js's _logHook exactly
     * (Count() scans used slots, matching Python's `len(self.logs)`, which
     * shrinks after Clear -- unlike the monotonic handle counter itself). */
    if (hook == PV_HOOK_LOG_WRITE) {
        int h;
        if (ctx->log_count >= PV_MAX_LOGS) { ctx->regs[rd] = 0; return; }
        h = ctx->log_count++;
        ctx->log_level[h] = ctx->regs[rs1];
        ctx->log_span[h] = ctx->regs[rs2];
        ctx->log_used[h] = 1;
        ctx->regs[rd] = h;
        return;
    }
    if (hook == PV_HOOK_LOG_COUNT) {
        int i, n = 0;
        for (i = 1; i < PV_MAX_LOGS; i++) if (ctx->log_used[i]) n++;
        ctx->regs[rd] = n;
        return;
    }
    if (hook == PV_HOOK_LOG_LEVEL || hook == PV_HOOK_LOG_MESSAGE) {
        int lh = ctx->regs[rs1];
        int l_ok = (lh > 0 && lh < PV_MAX_LOGS && ctx->log_used[lh]);
        ctx->regs[rd] = l_ok ? (hook == PV_HOOK_LOG_LEVEL ? ctx->log_level[lh] : ctx->log_span[lh]) : 0;
        return;
    }
    if (hook == PV_HOOK_LOG_CLEAR) {
        int i;
        for (i = 0; i < PV_MAX_LOGS; i++) ctx->log_used[i] = 0;
        ctx->regs[rd] = 1;
        return;
    }
    /* Kernel.*: WaitIRQ/WaitSWIRQ reuse the same cooperative-yield halt as the
     * raw OP_WAIT opcode; FireSWIRQ is an ack-only signal (no separate log --
     * this embedded runtime carries no free-text debug-log list, unlike
     * Python/JS -- but ProfileStart/ProfileEnd/TracePoint now reuse the
     * real Log.* table above, same as Python/JS). */
    if (hook == PV_HOOK_KERNEL_WAITIRQ || hook == PV_HOOK_KERNEL_WAITSWIRQ) {
        ctx->waiting = 1; ctx->halted = 1; return;
    }
    if (hook == PV_HOOK_KERNEL_FIRESWIRQ) { ctx->regs[rd] = 1; return; }
    if (hook == PV_HOOK_KERNEL_PROFILESTART || hook == PV_HOOK_KERNEL_PROFILEEND || hook == PV_HOOK_KERNEL_TRACEPOINT) {
        int h;
        if (ctx->log_count >= PV_MAX_LOGS) { ctx->regs[rd] = 0; return; }
        h = ctx->log_count++;
        ctx->log_level[h] = (hook == PV_HOOK_KERNEL_PROFILESTART) ? 100 : (hook == PV_HOOK_KERNEL_PROFILEEND) ? 101 : 102;
        ctx->log_span[h] = ctx->regs[rs1];
        ctx->log_used[h] = 1;
        ctx->regs[rd] = h;
        return;
    }
    /* Pack.Use: a lightweight "active pack" selector, independent of
     * Storage's own pack context (a different namespace/concept). */
    if (hook == PV_HOOK_PACK_USE) {
        ctx->active_pack = ctx->regs[rs1];
        ctx->regs[rd] = ctx->active_pack;
        return;
    }
    /* Thread.YieldCounted: deterministic cooperative-yield counter. */
    if (hook == PV_HOOK_THREAD_YIELDCOUNTED) {
        ctx->thread_yield_count++;
        ctx->regs[rd] = (int32_t)ctx->thread_yield_count;
        return;
    }
    /* Reserved namespaces: genuinely external/host-injected state this
     * deterministic VM has no way to source itself (identity provider,
     * physical card reader, live request/connection, OS facts, network
     * socket, PKI trust store). Every method returns a defined, documented
     * default (0, or an empty span for text-shaped results) instead of
     * silently falling through -- so every namespace/method is callable
     * from every dialect and VM. See docs/FEATURE_MATRIX.md. Mirrors
     * picoscript_vm.py's _reserved_stub / vm/picovm.js's _reservedStub. */
    if ((hook >= PV_HOOK_X509_FETCHCERTIFICATE && hook <= PV_HOOK_X509_GETKEYHANDLE) ||
        (hook >= PV_HOOK_AUTH_GETUSERCREDENTIALS && hook <= PV_HOOK_AUTH_REVOKETOKEN) ||
        hook == PV_HOOK_CARD_READ || hook == PV_HOOK_CARD_WRITE || hook == PV_HOOK_CARD_ADDRESS ||
        (hook >= PV_HOOK_ENVIRONMENT_GETOSVERSION && hook <= PV_HOOK_ENVIRONMENT_GETELAPSEDTIME) ||
        (hook >= PV_HOOK_CONTEXT_GETVERB && hook <= PV_HOOK_CONTEXT_GETTRACEID) ||
        (hook >= PV_HOOK_NET_LISTEN && hook <= PV_HOOK_NET_REGISTER)) {
        int is_span =
            hook == PV_HOOK_X509_FETCHCERTIFICATE || hook == PV_HOOK_X509_GENERATECSR ||
            hook == PV_HOOK_X509_GENERATEKEYPAIR || hook == PV_HOOK_X509_GETCERTINFO ||
            hook == PV_HOOK_AUTH_GETUSERCREDENTIALS || hook == PV_HOOK_AUTH_GETUSERPERMISSIONS ||
            hook == PV_HOOK_AUTH_REQUESTTOKEN || hook == PV_HOOK_AUTH_GETTOKEN ||
            hook == PV_HOOK_AUTH_REFRESHTOKEN ||
            hook == PV_HOOK_ENVIRONMENT_GETOSVERSION || hook == PV_HOOK_ENVIRONMENT_GETHOSTNAME ||
            hook == PV_HOOK_ENVIRONMENT_GETTIMEZONE ||
            hook == PV_HOOK_CONTEXT_GETVERB || hook == PV_HOOK_CONTEXT_GETPATH ||
            hook == PV_HOOK_CONTEXT_GETHOST || hook == PV_HOOK_CONTEXT_GETREMOTEADDR ||
            hook == PV_HOOK_CONTEXT_GETUSER || hook == PV_HOOK_CONTEXT_GETPERMISSIONS ||
            hook == PV_HOOK_CONTEXT_GETHEADERS || hook == PV_HOOK_CONTEXT_GETQUERYSTRING ||
            hook == PV_HOOK_CONTEXT_GETBODY || hook == PV_HOOK_CONTEXT_GETREQUESTID ||
            hook == PV_HOOK_CONTEXT_GETCLIENTCERT || hook == PV_HOOK_CONTEXT_GETTRACEID ||
            hook == PV_HOOK_NET_READ;
        ctx->regs[rd] = is_span ? pv_arena_finish(ctx, 0) : 0;
        ctx->host_status = 1; /* INV-18: NOT_FOUND -- no host binding installed */
        return;
    }
    if (hook == PV_HOOK_RANDOM_U32) {
        uint64_t x = ctx->rng_state;
        x ^= (x << 13) & MASK32;
        x ^= (x >> 7);
        x ^= (x << 17) & MASK32;
        ctx->rng_state = x;
        ctx->regs[rd] = (int32_t)(uint32_t)(x & MASK32);
        return;
    }
    if (hook == PV_HOOK_QUEUE_ENQUEUE) {
        int q = rs1 & 7;
        if (ctx->qdepth[q] < 64) ctx->queues[q][ctx->qdepth[q]++] = ctx->regs[rd];
        return;
    }
    if (hook == PV_HOOK_QUEUE_DEQUEUE) {
        int q = rs1 & 7;
        if (ctx->qdepth[q] > 0) {
            ctx->host_status = 0;
            ctx->regs[rd] = ctx->queues[q][0];
            for (int i = 1; i < ctx->qdepth[q]; i++) ctx->queues[q][i - 1] = ctx->queues[q][i];
            ctx->qdepth[q]--;
        } else {
            ctx->host_status = 3;       /* INV-18: EMPTY */
            ctx->regs[rd] = 0;
        }
        return;
    }
    if (hook == PV_HOOK_STATUS_LAST) {  /* INV-18: read out-of-band fallible-hook status */
        ctx->regs[rd] = ctx->host_status;
        return;
    }
    if (hook == PV_HOOK_QUEUE_DEPTH) {
        ctx->regs[rd] = ctx->qdepth[rs1 & 7];
        return;
    }
    if (hook == PV_HOOK_QUEUE_DEQUEUEBATCH || hook == PV_HOOK_QUEUE_ENQUEUEBATCH) {
        /* docs/CONFORMANCE_LEVELS.md's "L3: Profiling & Amortization
         * (Optional)" tier -- an aspirational v2 batch-container API ("no
         * correctness impact if omitted") with no existing container type
         * it can return without inventing new v2 semantics that would
         * preempt a future, deliberate design. Explicit defined default
         * (never a silent fallthrough leaving regs[rd] untouched), same
         * convention as every other deferred namespace in this codebase. */
        ctx->regs[rd] = 0;
        return;
    }
    if (hook == PV_HOOK_BITS_AND) {
        ctx->regs[rd] = (int32_t)((uint32_t)ctx->regs[rs1] & (uint32_t)ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_BITS_OR) {
        ctx->regs[rd] = (int32_t)((uint32_t)ctx->regs[rs1] | (uint32_t)ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_BITS_XOR) {
        ctx->regs[rd] = (int32_t)((uint32_t)ctx->regs[rs1] ^ (uint32_t)ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_BITS_SHL) {
        ctx->regs[rd] = (int32_t)(((uint32_t)ctx->regs[rs1] << ((uint32_t)ctx->regs[rs2] & 31)) & MASK32);
        return;
    }
    if (hook == PV_HOOK_BITS_SHR) {
        ctx->regs[rd] = (int32_t)((uint32_t)ctx->regs[rs1] >> ((uint32_t)ctx->regs[rs2] & 31));
        return;
    }
    if (hook == PV_HOOK_BITS_SAR) {
        ctx->regs[rd] = (int32_t)((int32_t)ctx->regs[rs1] >> ((uint32_t)ctx->regs[rs2] & 31));
        return;
    }
    if (hook == PV_HOOK_BITS_NOT) {
        ctx->regs[rd] = (int32_t)(~(uint32_t)ctx->regs[rs1]);
        return;
    }
    if (hook == PV_HOOK_DOT8_LEN) {
        ctx->dot_len = ctx->regs[rs1];
        return;
    }
    if (hook == PV_HOOK_DOT8_OF) {
        ctx->regs[rd] = pv_dot8(ctx, (uint32_t)ctx->regs[rs1], (uint32_t)ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_MEMORY_GET) {
        ctx->regs[rd] = pv_mem_get(ctx, (uint32_t)ctx->regs[rs1]);
        return;
    }
    if (hook == PV_HOOK_MEMORY_SET) {
        uint32_t a = (uint32_t)ctx->regs[rs1];
        if (ctx->mem_size) a %= (uint32_t)ctx->mem_size;   /* match Python/JS wrap before the const check */
        if (a >= ctx->const_floor && a < 0x8000u) {        /* INV-9: literal const region is read-only */
            pv_set_fault(ctx, PV_FAULT_CONST_WRITE, ctx->cur_pc, (int)a);
            return;
        }
        pv_mem_set(ctx, a, ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_MEMORY_SETCONST) {                  /* INV-9: compiler-only literal write */
        uint32_t a = (uint32_t)ctx->regs[rs1];
        uint8_t b = (uint8_t)(ctx->regs[rs2] & 0xFF);
        if (ctx->mem_size) a %= (uint32_t)ctx->mem_size;   /* match Python/JS wrap before lowering the floor */
        if (ctx->mem_size && pv_const_used(ctx, a) && ctx->mem[a] != b) {
            pv_set_fault(ctx, PV_FAULT_CONST_WRITE, ctx->cur_pc, (int)a);
            return;
        }
        if (ctx->mem_size) ctx->mem[a] = b;
        pv_const_mark(ctx, a);
        if (a < ctx->const_floor) ctx->const_floor = a;
        return;
    }
    if (hook == PV_HOOK_IO_WRITEBYTE) {
        pv_io_write(ctx, ctx->regs[rs1]);
        return;
    }

    /* ---- Span.* (handle = 1-based index into the span table) ---------- */
    if (hook == PV_HOOK_SPAN_MAKE) {
        ctx->regs[rd] = pv_span_make(ctx, (uint32_t)(ctx->regs[rs1] & 0xFFFF), ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_SPAN_SLICE) {
        int h = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, h);
        int32_t l = pv_span_n(ctx, h);
        int32_t off = ctx->regs[rs2];
        if (off < 0) off = 0;
        if (off > l) off = l;
        ctx->regs[rd] = pv_span_make(ctx, p + (uint32_t)off, l - off);
        return;
    }
    if (hook == PV_HOOK_SPAN_MATERIALIZE) {
        int h = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, h);
        int32_t l = pv_span_n(ctx, h);
        uint32_t k = 0;
        for (int32_t i = 0; i < l; i++) pv_arena_put(ctx, &k, pv_arena_get(ctx, p + (uint32_t)i));
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_SPAN_LEN) {
        ctx->regs[rd] = pv_span_n(ctx, ctx->regs[rs1]);
        return;
    }
    if (hook == PV_HOOK_SPAN_GET) {
        int h = ctx->regs[rs1];
        int32_t idx = ctx->regs[rs2];
        int32_t l = pv_span_n(ctx, h);
        ctx->regs[rd] = (idx >= 0 && idx < l)
                      ? (int32_t)pv_arena_get(ctx, pv_span_p(ctx, h) + (uint32_t)idx) : 0;
        return;
    }
    if (hook == PV_HOOK_IO_WRITE) {
        int h = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, h);
        int32_t l = pv_span_n(ctx, h);
        for (int32_t i = 0; i < l; i++)
            if (ctx->out_len < PV_MAX_OUT) ctx->out[ctx->out_len++] = pv_arena_get(ctx, p + (uint32_t)i);
        return;
    }
    /* ---- Req.* : read the native HTTP request context (ctx->req_*) ---------- */
    if (hook == PV_HOOK_REQ_METHOD) {
        ctx->regs[rd] = pv_span_from_cbytes(ctx, ctx->req_method, ctx->req_method_len);
        return;
    }
    if (hook == PV_HOOK_REQ_PATH) {
        ctx->regs[rd] = pv_span_from_cbytes(ctx, ctx->req_path, ctx->req_path_len);
        return;
    }
    if (hook == PV_HOOK_REQ_HEADER) {
        /* arg span = header name; search ctx->req_headers case-insensitively for
         * "name:" at a line start, return the trimmed value span (0 if absent). */
        int h = ctx->regs[rs1];
        uint32_t np = pv_span_p(ctx, h);
        int32_t nn = pv_span_n(ctx, h);
        const char *hdr = ctx->req_headers;
        int hl = ctx->req_headers_len;
        ctx->regs[rd] = 0;
        if (hdr && hl > 0 && nn > 0) {
            int i = 0;
            while (i < hl) {
                int ls = i;                       /* line start */
                while (i < hl && hdr[i] != '\n') i++;
                int le = i;                       /* line end (at \n) */
                if (le > ls && hdr[le - 1] == '\r') le--;
                /* match name (case-insensitive) followed by ':' */
                int j = 0, match = 1;
                while (j < nn) {
                    if (ls + j >= le) { match = 0; break; }
                    uint8_t a = (uint8_t)hdr[ls + j];
                    uint8_t b = pv_arena_get(ctx, np + (uint32_t)j);
                    if (a >= 'A' && a <= 'Z') a = (uint8_t)(a - 'A' + 'a');
                    if (b >= 'A' && b <= 'Z') b = (uint8_t)(b - 'A' + 'a');
                    if (a != b) { match = 0; break; }
                    j++;
                }
                if (match && ls + nn < le && hdr[ls + nn] == ':') {
                    int vs = ls + nn + 1;
                    while (vs < le && (hdr[vs] == ' ' || hdr[vs] == '\t')) vs++;
                    ctx->regs[rd] = pv_span_from_cbytes(ctx, hdr + vs, le - vs);
                    return;
                }
                i++;  /* skip the \n */
            }
        }
        return;
    }
    if (hook == PV_HOOK_REQ_BODYSPAN || hook == PV_HOOK_REQ_BODYSLICE) {
        ctx->regs[rd] = pv_span_from_cbytes(ctx, ctx->req_body, ctx->req_body_len);
        return;
    }
    if (hook == PV_HOOK_REQ_BODYLEN || hook == PV_HOOK_REQ_BODYCOUNT) {
        ctx->regs[rd] = ctx->req_body_len > 0 ? ctx->req_body_len : 0;
        return;
    }
    if (hook == PV_HOOK_REQ_PRINCIPAL) {
        /* Authenticated subject injected by the trusted auth front (proxy/kernel)
         * as the X-Forge-Principal header. Empty span = unauthenticated. The
         * front MUST strip any client-supplied copy of this header. */
        ctx->regs[rd] = pv_req_header_value(ctx, "x-forge-principal", 17);
        return;
    }
    if (hook == PV_HOOK_REQ_BODYMODE || hook == PV_HOOK_REQ_SEQ) {
        ctx->regs[rd] = 0;
        return;
    }
    if (hook == PV_HOOK_REQ_PARAMCOUNT) {
        /* number of non-empty '/'-separated path segments */
        const char *p = ctx->req_path; int pl = ctx->req_path_len, n = 0, i = 0;
        while (i < pl) {
            while (i < pl && p[i] == '/') i++;
            if (i < pl && p[i] != '?') { n++; while (i < pl && p[i] != '/' && p[i] != '?') i++; }
            if (i < pl && p[i] == '?') break;
        }
        ctx->regs[rd] = n;
        return;
    }
    if (hook == PV_HOOK_REQ_PARAM) {
        /* 0-based path segment by index (stops at '?'); 0 span if out of range */
        int want = ctx->regs[rs1];
        const char *p = ctx->req_path; int pl = ctx->req_path_len, idx = 0, i = 0;
        ctx->regs[rd] = 0;
        while (i < pl) {
            while (i < pl && p[i] == '/') i++;
            if (i >= pl || p[i] == '?') break;
            int s = i;
            while (i < pl && p[i] != '/' && p[i] != '?') i++;
            if (idx == want) { ctx->regs[rd] = pv_span_from_cbytes(ctx, p + s, i - s); return; }
            idx++;
        }
        return;
    }

    /* Resp.Write: same as Io.Write — append span bytes to ctx->out */
    if (hook == PV_HOOK_RESP_WRITE) {
        int h = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, h);
        int32_t l = pv_span_n(ctx, h);
        for (int32_t i = 0; i < l; i++)
            if (ctx->out_len < PV_MAX_OUT) ctx->out[ctx->out_len++] = pv_arena_get(ctx, p + (uint32_t)i);
        return;
    }
    /* Resp.Status: set HTTP status code */
    if (hook == PV_HOOK_RESP_STATUS) {
        ctx->http_status = ctx->regs[rs1] & 0xFFF;
        return;
    }
    /* Resp.Header: append "Name: Value\r\n" to ctx->out_headers, collected by
     * pv_send_http_response (picovm_pool.c) alongside the always-emitted
     * Content-Type/Content-Length/Connection. Was previously a documented
     * no-op in pool mode; fixed because host/picowal's app_router.eng needs
     * real CORS headers for the WebIDE (a different origin) to call it. */
    if (hook == PV_HOOK_RESP_HEADER) {
        int nh = ctx->regs[rs1], vh = ctx->regs[rs2];
        uint32_t np = pv_span_p(ctx, nh), vp = pv_span_p(ctx, vh);
        int32_t nn = pv_span_n(ctx, nh), vn = pv_span_n(ctx, vh);
        int32_t i, room = (int32_t)sizeof(ctx->out_headers) - ctx->out_headers_len - 4; /* ": \r\n" */
        if (nn + vn > room) return; /* header buffer full: drop silently, don't corrupt */
        for (i = 0; i < nn; i++) ctx->out_headers[ctx->out_headers_len++] = (char)pv_arena_get(ctx, np + (uint32_t)i);
        ctx->out_headers[ctx->out_headers_len++] = ':';
        ctx->out_headers[ctx->out_headers_len++] = ' ';
        for (i = 0; i < vn; i++) ctx->out_headers[ctx->out_headers_len++] = (char)pv_arena_get(ctx, vp + (uint32_t)i);
        ctx->out_headers[ctx->out_headers_len++] = '\r';
        ctx->out_headers[ctx->out_headers_len++] = '\n';
        return;
    }
    /* Resp.End: mark response complete (handler can return) */
    if (hook == PV_HOOK_RESP_END) {
        return;
    }
    /* Resp.Seal / Resp.Respond / Resp.Flush — no-ops in pool mode */
    if (hook >= 0x15 && hook <= 0x1F) {
        return;
    }

    /* ---- Arena scopes: Mark / Rewind / Reset the bump arena ----------- */
    if (hook == PV_HOOK_ARENA_MARK) {
        ctx->regs[rd] = (int32_t)((((uint32_t)ctx->span_count & 0x7FF) << 20) | (ctx->arena_top & 0xFFFFF));
        return;
    }
    if (hook == PV_HOOK_ARENA_REWIND) {
        uint32_t m = (uint32_t)ctx->regs[rs1];
        int cnt = (int)((m >> 20) & 0x7FF);
        ctx->arena_top = m & 0xFFFFF;
        if (cnt < 1) cnt = 1;
        if (cnt < ctx->span_count) ctx->span_count = cnt;
        return;
    }
    if (hook == PV_HOOK_ARENA_RESET) {
        ctx->arena_top = 0x8000;
        ctx->span_count = 1;
        return;
    }

    /* ---- String.* (spans in, span/int out) --------------------------- */
    if (hook == PV_HOOK_STRING_LENGTH) {
        ctx->regs[rd] = pv_span_n(ctx, ctx->regs[rs1]);
        return;
    }
    if (hook == PV_HOOK_STRING_CONCAT) {
        int ha = ctx->regs[rs1], hb = ctx->regs[rs2];
        uint32_t pa = pv_span_p(ctx, ha), pb = pv_span_p(ctx, hb);
        int32_t la = pv_span_n(ctx, ha), lb = pv_span_n(ctx, hb);
        uint32_t k = 0;
        for (int32_t i = 0; i < la; i++) pv_arena_put(ctx, &k, pv_arena_get(ctx, pa + (uint32_t)i));
        for (int32_t i = 0; i < lb; i++) pv_arena_put(ctx, &k, pv_arena_get(ctx, pb + (uint32_t)i));
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_STRING_SUBSTRING) {
        int ha = ctx->regs[rs1];
        uint32_t pa = pv_span_p(ctx, ha);
        int32_t la = pv_span_n(ctx, ha);
        int32_t start = ctx->regs[rs2];
        if (start < 0) start = 0;
        uint32_t k = 0;
        for (int32_t i = start; i < la; i++) pv_arena_put(ctx, &k, pv_arena_get(ctx, pa + (uint32_t)i));
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_STRING_INDEXOF) {
        int ha = ctx->regs[rs1], hb = ctx->regs[rs2];
        uint32_t pa = pv_span_p(ctx, ha), pb = pv_span_p(ctx, hb);
        int32_t la = pv_span_n(ctx, ha), lb = pv_span_n(ctx, hb);
        int32_t found = -1;
        if (lb == 0) {
            found = 0;
        } else {
            for (int32_t i = 0; i + lb <= la; i++) {
                int32_t j = 0;
                for (; j < lb; j++)
                    if (pv_arena_get(ctx, pa + (uint32_t)(i + j)) != pv_arena_get(ctx, pb + (uint32_t)j)) break;
                if (j == lb) { found = i; break; }
            }
        }
        ctx->host_status = (found < 0) ? 1 : 0;   /* INV-18: NOT_FOUND */
        ctx->regs[rd] = found;
        return;
    }
    if (hook == PV_HOOK_STRING_STARTSWITH || hook == PV_HOOK_STRING_ENDSWITH) {
        int ha = ctx->regs[rs1], hb = ctx->regs[rs2];
        uint32_t pa = pv_span_p(ctx, ha), pb = pv_span_p(ctx, hb);
        int32_t la = pv_span_n(ctx, ha), lb = pv_span_n(ctx, hb);
        int ok = (lb <= la);
        int32_t base = (hook == PV_HOOK_STRING_ENDSWITH) ? (la - lb) : 0;
        for (int32_t j = 0; ok && j < lb; j++)
            if (pv_arena_get(ctx, pa + (uint32_t)(base + j)) != pv_arena_get(ctx, pb + (uint32_t)j)) ok = 0;
        ctx->regs[rd] = ok ? 1 : 0;
        return;
    }
    /* String.Eq: full-content equality (not span-handle equality -- two
     * independently built spans with identical bytes are NOT the same
     * handle, so `a == b`/`is` on strings only coincidentally works when
     * both sides happen to alias the same span). This was allocated a hook
     * code (0x8D) and implemented in the Python reference VM
     * (picoscript_vm.py Strings.Eq) and documented in every hook table, but
     * had no C-VM implementation until now -- found while building
     * host/picowal/app_router.eng, whose route dispatch (`p0 is "schema"`)
     * silently always took the else branch because it was comparing span
     * handles, not string content. */
    if (hook == PV_HOOK_STRING_EQ) {
        int ha = ctx->regs[rs1], hb = ctx->regs[rs2];
        uint32_t pa = pv_span_p(ctx, ha), pb = pv_span_p(ctx, hb);
        int32_t la = pv_span_n(ctx, ha), lb = pv_span_n(ctx, hb);
        int ok = (la == lb);
        for (int32_t j = 0; ok && j < la; j++)
            if (pv_arena_get(ctx, pa + (uint32_t)j) != pv_arena_get(ctx, pb + (uint32_t)j)) ok = 0;
        ctx->regs[rd] = ok ? 1 : 0;
        return;
    }
    if (hook == PV_HOOK_STRING_TOUPPER || hook == PV_HOOK_STRING_TOLOWER) {
        int ha = ctx->regs[rs1];
        uint32_t pa = pv_span_p(ctx, ha);
        int32_t la = pv_span_n(ctx, ha);
        uint32_t k = 0;
        for (int32_t i = 0; i < la; i++) {
            uint8_t c = pv_arena_get(ctx, pa + (uint32_t)i);
            if (hook == PV_HOOK_STRING_TOUPPER) { if (c >= 97 && c <= 122) c = (uint8_t)(c - 32); }
            else { if (c >= 65 && c <= 90) c = (uint8_t)(c + 32); }
            pv_arena_put(ctx, &k, c);
        }
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_STRING_TRIM) {
        int ha = ctx->regs[rs1];
        uint32_t pa = pv_span_p(ctx, ha);
        int32_t la = pv_span_n(ctx, ha);
        int32_t s = 0, e = la;
        while (s < e) { uint8_t c = pv_arena_get(ctx, pa + (uint32_t)s);       if (c==0x20||c==0x09||c==0x0d||c==0x0a) s++; else break; }
        while (e > s) { uint8_t c = pv_arena_get(ctx, pa + (uint32_t)(e - 1)); if (c==0x20||c==0x09||c==0x0d||c==0x0a) e--; else break; }
        uint32_t k = 0;
        for (int32_t i = s; i < e; i++) pv_arena_put(ctx, &k, pv_arena_get(ctx, pa + (uint32_t)i));
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_STRING_SETREPLACE) {
        int ha = ctx->regs[rs1];
        ctx->str_repl_ptr = pv_span_p(ctx, ha);
        ctx->str_repl_len = pv_span_n(ctx, ha);
        return;
    }
    if (hook == PV_HOOK_STRING_REPLACE) {
        int ha = ctx->regs[rs1], hb = ctx->regs[rs2];
        uint32_t pa = pv_span_p(ctx, ha), pb = pv_span_p(ctx, hb);
        int32_t la = pv_span_n(ctx, ha), lb = pv_span_n(ctx, hb);
        uint32_t pr = ctx->str_repl_ptr;
        int32_t lr = ctx->str_repl_len;
        uint32_t k = 0;
        if (lb == 0) {
            /* bytes.replace(b"", repl): repl before every byte and once at the end */
            for (int32_t j = 0; j < lr; j++) pv_arena_put(ctx, &k, pv_arena_get(ctx, pr + (uint32_t)j));
            for (int32_t i = 0; i < la; i++) {
                pv_arena_put(ctx, &k, pv_arena_get(ctx, pa + (uint32_t)i));
                for (int32_t j = 0; j < lr; j++) pv_arena_put(ctx, &k, pv_arena_get(ctx, pr + (uint32_t)j));
            }
        } else {
            int32_t i = 0;
            while (i < la) {
                int match = (i + lb <= la);
                for (int32_t j = 0; match && j < lb; j++)
                    if (pv_arena_get(ctx, pa + (uint32_t)(i + j)) != pv_arena_get(ctx, pb + (uint32_t)j)) match = 0;
                if (match) {
                    for (int32_t j = 0; j < lr; j++) pv_arena_put(ctx, &k, pv_arena_get(ctx, pr + (uint32_t)j));
                    i += lb;
                } else {
                    pv_arena_put(ctx, &k, pv_arena_get(ctx, pa + (uint32_t)i));
                    i++;
                }
            }
        }
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_STRING_SPLIT) {
        /* Real, deterministic multi-value result: the 2-in/1-out host-hook ABI
         * has no array type, so parts are stored in a fresh Map (int key
         * 0..N-1 -> string part), reusing Map.*'s already-parity-tested
         * storage rather than inventing a new container. Allocated directly
         * (bypassing Map.New's side effect on map_active), so Split/Join
         * never disturb a map the caller already has open. Mirrors
         * picoscript_vm.py/picovm.js's String.Split exactly. */
        int ha = ctx->regs[rs1], hb = ctx->regs[rs2];
        uint32_t pa = pv_span_p(ctx, ha), pb = pv_span_p(ctx, hb);
        int32_t la = pv_span_n(ctx, ha), lb = pv_span_n(ctx, hb);
        int h, part_idx = 0, i = 0;
        if (ctx->map_nmaps >= PV_MAX_MAPS) { ctx->regs[rd] = 0; return; }
        h = ctx->map_nmaps++;
        ctx->map_used[h] = 1; ctx->map_head[h] = -1; ctx->map_tail[h] = -1; ctx->map_count[h] = 0;
        while (1) {
            int32_t found = -1, part_end, x, e_ok;
            uint32_t k2 = 0;
            int tmp, e;
            uint32_t off; int32_t len;
            if (lb > 0) {
                for (int32_t s = i; s + lb <= la; s++) {
                    int32_t j = 0;
                    for (; j < lb; j++) if (pv_arena_get(ctx, pa + (uint32_t)(s + j)) != pv_arena_get(ctx, pb + (uint32_t)j)) break;
                    if (j == lb) { found = s; break; }
                }
            }
            part_end = (found < 0) ? la : found;
            for (x = i; x < part_end; x++) pv_arena_put(ctx, &k2, pv_arena_get(ctx, pa + (uint32_t)x));
            tmp = pv_arena_finish(ctx, k2);
            e_ok = pv_map_intern(ctx, tmp, &off, &len);
            if (e_ok) {
                e = pv_map_new_entry(ctx, h);
                if (e >= 0) { ctx->me_kk[e] = 0; ctx->me_ki[e] = part_idx; ctx->me_vk[e] = 1; ctx->me_voff[e] = off; ctx->me_vlen[e] = len; }
            }
            part_idx++;
            if (lb == 0 || found < 0) break;
            i = found + lb;
        }
        ctx->regs[rd] = h;
        return;
    }
    if (hook == PV_HOOK_STRING_JOIN) {
        /* rs1 = separator span (as usual), rs2 = the Map handle returned by a
         * prior Split (or any int-keyed 0..N-1 string map). */
        uint32_t ps = pv_span_p(ctx, ctx->regs[rs1]);
        int32_t ls = pv_span_n(ctx, ctx->regs[rs1]);
        int mh = ctx->regs[rs2];
        uint32_t k3 = 0;
        int32_t idx = 0;
        if (!(mh > 0 && mh < ctx->map_nmaps && ctx->map_used[mh])) { ctx->regs[rd] = pv_arena_finish(ctx, 0); return; }
        while (1) {
            int e = pv_map_find_i(ctx, mh, idx);
            if (e < 0) break;
            if (idx > 0) { int32_t j; for (j = 0; j < ls; j++) pv_arena_put(ctx, &k3, pv_arena_get(ctx, ps + (uint32_t)j)); }
            if (ctx->me_vk[e] == 1) {
                int32_t j;
                for (j = 0; j < ctx->me_vlen[e]; j++) pv_arena_put(ctx, &k3, ctx->map_pool[ctx->me_voff[e] + (uint32_t)j]);
            }
            idx++;
        }
        ctx->regs[rd] = pv_arena_finish(ctx, k3);
        return;
    }

    /* ---- Number.* (int in, int/span out) ----------------------------- */
    if (hook == PV_HOOK_NUMBER_PARSE) {
        int ha = ctx->regs[rs1];
        uint32_t pa = pv_span_p(ctx, ha);
        int32_t la = pv_span_n(ctx, ha);
        int32_t i = 0, e = la;
        while (i < e) { uint8_t c = pv_arena_get(ctx, pa + (uint32_t)i);       if (c==0x20||c==0x09||c==0x0d||c==0x0a) i++; else break; }
        while (e > i) { uint8_t c = pv_arena_get(ctx, pa + (uint32_t)(e - 1)); if (c==0x20||c==0x09||c==0x0d||c==0x0a) e--; else break; }
        int neg = 0;
        if (i < e) { uint8_t c = pv_arena_get(ctx, pa + (uint32_t)i); if (c=='+'||c=='-') { neg = (c=='-'); i++; } }
        int32_t int_start = i, j = i;
        uint32_t val = 0;
        for (; j < e; j++) {
            uint8_t c = pv_arena_get(ctx, pa + (uint32_t)j);
            if (c < '0' || c > '9') break;
            val = val * 10u + (uint32_t)(c - '0');
        }
        int valid = (j > int_start);   /* at least one integer digit required */
        if (valid && j < e) {
            uint8_t c = pv_arena_get(ctx, pa + (uint32_t)j);
            if (c == '.') {
                /* Tolerate a decimal-point numeric string (e.g. "1000.0",
                 * "-3.75", "5.") by truncating the fractional part towards
                 * zero -- mirrors picoscript_vm.py's _parse_int_tolerant /
                 * vm/picovm.js's Number.Parse (same acceptance rule: exactly
                 * one dot, digits-only fraction, which may be empty). Number
                 * is 32-bit-integer only; a trailing fraction is numerically
                 * valid input that merely isn't integer-formatted, so this
                 * avoids a silent PARSE_ERROR/0 for it (e.g. a host
                 * language's default float-to-string of a whole currency
                 * amount, such as Python's str(1000.0) == "1000.0"). */
                int32_t k;
                for (k = j + 1; k < e; k++) {
                    uint8_t fc = pv_arena_get(ctx, pa + (uint32_t)k);
                    if (fc < '0' || fc > '9') { valid = 0; break; }
                }
            } else {
                valid = 0;   /* trailing garbage after the integer digits */
            }
        }
        if (!valid) val = 0;
        ctx->host_status = valid ? 0 : 2;          /* INV-18: PARSE_ERROR */
        ctx->regs[rd] = neg ? (int32_t)(0u - val) : (int32_t)val;
        return;
    }
    if (hook == PV_HOOK_NUMBER_ABS) {
        uint32_t a = (uint32_t)ctx->regs[rs1];
        ctx->regs[rd] = (ctx->regs[rs1] < 0) ? (int32_t)(0u - a) : (int32_t)a;
        return;
    }
    if (hook == PV_HOOK_NUMBER_MIN) {
        int32_t a = ctx->regs[rs1], b = ctx->regs[rs2];
        ctx->regs[rd] = (a < b) ? a : b;
        return;
    }
    if (hook == PV_HOOK_NUMBER_MAX) {
        int32_t a = ctx->regs[rs1], b = ctx->regs[rs2];
        ctx->regs[rd] = (a > b) ? a : b;
        return;
    }
    if (hook == PV_HOOK_NUMBER_FLOOR || hook == PV_HOOK_NUMBER_CEILING || hook == PV_HOOK_NUMBER_ROUND) {
        ctx->regs[rd] = ctx->regs[rs1];
        return;
    }
    if (hook == PV_HOOK_NUMBER_TOSTRING) {
        int32_t v = ctx->regs[rs1];
        uint8_t tmp[16];
        int t = 0, neg = 0;
        uint32_t u;
        if (v < 0) { neg = 1; u = 0u - (uint32_t)v; } else u = (uint32_t)v;
        if (u == 0) tmp[t++] = '0';
        while (u) { tmp[t++] = (uint8_t)('0' + (u % 10u)); u /= 10u; }
        uint32_t k = 0;
        if (neg) pv_arena_put(ctx, &k, '-');
        while (t > 0) pv_arena_put(ctx, &k, tmp[--t]);
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_NUMBER_TOHEX || hook == PV_HOOK_NUMBER_TOOCTAL || hook == PV_HOOK_NUMBER_TOBINARY) {
        uint32_t u = (uint32_t)ctx->regs[rs1];
        uint32_t base = (hook == PV_HOOK_NUMBER_TOHEX) ? 16u : (hook == PV_HOOK_NUMBER_TOOCTAL) ? 8u : 2u;
        uint8_t tmp[40];
        int t = 0;
        if (u == 0) tmp[t++] = '0';
        while (u) { uint32_t d = u % base; tmp[t++] = (uint8_t)(d < 10 ? '0' + d : 'a' + (d - 10)); u /= base; }
        uint32_t k = 0;
        while (t > 0) pv_arena_put(ctx, &k, tmp[--t]);
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }

    /* ---- Decimal.* (Q16.16 fixed-point; see pv_q16_parse_bytes/pv_q16_to_str above) --- */
    if (hook == PV_HOOK_DECIMAL_PARSE) {
        int ha = ctx->regs[rs1];
        uint32_t pa = pv_span_p(ctx, ha);
        int32_t la = pv_span_n(ctx, ha);
        uint8_t buf[64];
        int32_t v = 0; int ok = 0;
        if (la >= 0 && la <= (int32_t)sizeof(buf)) {
            int32_t bi;
            for (bi = 0; bi < la; bi++) buf[bi] = pv_arena_get(ctx, pa + (uint32_t)bi);
            ok = pv_q16_parse_bytes(buf, la, &v);
        }
        ctx->host_status = ok ? 0 : 2;          /* INV-18: PARSE_ERROR */
        ctx->regs[rd] = ok ? v : 0;
        return;
    }
    if (hook == PV_HOOK_DECIMAL_TOSTRING) {
        uint8_t buf[32];
        int32_t n = pv_q16_to_str(ctx->regs[rs1], buf);
        uint32_t k = 0; int32_t bi;
        for (bi = 0; bi < n; bi++) pv_arena_put(ctx, &k, buf[bi]);
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_DECIMAL_TOINT) {
        ctx->regs[rd] = pv_q16_idiv(ctx->regs[rs1], PV_Q16_ONE);   /* truncate towards zero */
        return;
    }
    if (hook == PV_HOOK_DECIMAL_ADD) {
        ctx->regs[rd] = (int32_t)((uint32_t)ctx->regs[rs1] + (uint32_t)ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_DECIMAL_SUB) {
        ctx->regs[rd] = (int32_t)((uint32_t)ctx->regs[rs1] - (uint32_t)ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_DECIMAL_MUL) {
        ctx->regs[rd] = pv_q16_fixmul(ctx->regs[rs1], ctx->regs[rs2]);
        return;
    }
    if (hook == PV_HOOK_DECIMAL_DIV) {
        ctx->regs[rd] = (ctx->regs[rs2] != 0) ? pv_q16_fixdiv(ctx->regs[rs1], ctx->regs[rs2]) : 0;
        return;
    }
    if (hook == PV_HOOK_DECIMAL_COMPARE) {
        int32_t a = ctx->regs[rs1], b = ctx->regs[rs2];
        ctx->regs[rd] = (a == b) ? 0 : (a > b ? 1 : -1);
        return;
    }

    /* ---- Template.* (AOT compile-at-save + render) ------------------- */
    if (hook == PV_HOOK_TEMPLATE_COMPILE) {
        int h = ctx->regs[rs1];
        ctx->regs[rd] = pv_template_compile(ctx, pv_span_p(ctx, h), pv_span_n(ctx, h));
        return;
    }
    if (hook == PV_HOOK_TEMPLATE_RENDER) {
        int hp = ctx->regs[rs1], hm = ctx->regs[rs2];
        ctx->regs[rd] = pv_template_render(ctx, pv_span_p(ctx, hp), pv_span_n(ctx, hp),
                                           pv_span_p(ctx, hm), pv_span_n(ctx, hm));
        return;
    }

    /* ---- Crypto.Sha256 (32-byte digest span) ------------------------- */
    if (hook == PV_HOOK_CRYPTO_SHA256) {
        int h = ctx->regs[rs1];
        uint8_t dig[32];
        pv_sha256(ctx, pv_span_p(ctx, h), pv_span_n(ctx, h), dig);
        uint32_t k = 0;
        for (int i = 0; i < 32; i++) pv_arena_put(ctx, &k, dig[i]);
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }

    /* ---- Crypto.HmacSha256 (key span, msg span -> 32-byte digest span) - */
    if (hook == PV_HOOK_CRYPTO_HMACSHA256) {
        int hk = ctx->regs[rs1], hm = ctx->regs[rs2];
        uint8_t dig[32];
        pv_hmac_sha256(ctx, pv_span_p(ctx, hk), pv_span_n(ctx, hk),
                       pv_span_p(ctx, hm), pv_span_n(ctx, hm), dig);
        uint32_t k = 0;
        for (int i = 0; i < 32; i++) pv_arena_put(ctx, &k, dig[i]);
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }

    /* ---- Crypto.Encrypt/Decrypt: AES-256-CTR. rs1=32-byte key span; rs2=data span
       whose first 16 bytes are the IV/counter and the rest is the payload. Returns
       IV || (payload ^ keystream); CTR is symmetric so encrypt == decrypt. ---- */
    if (hook == PV_HOOK_CRYPTO_ENCRYPT || hook == PV_HOOK_CRYPTO_DECRYPT) {
        int hk = ctx->regs[rs1], hd = ctx->regs[rs2];
        uint32_t kp = pv_span_p(ctx, hk), dp = pv_span_p(ctx, hd);
        int32_t kn = pv_span_n(ctx, hk), dn = pv_span_n(ctx, hd);
        uint8_t key[32], rk[240], ctr[16], ks[16];
        uint32_t k = 0;
        int32_t off, j, plen;
        int i;
        if (kn != 32 || dn < 16) { ctx->host_status = 2; ctx->regs[rd] = 0; return; }
        ctx->host_status = 0;
        for (i = 0; i < 32; i++) key[i] = pv_arena_get(ctx, kp + (uint32_t)i);
        for (i = 0; i < 16; i++) ctr[i] = pv_arena_get(ctx, dp + (uint32_t)i);
        pv_aes256_key_expand(key, rk);
        for (i = 0; i < 16; i++) pv_arena_put(ctx, &k, ctr[i]);   /* IV travels with the output */
        plen = dn - 16;
        for (off = 0; off < plen; off += 16) {
            pv_aes256_encrypt_block(ctr, rk, ks);
            for (j = 0; j < 16 && off + j < plen; j++) {
                uint8_t pb = pv_arena_get(ctx, dp + 16 + (uint32_t)(off + j));
                pv_arena_put(ctx, &k, (uint8_t)(pb ^ ks[j]));
            }
            for (j = 15; j >= 0; j--) { ctr[j] = (uint8_t)(ctr[j] + 1); if (ctr[j]) break; }
        }
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_HTTP_PARSEQUERY || hook == PV_HOOK_HTTP_PARSEFORM) {
        int h = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, h);
        int32_t l = pv_span_n(ctx, h), i = 0;
        uint32_t k = 0;
        while (i < l) {
            int32_t start = i;
            while (i < l && pv_arena_get(ctx, p + (uint32_t)i) != '&') i++;
            int32_t end = i;
            if (i < l) i++;                  /* skip '&' */
            if (end == start) continue;       /* empty pair */
            int32_t eq = start;
            while (eq < end && pv_arena_get(ctx, p + (uint32_t)eq) != '=') eq++;
            pv_urldecode_into(ctx, &k, p, start, eq);
            pv_arena_put(ctx, &k, '=');
            if (eq < end) pv_urldecode_into(ctx, &k, p, eq + 1, end);
            pv_arena_put(ctx, &k, '\n');
        }
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_HTTP_ENCODEJSON) {
        int h = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, h);
        int32_t l = pv_span_n(ctx, h), i = 0;
        uint32_t k = 0;
        pv_arena_put(ctx, &k, '{');
        int first = 1;
        while (i < l) {
            int32_t start = i;
            while (i < l && pv_arena_get(ctx, p + (uint32_t)i) != '\n') i++;
            int32_t end = i;
            if (i < l) i++;                  /* skip '\n' */
            int32_t eq = start;
            while (eq < end && pv_arena_get(ctx, p + (uint32_t)eq) != '=') eq++;
            if (eq >= end) continue;          /* no '=' -> skip line */
            if (!first) pv_arena_put(ctx, &k, ',');
            first = 0;
            pv_arena_put(ctx, &k, '"');
            pv_jsonesc_into(ctx, &k, p, start, eq);
            pv_arena_puts(ctx, &k, "\":\"");
            pv_jsonesc_into(ctx, &k, p, eq + 1, end);
            pv_arena_put(ctx, &k, '"');
        }
        pv_arena_put(ctx, &k, '}');
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_HTTP_PARSEJSON) {
        int h = ctx->regs[rs1];
        pv_pjs st;
        uint8_t pref[256];
        st.ctx = ctx; st.p = pv_span_p(ctx, h); st.n = pv_span_n(ctx, h); st.pos = 0; st.k = 0;
        pjs_emit(&st, pref, 0, 0);
        ctx->regs[rd] = pv_arena_finish(ctx, st.k);
        return;
    }
    /* ReadHeader/ReadBody/GenerateHeaders/GenerateResponse/Request/
     * RespStatus/RespHeaders/RespBody all read/write a live host connection
     * -- host-injected by design. Explicit default (never leave rd
     * untouched), matching picoscript_vm.py's _httplib exactly. */
    if (hook == PV_HOOK_HTTP_READHEADER || hook == PV_HOOK_HTTP_READBODY ||
        hook == PV_HOOK_HTTP_GENERATEHEADERS || hook == PV_HOOK_HTTP_GENERATERESPONSE ||
        hook == PV_HOOK_HTTP_RESPHEADERS || hook == PV_HOOK_HTTP_RESPBODY) {
        ctx->regs[rd] = pv_arena_finish(ctx, 0);
        return;
    }
    if (hook == PV_HOOK_HTTP_REQUEST || hook == PV_HOOK_HTTP_RESPSTATUS) {
        ctx->regs[rd] = 0;
        return;
    }
    /* Html.* DOM tree ops: a real, pure, deterministic node table -- see the
     * pv_html_* helpers above (mirrors picoscript_vm.py's _htmllib exactly). */
    if (hook == PV_HOOK_HTML_CREATENODE) {
        ctx->regs[rd] = pv_html_new_node(ctx, ctx->regs[rs1]);
        return;
    }
    if (hook == PV_HOOK_HTML_ADDCHILDNODE) {
        int ph = ctx->regs[rs1], ch = ctx->regs[rs2];
        int ok = pv_html_valid(ctx, ph) && pv_html_valid(ctx, ch) && ctx->html_child_count[ph] < PV_HTML_MAX_CHILDREN;
        if (ok) ctx->html_child[ph][ctx->html_child_count[ph]++] = ch;
        ctx->regs[rd] = ok ? 1 : 0;
        return;
    }
    if (hook == PV_HOOK_HTML_REMOVECHILDNODE) {
        int ph = ctx->regs[rs1], ch = ctx->regs[rs2];
        int ok = 0;
        if (pv_html_valid(ctx, ph)) {
            int cnt = ctx->html_child_count[ph];
            for (int i = 0; i < cnt; i++) {
                if (ctx->html_child[ph][i] == ch) {
                    for (int j = i; j < cnt - 1; j++) ctx->html_child[ph][j] = ctx->html_child[ph][j + 1];
                    ctx->html_child_count[ph] = (uint8_t)(cnt - 1);
                    ok = 1;
                    break;
                }
            }
        }
        ctx->regs[rd] = ok ? 1 : 0;
        return;
    }
    if (hook == PV_HOOK_HTML_SETATTRIBUTE) {
        /* rs2 packs "key=value" into a single span (2-in/1-out ABI has no 3rd
         * argument register -- see docs/NAMESPACE_STATUS.md's "3-argument
         * ops" section). No '=' present -> whole span is the key, empty value. */
        int nh = ctx->regs[rs1];
        if (pv_html_valid(ctx, nh)) {
            uint32_t p = pv_span_p(ctx, ctx->regs[rs2]);
            int32_t l = pv_span_n(ctx, ctx->regs[rs2]);
            int32_t eq = pv_html_find_char(ctx, p, l, 0, '=');
            int32_t klen = (eq >= 0) ? eq : l;
            uint32_t kk = 0;
            for (int32_t i = 0; i < klen; i++) pv_arena_put(ctx, &kk, pv_arena_get(ctx, p + (uint32_t)i));
            int key_span = pv_arena_finish(ctx, kk);
            int val_span;
            if (eq >= 0) {
                uint32_t vk = 0;
                for (int32_t i = eq + 1; i < l; i++) pv_arena_put(ctx, &vk, pv_arena_get(ctx, p + (uint32_t)i));
                val_span = pv_arena_finish(ctx, vk);
            } else {
                val_span = pv_arena_finish(ctx, 0);
            }
            pv_html_attr_set(ctx, nh, key_span, val_span);
            ctx->regs[rd] = 1;
        } else {
            ctx->regs[rd] = 0;
        }
        return;
    }
    if (hook == PV_HOOK_HTML_GETATTRIBUTE) {
        int val = 0;
        int found = pv_html_attr_get(ctx, ctx->regs[rs1], ctx->regs[rs2], &val);
        ctx->host_status = found ? 0 : 1;   /* INV-18: NOT_FOUND */
        ctx->regs[rd] = found ? val : pv_arena_finish(ctx, 0);
        return;
    }
    if (hook == PV_HOOK_HTML_PARSETREE) {
        int h = ctx->regs[rs1];
        ctx->regs[rd] = pv_html_parse(ctx, pv_span_p(ctx, h), pv_span_n(ctx, h));
        return;
    }
    if (hook == PV_HOOK_HTML_SERIALIZE) {
        uint32_t k = 0;
        pv_html_serialize_rec(ctx, ctx->regs[rs1], 0, &k);
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_HTML_QUERYSELECTOR) {
        int sh = ctx->regs[rs2];
        ctx->regs[rd] = pv_html_query_rec(ctx, ctx->regs[rs1], pv_span_p(ctx, sh), pv_span_n(ctx, sh), 0);
        return;
    }

    /* ---- Maths.* (pure integer: Power = modular pow, Sqrt = floor sqrt;
       Sin/Cos/Tan = Q16.16 CORDIC) - */
    if (hook == PV_HOOK_MATHS_SIN) {
        int32_t s, c; pv_q16_sincos(ctx->regs[rs1], &s, &c); ctx->regs[rd] = s; return;
    }
    if (hook == PV_HOOK_MATHS_COS) {
        int32_t s, c; pv_q16_sincos(ctx->regs[rs1], &s, &c); ctx->regs[rd] = c; return;
    }
    if (hook == PV_HOOK_MATHS_TAN) {
        ctx->regs[rd] = pv_q16_tan(ctx->regs[rs1]); return;
    }
    if (hook == PV_HOOK_MATHS_EXP) {
        ctx->regs[rd] = pv_q16_exp(ctx->regs[rs1]); return;
    }
    if (hook == PV_HOOK_MATHS_LOG) {
        ctx->regs[rd] = pv_q16_log(ctx->regs[rs1]); return;
    }
    if (hook == PV_HOOK_MATHS_LOG10) {
        ctx->regs[rd] = pv_q16_fixmul(pv_q16_log(ctx->regs[rs1]), PV_Q16_INV_LN10); return;
    }
    if (hook == PV_HOOK_MATHS_POWER) {
        int32_t base = ctx->regs[rs1], exp = ctx->regs[rs2];
        uint32_t r;
        if (exp <= 0) {
            r = (exp == 0) ? 1u : 0u;
        } else {
            int32_t e = (exp > 0xFFFF) ? 0xFFFF : exp;
            r = 1u;
            for (int32_t t = 0; t < e; t++) r = (uint32_t)(r * (uint32_t)base);
        }
        ctx->regs[rd] = (int32_t)r;
        return;
    }
    if (hook == PV_HOOK_MATHS_SQRT) {
        int32_t n = ctx->regs[rs1];
        if (n <= 0) { ctx->regs[rd] = 0; return; }
        uint32_t x = (uint32_t)n, res = 0, bit = 1u << 30;
        while (bit > (uint32_t)n) bit >>= 2;
        while (bit) {
            if (x >= res + bit) { x -= res + bit; res = (res >> 1) + bit; }
            else res >>= 1;
            bit >>= 2;
        }
        ctx->regs[rd] = (int32_t)res;
        return;
    }

    /* ---- Compress.PicoCompress / PicoDecompress: the real picocompress codec ----
     * Hosted builds only (picocompress.c needs <string.h>). On freestanding/embedded
     * targets these hooks fall through to the host-fillable path (PIOS supplies them). */
#if defined(__STDC_HOSTED__) && __STDC_HOSTED__
    if (hook == PV_HOOK_COMPRESS_PICOCOMPRESS || hook == PV_HOOK_COMPRESS_PICODECOMPRESS) {
        /* The result is written straight into the bump arena's free region (the input
         * span lives below arena_top, so they never overlap). */
        int hh = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, hh);
        int32_t l = pv_span_n(ctx, hh);
        uint8_t *outp = ctx->mem + ctx->arena_top;
        size_t cap = (ctx->arena_top < (uint32_t)ctx->mem_size)
                         ? (size_t)((uint32_t)ctx->mem_size - ctx->arena_top) : 0u;
        size_t out_len = 0;
        pc_result r = (hook == PV_HOOK_COMPRESS_PICOCOMPRESS)
            ? pc_compress_buffer(ctx->mem + p, (size_t)l, outp, cap, &out_len)
            : pc_decompress_buffer(ctx->mem + p, (size_t)l, outp, cap, &out_len);
        if (r != PC_OK) out_len = 0;
        ctx->regs[rd] = pv_arena_finish(ctx, (uint32_t)out_len);
        return;
    }
    /* ---- Compress.BrotliCompress / BrotliDecompress: the real micro-brotli codec
     * (vm/picobrotli.c), byte-identical with picobrotli.py / .js. Output is valid
     * RFC 7932 decodable by any browser. Written into the bump arena free region. */
    if (hook == PV_HOOK_COMPRESS_BROTLICOMPRESS || hook == PV_HOOK_COMPRESS_BROTLIDECOMPRESS) {
        int hh = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, hh);
        int32_t l = pv_span_n(ctx, hh);
        uint8_t *outp = ctx->mem + ctx->arena_top;
        size_t cap = (ctx->arena_top < (uint32_t)ctx->mem_size)
                         ? (size_t)((uint32_t)ctx->mem_size - ctx->arena_top) : 0u;
        int wrote = (hook == PV_HOOK_COMPRESS_BROTLICOMPRESS)
            ? brotli_encode(ctx->mem + p, (size_t)l, outp, cap)
            : brotli_decode(ctx->mem + p, (size_t)l, outp, cap);
        ctx->regs[rd] = pv_arena_finish(ctx, wrote < 0 ? 0u : (uint32_t)wrote);
        if (hook == PV_HOOK_COMPRESS_BROTLIDECOMPRESS) ctx->host_status = (wrote < 0) ? 2 : 0;
        return;
    }
#endif
    /* ---- Compress.DeflateDecompress / GzipDecompress: real INFLATE (RFC 1951)
     *      built into the runtime. Compression stays in the reference runtime +
     *      host (docs/COMPRESS.md). Malformed input -> empty span. ---- */
    if (hook == PV_HOOK_COMPRESS_DEFLATEDECOMPRESS || hook == PV_HOOK_COMPRESS_GZIPDECOMPRESS) {
        int h = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, h);
        int32_t l = pv_span_n(ctx, h);
        uint32_t k = 0;
        pv_puff s;
        int err;
        s.ctx = ctx; s.in = p; s.inlen = l; s.incnt = 0;
        s.bitbuf = 0; s.bitcnt = 0; s.outk = &k; s.err = 0;
        if (hook == PV_HOOK_COMPRESS_GZIPDECOMPRESS) {
            if (l < 18 || pv_arena_get(ctx, p) != 0x1F || pv_arena_get(ctx, p + 1) != 0x8B) {
                ctx->regs[rd] = pv_arena_finish(ctx, 0);
                return;
            }
            uint8_t flg = pv_arena_get(ctx, p + 3);
            int32_t pos = 10;
            if (flg & 4) { int xlen = pv_arena_get(ctx, p + (uint32_t)pos) | (pv_arena_get(ctx, p + (uint32_t)pos + 1) << 8); pos += 2 + xlen; }
            if (flg & 8) { while (pos < l && pv_arena_get(ctx, p + (uint32_t)pos) != 0) pos++; pos++; }
            if (flg & 16) { while (pos < l && pv_arena_get(ctx, p + (uint32_t)pos) != 0) pos++; pos++; }
            if (flg & 2) pos += 2;
            s.incnt = pos;
            s.inlen = l - 8;                 /* exclude the 8-byte CRC32 + ISIZE trailer */
        }
        err = pv_inflate(&s);
        if (s.err || err != 0) k = 0;        /* truncated/corrupt -> empty span */
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }

    /* ---- Html.* (entity escape; single-pass, byte-exact w/ Python) ----- */
    if (hook == PV_HOOK_HTML_ENCODE) {
        int h = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, h);
        int32_t l = pv_span_n(ctx, h);
        uint32_t k = 0;
        for (int32_t i = 0; i < l; i++) {
            uint8_t c = pv_arena_get(ctx, p + (uint32_t)i);
            if (c == '&') pv_arena_puts(ctx, &k, "&amp;");
            else if (c == '<') pv_arena_puts(ctx, &k, "&lt;");
            else if (c == '>') pv_arena_puts(ctx, &k, "&gt;");
            else if (c == '"') pv_arena_puts(ctx, &k, "&quot;");
            else if (c == 0x27) pv_arena_puts(ctx, &k, "&#39;");
            else pv_arena_put(ctx, &k, c);
        }
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_HTML_DECODE) {
        int h = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, h);
        int32_t l = pv_span_n(ctx, h), i = 0;
        uint32_t k = 0;
        while (i < l) {
            uint8_t c = pv_arena_get(ctx, p + (uint32_t)i);
            if (c == '&') {
                if (pv_arena_match(ctx, p + (uint32_t)i, l - i, "&lt;"))   { pv_arena_put(ctx, &k, '<');  i += 4; continue; }
                if (pv_arena_match(ctx, p + (uint32_t)i, l - i, "&gt;"))   { pv_arena_put(ctx, &k, '>');  i += 4; continue; }
                if (pv_arena_match(ctx, p + (uint32_t)i, l - i, "&quot;")) { pv_arena_put(ctx, &k, '"');  i += 6; continue; }
                if (pv_arena_match(ctx, p + (uint32_t)i, l - i, "&#39;"))  { pv_arena_put(ctx, &k, 0x27); i += 5; continue; }
                if (pv_arena_match(ctx, p + (uint32_t)i, l - i, "&amp;"))  { pv_arena_put(ctx, &k, '&');  i += 5; continue; }
            }
            pv_arena_put(ctx, &k, c); i++;
        }
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }

    /* Utf8Writer / Utf8Reader / Json / Xml (arena-backed text/binary builders). */
    if (pv_textio(ctx, hook, rd, rs1, rs2)) return;

    /* A defined host-fillable primitive whose binding this default runtime does
     * not implement (the host supplies it on a real target). Return the
     * documented INV-18 default -- 0 with host_status = NOT_FOUND -- exactly
     * like the reserved-namespace stub above and picoscript_vm.py / picovm.js,
     * so every dialect and VM stay byte-parity. This is a *defined* default that
     * returns here, never a silent fall-through leaving regs[rd] stale. A real
     * target overrides by installing ctx->host. */
    if (hook >= 1 && hook <= PV_HOOK_CODE_MAX) {
        ctx->regs[rd] = 0;
        ctx->host_status = 1; /* INV-18: NOT_FOUND -- no host binding installed */
        return;
    }

    /* Out-of-range / genuinely unknown hook id: the compiler never emits a code
     * outside the defined space, so this is only reachable from malformed or
     * hand-crafted bytecode. Fail closed (fault deterministically) instead of
     * silently returning and leaving regs[rd] stale -- the default runtime must
     * not fabricate success for a hook that does not exist. */
    pv_set_fault(ctx, PV_FAULT_BAD_HOOK, ctx->cur_pc, hook);
}

/* Value-based host entry: the SAME implementation as the interpreter, callable
 * directly from emitted C (toC backend) so compiled programs skip the bytecode
 * VM and the string-keyed pv_host. Compiled C never uses ctx->regs for data, so
 * they are free scratch here; we marshal (a,b) -> regs, dispatch by hook code,
 * and read back the result. Accelerated ops (e.g. Dot8 -> NEON SDOT / SMLAD)
 * keep their inline lowering in _emit_c and are reached via pv_dot8. */
int64_t pv_host2(pv_ctx *ctx, int hook, int64_t a, int64_t b)
{
    int imm16 = (hook <= 0xFF) ? (PV_HOST_HOOK_BASE | hook)
                               : (PV_EXT_HOST_HOOK_BASE | (hook & 0x0FFF));
    ctx->regs[1] = (int32_t)a;
    ctx->regs[2] = (int32_t)b;
    ctx->regs[0] = 0;
    (ctx->host ? ctx->host : pv_default_host)(ctx, hook, 0, 1, 2, imm16);
    return (int64_t)ctx->regs[0];
}

/* Value-kind introspection for `key` in the currently active map, for native
 * host code that needs more than the 2-arg hook ABI exposes (e.g. schema
 * validation telling an int field from a string one). Not a bytecode hook --
 * a plain C helper for embedders like host/picowal/storage_file.c.
 * Returns: 0 = int/bool, 1 = string/span, 2 = null, -1 = absent/no active map. */
int pv_map_value_kind(pv_ctx *ctx, int key_span_handle)
{
    int mi = ctx->map_active;
    int e;
    if (!(mi > 0 && mi < ctx->map_nmaps && ctx->map_used[mi])) return -1;
    e = pv_map_find_s(ctx, mi, key_span_handle);
    if (e < 0) return -1;
    return (int)ctx->me_vk[e];
}

int64_t pv_host(pv_ctx *ctx, const char *ns, const char *method, int64_t a, int64_t b)
{
    (void)ctx; (void)ns; (void)method; (void)a; (void)b;
    return 0;   /* emitted-C generic host stub; override per deployment */
}

/* ---- lifecycle ------------------------------------------------------- */

void pv_init(pv_ctx *ctx)
{
    pv_bzero(ctx, sizeof(*ctx));
    ctx->http_status = -1;
    ctx->http_type = 0;
    ctx->rng_state = 0x2545F4914F6CDD1DULL;
    ctx->max_steps = 1000000L;
    ctx->caps = PV_CAP_ALL;     /* default: every binding granted; host restricts to gate (INV-17) */
    ctx->const_floor = 0x8000;  /* INV-9: empty const region until literals are written */
    ctx->span_count = 1;        /* handle 0 reserved as the null span */
    ctx->arena_top = 0x8000;    /* bump pointer for span results (matches PicoVM) */
    ctx->w_count = 1;           /* Utf8Writer/Json/Xml: handle 0 reserved */
    ctx->r_count = 1;           /* Utf8Reader: handle 0 reserved */
    ctx->map_nmaps = 1;         /* Map.*: handle 0 reserved as the null map */
    ctx->desc_count = 1;        /* Descriptor.*: handle 0 reserved as null */
    ctx->lease_count = 1;       /* Lease.*: handle 0 reserved as null */
    ctx->fifo_count = 1;        /* Fifo.*: handle 0 reserved as null */
    ctx->log_count = 1;         /* Log.*: handle 0 reserved as null */
    ctx->html_count = 1;        /* Html.*: handle 0 reserved as null */
    ctx->host = pv_default_host;
    ctx->cur_pc = 0;
}

/* ---- interpreter core ------------------------------------------------ */

static int pv_branch(int mode, int32_t a, int32_t b)
{
    switch (mode) {
        case PV_BR_EQ: return a == b;
        case PV_BR_NE: return a != b;
        case PV_BR_LT: return a < b;
        case PV_BR_GT: return a > b;
        case PV_BR_LE: return a <= b;
        case PV_BR_GE: return a >= b;
        case PV_BR_Z:  return a == 0;
        case PV_BR_NZ: return a != 0;
        default:       return 0;
    }
}

static void pv_noop(pv_ctx *ctx, int rd, int rs1, int rs2, int imm16)
{
    if ((imm16 & 0xFF00) == PV_HOST_HOOK_BASE) {
        int hook = imm16 & 0x00FF;
        if (ctx->host) ctx->host(ctx, hook, rd, rs1, rs2, imm16);
    } else if ((imm16 & 0xF000) == PV_EXT_HOST_HOOK_BASE) {
        int hook = imm16 & 0x0FFF;                                  /* extended hostcall: hooks >= 0x100 */
        if (ctx->host) ctx->host(ctx, hook, rd, rs1, rs2, imm16);
    } else if ((imm16 & 0xF000) == PV_NET_STATUS_BASE) {
        ctx->http_status = imm16 & 0x0FFF;
    } else if ((imm16 & 0xF000) == 0xA000) {
        ctx->http_type = imm16;
    } else if (imm16 == PV_NET_BODY_MARKER) {
        /* body marker */
    } else if (imm16 == PV_NET_CLOSE_MARKER) {
        ctx->halted = 1;
    }
    /* else genuine NOOP */
}

/* INV-10: static verification before execution. Reject a program whose static (immediate)
 * JUMP/CALL/BRANCH targets are out of range, before running any instruction (fail-fast;
 * rejects a tampered/corrupt module up front). Register/indexed jumps are dynamic and stay
 * runtime-checked (INV-11). Returns a PV_FAULT_* code, or PV_FAULT_NONE if valid. */
int pv_verify(const uint32_t *program, int len, int *fault_pc, int *fault_detail)
{
    for (int i = 0; i < len; i++) {
        uint32_t w = program[i];
        int op    = (int)((w >> 28) & 0xF);
        int rs2   = (int)((w >> 16) & 0xF);
        int imm16 = (int)(w & 0xFFFF);
        int tgt;
        if (op == PV_OP_JUMP) {
            if (rs2 != PV_ADDR_IMM) continue;               /* register/indexed = dynamic */
            tgt = imm16;
        } else if (op == PV_OP_CALL) {
            tgt = imm16;
        } else if (op == PV_OP_BRANCH) {
            tgt = i + (int)(int16_t)(uint16_t)imm16;
        } else {
            continue;
        }
        if (tgt < 0 || tgt > len) {
            if (fault_pc) *fault_pc = i;
            if (fault_detail) *fault_detail = tgt;
            return PV_FAULT_BAD_JUMP;
        }
    }
    return PV_FAULT_NONE;
}

/* INV-23: validate a module container [MAGIC, ABI, HOOK_TABLE_VERSION, count, ...words]
 * (same wire format as pico_module.py / picovm.js). On success returns 0 and yields the
 * raw bytecode via out_words and out_count; otherwise a negative PV_MODULE_ERR_ code -- a
 * module built for a different ABI or host-hook table is refused before it can run. */
int pv_load_module(const uint32_t *container, int clen, const uint32_t **out_words, int *out_count)
{
    if (clen < 4) return PV_MODULE_ERR_TRUNCATED;
    if (container[0] != (uint32_t)PV_MODULE_MAGIC)        return PV_MODULE_ERR_MAGIC;
    if (container[1] != (uint32_t)PV_MODULE_ABI_VERSION)  return PV_MODULE_ERR_ABI;
    if (container[2] != (uint32_t)PV_HOOK_TABLE_VERSION)  return PV_MODULE_ERR_HOOKTABLE;
    if (container[3] != (uint32_t)(clen - 4))             return PV_MODULE_ERR_COUNT;
    if (out_words) *out_words = container + 4;
    if (out_count) *out_count = clen - 4;
    return 0;
}

long pv_vm_run(pv_ctx *ctx, const uint32_t *program, int len)
{
    int pc = 0;
    ctx->halted = 0;
    ctx->steps = 0;
    int vpc = 0, vdetail = 0;                                /* INV-10: verify before execution */
    int vf = pv_verify(program, len, &vpc, &vdetail);
    if (vf != PV_FAULT_NONE) { pv_set_fault(ctx, vf, vpc, vdetail); return ctx->steps; }
    while (!ctx->halted && pc < len) {
        int cur = pc;
        ctx->cur_pc = cur;
        if (ctx->steps >= ctx->max_steps) { pv_set_fault(ctx, PV_FAULT_STEP_BUDGET, cur, 0); break; }
        ctx->steps++;

        uint32_t w = program[pc];
        int op    = (int)((w >> 28) & 0xF);
        int rd    = (int)((w >> 24) & 0xF);
        int rs1   = (int)((w >> 20) & 0xF);
        int rs2   = (int)((w >> 16) & 0xF);
        int imm16 = (int)(w & 0xFFFF);
        pc++;

        switch (op) {
        case PV_OP_NOOP:
            pv_noop(ctx, rd, rs1, rs2, imm16);
            break;
        case PV_OP_LOAD:
            ctx->regs[rd] = pv_load(ctx, imm16);
            break;
        case PV_OP_SAVE:
            pv_save(ctx, imm16, ctx->regs[rs1]);
            break;
        case PV_OP_PIPE:
            pv_pipe(ctx, imm16, pv_load(ctx, imm16));
            break;
        case PV_OP_ADD: case PV_OP_SUB: case PV_OP_MUL: case PV_OP_DIV: {
            int32_t a = ctx->regs[rs1];
            int32_t b = (rs2 == PV_ADDR_REG) ? ctx->regs[imm16 & 0xF]
                                             : (int32_t)(int16_t)(uint16_t)imm16;
            int32_t r = 0;
            if (op == PV_OP_ADD) r = a + b;
            else if (op == PV_OP_SUB) r = a - b;
            else if (op == PV_OP_MUL) r = a * b;
            else if (b == 0) r = 0;
            else if (b == -1 && a == (int32_t)0x80000000) r = a;  /* INT_MIN/-1: wrap, avoid UB */
            else r = a / b;                                       /* truncates toward zero */
            ctx->regs[rd] = r;
            break;
        }
        case PV_OP_INC:
            ctx->regs[rd] = ctx->regs[rd] + 1;
            break;
        case PV_OP_JUMP: {
            int tgt;
            if (rs2 == PV_ADDR_REG)          tgt = ctx->regs[rs1] & 0xFFFF;          /* PC = Rs1 */
            else if (rs2 == PV_ADDR_REG_OFF) tgt = (ctx->regs[rs1] + imm16) & 0xFFFF; /* PC = Rs1 + imm16 */
            else                             tgt = imm16;
            if (tgt < 0 || tgt > len) { pv_set_fault(ctx, PV_FAULT_BAD_JUMP, cur, tgt); break; }
            pc = tgt;   /* tgt == len falls off the end == clean halt */
            break;
        }
        case PV_OP_BRANCH: {
            int off = (int)(int16_t)(uint16_t)imm16;
            if (pv_branch(rs2, ctx->regs[rd], ctx->regs[rs1])) {
                int tgt = cur + off;
                if (tgt < 0 || tgt > len) { pv_set_fault(ctx, PV_FAULT_BAD_JUMP, cur, tgt); break; }
                pc = tgt;
            }
            break;
        }
        case PV_OP_CALL:
            if (imm16 < 0 || imm16 > len) { pv_set_fault(ctx, PV_FAULT_BAD_JUMP, cur, imm16); break; }
            if (ctx->call_sp >= PV_MAX_CALL) { pv_set_fault(ctx, PV_FAULT_CALL_OVERFLOW, cur, 0); break; }
            ctx->call_stack[ctx->call_sp++] = pc;
            pc = imm16;
            break;
        case PV_OP_RETURN:
            if (ctx->call_sp > 0) pc = ctx->call_stack[--ctx->call_sp];
            else ctx->halted = 1;
            break;
        case PV_OP_WAIT:
            ctx->waiting = 1; ctx->halted = 1;
            break;
        case PV_OP_RAISE:
            break;
        case PV_OP_DSP: {
            int32_t a = ctx->regs[rs1];
            if (rs2 == 0x4)      ctx->regs[rd] = a < 0 ? 0 : a;
            else if (rs2 == 0x3) ctx->regs[rd] = a * (int32_t)(int16_t)(uint16_t)imm16;
            else if (rs2 == 0x9) ctx->regs[rd] = a + ctx->regs[imm16 & 0xF];
            break;
        }
        default:
            pv_set_fault(ctx, PV_FAULT_BAD_OPCODE, cur, op);
            break;
        }
        if (ctx->pending_jump_set) {
            pc = ctx->pending_jump;
            ctx->pending_jump_set = 0;
        }
    }
    return ctx->steps;
}

/* PicoCompress codec (vm/picocompress.c) compiled into this translation unit so
   every existing build that compiles picovm.c links pc_compress_buffer/_decompress.
   Hosted targets only -- picocompress.c needs <string.h>; freestanding builds skip
   it (the Compress.PicoCompress hooks are #if'd out to match). */
#if defined(__STDC_HOSTED__) && __STDC_HOSTED__
#include "picocompress.c"
#endif

/* PicoBrotli codec (vm/picobrotli.c) compiled into this translation unit so every
   build that compiles picovm.c links brotli_encode/brotli_decode. Hosted targets
   only -- picobrotli.c needs <string.h>/<stdlib.h>; freestanding builds skip it
   (the Compress.Brotli* hooks are #if'd out to match). */
#if defined(__STDC_HOSTED__) && __STDC_HOSTED__
#include "picobrotli.c"
#endif
