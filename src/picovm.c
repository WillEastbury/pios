/* picovm.c -- portable C implementation of the PicoScript 16-opcode VM.
 *
 * Mirrors picoscript_vm.PicoVM._step exactly so the same bytecode yields the
 * same register file, output bytes and HTTP markers on host and on bare metal.
 */
#include "picovm.h"
#include "pico_hooks.h"

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
    pv_emit_word(ctx, (uint32_t)val);
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
    if (!ctx->mem || ctx->mem_size <= 0) return 0;
    {
        const int8_t *w = (const int8_t *)(ctx->mem + (wptr % (uint32_t)ctx->mem_size));
        const int8_t *a = (const int8_t *)(ctx->mem + (aptr % (uint32_t)ctx->mem_size));
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
    int h = pv_span_make(ctx, ctx->arena_top, (int32_t)k);
    ctx->arena_top += k;
    return h;
}
static void pv_arena_puts(pv_ctx *ctx, uint32_t *k, const char *s)
{
    while (*s) pv_arena_put(ctx, k, (uint8_t)*s++);
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
    if (hook >= 0x07 && hook <= 0x0E) return PV_CAP_NET;     /* Req.* */
    if (hook >= 0x10 && hook <= 0x14) return PV_CAP_QUEUE;   /* Queue.* */
    if ((hook >= 0x15 && hook <= 0x1F) || hook == 0x38 || hook == 0x39) return PV_CAP_NET;  /* Resp.* */
    if (hook == PV_HOOK_RANDOM_U32) return PV_CAP_RANDOM;    /* Random.U32 */
    if (hook >= 0x60 && hook <= 0x6E) return PV_CAP_STORAGE; /* Storage.* */
    if (hook >= 0xB0 && hook <= 0xBA) return PV_CAP_TIME;    /* DateTime.* */
    if (hook >= 0xC0 && hook <= 0xC6) return PV_CAP_ENV;     /* Locale.* */
    if (hook >= 0xD0 && hook <= 0xD8) return PV_CAP_ENV;     /* Environment.* */
    if (hook >= 0xE0 && hook <= 0xEE) return PV_CAP_CONTEXT; /* Context.* */
    if (hook >= 0x110 && hook <= 0x117) return PV_CAP_AUTH;  /* X509.* */
    if (hook >= 0x120 && hook <= 0x129) return PV_CAP_AUTH;  /* Auth.* */
    if (hook >= 0x150 && hook <= 0x156) return PV_CAP_GPIO;  /* Gpio.* */
    if (hook >= 0x160 && hook <= 0x167) return PV_CAP_CAPSULE; /* Pack/Card/Fifo */
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

void pv_default_host(pv_ctx *ctx, int hook, int rd, int rs1, int rs2, int imm16)
{
    (void)imm16;
    /* INV-17: bindings are not ambient -- deny the hook unless its class is granted. */
    uint32_t need = pv_hook_cap(hook);
    if (need && !(ctx->caps & need)) { pv_set_fault(ctx, PV_FAULT_CAPABILITY, ctx->cur_pc, hook); return; }
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
        if (ctx->mem_size) a %= (uint32_t)ctx->mem_size;   /* match Python/JS wrap before lowering the floor */
        pv_mem_set(ctx, a, ctx->regs[rs2]);
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
        int valid = (i < e);
        uint32_t val = 0;
        for (; i < e; i++) {
            uint8_t c = pv_arena_get(ctx, pa + (uint32_t)i);
            if (c < '0' || c > '9') { valid = 0; break; }
            val = val * 10u + (uint32_t)(c - '0');
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

    /* ---- Compress.* (reversible byte-run RLE -> (count,byte) pairs) ---- */
    if (hook == PV_HOOK_COMPRESS_PICOCOMPRESS) {
        int h = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, h);
        int32_t l = pv_span_n(ctx, h), i = 0;
        uint32_t k = 0;
        while (i < l) {
            uint8_t b0 = pv_arena_get(ctx, p + (uint32_t)i);
            int32_t c = 1;
            while (i + c < l && pv_arena_get(ctx, p + (uint32_t)(i + c)) == b0 && c < 255) c++;
            pv_arena_put(ctx, &k, (uint8_t)c);
            pv_arena_put(ctx, &k, b0);
            i += c;
        }
        ctx->regs[rd] = pv_arena_finish(ctx, k);
        return;
    }
    if (hook == PV_HOOK_COMPRESS_PICODECOMPRESS) {
        int h = ctx->regs[rs1];
        uint32_t p = pv_span_p(ctx, h);
        int32_t l = pv_span_n(ctx, h), i = 0;
        uint32_t k = 0;
        while (i + 1 < l) {
            uint8_t cnt = pv_arena_get(ctx, p + (uint32_t)i);
            uint8_t b0 = pv_arena_get(ctx, p + (uint32_t)(i + 1));
            for (uint8_t t = 0; t < cnt; t++) pv_arena_put(ctx, &k, b0);
            i += 2;
        }
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

    /* unknown host-fillable primitive: ignore (host supplies on real target) */
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
    pv_default_host(ctx, hook, 0, 1, 2, imm16);
    return (int64_t)ctx->regs[0];
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
    }
    return ctx->steps;
}
