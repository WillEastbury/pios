/*
 * Poly1305 (RFC 8439 §2.5) — one-time authenticator.
 *
 * Reference (slow but compact) implementation using a 5x26-bit limb
 * representation for the accumulator. We do not promise this is
 * constant-time on all targets — the additions and multiplications
 * are constant-time, but the modular reduction has a couple of
 * data-dependent carry chains. Good enough for the spike; production
 * code should swap in a tight constant-time implementation.
 *
 * Math:
 *   acc = 0
 *   for each 16-byte block m:
 *     acc = ((acc + (m | 0x01_appended)) * r) mod (2^130 - 5)
 *   tag = (acc + s) mod 2^128
 */

#include "poly1305.h"

#include <string.h>

#include "util.h"

static inline uint32_t U8TO32_LE(const uint8_t* p) {
    return  (uint32_t)p[0]        | ((uint32_t)p[1] <<  8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void U32TO8_LE(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v;         p[1] = (uint8_t)(v >>  8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

void poly1305_init(poly1305_ctx_t* ctx, const uint8_t key[POLY1305_KEY_LEN]) {
    /* Clamp r per RFC 8439 §2.5.1. */
    ctx->r[0] = (U8TO32_LE(key +  0))      & 0x03ffffff;
    ctx->r[1] = (U8TO32_LE(key +  3) >> 2) & 0x03ffff03;
    ctx->r[2] = (U8TO32_LE(key +  6) >> 4) & 0x03ffc0ff;
    ctx->r[3] = (U8TO32_LE(key +  9) >> 6) & 0x03f03fff;
    ctx->r[4] = (U8TO32_LE(key + 12) >> 8) & 0x000fffff;

    ctx->s[0] = U8TO32_LE(key + 16);
    ctx->s[1] = U8TO32_LE(key + 20);
    ctx->s[2] = U8TO32_LE(key + 24);
    ctx->s[3] = U8TO32_LE(key + 28);

    ctx->h[0] = ctx->h[1] = ctx->h[2] = ctx->h[3] = ctx->h[4] = 0;
    ctx->buf_len = 0;
}

/* Process exactly N full 16-byte blocks from `m`. `final_block` =
 * 1 means we are processing the last (possibly short, already-padded)
 * block and the implicit hibit was already folded in by the caller. */
static void poly1305_blocks(poly1305_ctx_t* ctx,
                            const uint8_t* m, size_t bytes,
                            int hibit_present) {
    uint32_t r0 = ctx->r[0], r1 = ctx->r[1], r2 = ctx->r[2],
             r3 = ctx->r[3], r4 = ctx->r[4];
    uint32_t s1 = r1 * 5, s2 = r2 * 5, s3 = r3 * 5, s4 = r4 * 5;

    uint32_t h0 = ctx->h[0], h1 = ctx->h[1], h2 = ctx->h[2],
             h3 = ctx->h[3], h4 = ctx->h[4];

    while (bytes >= 16) {
        h0 += ((U8TO32_LE(m +  0))     ) & 0x03ffffff;
        h1 += ((U8TO32_LE(m +  3) >> 2)) & 0x03ffffff;
        h2 += ((U8TO32_LE(m +  6) >> 4)) & 0x03ffffff;
        h3 += ((U8TO32_LE(m +  9) >> 6)) & 0x03ffffff;
        h4 += ((U8TO32_LE(m + 12) >> 8)) | (hibit_present ? (1u << 24) : 0u);

        uint64_t d0 = (uint64_t)h0 * r0 + (uint64_t)h1 * s4 +
                      (uint64_t)h2 * s3 + (uint64_t)h3 * s2 +
                      (uint64_t)h4 * s1;
        uint64_t d1 = (uint64_t)h0 * r1 + (uint64_t)h1 * r0 +
                      (uint64_t)h2 * s4 + (uint64_t)h3 * s3 +
                      (uint64_t)h4 * s2;
        uint64_t d2 = (uint64_t)h0 * r2 + (uint64_t)h1 * r1 +
                      (uint64_t)h2 * r0 + (uint64_t)h3 * s4 +
                      (uint64_t)h4 * s3;
        uint64_t d3 = (uint64_t)h0 * r3 + (uint64_t)h1 * r2 +
                      (uint64_t)h2 * r1 + (uint64_t)h3 * r0 +
                      (uint64_t)h4 * s4;
        uint64_t d4 = (uint64_t)h0 * r4 + (uint64_t)h1 * r3 +
                      (uint64_t)h2 * r2 + (uint64_t)h3 * r1 +
                      (uint64_t)h4 * r0;

        uint32_t c;
        c  = (uint32_t)(d0 >> 26); h0 = (uint32_t)d0 & 0x03ffffff;
        d1 += c;
        c  = (uint32_t)(d1 >> 26); h1 = (uint32_t)d1 & 0x03ffffff;
        d2 += c;
        c  = (uint32_t)(d2 >> 26); h2 = (uint32_t)d2 & 0x03ffffff;
        d3 += c;
        c  = (uint32_t)(d3 >> 26); h3 = (uint32_t)d3 & 0x03ffffff;
        d4 += c;
        c  = (uint32_t)(d4 >> 26); h4 = (uint32_t)d4 & 0x03ffffff;
        h0 += c * 5;
        c  = h0 >> 26;             h0 &= 0x03ffffff;
        h1 += c;

        m += 16;
        bytes -= 16;
    }

    ctx->h[0] = h0; ctx->h[1] = h1; ctx->h[2] = h2;
    ctx->h[3] = h3; ctx->h[4] = h4;
}

void poly1305_update(poly1305_ctx_t* ctx, const uint8_t* msg, size_t len) {
    if (len == 0) return;

    /* Fill the partial-block buffer first. */
    if (ctx->buf_len) {
        size_t want = 16 - ctx->buf_len;
        size_t take = len < want ? len : want;
        memcpy(ctx->buf + ctx->buf_len, msg, take);
        ctx->buf_len += take;
        msg += take;
        len -= take;
        if (ctx->buf_len == 16) {
            poly1305_blocks(ctx, ctx->buf, 16, /*hibit*/1);
            ctx->buf_len = 0;
        }
    }

    /* Bulk full blocks straight from the input. */
    if (len >= 16) {
        size_t bulk = len & ~(size_t)15;
        poly1305_blocks(ctx, msg, bulk, /*hibit*/1);
        msg += bulk;
        len -= bulk;
    }

    /* Stash any tail bytes for the next call (or finish). */
    if (len) {
        memcpy(ctx->buf + ctx->buf_len, msg, len);
        ctx->buf_len += len;
    }
}

void poly1305_finish(poly1305_ctx_t* ctx, uint8_t tag[POLY1305_TAG_LEN]) {
    /* Pad the partial block per RFC §2.5.1: append 0x01, then zeros to
     * 16 bytes, then process WITHOUT folding in the implicit hibit
     * (because the 0x01 we appended is the explicit hibit). */
    if (ctx->buf_len) {
        ctx->buf[ctx->buf_len] = 0x01;
        for (size_t i = ctx->buf_len + 1; i < 16; i++) ctx->buf[i] = 0;
        poly1305_blocks(ctx, ctx->buf, 16, /*hibit*/0);
        ctx->buf_len = 0;
    }

    uint32_t h0 = ctx->h[0], h1 = ctx->h[1], h2 = ctx->h[2],
             h3 = ctx->h[3], h4 = ctx->h[4];

    uint32_t c;
    c  = h1 >> 26; h1 &= 0x03ffffff; h2 += c;
    c  = h2 >> 26; h2 &= 0x03ffffff; h3 += c;
    c  = h3 >> 26; h3 &= 0x03ffffff; h4 += c;
    c  = h4 >> 26; h4 &= 0x03ffffff; h0 += c * 5;
    c  = h0 >> 26; h0 &= 0x03ffffff; h1 += c;

    uint32_t g0 = h0 + 5;
    c  = g0 >> 26; g0 &= 0x03ffffff;
    uint32_t g1 = h1 + c;
    c  = g1 >> 26; g1 &= 0x03ffffff;
    uint32_t g2 = h2 + c;
    c  = g2 >> 26; g2 &= 0x03ffffff;
    uint32_t g3 = h3 + c;
    c  = g3 >> 26; g3 &= 0x03ffffff;
    uint32_t g4 = h4 + c - (1u << 26);

    /* Constant-time select between h (if h < p) and g (if h >= p).
     * Matches RFC 8439 §2.5.1 reference exactly: g4>>31 is 1 when
     * the trial subtraction underflowed (h<p, keep h) and 0 otherwise
     * (h>=p, keep g). Both `(g4>>31)-1` and `0u-(g4>>31)` are
     * well-defined unsigned wrap; we use the RFC's literal form. */
    uint32_t mask = (g4 >> 31) - 1;
    g0 &= mask; g1 &= mask; g2 &= mask; g3 &= mask; g4 &= mask;
    mask = ~mask;
    h0 = (h0 & mask) | g0;
    h1 = (h1 & mask) | g1;
    h2 = (h2 & mask) | g2;
    h3 = (h3 & mask) | g3;
    h4 = (h4 & mask) | g4;

    uint32_t f0 = (h0      ) | (h1 << 26);
    uint32_t f1 = (h1 >>  6) | (h2 << 20);
    uint32_t f2 = (h2 >> 12) | (h3 << 14);
    uint32_t f3 = (h3 >> 18) | (h4 <<  8);

    uint64_t f;
    f  = (uint64_t)f0 + ctx->s[0];                f0 = (uint32_t)f;
    f  = (uint64_t)f1 + ctx->s[1] + (f >> 32);    f1 = (uint32_t)f;
    f  = (uint64_t)f2 + ctx->s[2] + (f >> 32);    f2 = (uint32_t)f;
    f  = (uint64_t)f3 + ctx->s[3] + (f >> 32);    f3 = (uint32_t)f;

    U32TO8_LE(tag +  0, f0);
    U32TO8_LE(tag +  4, f1);
    U32TO8_LE(tag +  8, f2);
    U32TO8_LE(tag + 12, f3);

    /* Wipe accumulator + key material — defence in depth even though
     * the caller's supposed to do this if they care. */
    secure_zero(ctx, sizeof(*ctx));
}

void poly1305(const uint8_t key[POLY1305_KEY_LEN],
              const uint8_t* msg, size_t len,
              uint8_t       tag[POLY1305_TAG_LEN]) {
    poly1305_ctx_t ctx;
    poly1305_init(&ctx, key);
    poly1305_update(&ctx, msg, len);
    poly1305_finish(&ctx, tag);
}
