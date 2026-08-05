/*
 * sha256_hkdf.c - SHA-256, HMAC-SHA256, and HKDF (RFC 5869).
 *
 * Extracted from crypto.c so this pure-logic code (no MMIO, no inline
 * asm/ARM crypto-extension dependency) can be host-compiled and tested
 * directly -- see tests/test_tls13_keysched.c, which validates the whole
 * chain (via src/tls13_keysched.c) against the official RFC 8448 byte
 * trace. crypto.c keeps its own private copies of load_be32/store_be32/
 * store_be64 for its AES/AES-GCM code (which stays ARM-crypto-extension
 * accelerated); those three tiny, stable byte-order helpers are
 * intentionally duplicated here rather than shared through a header, to
 * avoid coupling this freestanding-and-hostable file to crypto.c's ASM.
 */
#include "crypto.h"
#include "simd.h"

static const u32 sha256_k[64] = {
    0x428A2F98U,0x71374491U,0xB5C0FBCFU,0xE9B5DBA5U,0x3956C25BU,0x59F111F1U,0x923F82A4U,0xAB1C5ED5U,
    0xD807AA98U,0x12835B01U,0x243185BEU,0x550C7DC3U,0x72BE5D74U,0x80DEB1FEU,0x9BDC06A7U,0xC19BF174U,
    0xE49B69C1U,0xEFBE4786U,0x0FC19DC6U,0x240CA1CCU,0x2DE92C6FU,0x4A7484AAU,0x5CB0A9DCU,0x76F988DAU,
    0x983E5152U,0xA831C66DU,0xB00327C8U,0xBF597FC7U,0xC6E00BF3U,0xD5A79147U,0x06CA6351U,0x14292967U,
    0x27B70A85U,0x2E1B2138U,0x4D2C6DFCU,0x53380D13U,0x650A7354U,0x766A0ABBU,0x81C2C92EU,0x92722C85U,
    0xA2BFE8A1U,0xA81A664BU,0xC24B8B70U,0xC76C51A3U,0xD192E819U,0xD6990624U,0xF40E3585U,0x106AA070U,
    0x19A4C116U,0x1E376C08U,0x2748774CU,0x34B0BCB5U,0x391C0CB3U,0x4ED8AA4AU,0x5B9CCA4FU,0x682E6FF3U,
    0x748F82EEU,0x78A5636FU,0x84C87814U,0x8CC70208U,0x90BEFFFAU,0xA4506CEBU,0xBEF9A3F7U,0xC67178F2U
};

static inline u32 rotr32(u32 x, u32 n) {
    return (x >> n) | (x << (32U - n));
}

static inline u32 load_be32(const u8 *p) {
    return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | (u32)p[3];
}

static inline void store_be32(u8 *p, u32 v) {
    p[0] = (u8)(v >> 24);
    p[1] = (u8)(v >> 16);
    p[2] = (u8)(v >> 8);
    p[3] = (u8)v;
}

static inline void store_be64(u8 *p, u64 v) {
    p[0] = (u8)(v >> 56);
    p[1] = (u8)(v >> 48);
    p[2] = (u8)(v >> 40);
    p[3] = (u8)(v >> 32);
    p[4] = (u8)(v >> 24);
    p[5] = (u8)(v >> 16);
    p[6] = (u8)(v >> 8);
    p[7] = (u8)v;
}

static void sha256_transform(struct sha256_ctx *ctx, const u8 *block) {
    u32 w[64];
    for (u32 i = 0; i < 16; i++)
        w[i] = load_be32(block + (i * 4));
    for (u32 i = 16; i < 64; i++) {
        u32 s0 = rotr32(w[i - 15], 7) ^ rotr32(w[i - 15], 18) ^ (w[i - 15] >> 3);
        u32 s1 = rotr32(w[i - 2], 17) ^ rotr32(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    u32 a = ctx->state[0], b = ctx->state[1], c = ctx->state[2], d = ctx->state[3];
    u32 e = ctx->state[4], f = ctx->state[5], g = ctx->state[6], h = ctx->state[7];

    for (u32 i = 0; i < 64; i++) {
        u32 S1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
        u32 ch = (e & f) ^ ((~e) & g);
        u32 temp1 = h + S1 + ch + sha256_k[i] + w[i];
        u32 S0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
        u32 maj = (a & b) ^ (a & c) ^ (b & c);
        u32 temp2 = S0 + maj;

        h = g;
        g = f;
        f = e;
        e = d + temp1;
        d = c;
        c = b;
        b = a;
        a = temp1 + temp2;
    }

    ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c; ctx->state[3] += d;
    ctx->state[4] += e; ctx->state[5] += f; ctx->state[6] += g; ctx->state[7] += h;
}

void sha256_init(struct sha256_ctx *ctx) {
    if (!ctx) return;
    ctx->state[0] = 0x6A09E667U;
    ctx->state[1] = 0xBB67AE85U;
    ctx->state[2] = 0x3C6EF372U;
    ctx->state[3] = 0xA54FF53AU;
    ctx->state[4] = 0x510E527FU;
    ctx->state[5] = 0x9B05688CU;
    ctx->state[6] = 0x1F83D9ABU;
    ctx->state[7] = 0x5BE0CD19U;
    ctx->total_len = 0;
    ctx->buf_len = 0;
}

void sha256_update(struct sha256_ctx *ctx, const u8 *data, u32 len) {
    if (!ctx || (!data && len)) return;

    ctx->total_len += len;

    if (ctx->buf_len) {
        u32 need = 64 - ctx->buf_len;
        if (need > len) need = len;
        simd_memcpy(ctx->buf + ctx->buf_len, data, need);
        ctx->buf_len += need;
        data += need;
        len -= need;
        if (ctx->buf_len == 64) {
            sha256_transform(ctx, ctx->buf);
            ctx->buf_len = 0;
        }
    }

    while (len >= 64) {
        sha256_transform(ctx, data);
        data += 64;
        len -= 64;
    }

    if (len) {
        simd_memcpy(ctx->buf, data, len);
        ctx->buf_len = len;
    }
}

void sha256_final(struct sha256_ctx *ctx, u8 *hash) {
    u64 bits_len;
    u8 len_bytes[8];
    if (!ctx || !hash) return;

    bits_len = ctx->total_len * 8U;
    store_be64(len_bytes, bits_len);

    ctx->buf[ctx->buf_len++] = 0x80;
    if (ctx->buf_len > 56) {
        while (ctx->buf_len < 64) ctx->buf[ctx->buf_len++] = 0;
        sha256_transform(ctx, ctx->buf);
        ctx->buf_len = 0;
    }
    while (ctx->buf_len < 56) ctx->buf[ctx->buf_len++] = 0;
    simd_memcpy(ctx->buf + 56, len_bytes, 8);
    sha256_transform(ctx, ctx->buf);

    for (u32 i = 0; i < 8; i++)
        store_be32(hash + i * 4, ctx->state[i]);
}

void sha256(const u8 *data, u32 len, u8 *hash) {
    struct sha256_ctx ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, data, len);
    sha256_final(&ctx, hash);
}

static void hmac_sha256_parts(const u8 *key, u32 key_len,
                              const u8 *p1, u32 l1,
                              const u8 *p2, u32 l2,
                              const u8 *p3, u32 l3,
                              u8 *mac) {
    u8 k0[64];
    u8 tk[32];
    u8 ipad[64];
    u8 opad[64];
    u8 inner[32];
    struct sha256_ctx s;

    simd_zero(k0, sizeof(k0));
    if (key_len > 64) {
        sha256(key, key_len, tk);
        simd_memcpy(k0, tk, 32);
    } else if (key_len) {
        simd_memcpy(k0, key, key_len);
    }

    for (u32 i = 0; i < 64; i++) {
        ipad[i] = k0[i] ^ 0x36U;
        opad[i] = k0[i] ^ 0x5CU;
    }

    sha256_init(&s);
    sha256_update(&s, ipad, 64);
    if (l1) sha256_update(&s, p1, l1);
    if (l2) sha256_update(&s, p2, l2);
    if (l3) sha256_update(&s, p3, l3);
    sha256_final(&s, inner);

    sha256_init(&s);
    sha256_update(&s, opad, 64);
    sha256_update(&s, inner, 32);
    sha256_final(&s, mac);
}

void hmac_sha256(const u8 *key, u32 key_len,
                 const u8 *data, u32 data_len,
                 u8 *mac) {
    hmac_sha256_parts(key, key_len, data, data_len, NULL, 0, NULL, 0, mac);
}

void hkdf_extract(const u8 *salt, u32 salt_len,
                  const u8 *ikm, u32 ikm_len,
                  u8 *prk) {
    u8 zero_salt[32];
    if (!prk || (!ikm && ikm_len))
        return;

    if (!salt || salt_len == 0) {
        simd_zero(zero_salt, sizeof(zero_salt));
        hmac_sha256(zero_salt, sizeof(zero_salt), ikm, ikm_len, prk);
        return;
    }

    hmac_sha256(salt, salt_len, ikm, ikm_len, prk);
}

void hkdf_expand(const u8 *prk, u32 prk_len,
                 const u8 *info, u32 info_len,
                 u8 *okm, u32 okm_len) {
    u8 t[32];
    u8 prev[32];
    u32 produced = 0;
    u8 ctr = 1;
    u32 prev_len = 0;

    if (!prk || !okm || prk_len == 0 || okm_len == 0)
        return;

    while (produced < okm_len) {
        hmac_sha256_parts(prk, prk_len,
                          prev, prev_len,
                          info, info_len,
                          &ctr, 1,
                          t);

        u32 chunk = okm_len - produced;
        if (chunk > 32) chunk = 32;
        simd_memcpy(okm + produced, t, chunk);
        simd_memcpy(prev, t, 32);
        prev_len = 32;
        produced += chunk;
        ctr++;
    }
}
