#include "crypto.h"
#include "simd.h"

/* ---- AES tables ---- */

static const u8 aes_sbox[256] = {
    0x63,0x7C,0x77,0x7B,0xF2,0x6B,0x6F,0xC5,0x30,0x01,0x67,0x2B,0xFE,0xD7,0xAB,0x76,
    0xCA,0x82,0xC9,0x7D,0xFA,0x59,0x47,0xF0,0xAD,0xD4,0xA2,0xAF,0x9C,0xA4,0x72,0xC0,
    0xB7,0xFD,0x93,0x26,0x36,0x3F,0xF7,0xCC,0x34,0xA5,0xE5,0xF1,0x71,0xD8,0x31,0x15,
    0x04,0xC7,0x23,0xC3,0x18,0x96,0x05,0x9A,0x07,0x12,0x80,0xE2,0xEB,0x27,0xB2,0x75,
    0x09,0x83,0x2C,0x1A,0x1B,0x6E,0x5A,0xA0,0x52,0x3B,0xD6,0xB3,0x29,0xE3,0x2F,0x84,
    0x53,0xD1,0x00,0xED,0x20,0xFC,0xB1,0x5B,0x6A,0xCB,0xBE,0x39,0x4A,0x4C,0x58,0xCF,
    0xD0,0xEF,0xAA,0xFB,0x43,0x4D,0x33,0x85,0x45,0xF9,0x02,0x7F,0x50,0x3C,0x9F,0xA8,
    0x51,0xA3,0x40,0x8F,0x92,0x9D,0x38,0xF5,0xBC,0xB6,0xDA,0x21,0x10,0xFF,0xF3,0xD2,
    0xCD,0x0C,0x13,0xEC,0x5F,0x97,0x44,0x17,0xC4,0xA7,0x7E,0x3D,0x64,0x5D,0x19,0x73,
    0x60,0x81,0x4F,0xDC,0x22,0x2A,0x90,0x88,0x46,0xEE,0xB8,0x14,0xDE,0x5E,0x0B,0xDB,
    0xE0,0x32,0x3A,0x0A,0x49,0x06,0x24,0x5C,0xC2,0xD3,0xAC,0x62,0x91,0x95,0xE4,0x79,
    0xE7,0xC8,0x37,0x6D,0x8D,0xD5,0x4E,0xA9,0x6C,0x56,0xF4,0xEA,0x65,0x7A,0xAE,0x08,
    0xBA,0x78,0x25,0x2E,0x1C,0xA6,0xB4,0xC6,0xE8,0xDD,0x74,0x1F,0x4B,0xBD,0x8B,0x8A,
    0x70,0x3E,0xB5,0x66,0x48,0x03,0xF6,0x0E,0x61,0x35,0x57,0xB9,0x86,0xC1,0x1D,0x9E,
    0xE1,0xF8,0x98,0x11,0x69,0xD9,0x8E,0x94,0x9B,0x1E,0x87,0xE9,0xCE,0x55,0x28,0xDF,
    0x8C,0xA1,0x89,0x0D,0xBF,0xE6,0x42,0x68,0x41,0x99,0x2D,0x0F,0xB0,0x54,0xBB,0x16
};

static const u8 aes_inv_sbox[256] = {
    0x52,0x09,0x6A,0xD5,0x30,0x36,0xA5,0x38,0xBF,0x40,0xA3,0x9E,0x81,0xF3,0xD7,0xFB,
    0x7C,0xE3,0x39,0x82,0x9B,0x2F,0xFF,0x87,0x34,0x8E,0x43,0x44,0xC4,0xDE,0xE9,0xCB,
    0x54,0x7B,0x94,0x32,0xA6,0xC2,0x23,0x3D,0xEE,0x4C,0x95,0x0B,0x42,0xFA,0xC3,0x4E,
    0x08,0x2E,0xA1,0x66,0x28,0xD9,0x24,0xB2,0x76,0x5B,0xA2,0x49,0x6D,0x8B,0xD1,0x25,
    0x72,0xF8,0xF6,0x64,0x86,0x68,0x98,0x16,0xD4,0xA4,0x5C,0xCC,0x5D,0x65,0xB6,0x92,
    0x6C,0x70,0x48,0x50,0xFD,0xED,0xB9,0xDA,0x5E,0x15,0x46,0x57,0xA7,0x8D,0x9D,0x84,
    0x90,0xD8,0xAB,0x00,0x8C,0xBC,0xD3,0x0A,0xF7,0xE4,0x58,0x05,0xB8,0xB3,0x45,0x06,
    0xD0,0x2C,0x1E,0x8F,0xCA,0x3F,0x0F,0x02,0xC1,0xAF,0xBD,0x03,0x01,0x13,0x8A,0x6B,
    0x3A,0x91,0x11,0x41,0x4F,0x67,0xDC,0xEA,0x97,0xF2,0xCF,0xCE,0xF0,0xB4,0xE6,0x73,
    0x96,0xAC,0x74,0x22,0xE7,0xAD,0x35,0x85,0xE2,0xF9,0x37,0xE8,0x1C,0x75,0xDF,0x6E,
    0x47,0xF1,0x1A,0x71,0x1D,0x29,0xC5,0x89,0x6F,0xB7,0x62,0x0E,0xAA,0x18,0xBE,0x1B,
    0xFC,0x56,0x3E,0x4B,0xC6,0xD2,0x79,0x20,0x9A,0xDB,0xC0,0xFE,0x78,0xCD,0x5A,0xF4,
    0x1F,0xDD,0xA8,0x33,0x88,0x07,0xC7,0x31,0xB1,0x12,0x10,0x59,0x27,0x80,0xEC,0x5F,
    0x60,0x51,0x7F,0xA9,0x19,0xB5,0x4A,0x0D,0x2D,0xE5,0x7A,0x9F,0x93,0xC9,0x9C,0xEF,
    0xA0,0xE0,0x3B,0x4D,0xAE,0x2A,0xF5,0xB0,0xC8,0xEB,0xBB,0x3C,0x83,0x53,0x99,0x61,
    0x17,0x2B,0x04,0x7E,0xBA,0x77,0xD6,0x26,0xE1,0x69,0x14,0x63,0x55,0x21,0x0C,0x7D
};

static const u8 aes_rcon[15] = {
    0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1B,0x36,0x6C,0xD8,0xAB,0x4D
};

/* ---- SHA-256 constants ---- */

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

/* clang-format off */
static const u8 zero_block[16] = {0};
/* clang-format on */

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

static inline u8 gf_xtime(u8 x) {
    return (u8)((x << 1) ^ ((x & 0x80) ? 0x1BU : 0x00U));
}

static u8 gf_mul(u8 a, u8 b) {
    u8 p = 0;
    for (u32 i = 0; i < 8; i++) {
        if (b & 1) p ^= a;
        b >>= 1;
        a = gf_xtime(a);
    }
    return p;
}

static inline u32 aes_sub_word(u32 w) {
    return ((u32)aes_sbox[(w >> 24) & 0xFF] << 24) |
           ((u32)aes_sbox[(w >> 16) & 0xFF] << 16) |
           ((u32)aes_sbox[(w >> 8) & 0xFF] << 8) |
           (u32)aes_sbox[w & 0xFF];
}

static inline u32 aes_rot_word(u32 w) {
    return (w << 8) | (w >> 24);
}

static void add_round_key(u8 *state, const u8 *rk) {
    for (u32 i = 0; i < 16; i++)
        state[i] ^= rk[i];
}

static void inv_shift_rows(u8 *s) {
    u8 t;
    t = s[13]; s[13] = s[9];  s[9]  = s[5];  s[5]  = s[1];  s[1]  = t;
    t = s[2];  s[2]  = s[10]; s[10] = t;     t = s[6]; s[6] = s[14]; s[14] = t;
    t = s[3];  s[3]  = s[7];  s[7]  = s[11]; s[11] = s[15]; s[15] = t;
}

static void inv_sub_bytes(u8 *s) {
    for (u32 i = 0; i < 16; i++)
        s[i] = aes_inv_sbox[s[i]];
}

static void inv_mix_columns(u8 *s) {
    for (u32 c = 0; c < 4; c++) {
        u32 i = c * 4;
        u8 a0 = s[i + 0], a1 = s[i + 1], a2 = s[i + 2], a3 = s[i + 3];
        s[i + 0] = gf_mul(a0, 0x0E) ^ gf_mul(a1, 0x0B) ^ gf_mul(a2, 0x0D) ^ gf_mul(a3, 0x09);
        s[i + 1] = gf_mul(a0, 0x09) ^ gf_mul(a1, 0x0E) ^ gf_mul(a2, 0x0B) ^ gf_mul(a3, 0x0D);
        s[i + 2] = gf_mul(a0, 0x0D) ^ gf_mul(a1, 0x09) ^ gf_mul(a2, 0x0E) ^ gf_mul(a3, 0x0B);
        s[i + 3] = gf_mul(a0, 0x0B) ^ gf_mul(a1, 0x0D) ^ gf_mul(a2, 0x09) ^ gf_mul(a3, 0x0E);
    }
}

static inline void aes_round_arm(u8 *state, const u8 *rk, bool mix_columns) {
    if (mix_columns) {
        __asm__ volatile(
            "ldr q0, [%0]\n"
            "ldr q1, [%1]\n"
            "aese v0.16b, v1.16b\n"
            "aesmc v0.16b, v0.16b\n"
            "str q0, [%0]\n"
            :
            : "r"(state), "r"(rk)
            : "v0", "v1", "memory");
    } else {
        __asm__ volatile(
            "ldr q0, [%0]\n"
            "ldr q1, [%1]\n"
            "aese v0.16b, v1.16b\n"
            "str q0, [%0]\n"
            :
            : "r"(state), "r"(rk)
            : "v0", "v1", "memory");
    }
}

static void ghash_shift_right_one(u8 *v) {
    u8 carry = 0;
    for (u32 i = 0; i < 16; i++) {
        u32 idx = 15 - i;
        u8 new_carry = v[idx] & 1U;
        v[idx] = (u8)((v[idx] >> 1) | (carry << 7));
        carry = new_carry;
    }
}

static void ghash_mul(u8 *x, const u8 *h) {
    u8 z[16];
    u8 v[16];
    simd_zero(z, sizeof(z));
    simd_memcpy(v, h, sizeof(v));

    for (u32 i = 0; i < 16; i++) {
        u8 xi = x[i];
        for (u32 b = 0; b < 8; b++) {
            if (xi & 0x80) {
                for (u32 j = 0; j < 16; j++)
                    z[j] ^= v[j];
            }

            bool lsb = (v[15] & 1U) != 0;
            ghash_shift_right_one(v);
            if (lsb)
                v[0] ^= 0xE1U;
            xi <<= 1;
        }
    }

    simd_memcpy(x, z, sizeof(z));
}

static void ghash_update(u8 *y, const u8 *h, const u8 *data, u32 len) {
    while (len >= 16) {
        for (u32 i = 0; i < 16; i++)
            y[i] ^= data[i];
        ghash_mul(y, h);
        data += 16;
        len  -= 16;
    }

    if (len) {
        u8 last[16];
        simd_zero(last, sizeof(last));
        simd_memcpy(last, data, len);
        for (u32 i = 0; i < 16; i++)
            y[i] ^= last[i];
        ghash_mul(y, h);
    }
}

static void gcm_make_j0(u8 *j0, const u8 *nonce, u32 nonce_len) {
    if (nonce_len == 12) {
        simd_memcpy(j0, nonce, 12);
        j0[12] = 0;
        j0[13] = 0;
        j0[14] = 0;
        j0[15] = 1;
        return;
    }

    /* Conservative fallback: only 96-bit nonce is supported for now. */
    simd_zero(j0, 16);
}

static void gcm_inc32(u8 *counter) {
    u32 n = ((u32)counter[12] << 24) | ((u32)counter[13] << 16) |
            ((u32)counter[14] << 8) | (u32)counter[15];
    n++;
    counter[12] = (u8)(n >> 24);
    counter[13] = (u8)(n >> 16);
    counter[14] = (u8)(n >> 8);
    counter[15] = (u8)n;
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

void aes_key_expand(struct aes_key *ctx, const u8 *key, u32 key_bits) {
    u32 nk, nr;
    u32 words;
    u32 w[60];

    if (!ctx || !key) return;

    if (key_bits == 128) {
        nk = 4;
        nr = 10;
    } else if (key_bits == 256) {
        nk = 8;
        nr = 14;
    } else {
        simd_zero(ctx, sizeof(*ctx));
        return;
    }

    words = 4 * (nr + 1);

    for (u32 i = 0; i < nk; i++)
        w[i] = load_be32(key + i * 4);

    for (u32 i = nk; i < words; i++) {
        u32 temp = w[i - 1];
        if ((i % nk) == 0) {
            temp = aes_sub_word(aes_rot_word(temp));
            temp ^= (u32)aes_rcon[i / nk] << 24;
        } else if (nk > 6 && (i % nk) == 4) {
            temp = aes_sub_word(temp);
        }
        w[i] = w[i - nk] ^ temp;
    }

    for (u32 i = 0; i < words; i++)
        store_be32(ctx->round_keys + i * 4, w[i]);

    ctx->rounds = nr;
}

void aes_encrypt_block(const struct aes_key *ctx, const u8 *in, u8 *out) {
    u8 state[16];

    if (!ctx || !in || !out || (ctx->rounds != 10 && ctx->rounds != 14))
        return;

    simd_memcpy(state, in, 16);
    add_round_key(state, &ctx->round_keys[0]);

    for (u32 r = 1; r < ctx->rounds; r++)
        aes_round_arm(state, &ctx->round_keys[r * 16], true);

    aes_round_arm(state, &ctx->round_keys[ctx->rounds * 16], false);
    simd_memcpy(out, state, 16);
}

void aes_decrypt_block(const struct aes_key *ctx, const u8 *in, u8 *out) {
    u8 state[16];
    if (!ctx || !in || !out || (ctx->rounds != 10 && ctx->rounds != 14))
        return;

    simd_memcpy(state, in, 16);
    add_round_key(state, &ctx->round_keys[ctx->rounds * 16]);

    for (u32 r = ctx->rounds - 1; r > 0; r--) {
        inv_shift_rows(state);
        inv_sub_bytes(state);
        add_round_key(state, &ctx->round_keys[r * 16]);
        inv_mix_columns(state);
    }

    inv_shift_rows(state);
    inv_sub_bytes(state);
    add_round_key(state, &ctx->round_keys[0]);
    simd_memcpy(out, state, 16);
}

void aes_gcm_init(struct aes_gcm_ctx *ctx, const u8 *key, u32 key_bits) {
    if (!ctx || !key) return;
    aes_key_expand(&ctx->key, key, key_bits);
    aes_encrypt_block(&ctx->key, zero_block, ctx->h);
    simd_zero(ctx->j0, sizeof(ctx->j0));
}

bool aes_gcm_encrypt(struct aes_gcm_ctx *ctx,
                     const u8 *nonce, u32 nonce_len,
                     const u8 *aad, u32 aad_len,
                     const u8 *plain, u32 plain_len,
                     u8 *cipher, u8 *tag) {
    u8 y[16];
    u8 ctr[16];
    u8 stream[16];
    u8 len_block[16];
    u8 s[16];
    u32 offset = 0;

    if (!ctx || !nonce || !plain || !cipher || !tag)
        return false;
    if (nonce_len != 12)
        return false;

    gcm_make_j0(ctx->j0, nonce, nonce_len);
    simd_memcpy(ctr, ctx->j0, 16);
    gcm_inc32(ctr);

    while (offset < plain_len) {
        u32 chunk = plain_len - offset;
        if (chunk > 16) chunk = 16;

        aes_encrypt_block(&ctx->key, ctr, stream);
        for (u32 i = 0; i < chunk; i++)
            cipher[offset + i] = plain[offset + i] ^ stream[i];

        offset += chunk;
        gcm_inc32(ctr);
    }

    simd_zero(y, sizeof(y));
    if (aad && aad_len)
        ghash_update(y, ctx->h, aad, aad_len);
    if (plain_len)
        ghash_update(y, ctx->h, cipher, plain_len);

    store_be64(len_block + 0, (u64)aad_len * 8U);
    store_be64(len_block + 8, (u64)plain_len * 8U);
    for (u32 i = 0; i < 16; i++)
        y[i] ^= len_block[i];
    ghash_mul(y, ctx->h);

    aes_encrypt_block(&ctx->key, ctx->j0, s);
    for (u32 i = 0; i < 16; i++)
        tag[i] = s[i] ^ y[i];

    return true;
}

bool aes_gcm_decrypt(struct aes_gcm_ctx *ctx,
                     const u8 *nonce, u32 nonce_len,
                     const u8 *aad, u32 aad_len,
                     const u8 *cipher, u32 cipher_len,
                     u8 *plain, const u8 *tag) {
    u8 y[16];
    u8 ctr[16];
    u8 stream[16];
    u8 len_block[16];
    u8 s[16];
    u8 calc_tag[16];
    u32 diff = 0;
    u32 offset = 0;

    if (!ctx || !nonce || !cipher || !plain || !tag)
        return false;
    if (nonce_len != 12)
        return false;

    gcm_make_j0(ctx->j0, nonce, nonce_len);

    simd_zero(y, sizeof(y));
    if (aad && aad_len)
        ghash_update(y, ctx->h, aad, aad_len);
    if (cipher_len)
        ghash_update(y, ctx->h, cipher, cipher_len);

    store_be64(len_block + 0, (u64)aad_len * 8U);
    store_be64(len_block + 8, (u64)cipher_len * 8U);
    for (u32 i = 0; i < 16; i++)
        y[i] ^= len_block[i];
    ghash_mul(y, ctx->h);

    aes_encrypt_block(&ctx->key, ctx->j0, s);
    for (u32 i = 0; i < 16; i++)
        calc_tag[i] = s[i] ^ y[i];
    for (u32 i = 0; i < 16; i++)
        diff |= (u32)(calc_tag[i] ^ tag[i]);
    if (diff != 0)
        return false;

    simd_memcpy(ctr, ctx->j0, 16);
    gcm_inc32(ctr);

    while (offset < cipher_len) {
        u32 chunk = cipher_len - offset;
        if (chunk > 16) chunk = 16;

        aes_encrypt_block(&ctx->key, ctr, stream);
        for (u32 i = 0; i < chunk; i++)
            plain[offset + i] = cipher[offset + i] ^ stream[i];

        offset += chunk;
        gcm_inc32(ctr);
    }

    return true;
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
