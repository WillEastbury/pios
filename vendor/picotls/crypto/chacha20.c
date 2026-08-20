/*
 * ChaCha20 (RFC 8439 §2.4) — pure C reference + dispatch.
 *
 * The 64-byte initial state is laid out as 16 32-bit little-endian words:
 *   [ "expa" "nd 3" "2-by" "te k" ]    (constants)
 *   [    key[0..15]              ]
 *   [    key[16..31]             ]
 *   [ counter | nonce[0..11]     ]
 *
 * One quarter-round on (a,b,c,d):
 *   a += b; d ^= a; d <<<= 16
 *   c += d; b ^= c; b <<<= 12
 *   a += b; d ^= a; d <<<=  8
 *   c += d; b ^= c; b <<<=  7
 *
 * Twenty rounds = 10x (column round + diagonal round). After twenty
 * rounds we add the original state back and serialise as little-endian
 * — that's the keystream block.
 *
 * The HW-accelerated path lives in chacha20_sse2.c; this file owns
 * the scalar fallback and the dispatch.
 */

#include "chacha20.h"

#include <string.h>

#include "pw_cpuid.h"

static inline uint32_t rotl32(uint32_t x, unsigned n) {
    return (x << n) | (x >> (32u - n));
}

static inline uint32_t load_le32(const uint8_t* p) {
    return  (uint32_t)p[0]        | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void store_le32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v;         p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

#define QR(a,b,c,d) do { \
    a += b; d ^= a; d = rotl32(d, 16); \
    c += d; b ^= c; b = rotl32(b, 12); \
    a += b; d ^= a; d = rotl32(d,  8); \
    c += d; b ^= c; b = rotl32(b,  7); \
} while (0)

void chacha20_block(const uint8_t key[CHACHA20_KEY_LEN],
                    uint32_t      counter,
                    const uint8_t nonce[CHACHA20_NONCE_LEN],
                    uint8_t       out[CHACHA20_BLOCK_LEN]) {
    /* Initial state. */
    uint32_t s[16];
    s[0] = 0x61707865; s[1] = 0x3320646e;
    s[2] = 0x79622d32; s[3] = 0x6b206574;
    for (int i = 0; i < 8; i++) s[4 + i] = load_le32(key + i * 4);
    s[12] = counter;
    s[13] = load_le32(nonce + 0);
    s[14] = load_le32(nonce + 4);
    s[15] = load_le32(nonce + 8);

    uint32_t x[16];
    memcpy(x, s, sizeof(x));

    for (int i = 0; i < 10; i++) {
        /* Column round */
        QR(x[0], x[4], x[ 8], x[12]);
        QR(x[1], x[5], x[ 9], x[13]);
        QR(x[2], x[6], x[10], x[14]);
        QR(x[3], x[7], x[11], x[15]);
        /* Diagonal round */
        QR(x[0], x[5], x[10], x[15]);
        QR(x[1], x[6], x[11], x[12]);
        QR(x[2], x[7], x[ 8], x[13]);
        QR(x[3], x[4], x[ 9], x[14]);
    }

    for (int i = 0; i < 16; i++) {
        store_le32(out + i * 4, x[i] + s[i]);
    }
}

void chacha20_xor_scalar(const uint8_t key[CHACHA20_KEY_LEN],
                         uint32_t      counter,
                         const uint8_t nonce[CHACHA20_NONCE_LEN],
                         const uint8_t* in, uint8_t* out, size_t len) {
    uint8_t ks[CHACHA20_BLOCK_LEN];
    while (len > 0) {
        chacha20_block(key, counter, nonce, ks);
        size_t take = len < CHACHA20_BLOCK_LEN ? len : CHACHA20_BLOCK_LEN;
        for (size_t i = 0; i < take; i++) out[i] = in[i] ^ ks[i];
        in += take; out += take; len -= take;
        counter++;
    }
    memset(ks, 0, sizeof(ks));
}

chacha20_xor_fn_t chacha20_xor_fn = chacha20_xor_scalar;
static const char* chacha20_impl_label = "scalar";

void chacha20_select_impl(void) {
    const cpu_features_t* f = cpu_features_init();
#if defined(__x86_64__) || defined(__i386__)
    if (f->x86_sse2) {
        chacha20_xor_fn = chacha20_xor_sse2;
        chacha20_impl_label = "sse2-4way";
        return;
    }
#endif
    (void)f;
    chacha20_xor_fn = chacha20_xor_scalar;
    chacha20_impl_label = "scalar";
}

const char* chacha20_impl_name(void) {
    return chacha20_impl_label;
}

void chacha20_xor(const uint8_t key[CHACHA20_KEY_LEN],
                  uint32_t      counter,
                  const uint8_t nonce[CHACHA20_NONCE_LEN],
                  const uint8_t* in, uint8_t* out, size_t len) {
    chacha20_xor_fn(key, counter, nonce, in, out, len);
}

/* ---------------- streaming API ---------------- */

void chacha20_stream_init(chacha20_stream_t* cs,
                          const uint8_t key[CHACHA20_KEY_LEN],
                          const uint8_t nonce[CHACHA20_NONCE_LEN],
                          uint32_t initial_counter) {
    memcpy(cs->key, key, CHACHA20_KEY_LEN);
    memcpy(cs->nonce, nonce, CHACHA20_NONCE_LEN);
    cs->counter   = initial_counter;
    cs->carry_off = CHACHA20_BLOCK_LEN;     /* empty */
    /* ks_carry left undefined until first refill. */
}

void chacha20_stream_xor(chacha20_stream_t* cs,
                         const uint8_t* in, uint8_t* out, size_t len) {
    /* 1) Drain any leftover keystream from the previous call. */
    if (cs->carry_off < CHACHA20_BLOCK_LEN && len > 0) {
        size_t avail = (size_t)CHACHA20_BLOCK_LEN - cs->carry_off;
        size_t take  = len < avail ? len : avail;
        for (size_t i = 0; i < take; i++) {
            out[i] = in[i] ^ cs->ks_carry[cs->carry_off + i];
        }
        cs->carry_off += (uint8_t)take;
        in  += take;
        out += take;
        len -= take;
    }

    /* 2) XOR full middle blocks via the dispatched (possibly SIMD)
     *    bulk path. This advances `counter` by `len / 64`. */
    if (len >= CHACHA20_BLOCK_LEN) {
        size_t full_bytes = len & ~(size_t)(CHACHA20_BLOCK_LEN - 1);
        chacha20_xor_fn(cs->key, cs->counter, cs->nonce, in, out, full_bytes);
        cs->counter += (uint32_t)(full_bytes / CHACHA20_BLOCK_LEN);
        in  += full_bytes;
        out += full_bytes;
        len -= full_bytes;
    }

    /* 3) Tail (< 64 bytes): generate one fresh block into ks_carry,
     *    XOR what we need, and remember the offset so the next call
     *    picks up where we left off. */
    if (len > 0) {
        chacha20_block(cs->key, cs->counter, cs->nonce, cs->ks_carry);
        cs->counter++;
        for (size_t i = 0; i < len; i++) {
            out[i] = in[i] ^ cs->ks_carry[i];
        }
        cs->carry_off = (uint8_t)len;
    }
}
