/*
 * SHA-512 (FIPS 180-4) — pure C scalar implementation.
 *
 * Structure mirrors sha256.c. Differences from SHA-256:
 *   - 64-bit words, 128-byte block, 80 rounds, 16-byte length field
 *   - different IV (square roots of first 8 primes, 64 bits each)
 *   - different K (cube roots of first 80 primes, 64 bits each)
 *   - different sigma rotations
 *
 * Memory: all state is on the caller-provided sha512_ctx. The
 * scalar compression copies one block into 80 64-bit words on
 * the stack (640 B) plus 8 working registers a..h.
 */

#include "sha512.h"

#include <string.h>

/* FIPS 180-4 §4.2.3 — first 64 bits of the fractional parts of the
 * cube roots of the first 80 primes (2..409). */
static const uint64_t K[80] = {
    0x428a2f98d728ae22ULL, 0x7137449123ef65cdULL, 0xb5c0fbcfec4d3b2fULL, 0xe9b5dba58189dbbcULL,
    0x3956c25bf348b538ULL, 0x59f111f1b605d019ULL, 0x923f82a4af194f9bULL, 0xab1c5ed5da6d8118ULL,
    0xd807aa98a3030242ULL, 0x12835b0145706fbeULL, 0x243185be4ee4b28cULL, 0x550c7dc3d5ffb4e2ULL,
    0x72be5d74f27b896fULL, 0x80deb1fe3b1696b1ULL, 0x9bdc06a725c71235ULL, 0xc19bf174cf692694ULL,
    0xe49b69c19ef14ad2ULL, 0xefbe4786384f25e3ULL, 0x0fc19dc68b8cd5b5ULL, 0x240ca1cc77ac9c65ULL,
    0x2de92c6f592b0275ULL, 0x4a7484aa6ea6e483ULL, 0x5cb0a9dcbd41fbd4ULL, 0x76f988da831153b5ULL,
    0x983e5152ee66dfabULL, 0xa831c66d2db43210ULL, 0xb00327c898fb213fULL, 0xbf597fc7beef0ee4ULL,
    0xc6e00bf33da88fc2ULL, 0xd5a79147930aa725ULL, 0x06ca6351e003826fULL, 0x142929670a0e6e70ULL,
    0x27b70a8546d22ffcULL, 0x2e1b21385c26c926ULL, 0x4d2c6dfc5ac42aedULL, 0x53380d139d95b3dfULL,
    0x650a73548baf63deULL, 0x766a0abb3c77b2a8ULL, 0x81c2c92e47edaee6ULL, 0x92722c851482353bULL,
    0xa2bfe8a14cf10364ULL, 0xa81a664bbc423001ULL, 0xc24b8b70d0f89791ULL, 0xc76c51a30654be30ULL,
    0xd192e819d6ef5218ULL, 0xd69906245565a910ULL, 0xf40e35855771202aULL, 0x106aa07032bbd1b8ULL,
    0x19a4c116b8d2d0c8ULL, 0x1e376c085141ab53ULL, 0x2748774cdf8eeb99ULL, 0x34b0bcb5e19b48a8ULL,
    0x391c0cb3c5c95a63ULL, 0x4ed8aa4ae3418acbULL, 0x5b9cca4f7763e373ULL, 0x682e6ff3d6b2b8a3ULL,
    0x748f82ee5defb2fcULL, 0x78a5636f43172f60ULL, 0x84c87814a1f0ab72ULL, 0x8cc702081a6439ecULL,
    0x90befffa23631e28ULL, 0xa4506cebde82bde9ULL, 0xbef9a3f7b2c67915ULL, 0xc67178f2e372532bULL,
    0xca273eceea26619cULL, 0xd186b8c721c0c207ULL, 0xeada7dd6cde0eb1eULL, 0xf57d4f7fee6ed178ULL,
    0x06f067aa72176fbaULL, 0x0a637dc5a2c898a6ULL, 0x113f9804bef90daeULL, 0x1b710b35131c471bULL,
    0x28db77f523047d84ULL, 0x32caab7b40c72493ULL, 0x3c9ebe0a15c9bebcULL, 0x431d67c49c100d4cULL,
    0x4cc5d4becb3e42b6ULL, 0x597f299cfc657e2aULL, 0x5fcb6fab3ad6faecULL, 0x6c44198c4a475817ULL,
};

static inline uint64_t rotr64(uint64_t x, unsigned n) {
    return (x >> n) | (x << (64u - n));
}

static inline uint64_t big_sigma0(uint64_t x) {
    return rotr64(x, 28) ^ rotr64(x, 34) ^ rotr64(x, 39);
}
static inline uint64_t big_sigma1(uint64_t x) {
    return rotr64(x, 14) ^ rotr64(x, 18) ^ rotr64(x, 41);
}
static inline uint64_t small_sigma0(uint64_t x) {
    return rotr64(x, 1)  ^ rotr64(x, 8)  ^ (x >> 7);
}
static inline uint64_t small_sigma1(uint64_t x) {
    return rotr64(x, 19) ^ rotr64(x, 61) ^ (x >> 6);
}
static inline uint64_t Ch(uint64_t x, uint64_t y, uint64_t z) {
    return (x & y) ^ (~x & z);
}
static inline uint64_t Maj(uint64_t x, uint64_t y, uint64_t z) {
    return (x & y) ^ (x & z) ^ (y & z);
}

static inline uint64_t load_be64(const uint8_t* p) {
    return ((uint64_t)p[0] << 56) | ((uint64_t)p[1] << 48) |
           ((uint64_t)p[2] << 40) | ((uint64_t)p[3] << 32) |
           ((uint64_t)p[4] << 24) | ((uint64_t)p[5] << 16) |
           ((uint64_t)p[6] <<  8) |  (uint64_t)p[7];
}

static inline void store_be64(uint8_t* p, uint64_t v) {
    p[0] = (uint8_t)(v >> 56); p[1] = (uint8_t)(v >> 48);
    p[2] = (uint8_t)(v >> 40); p[3] = (uint8_t)(v >> 32);
    p[4] = (uint8_t)(v >> 24); p[5] = (uint8_t)(v >> 16);
    p[6] = (uint8_t)(v >>  8); p[7] = (uint8_t)v;
}

static void compress_one(uint64_t state[8], const uint8_t block[128]) {
    uint64_t w[80];
    for (int i = 0; i < 16; i++) {
        w[i] = load_be64(block + i * 8);
    }
    for (int i = 16; i < 80; i++) {
        w[i] = small_sigma1(w[i-2]) + w[i-7] + small_sigma0(w[i-15]) + w[i-16];
    }

    uint64_t a = state[0], b = state[1], c = state[2], d = state[3];
    uint64_t e = state[4], f = state[5], g = state[6], h = state[7];

    for (int i = 0; i < 80; i++) {
        uint64_t t1 = h + big_sigma1(e) + Ch(e, f, g) + K[i] + w[i];
        uint64_t t2 = big_sigma0(a) + Maj(a, b, c);
        h = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }

    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
    state[4] += e; state[5] += f; state[6] += g; state[7] += h;
}

/* FIPS 180-4 §5.3.5 — initial hash values for SHA-512:
 * first 64 bits of the fractional parts of the square roots of the
 * first 8 primes. */
void sha512_init(sha512_ctx* c) {
    c->state[0] = 0x6a09e667f3bcc908ULL;
    c->state[1] = 0xbb67ae8584caa73bULL;
    c->state[2] = 0x3c6ef372fe94f82bULL;
    c->state[3] = 0xa54ff53a5f1d36f1ULL;
    c->state[4] = 0x510e527fade682d1ULL;
    c->state[5] = 0x9b05688c2b3e6c1fULL;
    c->state[6] = 0x1f83d9abfb41bd6bULL;
    c->state[7] = 0x5be0cd19137e2179ULL;
    c->bitlen_lo = 0;
    c->bitlen_hi = 0;
    c->buf_len = 0;
}

/* 128-bit add: bitlen += len*8. Tracks carry into bitlen_hi. */
static inline void add_bitlen(sha512_ctx* c, uint64_t bytes) {
    uint64_t add = bytes << 3;
    uint64_t carry_into_lo = (bytes >> 61); /* high 3 bits of bytes shift up */
    uint64_t old = c->bitlen_lo;
    c->bitlen_lo = old + add;
    if (c->bitlen_lo < old) c->bitlen_hi++;
    c->bitlen_hi += carry_into_lo;
}

void sha512_update(sha512_ctx* c, const void* data, size_t len) {
    const uint8_t* p = (const uint8_t*)data;
    add_bitlen(c, (uint64_t)len);

    /* Drain a partial block first. */
    if (c->buf_len) {
        size_t need = SHA512_BLOCK_LEN - c->buf_len;
        if (len < need) {
            memcpy(c->buf + c->buf_len, p, len);
            c->buf_len += len;
            return;
        }
        memcpy(c->buf + c->buf_len, p, need);
        compress_one(c->state, c->buf);
        c->buf_len = 0;
        p   += need;
        len -= need;
    }

    /* Consume full blocks directly. */
    while (len >= SHA512_BLOCK_LEN) {
        compress_one(c->state, p);
        p   += SHA512_BLOCK_LEN;
        len -= SHA512_BLOCK_LEN;
    }

    if (len) {
        memcpy(c->buf, p, len);
        c->buf_len = len;
    }
}

void sha512_final(sha512_ctx* c, uint8_t out[SHA512_DIGEST_LEN]) {
    /* Padding: append 0x80 then zeros until 112 bytes mod 128, then
     * the 128-bit big-endian message length in bits. */
    c->buf[c->buf_len++] = 0x80;
    if (c->buf_len > 112) {
        memset(c->buf + c->buf_len, 0, SHA512_BLOCK_LEN - c->buf_len);
        compress_one(c->state, c->buf);
        c->buf_len = 0;
    }
    memset(c->buf + c->buf_len, 0, 112 - c->buf_len);

    /* Big-endian 128-bit length: hi half then lo half. */
    store_be64(c->buf + 112, c->bitlen_hi);
    store_be64(c->buf + 120, c->bitlen_lo);
    compress_one(c->state, c->buf);

    for (int i = 0; i < 8; i++) {
        store_be64(out + i * 8, c->state[i]);
    }

    memset(c, 0, sizeof(*c));
}

void sha512(const void* data, size_t len, uint8_t out[SHA512_DIGEST_LEN]) {
    sha512_ctx c;
    sha512_init(&c);
    sha512_update(&c, data, len);
    sha512_final(&c, out);
}
