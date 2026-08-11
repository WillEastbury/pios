/*
 * Poly1305 (RFC 8439 §2.5) — one-time authenticator.
 *
 * Tag = (((sum_of_blocks * r) + s) mod (2^130 - 5)) mod 2^128
 *
 * The key is 32 bytes split into r (16) and s (16). r has 22 bits
 * masked out per RFC 8439 §2.5 ("clamping"). s is added at the end.
 *
 * NEVER reuse the same (r,s) for two different messages — the
 * mathematical assumptions break and key recovery is trivial. In our
 * AEAD construction we derive a fresh (r,s) per record from the
 * ChaCha20 keystream block 0; that's the standard pattern.
 *
 * Two APIs are exposed:
 *
 *   1. One-shot:  poly1305(key, msg, len, tag)
 *
 *   2. Incremental: poly1305_init / poly1305_update / poly1305_finish
 *      Use this when the message is composed of multiple disjoint
 *      buffers (e.g. AEAD aad || pad || ciphertext || pad || lens)
 *      and you can't or don't want to concatenate first.
 *      Internally buffers up partial 16-byte blocks so the caller can
 *      feed any byte counts.
 */
#ifndef PICOWEB_USERSPACE_CRYPTO_POLY1305_H
#define PICOWEB_USERSPACE_CRYPTO_POLY1305_H

#include <stddef.h>
#include <stdint.h>

#define POLY1305_KEY_LEN 32u
#define POLY1305_TAG_LEN 16u

typedef struct {
    uint32_t r[5];        /* clamped key */
    uint32_t s[4];        /* additive part of key */
    uint32_t h[5];        /* accumulator */
    uint8_t  buf[16];     /* partial block buffer */
    size_t   buf_len;     /* bytes currently in buf, 0..15 */
} poly1305_ctx_t;

void poly1305_init(poly1305_ctx_t* ctx, const uint8_t key[POLY1305_KEY_LEN]);
void poly1305_update(poly1305_ctx_t* ctx, const uint8_t* msg, size_t len);
void poly1305_finish(poly1305_ctx_t* ctx, uint8_t tag[POLY1305_TAG_LEN]);

/* One-shot convenience wrapper. */
void poly1305(const uint8_t key[POLY1305_KEY_LEN],
              const uint8_t* msg, size_t len,
              uint8_t       tag[POLY1305_TAG_LEN]);

#endif
