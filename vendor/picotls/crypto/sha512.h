/*
 * SHA-512 (FIPS 180-4) — pure C scalar implementation.
 *
 * Used by Ed25519 (RFC 8032) for signing/verification. We don't
 * dispatch to a HW path here: SHA-512 isn't on the TLS hot path
 * (records use SHA-256/HMAC for the key schedule), and Ed25519
 * hashes are short (96–192 bytes typically), so the scalar core
 * suits us. If we ever bulk-hash with SHA-512 we can add ARMv8.2
 * SHA512 / Intel SHA-512 dispatch the same way sha256.c does.
 *
 * Constant-time-ish: no data-dependent branches, no table lookups
 * indexed by secret data.
 *
 * Reference: NIST FIPS PUB 180-4, August 2015 (§5.3.5, §6.4).
 */
#ifndef PICOWEB_USERSPACE_CRYPTO_SHA512_H
#define PICOWEB_USERSPACE_CRYPTO_SHA512_H

#include <stddef.h>
#include <stdint.h>

#define SHA512_DIGEST_LEN 64u
#define SHA512_BLOCK_LEN 128u

typedef struct {
    uint64_t state[8];
    /* 128-bit message length in bits (FIPS 180-4 §5.1.2). The
     * high half is virtually never non-zero in practice, but we
     * track it for spec compliance. */
    uint64_t bitlen_lo;
    uint64_t bitlen_hi;
    uint8_t  buf[SHA512_BLOCK_LEN];
    size_t   buf_len;
} sha512_ctx;

void sha512_init(sha512_ctx* c);
void sha512_update(sha512_ctx* c, const void* data, size_t len);
void sha512_final(sha512_ctx* c, uint8_t out[SHA512_DIGEST_LEN]);

/* One-shot convenience. */
void sha512(const void* data, size_t len, uint8_t out[SHA512_DIGEST_LEN]);

#endif
