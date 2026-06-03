/*
 * SHA-512 (FIPS 180-4) - pure C scalar implementation.
 *
 * Used by Ed25519 (RFC 8032) for signing/verification. We don't
 * dispatch to a HW path here: SHA-512 isn't on the TLS hot path
 * (records use SHA-256/HMAC for the key schedule), and Ed25519
 * hashes are short (96-192 bytes typically), so the scalar core
 * suits us. If we ever bulk-hash with SHA-512 we can add ARMv8.2
 * SHA512 / Intel SHA-512 dispatch the same way sha256.c does.
 *
 * Constant-time-ish: no data-dependent branches, no table lookups
 * indexed by secret data.
 *
 * Reference: NIST FIPS PUB 180-4, August 2015 (sections 5.3.5, 6.4).
 */
#ifndef PIOS_SHA512_H
#define PIOS_SHA512_H

#include "types.h"

#define SHA512_DIGEST_LEN 64u
#define SHA512_BLOCK_LEN 128u

typedef struct {
    u64 state[8];
    /* 128-bit message length in bits (FIPS 180-4 section 5.1.2). The
     * high half is virtually never non-zero in practice, but we
     * track it for spec compliance. */
    u64 bitlen_lo;
    u64 bitlen_hi;
    u8  buf[SHA512_BLOCK_LEN];
    usize   buf_len;
} sha512_ctx;

void sha512_init(sha512_ctx* c);
void sha512_update(sha512_ctx* c, const void* data, usize len);
void sha512_final(sha512_ctx* c, u8 out[SHA512_DIGEST_LEN]);

/* One-shot convenience. */
void sha512(const void* data, usize len, u8 out[SHA512_DIGEST_LEN]);

#endif
