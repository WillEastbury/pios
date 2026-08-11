/*
 * SHA-256 (FIPS 180-4) — pure C, with optional hardware acceleration.
 *
 * The compression function `sha256_compress` is dispatched at runtime
 * via `sha256_compress_fn`. cpu_features_init() picks the best
 * available impl (scalar / SHA-NI / ARMv8 SHA2). The fallback is the
 * pure-C `sha256_compress_scalar`.
 *
 * Constant-time-ish: no data-dependent branches, no table lookups
 * indexed by secret data. Suitable for use inside HMAC keying.
 *
 * Reference: NIST FIPS PUB 180-4, August 2015.
 */
#ifndef PICOWEB_USERSPACE_CRYPTO_SHA256_H
#define PICOWEB_USERSPACE_CRYPTO_SHA256_H

#include <stddef.h>
#include <stdint.h>

#define SHA256_DIGEST_LEN 32u
#define SHA256_BLOCK_LEN  64u

typedef struct {
    uint32_t state[8];
    uint64_t bitlen;
    uint8_t  buf[SHA256_BLOCK_LEN];
    size_t   buf_len;
} sha256_ctx;

void sha256_init(sha256_ctx* c);
void sha256_update(sha256_ctx* c, const void* data, size_t len);
void sha256_final(sha256_ctx* c, uint8_t out[SHA256_DIGEST_LEN]);

/* One-shot convenience. */
void sha256(const void* data, size_t len, uint8_t out[SHA256_DIGEST_LEN]);

/* Compression function variants. The scalar fallback is always
 * available; HW variants exist on architectures that have the
 * relevant instructions. Each processes `nblocks` 64-byte blocks. */
typedef void (*sha256_compress_fn_t)(uint32_t state[8],
                                     const uint8_t* blocks,
                                     size_t nblocks);

void sha256_compress_scalar(uint32_t state[8], const uint8_t* blocks, size_t nblocks);

#if defined(__x86_64__) || defined(__i386__)
void sha256_compress_shani(uint32_t state[8], const uint8_t* blocks, size_t nblocks);
#endif
#if defined(__aarch64__)
void sha256_compress_armv8(uint32_t state[8], const uint8_t* blocks, size_t nblocks);
#endif

/* Dispatch pointer. Initialised on first call to sha256_init() or
 * the one-shot sha256(); also re-initialised by cpu_features_init().
 * Direct extern access is permitted for benches / micro-tests. */
extern sha256_compress_fn_t sha256_compress_fn;

/* Force re-selection (called once at startup; rarely useful otherwise). */
void sha256_select_impl(void);

/* Returns a short label identifying the active impl: "scalar",
 * "sha-ni", "armv8-sha2". Useful for the startup banner. */
const char* sha256_impl_name(void);

#endif
