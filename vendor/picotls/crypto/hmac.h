/*
 * HMAC-SHA256 (RFC 2104) — pure C, on top of our SHA-256.
 *
 * Constant-time-ish where it matters: the key processing step is
 * fixed-shape regardless of key length. There is no comparison of the
 * computed tag against a caller-supplied tag in this module — we just
 * produce the tag. The TLS layer compares with a constant-time helper.
 */
#ifndef PICOWEB_USERSPACE_CRYPTO_HMAC_H
#define PICOWEB_USERSPACE_CRYPTO_HMAC_H

#include <stddef.h>
#include <stdint.h>

#include "sha256.h"

#define HMAC_SHA256_TAG_LEN SHA256_DIGEST_LEN

typedef struct {
    sha256_ctx inner;
    sha256_ctx outer;
} hmac_sha256_ctx;

void hmac_sha256_init(hmac_sha256_ctx* h, const void* key, size_t key_len);
void hmac_sha256_update(hmac_sha256_ctx* h, const void* data, size_t len);
void hmac_sha256_final(hmac_sha256_ctx* h, uint8_t out[HMAC_SHA256_TAG_LEN]);

/* One-shot. */
void hmac_sha256(const void* key, size_t key_len,
                 const void* data, size_t len,
                 uint8_t out[HMAC_SHA256_TAG_LEN]);

#endif
