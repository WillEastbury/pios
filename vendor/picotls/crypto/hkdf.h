/*
 * HKDF-SHA256 (RFC 5869) — Extract + Expand.
 *
 * HKDF-Extract(salt, IKM)        -> PRK            (32 bytes)
 * HKDF-Expand(PRK, info, L)      -> OKM            (L <= 255*32 bytes)
 *
 * TLS 1.3 also defines a wrapper HKDF-Expand-Label on top of
 * HKDF-Expand which lives in tls/keysched.{c,h}.
 */
#ifndef PICOWEB_USERSPACE_CRYPTO_HKDF_H
#define PICOWEB_USERSPACE_CRYPTO_HKDF_H

#include <stddef.h>
#include <stdint.h>

#include "sha256.h"

#define HKDF_PRK_LEN SHA256_DIGEST_LEN

void hkdf_extract(const void* salt, size_t salt_len,
                  const void* ikm,  size_t ikm_len,
                  uint8_t prk[HKDF_PRK_LEN]);

/* Returns 0 on success, -1 if `out_len` exceeds the HKDF maximum. */
int hkdf_expand(const uint8_t prk[HKDF_PRK_LEN],
                const void* info, size_t info_len,
                uint8_t* out, size_t out_len);

#endif
