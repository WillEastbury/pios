/*
 * HMAC-SHA256 (RFC 2104) on top of our SHA-256 implementation.
 *
 * Construction:
 *   K' = SHA-256(K)               if K is longer than the block size
 *      = K || 0x00...00           if K is shorter than the block size
 *      = K                        otherwise
 *   ipad = K' XOR 0x36363636...
 *   opad = K' XOR 0x5c5c5c5c...
 *   HMAC(K, m) = SHA-256( opad || SHA-256( ipad || m ) )
 */

#include "hmac.h"

#include <string.h>

void hmac_sha256_init(hmac_sha256_ctx* h, const void* key, size_t key_len) {
    uint8_t kprime[SHA256_BLOCK_LEN];
    uint8_t ipad[SHA256_BLOCK_LEN];
    uint8_t opad[SHA256_BLOCK_LEN];

    if (key_len > SHA256_BLOCK_LEN) {
        sha256(key, key_len, kprime);
        memset(kprime + SHA256_DIGEST_LEN, 0,
               SHA256_BLOCK_LEN - SHA256_DIGEST_LEN);
    } else {
        memcpy(kprime, key, key_len);
        memset(kprime + key_len, 0, SHA256_BLOCK_LEN - key_len);
    }

    for (size_t i = 0; i < SHA256_BLOCK_LEN; i++) {
        ipad[i] = kprime[i] ^ 0x36;
        opad[i] = kprime[i] ^ 0x5c;
    }

    sha256_init(&h->inner);
    sha256_update(&h->inner, ipad, SHA256_BLOCK_LEN);
    sha256_init(&h->outer);
    sha256_update(&h->outer, opad, SHA256_BLOCK_LEN);

    /* Wipe the temporary key material — TLS keys often pass through here. */
    memset(kprime, 0, sizeof(kprime));
    memset(ipad,   0, sizeof(ipad));
    memset(opad,   0, sizeof(opad));
}

void hmac_sha256_update(hmac_sha256_ctx* h, const void* data, size_t len) {
    sha256_update(&h->inner, data, len);
}

void hmac_sha256_final(hmac_sha256_ctx* h, uint8_t out[HMAC_SHA256_TAG_LEN]) {
    uint8_t inner_tag[SHA256_DIGEST_LEN];
    sha256_final(&h->inner, inner_tag);
    sha256_update(&h->outer, inner_tag, sizeof(inner_tag));
    sha256_final(&h->outer, out);
    memset(inner_tag, 0, sizeof(inner_tag));
    memset(h, 0, sizeof(*h));
}

void hmac_sha256(const void* key, size_t key_len,
                 const void* data, size_t len,
                 uint8_t out[HMAC_SHA256_TAG_LEN]) {
    hmac_sha256_ctx h;
    hmac_sha256_init(&h, key, key_len);
    hmac_sha256_update(&h, data, len);
    hmac_sha256_final(&h, out);
}
