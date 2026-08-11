/*
 * HKDF-SHA256 (RFC 5869).
 */

#include "hkdf.h"

#include <string.h>

#include "hmac.h"

void hkdf_extract(const void* salt, size_t salt_len,
                  const void* ikm,  size_t ikm_len,
                  uint8_t prk[HKDF_PRK_LEN]) {
    /* If the caller passed NULL for the salt, RFC 5869 §2.2 says we
     * use HashLen zero bytes. */
    static const uint8_t zero_salt[SHA256_DIGEST_LEN] = {0};
    if (salt == NULL || salt_len == 0) {
        salt = zero_salt;
        salt_len = SHA256_DIGEST_LEN;
    }
    hmac_sha256(salt, salt_len, ikm, ikm_len, prk);
}

int hkdf_expand(const uint8_t prk[HKDF_PRK_LEN],
                const void* info, size_t info_len,
                uint8_t* out, size_t out_len) {
    /* RFC 5869 §2.3: L <= 255 * HashLen */
    if (out_len > 255u * SHA256_DIGEST_LEN) return -1;

    uint8_t T[SHA256_DIGEST_LEN];
    size_t  T_len = 0;
    uint8_t counter = 1;
    size_t  pos = 0;

    while (pos < out_len) {
        hmac_sha256_ctx h;
        hmac_sha256_init(&h, prk, HKDF_PRK_LEN);
        if (T_len) hmac_sha256_update(&h, T, T_len);
        if (info_len) hmac_sha256_update(&h, info, info_len);
        hmac_sha256_update(&h, &counter, 1);
        hmac_sha256_final(&h, T);
        T_len = SHA256_DIGEST_LEN;

        size_t take = out_len - pos;
        if (take > SHA256_DIGEST_LEN) take = SHA256_DIGEST_LEN;
        memcpy(out + pos, T, take);
        pos += take;
        counter++;
    }

    memset(T, 0, sizeof(T));
    return 0;
}
