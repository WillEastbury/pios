/*
 * TLS 1.3 key schedule (RFC 8446 §7.1).
 */

#include "keysched.h"

#include <string.h>

#include "../crypto/hkdf.h"
#include "../crypto/hmac.h"
#include "../crypto/sha256.h"
#include "../crypto/util.h"

int tls13_hkdf_expand_label(const uint8_t secret[TLS13_HASH_LEN],
                            const char* label,
                            const uint8_t* context, size_t context_len,
                            uint8_t* out, size_t out_len) {
    /* HkdfLabel struct:
     *   uint16 length             (network byte order)
     *   opaque label<7..255>      "tls13 " + label
     *   opaque context<0..255>    context bytes
     *
     * Max sizes: length=2, label=1+(6+255)=262, context=1+255=256
     * Total cap: 520. We size to 600 for headroom. */
    uint8_t hl[600];
    size_t  off = 0;
    size_t  label_len = strlen(label);
    if (label_len + 6 > 255) return -1;     /* label list too long */
    if (context_len > 255)   return -1;     /* context too long */
    if (out_len > 0xffff)    return -1;     /* length doesn't fit u16 */

    hl[off++] = (uint8_t)(out_len >> 8);
    hl[off++] = (uint8_t)(out_len);
    hl[off++] = (uint8_t)(6 + label_len);
    memcpy(hl + off, "tls13 ", 6); off += 6;
    memcpy(hl + off, label, label_len); off += label_len;
    hl[off++] = (uint8_t)context_len;
    if (context_len) { memcpy(hl + off, context, context_len); off += context_len; }

    return hkdf_expand(secret, hl, off, out, out_len);
}

int tls13_derive_secret(const uint8_t secret[TLS13_HASH_LEN],
                        const char* label,
                        const uint8_t* messages, size_t messages_len,
                        uint8_t out[TLS13_HASH_LEN]) {
    uint8_t transcript[TLS13_HASH_LEN];
    sha256(messages, messages_len, transcript);
    return tls13_hkdf_expand_label(secret, label,
                                   transcript, sizeof(transcript),
                                   out, TLS13_HASH_LEN);
}

void tls13_derive_traffic_keys(const uint8_t traffic_secret[TLS13_HASH_LEN],
                               uint8_t key[32], uint8_t iv[12]) {
    /* RFC 8446 §7.3:
     *   key = HKDF-Expand-Label(traffic_secret, "key", "", key_length)
     *   iv  = HKDF-Expand-Label(traffic_secret, "iv",  "", iv_length)
     */
    tls13_hkdf_expand_label(traffic_secret, "key", NULL, 0, key, 32);
    tls13_hkdf_expand_label(traffic_secret, "iv",  NULL, 0, iv,  12);
}

int tls13_compute_finished(const uint8_t base_key[TLS13_HASH_LEN],
                           const uint8_t transcript_hash[TLS13_HASH_LEN],
                           uint8_t verify_data[TLS13_HASH_LEN]) {
    /* RFC 8446 §4.4.4:
     *   finished_key = HKDF-Expand-Label(BaseKey, "finished", "", Hash.length)
     *   verify_data  = HMAC(finished_key, Transcript-Hash)
     */
    uint8_t finished_key[TLS13_HASH_LEN];
    int rc = tls13_hkdf_expand_label(base_key, "finished",
                                     NULL, 0,
                                     finished_key, sizeof(finished_key));
    if (rc != 0) {
        secure_zero(finished_key, sizeof(finished_key));
        return -1;
    }
    hmac_sha256(finished_key, sizeof(finished_key),
                transcript_hash, TLS13_HASH_LEN,
                verify_data);
    secure_zero(finished_key, sizeof(finished_key));
    return 0;
}

int tls13_verify_finished(const uint8_t base_key[TLS13_HASH_LEN],
                          const uint8_t transcript_hash[TLS13_HASH_LEN],
                          const uint8_t verify_data[TLS13_HASH_LEN]) {
    uint8_t expected[TLS13_HASH_LEN];
    if (tls13_compute_finished(base_key, transcript_hash, expected) != 0) {
        secure_zero(expected, sizeof(expected));
        return -1;
    }
    int ok = crypto_consttime_eq(expected, verify_data, TLS13_HASH_LEN);
    secure_zero(expected, sizeof(expected));
    return ok ? 0 : -1;
}
