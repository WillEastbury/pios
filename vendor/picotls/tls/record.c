/*
 * TLS 1.3 record layer (RFC 8446 §5).
 *
 * Spike-grade: handles a single AEAD (ChaCha20-Poly1305) and assumes
 * the caller already negotiated the cipher suite.
 */

#include "record.h"

#include <string.h>

#include "../crypto/chacha20_poly1305.h"
#include "../crypto/util.h"
#include "../iov.h"

void tls13_build_nonce(const tls_record_dir_t* dir, uint8_t nonce[12]) {
    /* Per RFC 8446 §5.3: nonce = static_iv XOR (seq padded with leading zeros). */
    memcpy(nonce, dir->static_iv, 12);
    for (int i = 0; i < 8; i++) {
        nonce[12 - 1 - i] ^= (uint8_t)(dir->seq >> (i * 8));
    }
}

size_t tls13_seal_record(tls_record_dir_t* dir,
                         tls_content_type_t inner_type,
                         tls_content_type_t outer_type,
                         const uint8_t* plaintext, size_t plaintext_len,
                         uint8_t* out, size_t out_cap) {
    size_t inner_len = plaintext_len + 1;            /* type trailer */
    size_t cipher_len = inner_len + TLS13_AEAD_TAG_LEN;
    size_t wire_len = TLS13_RECORD_HEADER_LEN + cipher_len;

    if (cipher_len > 0xffff)        return 0;        /* doesn't fit u16 */
    if (wire_len > out_cap)         return 0;
    if (plaintext_len > TLS13_MAX_PLAINTEXT) return 0;

    /* Build header (this is also the AEAD AAD). */
    out[0] = (uint8_t)outer_type;
    out[1] = 0x03;                /* legacy_record_version 0x0303 */
    out[2] = 0x03;
    out[3] = (uint8_t)(cipher_len >> 8);
    out[4] = (uint8_t)cipher_len;

    /* Refuse to seal if the next increment would wrap dir->seq.
     * RFC 8446 §5.3: the per-direction sequence number MUST NOT
     * wrap. Wrapping reuses an AEAD nonce and breaks confidentiality
     * + integrity guarantees catastrophically. */
    if (dir->seq == UINT64_MAX) return 0;

    /* Build TLSInnerPlaintext = plaintext || type byte. We overwrite
     * `out + 5` with the encrypted form. The type byte must live in
     * the buffer that gets encrypted, so copy plaintext to its final
     * spot and then append the type. */
    uint8_t* body = out + TLS13_RECORD_HEADER_LEN;
    if (plaintext_len) memcpy(body, plaintext, plaintext_len);
    body[plaintext_len] = (uint8_t)inner_type;

    uint8_t nonce[12];
    tls13_build_nonce(dir, nonce);
    aead_chacha20_poly1305_seal(dir->key, nonce,
                                out, TLS13_RECORD_HEADER_LEN,
                                body, inner_len,
                                body, body + inner_len);
    secure_zero(nonce, sizeof(nonce));
    dir->seq++;
    return wire_len;
}

size_t tls13_seal_record_iov(tls_record_dir_t* dir,
                             tls_content_type_t inner_type,
                             tls_content_type_t outer_type,
                             const struct pw_iov* pt_iov, unsigned pt_iov_n,
                             size_t total_plaintext_len,
                             uint8_t* out, size_t out_cap) {
    /* Same length math as the contiguous path; we can pre-write the
     * record header (which is also the AEAD AAD) before any encrypt
     * work happens. This is exactly what the user wanted from the
     * "calculate length before TLS is hit" property: the wire
     * preamble is determined the moment the iov chain is handed to
     * us. */
    size_t inner_len  = total_plaintext_len + 1;
    size_t cipher_len = inner_len + TLS13_AEAD_TAG_LEN;
    size_t wire_len   = TLS13_RECORD_HEADER_LEN + cipher_len;

    if (cipher_len > 0xffff)                     return 0;
    if (wire_len   > out_cap)                    return 0;
    if (total_plaintext_len > TLS13_MAX_PLAINTEXT) return 0;

    out[0] = (uint8_t)outer_type;
    out[1] = 0x03; out[2] = 0x03;
    out[3] = (uint8_t)(cipher_len >> 8);
    out[4] = (uint8_t)cipher_len;

    /* Refuse to seal if dir->seq is about to wrap (would reuse nonce). */
    if (dir->seq == UINT64_MAX) return 0;

    uint8_t* body     = out + TLS13_RECORD_HEADER_LEN;
    uint8_t* tag_dst  = body + inner_len;

    /* Append the inner type trailer as a single 1-byte fragment so
     * the AEAD seals (plaintext_iov || type_byte) without any other
     * code path needing to know about the trailer convention. */
    pw_iov_t local[PW_IOV_MAX_FRAGS + 1];
    if (pt_iov_n + 1 > sizeof(local) / sizeof(local[0])) return 0;
    for (unsigned i = 0; i < pt_iov_n; i++) local[i] = pt_iov[i];
    uint8_t type_byte = (uint8_t)inner_type;
    local[pt_iov_n].base = &type_byte;
    local[pt_iov_n].len  = 1;

    uint8_t nonce[12];
    tls13_build_nonce(dir, nonce);
    aead_chacha20_poly1305_seal_iov(dir->key, nonce,
                                    out, TLS13_RECORD_HEADER_LEN,
                                    local, pt_iov_n + 1,
                                    inner_len,
                                    body, tag_dst);
    secure_zero(nonce, sizeof(nonce));
    dir->seq++;
    return wire_len;
}

int tls13_open_record(tls_record_dir_t* dir,
                      uint8_t* record, size_t record_len,
                      tls_content_type_t* inner_type_out,
                      uint8_t** plaintext_out, size_t* plaintext_len_out) {
    if (record_len < TLS13_RECORD_HEADER_LEN + TLS13_AEAD_TAG_LEN) return -1;
    /* Refuse to open if dir->seq is at its max (would reuse nonce on
     * the next record). Treat this as a hard protocol failure. */
    if (dir->seq == UINT64_MAX) return -1;
    size_t cipher_len = ((size_t)record[3] << 8) | record[4];
    if (cipher_len + TLS13_RECORD_HEADER_LEN != record_len) return -1;
    if (cipher_len < TLS13_AEAD_TAG_LEN) return -1;
    size_t inner_len = cipher_len - TLS13_AEAD_TAG_LEN;

    uint8_t nonce[12];
    tls13_build_nonce(dir, nonce);
    int rc = aead_chacha20_poly1305_open(dir->key, nonce,
                                         record, TLS13_RECORD_HEADER_LEN,
                                         record + TLS13_RECORD_HEADER_LEN,
                                         inner_len,
                                         record + TLS13_RECORD_HEADER_LEN + inner_len,
                                         record + TLS13_RECORD_HEADER_LEN);
    secure_zero(nonce, sizeof(nonce));
    if (rc != 0) return -1;

    /* Strip trailing zero padding from the inner plaintext to find
     * the type byte (RFC 8446 §5.4). */
    while (inner_len > 0 && record[TLS13_RECORD_HEADER_LEN + inner_len - 1] == 0) {
        inner_len--;
    }
    if (inner_len == 0) return -1;            /* malformed: no type byte */

    *inner_type_out = (tls_content_type_t)record[TLS13_RECORD_HEADER_LEN + inner_len - 1];
    *plaintext_out = record + TLS13_RECORD_HEADER_LEN;
    *plaintext_len_out = inner_len - 1;
    dir->seq++;
    return 0;
}
