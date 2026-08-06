#include "tls13_record.h"
#include "simd.h"

void tls13_record_dir_init(struct tls13_record_dir *dir, const u8 key[16], const u8 iv[12])
{
    aes_gcm_init(&dir->gcm, key, 128U);
    simd_memcpy(dir->iv, iv, 12);
    dir->seq = 0;
}

void tls13_record_build_nonce(const u8 iv[12], u64 seq, u8 nonce[12])
{
    u8 seq_be[12];
    simd_zero(seq_be, 4); /* top 4 bytes of the 12-byte field are always 0 */
    for (u32 i = 0; i < 8; i++)
        seq_be[11 - i] = (u8)(seq >> (8U * i));
    for (u32 i = 0; i < 12; i++)
        nonce[i] = iv[i] ^ seq_be[i];
}

void tls13_record_build_header(u32 ciphertext_len, u8 hdr[TLS13_RECORD_HDR_LEN])
{
    hdr[0] = (u8)TLS13_CONTENT_APPLICATION_DATA; /* opaque_type: always 0x17 once protected */
    hdr[1] = 0x03U;                              /* legacy_record_version */
    hdr[2] = 0x03U;
    hdr[3] = (u8)(ciphertext_len >> 8);
    hdr[4] = (u8)(ciphertext_len & 0xFFU);
}

u32 tls13_record_seal(struct tls13_record_dir *dir, u8 content_type,
                     const u8 *plain, u32 plain_len, u32 pad_len,
                     u8 *out, u32 out_cap)
{
    if (!dir || !out || (plain_len && !plain)) return 0;

    u32 inner_len = plain_len + pad_len + 1U;
    if (inner_len > TLS13_MAX_INNER_PLAINTEXT) return 0;

    u32 cipher_len = inner_len + TLS13_TAG_LEN;
    if (out_cap < TLS13_RECORD_HDR_LEN + cipher_len) return 0;

    u8 inner[TLS13_MAX_INNER_PLAINTEXT];
    if (plain_len) simd_memcpy(inner, plain, plain_len);
    if (pad_len) simd_zero(inner + plain_len, pad_len);
    inner[plain_len + pad_len] = content_type;

    u8 hdr[TLS13_RECORD_HDR_LEN];
    tls13_record_build_header(cipher_len, hdr);

    u8 nonce[12];
    tls13_record_build_nonce(dir->iv, dir->seq, nonce);

    u8 tag[TLS13_TAG_LEN];
    if (!aes_gcm_encrypt(&dir->gcm, nonce, 12U, hdr, TLS13_RECORD_HDR_LEN,
                         inner, inner_len, out + TLS13_RECORD_HDR_LEN, tag))
        return 0;

    simd_memcpy(out, hdr, TLS13_RECORD_HDR_LEN);
    simd_memcpy(out + TLS13_RECORD_HDR_LEN + inner_len, tag, TLS13_TAG_LEN);
    dir->seq++;
    return TLS13_RECORD_HDR_LEN + cipher_len;
}

bool tls13_record_open(struct tls13_record_dir *dir, const u8 *in, u32 record_len,
                       u8 *content_type, u8 *out, u32 out_cap, u32 *out_len)
{
    if (!dir || !in || !content_type || !out || !out_len) return false;
    if (record_len < TLS13_RECORD_HDR_LEN + TLS13_TAG_LEN + 1U) return false;

    u32 cipher_len = ((u32)in[3] << 8) | (u32)in[4];
    if (cipher_len != record_len - TLS13_RECORD_HDR_LEN) return false; /* header/actual length mismatch */
    if (cipher_len < TLS13_TAG_LEN + 1U) return false;

    u32 inner_len = cipher_len - TLS13_TAG_LEN;
    if (inner_len > TLS13_MAX_INNER_PLAINTEXT || inner_len > out_cap) return false;

    const u8 *hdr = in;
    const u8 *ciphertext = in + TLS13_RECORD_HDR_LEN;
    const u8 *tag = ciphertext + inner_len;

    u8 nonce[12];
    tls13_record_build_nonce(dir->iv, dir->seq, nonce);

    u8 inner[TLS13_MAX_INNER_PLAINTEXT];
    if (!aes_gcm_decrypt(&dir->gcm, nonce, 12U, hdr, TLS13_RECORD_HDR_LEN,
                         ciphertext, inner_len, inner, tag))
        return false; /* auth failure -- fail closed, do not advance seq */

    /* RFC 8446 Section 5.4: scan back from the end for the last non-zero
     * byte -- that is the real ContentType; everything after it must have
     * been zero padding (already implied by construction), everything
     * before it is the real content. An all-zero inner plaintext (no
     * ContentType byte at all) is a fatal "unexpected_message" condition. */
    i32 i = (i32)inner_len - 1;
    while (i >= 0 && inner[i] == 0) i--;
    if (i < 0) return false;

    *content_type = inner[i];
    *out_len = (u32)i;
    if ((u32)i) simd_memcpy(out, inner, (u32)i);
    dir->seq++;
    return true;
}

/* Bare-metal-only selftest (needs the real ARM-crypto-extension AES-GCM in
 * crypto.c, so this cannot be host-tested -- see tests/test_p256_ecdh.c and
 * tests/test_tls13_keysched.c for the host-testable pure-logic pieces of
 * this TLS 1.3 server work). Reproduces the OFFICIAL RFC 8448 Section 3
 * "Simple 1-RTT Handshake" client Finished record byte-for-byte: seals the
 * exact plaintext Finished message with the exact client handshake write
 * key/IV from that trace and checks the result against the RFC's own
 * "complete record" bytes. Register this in the kernel's `selftest`
 * battery (src/kernel.c's `bat[]` array). */
bool tls13_record_selftest(void)
{
    static const u8 key[16] = {
        0xdb,0xfa,0xa6,0x93,0xd1,0x76,0x2c,0x5b,0x66,0x6a,0xf5,0xd9,0x50,0x25,0x8d,0x01
    };
    static const u8 iv[12] = {
        0x5b,0xd3,0xc7,0x1b,0x83,0x6e,0x0b,0x76,0xbb,0x73,0x26,0x5f
    };
    static const u8 finished_msg[36] = {
        0x14,0x00,0x00,0x20,
        0xa8,0xec,0x43,0x6d,0x67,0x76,0x34,0xae,0x52,0x5a,0xc1,0xfc,0xeb,0xe1,0x1a,0x03,
        0x9e,0xc1,0x76,0x94,0xfa,0xc6,0xe9,0x85,0x27,0xb6,0x42,0xf2,0xed,0xd5,0xce,0x61
    };
    static const u8 expected_record[58] = {
        0x17,0x03,0x03,0x00,0x35,
        0x75,0xec,0x4d,0xc2,0x38,0xcc,0xe6,0x0b,0x29,0x80,0x44,0xa7,0x1e,0x21,0x9c,0x56,
        0xcc,0x77,0xb0,0x51,0x7f,0xe9,0xb9,0x3c,0x7a,0x4b,0xfc,0x44,0xd8,0x7f,0x38,0xf8,
        0x03,0x38,0xac,0x98,0xfc,0x46,0xde,0xb3,0x84,0xbd,0x1c,0xae,0xac,0xab,0x68,0x67,
        0xd7,0x26,0xc4,0x05,0x46
    };
    /* Key/IV: "client handshake write" = "server handshake read" traffic
     * keys from the RFC 8448 trace (derived from client_hs_traffic_secret
     * = b3eddb12...a55a21), used for the client's very first (seq=0)
     * post-handshake-keys record: its Finished message. */

    struct tls13_record_dir dir;
    tls13_record_dir_init(&dir, key, iv);

    u8 out[64];
    u32 n = tls13_record_seal(&dir, TLS13_CONTENT_HANDSHAKE,
                              finished_msg, sizeof(finished_msg), 0,
                              out, sizeof(out));
    if (n != sizeof(expected_record)) return false;
    for (u32 i = 0; i < n; i++)
        if (out[i] != expected_record[i]) return false;

    /* Round-trip: open() must recover the exact plaintext + content type. */
    struct tls13_record_dir dir2;
    tls13_record_dir_init(&dir2, key, iv);
    u8 content_type = 0;
    u8 recovered[64];
    u32 recovered_len = 0;
    if (!tls13_record_open(&dir2, out, n, &content_type, recovered, sizeof(recovered), &recovered_len))
        return false;
    if (content_type != TLS13_CONTENT_HANDSHAKE) return false;
    if (recovered_len != sizeof(finished_msg)) return false;
    for (u32 i = 0; i < recovered_len; i++)
        if (recovered[i] != finished_msg[i]) return false;

    return true;
}

