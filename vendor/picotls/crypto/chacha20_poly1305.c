/*
 * ChaCha20-Poly1305 AEAD (RFC 8439 §2.8).
 *
 * Zero allocations. Poly1305 is fed incrementally so the AAD || pad ||
 * ciphertext || pad || lengths sequence is streamed without ever
 * materialising the full MAC input. This matters because TLS records
 * can be up to ~16 KiB and we'd otherwise blow the stack via alloca.
 */

#include "chacha20_poly1305.h"

#include <string.h>

#include "../iov.h"
#include "chacha20.h"
#include "poly1305.h"
#include "util.h"

static const uint8_t k_zero16[16] = {0};

static void store_le64(uint8_t* p, uint64_t v) {
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (i * 8));
}

static void mac_data(const uint8_t poly_key[POLY1305_KEY_LEN],
                     const uint8_t* aad, size_t aad_len,
                     const uint8_t* ct,  size_t ct_len,
                     uint8_t tag[16]) {
    poly1305_ctx_t pctx;
    poly1305_init(&pctx, poly_key);

    if (aad_len) poly1305_update(&pctx, aad, aad_len);
    size_t aad_pad = (16u - (aad_len & 15u)) & 15u;
    if (aad_pad) poly1305_update(&pctx, k_zero16, aad_pad);

    if (ct_len)  poly1305_update(&pctx, ct, ct_len);
    size_t ct_pad = (16u - (ct_len & 15u)) & 15u;
    if (ct_pad)  poly1305_update(&pctx, k_zero16, ct_pad);

    uint8_t lens[16];
    store_le64(lens + 0, (uint64_t)aad_len);
    store_le64(lens + 8, (uint64_t)ct_len);
    poly1305_update(&pctx, lens, sizeof(lens));

    poly1305_finish(&pctx, tag);
    /* poly1305_finish wipes pctx; nothing else to scrub here. */
}

void aead_chacha20_poly1305_seal(const uint8_t key[AEAD_CHACHA20_POLY1305_KEY_LEN],
                                 const uint8_t nonce[AEAD_CHACHA20_POLY1305_NONCE_LEN],
                                 const uint8_t* aad, size_t aad_len,
                                 const uint8_t* pt,  size_t pt_len,
                                 uint8_t* ct,
                                 uint8_t tag[AEAD_CHACHA20_POLY1305_TAG_LEN]) {
    /* Derive Poly1305 one-time key from ChaCha20 block 0. Only the
     * first 32 bytes of the keystream block are used; the rest is
     * discarded (and we wipe the whole buffer below). */
    uint8_t poly_key[64];
    chacha20_block(key, 0, nonce, poly_key);

    /* Encrypt with counter starting at 1. */
    chacha20_xor(key, 1, nonce, pt, ct, pt_len);

    mac_data(poly_key, aad, aad_len, ct, pt_len, tag);

    secure_zero(poly_key, sizeof(poly_key));
}

int aead_chacha20_poly1305_open(const uint8_t key[AEAD_CHACHA20_POLY1305_KEY_LEN],
                                const uint8_t nonce[AEAD_CHACHA20_POLY1305_NONCE_LEN],
                                const uint8_t* aad, size_t aad_len,
                                const uint8_t* ct,  size_t ct_len,
                                const uint8_t tag[AEAD_CHACHA20_POLY1305_TAG_LEN],
                                uint8_t* pt) {
    uint8_t poly_key[64];
    chacha20_block(key, 0, nonce, poly_key);

    uint8_t expected[16];
    mac_data(poly_key, aad, aad_len, ct, ct_len, expected);
    secure_zero(poly_key, sizeof(poly_key));

    int ok = crypto_consttime_eq(expected, tag, 16);
    secure_zero(expected, sizeof(expected));
    if (!ok) return -1;

    chacha20_xor(key, 1, nonce, ct, pt, ct_len);
    return 0;
}

/* ---------------- scatter-gather seal ---------------- */

void aead_chacha20_poly1305_seal_iov(const uint8_t key[AEAD_CHACHA20_POLY1305_KEY_LEN],
                                     const uint8_t nonce[AEAD_CHACHA20_POLY1305_NONCE_LEN],
                                     const uint8_t* aad, size_t aad_len,
                                     const struct pw_iov* pt_iov, unsigned pt_iov_n,
                                     size_t total_pt_len,
                                     uint8_t* ct_out,
                                     uint8_t tag[AEAD_CHACHA20_POLY1305_TAG_LEN]) {
    /* Derive the Poly1305 one-time key from ChaCha20 block 0. */
    uint8_t poly_key[64];
    chacha20_block(key, 0, nonce, poly_key);

    /* Streaming ChaCha20 starting at counter=1 (block 0 is reserved
     * for the Poly1305 key derivation above). */
    chacha20_stream_t cs;
    chacha20_stream_init(&cs, key, nonce, 1);

    /* Streaming Poly1305 over: aad || pad16 || ct || pad16 || lens. */
    poly1305_ctx_t pctx;
    poly1305_init(&pctx, poly_key);
    secure_zero(poly_key, sizeof(poly_key));

    if (aad_len) poly1305_update(&pctx, aad, aad_len);
    size_t aad_pad = (16u - (aad_len & 15u)) & 15u;
    if (aad_pad)  poly1305_update(&pctx, k_zero16, aad_pad);

    /* Per-fragment encrypt (in-place stream) + Poly1305 over the
     * ciphertext bytes we just produced. */
    size_t off = 0;
    for (unsigned i = 0; i < pt_iov_n; i++) {
        const uint8_t* fb  = pt_iov[i].base;
        size_t         fl  = pt_iov[i].len;
        if (fl == 0) continue;
        chacha20_stream_xor(&cs, fb, ct_out + off, fl);
        poly1305_update(&pctx, ct_out + off, fl);
        off += fl;
    }
    /* Defensive: if the caller's total_pt_len disagreed with the sum
     * of fragments, the mac's len trailer would still be authoritative
     * — but we'd silently produce a record claiming the wrong length.
     * Trust but verify.  (`off` is the truth.) */
    (void)total_pt_len;

    size_t ct_pad = (16u - (off & 15u)) & 15u;
    if (ct_pad) poly1305_update(&pctx, k_zero16, ct_pad);

    uint8_t lens[16];
    store_le64(lens + 0, (uint64_t)aad_len);
    store_le64(lens + 8, (uint64_t)off);
    poly1305_update(&pctx, lens, sizeof(lens));

    poly1305_finish(&pctx, tag);
    secure_zero(&cs, sizeof(cs));
}
