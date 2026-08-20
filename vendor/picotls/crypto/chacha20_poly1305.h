/*
 * ChaCha20-Poly1305 AEAD (RFC 8439 §2.8) — what TLS 1.3 uses.
 *
 * Ciphertext = ChaCha20(key, counter=1, nonce, plaintext)
 * Otp        = ChaCha20(key, counter=0, nonce)[0..31]   (one-time key for Poly1305)
 * MAC input  = aad || pad16(aad) || ct || pad16(ct) || len64_le(aad) || len64_le(ct)
 * Tag        = Poly1305(Otp, MAC input)
 *
 * The function does NOT do constant-time tag comparison; the caller
 * is responsible for that on decrypt. We provide a helper.
 */
#ifndef PICOWEB_USERSPACE_CRYPTO_CHACHA20_POLY1305_H
#define PICOWEB_USERSPACE_CRYPTO_CHACHA20_POLY1305_H

#include <stddef.h>
#include <stdint.h>

#define AEAD_CHACHA20_POLY1305_KEY_LEN   32u
#define AEAD_CHACHA20_POLY1305_NONCE_LEN 12u
#define AEAD_CHACHA20_POLY1305_TAG_LEN   16u

/* Encrypts `pt_len` bytes from `pt` to `ct`, computing the auth tag
 * over `aad` (length `aad_len`) and `ct`. `pt` and `ct` may alias. */
void aead_chacha20_poly1305_seal(const uint8_t key[AEAD_CHACHA20_POLY1305_KEY_LEN],
                                 const uint8_t nonce[AEAD_CHACHA20_POLY1305_NONCE_LEN],
                                 const uint8_t* aad, size_t aad_len,
                                 const uint8_t* pt,  size_t pt_len,
                                 uint8_t* ct,
                                 uint8_t tag[AEAD_CHACHA20_POLY1305_TAG_LEN]);

/* Decrypts and verifies. Returns 0 on success, -1 on auth failure
 * (in which case `pt` MUST be discarded by the caller). */
int aead_chacha20_poly1305_open(const uint8_t key[AEAD_CHACHA20_POLY1305_KEY_LEN],
                                const uint8_t nonce[AEAD_CHACHA20_POLY1305_NONCE_LEN],
                                const uint8_t* aad, size_t aad_len,
                                const uint8_t* ct,  size_t ct_len,
                                const uint8_t tag[AEAD_CHACHA20_POLY1305_TAG_LEN],
                                uint8_t* pt);

/* Scatter-gather seal: encrypts a chain of plaintext fragments into a
 * single contiguous ciphertext buffer + tag. Bit-identical to
 * `aead_chacha20_poly1305_seal` over the concatenated plaintext.
 *
 * `total_pt_len` MUST equal the sum of `pt_iov[].len`; the caller
 * normally precomputes this so the TLS record header length field can
 * be written before any encryption work is done.
 *
 * `ct_out` must have room for `total_pt_len` bytes.
 *
 * The fragments MUST point at distinct, non-overlapping memory; they
 * are also not aliased with `ct_out`. (Both restrictions could be
 * relaxed but are not needed for the picoweb use case where fragments
 * come from the immutable static arena and ct goes into a per-worker
 * pool slot.) */
struct pw_iov;     /* fwd: real def in userspace/iov.h */
void aead_chacha20_poly1305_seal_iov(const uint8_t key[AEAD_CHACHA20_POLY1305_KEY_LEN],
                                     const uint8_t nonce[AEAD_CHACHA20_POLY1305_NONCE_LEN],
                                     const uint8_t* aad, size_t aad_len,
                                     const struct pw_iov* pt_iov, unsigned pt_iov_n,
                                     size_t total_pt_len,
                                     uint8_t* ct_out,
                                     uint8_t tag[AEAD_CHACHA20_POLY1305_TAG_LEN]);

#endif
