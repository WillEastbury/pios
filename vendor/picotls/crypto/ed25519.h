/*
 * Ed25519 (RFC 8032 §5.1) — pure C reference implementation.
 *
 * Provides:
 *   ed25519_pubkey_from_seed(pk, seed)
 *   ed25519_sign(sig, msg, len, seed, pk)
 *   ed25519_verify(sig, msg, len, pk)  -> 1 valid, 0 invalid
 *
 * Uses SHA-512 (sha512.h) per RFC 8032. Field arithmetic over
 * GF(2^255 - 19) is implemented internally in ed25519.c (5x51-bit
 * limbs); we duplicate the layout from x25519.c rather than refactor
 * the shipping X25519 module.
 *
 * Spike scope. Variable-time sign and verify. No precomputed
 * base-point table. No SIMD. Does NOT yet reject small-order public
 * keys — fine while picoweb only signs with its own cert and never
 * verifies attacker-controlled certs (no mTLS).
 *
 * Reference: RFC 8032 (EdDSA), Bernstein/Lange/Schwabe et al. The
 * sc_reduce reduction constants (666643, 470296, 654183, -997805,
 * 136657, -683901) are L_low expressed in balanced base 2^21 limbs;
 * they are mathematical facts derived from L = 2^252 +
 * 0x14def9dea2f79cd65812631a5cf5d3ed and not third-party code.
 */
#ifndef PICOWEB_USERSPACE_CRYPTO_ED25519_H
#define PICOWEB_USERSPACE_CRYPTO_ED25519_H

#include <stddef.h>
#include <stdint.h>

#define ED25519_SEED_LEN     32u   /* private key seed */
#define ED25519_PUBKEY_LEN   32u   /* compressed public key */
#define ED25519_SIG_LEN      64u   /* R || S */

void ed25519_pubkey_from_seed(uint8_t pk[ED25519_PUBKEY_LEN],
                              const uint8_t seed[ED25519_SEED_LEN]);

void ed25519_sign(uint8_t sig[ED25519_SIG_LEN],
                  const uint8_t* msg, size_t msg_len,
                  const uint8_t seed[ED25519_SEED_LEN],
                  const uint8_t pk[ED25519_PUBKEY_LEN]);

int ed25519_verify(const uint8_t sig[ED25519_SIG_LEN],
                   const uint8_t* msg, size_t msg_len,
                   const uint8_t pk[ED25519_PUBKEY_LEN]);

#endif
