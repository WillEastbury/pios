/*
 * X25519 (RFC 7748 §5) — Curve25519 ECDH.
 *
 * Public values are 32-byte little-endian curve points; private keys
 * are 32-byte little-endian scalars (clamped per RFC 7748 §5).
 *
 * X25519(scalar, base_point=9) -> our public key
 * X25519(scalar, peer_pub)     -> shared secret
 *
 * The shared secret MUST be passed through the TLS 1.3 key schedule
 * (HKDF) before use; raw X25519 output is biased and not suitable as
 * a key directly.
 */
#ifndef PICOWEB_USERSPACE_CRYPTO_X25519_H
#define PICOWEB_USERSPACE_CRYPTO_X25519_H

#include <stddef.h>
#include <stdint.h>

#define X25519_KEY_LEN 32u

void x25519(uint8_t out[X25519_KEY_LEN],
            const uint8_t scalar[X25519_KEY_LEN],
            const uint8_t point[X25519_KEY_LEN]);

/* The base point for X25519 is u=9 little-endian. */
extern const uint8_t X25519_BASE_POINT[X25519_KEY_LEN];

#endif
