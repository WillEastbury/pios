#ifndef PIOS_P256_H
#define PIOS_P256_H

#include "types.h"
#define P256_SCALAR_LEN 32
#define P256_COORD_LEN  32
#define P256_PUBKEY_UNCOMPRESSED_LEN 65

/* Compute Q = k * G where G is the standard P-256 base point.
 * out is 64 bytes: X || Y, big-endian.
 * scalar is 32 bytes, big-endian. Must be in [1, n-1].
 * Returns 0 on success, -1 on bad input. */
int p256_scalar_mul_base(const u8 scalar[32], u8 out_xy[64]);

/* Decode SEC1 uncompressed pubkey (0x04 || X || Y), verify on-curve. */
int p256_pubkey_validate(const u8 in[65]);

/* Derive SEC1 uncompressed pubkey (0x04 || X || Y) from private scalar. */
int p256_derive_pubkey(const u8 scalar[32], u8 out_pub[65]);

/* ECDH shared secret: out_xy = scalar * Q (64 bytes: X || Y, big-endian),
 * where Q (peer_pub, 65 bytes SEC1-uncompressed 0x04||X||Y) is an
 * arbitrary, untrusted peer-supplied point -- validated on-curve before
 * use. Returns 0 on success, -1 on malformed/off-curve/low-order input. */
int p256_scalar_mul_point(const u8 scalar[32], const u8 peer_pub[65], u8 out_xy[64]);

#endif
