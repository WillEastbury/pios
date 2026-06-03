#ifndef PIOS_ECDSA_H
#define PIOS_ECDSA_H

#include "types.h"
/* ECDSA-P256-SHA256 sign with RFC 6979 deterministic k.
 * msg is hashed with SHA-256 internally. */
int ecdsa_p256_sha256_sign(const u8 priv[32],
                           const u8* msg, usize msg_n,
                           u8 out_r[32], u8 out_s[32]);

/* DER-encode (r, s) as SEQUENCE { INTEGER r, INTEGER s }. */
int ecdsa_p256_encode_der(const u8 r[32], const u8 s[32],
                          u8* out, usize out_cap);

/* Raw TLS-legacy/JWS style helper: out = r || s. TLS 1.3 CV uses DER. */
int ecdsa_p256_encode_raw64(const u8 r[32], const u8 s[32],
                            u8 out[64]);

#endif
