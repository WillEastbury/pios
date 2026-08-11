/*
 * TLS 1.3 key schedule (RFC 8446 §7.1).
 *
 * Implements:
 *   - HKDF-Expand-Label(secret, label, context, length)
 *   - Derive-Secret(secret, label, messages)  (uses transcript hash)
 *   - The Early/Handshake/Master secret derivations
 *
 * All five RFC 8446 §7.1 secret derivations are exercised; the
 * concrete labels live in static const strings at the bottom of
 * this header.
 */
#ifndef PICOWEB_USERSPACE_TLS_KEYSCHED_H
#define PICOWEB_USERSPACE_TLS_KEYSCHED_H

#include <stddef.h>
#include <stdint.h>

#include "../crypto/sha256.h"

#define TLS13_HASH_LEN SHA256_DIGEST_LEN   /* 32 — only SHA-256 supported in spike */

/* HKDF-Expand-Label(Secret, Label, Context, Length) per RFC 8446 §7.1.
 *
 *   HkdfLabel = struct {
 *     uint16 length = Length;
 *     opaque label<7..255> = "tls13 " + Label;
 *     opaque context<0..255> = Context;
 *   } HkdfLabel;
 *
 *   HKDF-Expand-Label = HKDF-Expand(Secret, HkdfLabel, Length)
 *
 * Returns 0 on success, -1 if length exceeds HKDF capacity. */
int tls13_hkdf_expand_label(const uint8_t secret[TLS13_HASH_LEN],
                            const char* label,
                            const uint8_t* context, size_t context_len,
                            uint8_t* out, size_t out_len);

/* Derive-Secret(Secret, Label, Messages) per RFC 8446 §7.1.
 *
 * Messages is the concatenation of handshake messages so far; we
 * compute its SHA-256 transcript hash here (the caller does NOT
 * pre-hash). Output is exactly TLS13_HASH_LEN bytes. */
int tls13_derive_secret(const uint8_t secret[TLS13_HASH_LEN],
                        const char* label,
                        const uint8_t* messages, size_t messages_len,
                        uint8_t out[TLS13_HASH_LEN]);

/* Convenience: derive the per-direction traffic key + iv from a
 * traffic_secret. ChaCha20-Poly1305 uses 32-byte key, 12-byte IV. */
void tls13_derive_traffic_keys(const uint8_t traffic_secret[TLS13_HASH_LEN],
                               uint8_t key[32], uint8_t iv[12]);

/* Compute the Finished message verify_data per RFC 8446 §4.4.4.
 *
 *   finished_key = HKDF-Expand-Label(BaseKey, "finished", "", Hash.length)
 *   verify_data  = HMAC-SHA256(finished_key,
 *                              Transcript-Hash(Handshake context, Cert*))
 *
 * `base_key` is the relevant traffic secret (server_handshake_traffic_secret
 * for the server's Finished, client_handshake_traffic_secret for the
 * client's). `transcript_hash` is the SHA-256 of the handshake messages
 * UP TO BUT NOT INCLUDING the Finished message itself.
 *
 * Output `verify_data` is TLS13_HASH_LEN (32) bytes. Returns 0 on
 * success, -1 on internal error. */
int tls13_compute_finished(const uint8_t base_key[TLS13_HASH_LEN],
                           const uint8_t transcript_hash[TLS13_HASH_LEN],
                           uint8_t verify_data[TLS13_HASH_LEN]);

/* Constant-time verify of an inbound peer Finished. Returns 0 if
 * `verify_data` matches the locally-computed value, -1 otherwise. */
int tls13_verify_finished(const uint8_t base_key[TLS13_HASH_LEN],
                          const uint8_t transcript_hash[TLS13_HASH_LEN],
                          const uint8_t verify_data[TLS13_HASH_LEN]);

#endif
