#pragma once
#include "types.h"

/*
 * tls13_keysched.h - TLS 1.3 (RFC 8446 §7.1) HKDF-Expand-Label and key
 * schedule helpers, built on top of the existing crypto.h primitives
 * (hkdf_extract/hkdf_expand/hmac_sha256/sha256). Pure logic, no MMIO/asm
 * dependencies -- host-testable (see tests/test_tls13_keysched.c, checked
 * against the official RFC 8448 "Simple 1-RTT Handshake" byte vectors).
 *
 * Server-side usage mirrors the client-side schedule exactly (RFC 8446
 * §7.1 is symmetric) -- only which secret is used for TX vs RX differs:
 *   server TX (handshake) = server_hs_traffic_secret
 *   server RX (handshake) = client_hs_traffic_secret
 *   server TX (application) = server_ap_traffic_secret
 *   server RX (application) = client_ap_traffic_secret
 */

#define TLS13_HASH_LEN     32U   /* SHA-256 output length */
#define TLS13_AEAD_KEY_LEN 16U   /* TLS_AES_128_GCM_SHA256 key length */
#define TLS13_AEAD_IV_LEN  12U   /* TLS 1.3 record nonce/IV length */

/* HKDF-Expand-Label(Secret, Label, Context, Length) per RFC 8446 §7.1:
 *   HkdfLabel = uint16 Length || opaque "tls13 " + Label <7..255>
 *               || opaque Context <0..255>
 *   result = HKDF-Expand(Secret, HkdfLabel, Length)
 * label_len must be <= 249 (255 - strlen("tls13 ")) and context_len <= 255;
 * out_len must be <= 255 (sufficient for every TLS 1.3 SHA-256 derivation
 * this server needs -- traffic secrets, keys/IVs, Finished keys are all
 * well under this). Returns false on oversized inputs, never truncates
 * silently. */
bool tls13_hkdf_expand_label(const u8 *secret, u32 secret_len,
                             const char *label, u32 label_len,
                             const u8 *context, u32 context_len,
                             u8 *out, u32 out_len);

/* Derive-Secret(Secret, Label, Messages) = HKDF-Expand-Label(Secret, Label,
 * Transcript-Hash(Messages), Hash.length). transcript_hash must be exactly
 * TLS13_HASH_LEN bytes (pass a SHA-256 hash of "" for the empty-transcript
 * case, e.g. the "derived" label before any messages exist). */
bool tls13_derive_secret(const u8 *secret, u32 secret_len,
                         const char *label, u32 label_len,
                         const u8 transcript_hash[TLS13_HASH_LEN],
                         u8 out[TLS13_HASH_LEN]);

/* Full key-schedule secrets for one connection, derived in the RFC 8446
 * §7.1 order. shared_secret is the raw ECDHE shared value (for this
 * server: p256_scalar_mul_point()'s 64-byte X||Y output -- only the first
 * 32 bytes, the X coordinate, are used as IKM, matching every TLS 1.3
 * ECDHE group). No PSK/early-data support, so early_secret always uses an
 * all-zero IKM (RFC 8446 §7.1, "unless" this connection uses a PSK). */
struct tls13_secrets {
    u8 early_secret[TLS13_HASH_LEN];
    u8 handshake_secret[TLS13_HASH_LEN];
    u8 master_secret[TLS13_HASH_LEN];
    u8 client_hs_traffic[TLS13_HASH_LEN];
    u8 server_hs_traffic[TLS13_HASH_LEN];
    u8 client_ap_traffic[TLS13_HASH_LEN];
    u8 server_ap_traffic[TLS13_HASH_LEN];
};

/* Computes early_secret, handshake_secret, client/server_hs_traffic from
 * the ECDHE shared secret (32-byte X coordinate) and the transcript hash
 * through ServerHello (Transcript-Hash(ClientHello..ServerHello)). Must be
 * called before application secrets (tls13_derive_application_secrets). */
bool tls13_derive_handshake_secrets(struct tls13_secrets *s,
                                    const u8 shared_secret_x[32],
                                    const u8 transcript_hash_ch_sh[TLS13_HASH_LEN]);

/* Computes master_secret and client/server_ap_traffic from the already-
 * derived handshake_secret and the transcript hash through server Finished
 * (Transcript-Hash(ClientHello..Server Finished)). */
bool tls13_derive_application_secrets(struct tls13_secrets *s,
                                      const u8 transcript_hash_through_sf[TLS13_HASH_LEN]);

/* key = HKDF-Expand-Label(traffic_secret, "key", "", TLS13_AEAD_KEY_LEN)
 * iv  = HKDF-Expand-Label(traffic_secret, "iv",  "", TLS13_AEAD_IV_LEN) */
bool tls13_derive_traffic_keys(const u8 traffic_secret[TLS13_HASH_LEN],
                               u8 key[TLS13_AEAD_KEY_LEN],
                               u8 iv[TLS13_AEAD_IV_LEN]);

/* finished_key = HKDF-Expand-Label(base_key, "finished", "", Hash.length)
 * verify_data  = HMAC-SHA256(finished_key, transcript_hash) */
bool tls13_compute_finished(const u8 base_key[TLS13_HASH_LEN],
                           const u8 transcript_hash[TLS13_HASH_LEN],
                           u8 verify_data[TLS13_HASH_LEN]);

/* Constant-time comparison of two Finished verify_data values (or any
 * other secret-derived comparison in this module) -- callers MUST use
 * this instead of memcmp for anything that gates handshake acceptance. */
bool tls13_consttime_eq(const u8 *a, const u8 *b, u32 len);
