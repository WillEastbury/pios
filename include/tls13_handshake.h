#pragma once
#include "types.h"
#include "crypto.h"

/*
 * tls13_handshake.h - TLS 1.3 (RFC 8446 Section 4) handshake message
 * parsing/construction for the SERVER role. Pure logic (no MMIO/asm,
 * no network I/O) -- host-testable, see tests/test_tls13_clienthello.c.
 *
 * Every parse function is strict and bounds-checked (per this repo's
 * "length is authority" / "bounds checked before touch" invariants):
 * malformed, truncated, or oversized input is rejected outright, never
 * truncated or best-effort-parsed.
 */

#define TLS13_HS_CLIENT_HELLO         1U
#define TLS13_HS_SERVER_HELLO         2U
#define TLS13_HS_NEW_SESSION_TICKET   4U
#define TLS13_HS_ENCRYPTED_EXTENSIONS 8U
#define TLS13_HS_CERTIFICATE          11U
#define TLS13_HS_CERTIFICATE_VERIFY   15U
#define TLS13_HS_FINISHED             20U

#define TLS13_CIPHER_AES_128_GCM_SHA256 0x1301U
#define TLS13_GROUP_SECP256R1           0x0017U
#define TLS13_SIGALG_ECDSA_SECP256R1_SHA256 0x0403U
#define TLS13_VERSION_1_3               0x0304U

#define TLS13_CH_MAX_SESSION_ID   32U
#define TLS13_CH_MAX_CIPHERS      64U   /* real ClientHellos offer a handful; well clear of any real client */
#define TLS13_CH_MAX_SIGALGS      32U
#define TLS13_CH_MAX_GROUPS       16U
#define TLS13_CH_SNI_MAX          128U

/* Parsed, validated fields this server actually needs from a ClientHello.
 * Everything else (padding, unknown extensions, PSK/early-data, etc.) is
 * skipped over during parsing (per RFC 8446, unknown extensions MUST be
 * ignored) but not retained. */
struct tls13_client_hello {
    u8  legacy_session_id[TLS13_CH_MAX_SESSION_ID];
    u32 legacy_session_id_len;

    u16 cipher_suites[TLS13_CH_MAX_CIPHERS];
    u32 cipher_suite_count;

    bool has_supported_versions;
    bool offers_tls13;                 /* supported_versions list includes 0x0304 */

    u16 sig_algs[TLS13_CH_MAX_SIGALGS];
    u32 sig_alg_count;

    u16 groups[TLS13_CH_MAX_GROUPS];
    u32 group_count;

    /* secp256r1 key_share, if the client offered one directly (no
     * HelloRetryRequest support in this server -- if the client didn't
     * include a secp256r1 key_share, the connection is simply rejected,
     * matching this server's minimal/no-HRR design). */
    bool has_p256_key_share;
    u8   p256_key_share[65]; /* SEC1 uncompressed 0x04||X||Y */

    bool has_sni;
    char sni[TLS13_CH_SNI_MAX];
    u32  sni_len;
};

/* Parses a ClientHello handshake message BODY (i.e. NOT including the
 * 4-byte handshake header msg_type+length -- caller strips that first;
 * see tls13_next_handshake_header()). Returns true and fills *out on
 * success. Rejects anything malformed/truncated/oversized (never
 * best-effort parses). Does not validate that the offered parameters are
 * ACCEPTABLE to this server (no TLS_AES_128_GCM_SHA256, no secp256r1
 * key_share, etc.) -- callers check *out's fields for that after a
 * successful parse. */
bool tls13_parse_client_hello(const u8 *body, u32 body_len, struct tls13_client_hello *out);

/* Reads a 4-byte handshake message header (1-byte msg_type, 3-byte
 * big-endian length) from `buf` (buf_len bytes available). On success,
 * sets *msg_type and *body_len and returns true; the message body starts
 * at buf+4. Returns false if fewer than 4 bytes are available, or if
 * body_len would exceed buf_len-4 (caller must accumulate more bytes
 * from the record layer before retrying -- this function never partially
 * consumes a header). */
bool tls13_next_handshake_header(const u8 *buf, u32 buf_len, u8 *msg_type, u32 *body_len);

/* ==================================================================
 * Transcript hash (RFC 8446 Section 4.4.1): a running SHA-256 over the
 * raw bytes of every handshake message IN ORDER (the 4-byte handshake
 * header included, TLS record-layer framing excluded). tls13_transcript_
 * snapshot() takes the hash of everything fed so far WITHOUT disturbing
 * the live running state, so it can be called at each of the several
 * points the handshake needs a hash (through ServerHello, through
 * Certificate, through CertificateVerify, through server Finished,
 * through client Finished) while continuing to accumulate afterward.
 * ================================================================== */
struct tls13_transcript {
    struct sha256_ctx ctx;
};

void tls13_transcript_init(struct tls13_transcript *t);
void tls13_transcript_update(struct tls13_transcript *t, const u8 *data, u32 len);
void tls13_transcript_snapshot(const struct tls13_transcript *t, u8 hash[32]);

/* ==================================================================
 * Server-side handshake message builders. Every builder writes a
 * complete handshake message (4-byte header + body) to `out` and
 * returns the total length written, or 0 on failure (buffer too small
 * or invalid input) -- never partially writes a message. Callers are
 * responsible for feeding the returned bytes into both the transcript
 * hash and the record layer (tls13_record_seal(), content type
 * TLS13_CONTENT_HANDSHAKE) in the same order they were built.
 * ================================================================== */

/* ServerHello: echoes session_id_len/session_id verbatim from the
 * ClientHello, selects the given cipher_suite (caller has already
 * checked it was offered), and includes the server's own P-256
 * ephemeral public key (65-byte SEC1 uncompressed) in a secp256r1
 * key_share plus a supported_versions extension selecting TLS 1.3. */
u32 tls13_build_server_hello(const u8 *session_id, u32 session_id_len,
                             u16 cipher_suite,
                             const u8 server_p256_pub[65],
                             const u8 server_random[32],
                             u8 *out, u32 out_cap);

/* EncryptedExtensions with an empty extensions list -- sufficient for a
 * minimal server that doesn't negotiate ALPN or other optional features. */
u32 tls13_build_encrypted_extensions(u8 *out, u32 out_cap);

/* Certificate: empty certificate_request_context, one CertificateEntry
 * (the leaf DER certificate, no extensions) -- this server only ever
 * presents its single self-signed leaf, no intermediate chain. */
u32 tls13_build_certificate(const u8 *cert_der, u32 cert_der_len, u8 *out, u32 out_cap);

/* Builds the RFC 8446 Section 4.4.3 "signed data" for a server
 * CertificateVerify: 64 x 0x20, the context string "TLS 1.3, server
 * CertificateVerify", a 0x00 separator, then the transcript hash
 * (through Certificate). out must have room for 64+33+1+32 = 130 bytes;
 * returns the exact length written (always 130) or 0 on bad input. */
u32 tls13_build_certificate_verify_signed_data(const u8 transcript_hash[32],
                                               u8 *out, u32 out_cap);

/* CertificateVerify using ecdsa_secp256r1_sha256 (0x0403): signs the
 * signed-data (see above) with the server's raw 32-byte P-256 private
 * scalar via ecdsa_p256_sha256_sign()+ecdsa_p256_encode_der() (already
 * used by x509.c for P-256 CSR/cert signing) and wraps it in the
 * CertificateVerify message format (2-byte SignatureScheme + 2-byte
 * length + DER signature). */
u32 tls13_build_certificate_verify_p256(const u8 p256_private_scalar[32],
                                        const u8 transcript_hash[32],
                                        u8 *out, u32 out_cap);

/* Finished: verify_data = HMAC-SHA256(finished_key, transcript_hash),
 * where finished_key = HKDF-Expand-Label(base_key, "finished", "", 32)
 * -- base_key is the server_hs_traffic_secret for the server's own
 * Finished (see tls13_keysched.h). Wraps the resulting 32 bytes in the
 * Finished message format (just the raw verify_data, no substructure). */
u32 tls13_build_finished(const u8 verify_data[32], u8 *out, u32 out_cap);

/* Parses a Finished message BODY (post handshake-header) and returns its
 * 32-byte verify_data via *out. Rejects anything not exactly 32 bytes. */
bool tls13_parse_finished(const u8 *body, u32 body_len, u8 out_verify_data[32]);
