#pragma once
#include "types.h"
#include "crypto.h"

/*
 * tls13_record.h - TLS 1.3 (RFC 8446 Section 5) record-layer protection:
 * per-direction AEAD state, nonce construction (static_iv XOR sequence
 * number), and TLSInnerPlaintext framing (content || zeros* || real
 * ContentType byte). Symmetric / role-agnostic: server and client use
 * the exact same seal()/open() calls, just with their own traffic keys
 * assigned to TX vs RX (see tls13_keysched.h's direction-assignment doc
 * comment).
 *
 * This server only ever negotiates TLS_AES_128_GCM_SHA256, so the AEAD
 * is always AES-128-GCM (crypto.h's existing aes_gcm_encrypt/decrypt).
 */

#define TLS13_CONTENT_CHANGE_CIPHER_SPEC 0x14U
#define TLS13_CONTENT_ALERT              0x15U
#define TLS13_CONTENT_HANDSHAKE          0x16U
#define TLS13_CONTENT_APPLICATION_DATA   0x17U

#define TLS13_TAG_LEN        16U
#define TLS13_RECORD_HDR_LEN 5U
/* This server intentionally supports a much smaller max fragment than TLS's
 * spec-permitted 16384 bytes (RFC 8446 Section 5.1): its own ClientHello/
 * Certificate/CertificateVerify/Finished messages and HTTP response bodies
 * are all well under a few KB, and capping this bounds every scratch
 * buffer used during framing. Oversized inbound records are rejected
 * (fail closed), not truncated. */
#define TLS13_MAX_INNER_PLAINTEXT 4096U

struct tls13_record_dir {
    struct aes_gcm_ctx gcm;
    u8  iv[12];
    u64 seq;
};

/* Initializes a direction's AEAD state from a 16-byte AES-128 key and
 * 12-byte static IV (both produced by tls13_derive_traffic_keys()).
 * Resets the sequence number to 0 -- callers must call this again after
 * every key-schedule transition (handshake keys -> application keys). */
void tls13_record_dir_init(struct tls13_record_dir *dir, const u8 key[16], const u8 iv[12]);

/* Builds the 12-byte per-record AEAD nonce = static_iv XOR
 * left-zero-padded-big-endian(seq) per RFC 8446 Section 5.3. Exposed
 * separately (pure, no AEAD call) so it can be unit tested without a
 * real AES-GCM implementation. */
void tls13_record_build_nonce(const u8 iv[12], u64 seq, u8 nonce[12]);

/* Builds the 5-byte TLSCiphertext record header (opaque_type=0x17,
 * legacy_record_version=0x0303, length). Exposed separately for the same
 * reason as tls13_record_build_nonce(). */
void tls13_record_build_header(u32 ciphertext_len, u8 hdr[TLS13_RECORD_HDR_LEN]);

/* Seals one handshake/application/alert message (content_type, plain,
 * plain_len) into a complete on-wire TLSCiphertext record at `out`
 * (capacity out_cap). pad_len extra zero bytes are inserted before the
 * trailing real ContentType byte (0 is normal/typical -- this server
 * does not pad to obscure lengths). Advances dir->seq only on success.
 * Returns the total record length written (header + ciphertext + tag),
 * or 0 on failure (oversized input for out_cap, or plain_len+pad_len+1
 * exceeding TLS's own max fragment size). */
u32 tls13_record_seal(struct tls13_record_dir *dir, u8 content_type,
                     const u8 *plain, u32 plain_len, u32 pad_len,
                     u8 *out, u32 out_cap);

/* Opens one complete on-wire TLSCiphertext record (`in`, record_len bytes,
 * beginning with the 5-byte header). On success, returns true, sets
 * *content_type to the real (unpadded) ContentType, writes the decrypted
 * inner content (with any zero padding and the trailing ContentType byte
 * already stripped) to `out` (capacity out_cap -- record_len is always a
 * safe upper bound), sets *out_len, and advances dir->seq. On any
 * decryption/framing failure returns false and does NOT advance dir->seq
 * (caller must treat this as fatal per RFC 8446 Section 5.2 -- do not
 * retry the same record). */
bool tls13_record_open(struct tls13_record_dir *dir, const u8 *in, u32 record_len,
                       u8 *content_type, u8 *out, u32 out_cap, u32 *out_len);

/* Bare-metal-only RFC 8448 known-answer selftest (needs the real AES-GCM
 * in crypto.c). Register in the kernel's `selftest` command battery. */
bool tls13_record_selftest(void);
