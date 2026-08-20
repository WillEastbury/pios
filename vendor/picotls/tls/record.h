/*
 * TLS 1.3 record layer (RFC 8446 §5).
 *
 * On the wire, every record is:
 *
 *   ContentType opaque_type = application_data (23)
 *   ProtocolVersion legacy_record_version = 0x0303
 *   uint16 length
 *   opaque encrypted_record[length]
 *
 * The encrypted_record is the ciphertext + 16-byte AEAD tag.
 * The plaintext inside is:
 *
 *   opaque content[TLSPlaintext.length]
 *   ContentType type            -- the actual type
 *   uint8 zeros[length_of_padding]
 *
 * The 5-byte record header is the AEAD additional_data (RFC 8446
 * §5.2):  opaque_type || legacy_record_version || length.
 *
 * The nonce is the per-direction static_iv XORed with the record
 * sequence number padded out to iv_length (RFC 8446 §5.3).
 */
#ifndef PICOWEB_USERSPACE_TLS_RECORD_H
#define PICOWEB_USERSPACE_TLS_RECORD_H

#include <stddef.h>
#include <stdint.h>

#define TLS13_RECORD_HEADER_LEN 5u
#define TLS13_AEAD_TAG_LEN      16u
#define TLS13_MAX_PLAINTEXT     16384u    /* 2^14 */
/* Plaintext + 1 type byte + max ~256 bytes of padding + AEAD tag.
 * RFC 8446 §5.2 caps at TLSPlaintext.length + 256. */
#define TLS13_MAX_CIPHERTEXT    (TLS13_MAX_PLAINTEXT + 256u + TLS13_AEAD_TAG_LEN)

/* Wire size of one maximally-sized TLS 1.3 record (header + body).
 * This is the canonical "fits one TLS record on the wire" constant.
 * Use it to size:
 *
 *   - mbuf / RX reassembly pool slots that buffer one TLS record off
 *     the network before feeding it to the engine
 *   - any caller-side scratch that must hold a full ciphertext record
 *
 * The engine's RX + APP buffers use this size (PW_TLS_BUF_CAP in
 * engine.h). TX uses a larger dedicated cap because responses may span
 * multiple records.
 *
 * 5 + (16384 + 256 + 16) = 16661 bytes (~16.27 KiB).
 */
#define PW_TLS_WIRE_RECORD_MAX  (TLS13_RECORD_HEADER_LEN + TLS13_MAX_CIPHERTEXT)

/* Recommended slot_size for a buffer_pool_t whose slots each hold
 * one inbound TLS record awaiting reassembly + decryption. Sized
 * generously to absorb any future growth in TLS13_MAX_CIPHERTEXT
 * (e.g. if record_size_limit ever bumps the plaintext cap). */
#define PW_RX_REASSEMBLY_SLOT   PW_TLS_WIRE_RECORD_MAX

typedef enum {
    TLS_CT_INVALID            = 0,
    TLS_CT_CHANGE_CIPHER_SPEC = 20,
    TLS_CT_ALERT              = 21,
    TLS_CT_HANDSHAKE          = 22,
    TLS_CT_APPLICATION_DATA   = 23,
} tls_content_type_t;

/* Per-direction record state. The static_iv and key are derived from
 * the relevant traffic_secret via tls13_derive_traffic_keys(). */
typedef struct {
    uint8_t key[32];          /* ChaCha20 key */
    uint8_t static_iv[12];    /* TLS 1.3 §5.3 base nonce */
    uint64_t seq;             /* incremented per record */
} tls_record_dir_t;

/* Build the per-record nonce by XORing seq into the rightmost
 * 8 bytes of static_iv (RFC 8446 §5.3). */
void tls13_build_nonce(const tls_record_dir_t* dir, uint8_t nonce[12]);

/* Encrypt a single record. `inner_type` becomes the trailer byte of
 * the TLSInnerPlaintext; `outer_type` (typically application_data)
 * goes in the record header. Output is the full record on the wire,
 * including the 5-byte header. Returns the total wire length, or 0
 * on overflow.
 *
 * `out` must have room for TLS13_RECORD_HEADER_LEN + plaintext_len +
 * 1 (type) + AEAD tag. */
size_t tls13_seal_record(tls_record_dir_t* dir,
                         tls_content_type_t inner_type,
                         tls_content_type_t outer_type,
                         const uint8_t* plaintext, size_t plaintext_len,
                         uint8_t* out, size_t out_cap);

/* Scatter-gather variant. The plaintext is the concatenation of
 * `pt_iov[0..pt_iov_n-1]` (total = `total_plaintext_len`); the type
 * trailer is appended internally. Output and bounds rules match the
 * contiguous variant. */
struct pw_iov;
size_t tls13_seal_record_iov(tls_record_dir_t* dir,
                             tls_content_type_t inner_type,
                             tls_content_type_t outer_type,
                             const struct pw_iov* pt_iov, unsigned pt_iov_n,
                             size_t total_plaintext_len,
                             uint8_t* out, size_t out_cap);

/* Decrypt a record in place. `record` points at the wire bytes
 * starting with the 5-byte header. On success returns the inner
 * plaintext length and writes the recovered TLSInnerPlaintext.type
 * to *inner_type_out; on failure returns -1. */
int tls13_open_record(tls_record_dir_t* dir,
                      uint8_t* record, size_t record_len,
                      tls_content_type_t* inner_type_out,
                      uint8_t** plaintext_out, size_t* plaintext_len_out);

#endif
