#include "tls13_client.h"
#include "tls13_handshake.h"
#include "tls13_keysched.h"
#include "tls13_record.h"
#include "tls13_verify.h"
#include "p256.h"
#include "simd.h"

#define PTC_WIRE_MAX \
    (TLS13_RECORD_HDR_LEN + TLS13_MAX_INNER_PLAINTEXT + TLS13_TAG_LEN)
#define PTC_TX_WIRE_MAX (PTC_WIRE_MAX + TLS13_RECORD_HDR_LEN)
#define PTC_HANDSHAKE_MAX TLS13_MAX_INNER_PLAINTEXT
#define PTC_HANDSHAKE_FRAGMENT 96U

enum ptc_state {
    PTC_STATE_IDLE = 0,
    PTC_STATE_SEND_CLIENT_HELLO,
    PTC_STATE_WAIT_SERVER_HELLO,
    PTC_STATE_WAIT_ENCRYPTED_EXTENSIONS,
    PTC_STATE_WAIT_CERTIFICATE,
    PTC_STATE_WAIT_CERTIFICATE_VERIFY,
    PTC_STATE_WAIT_SERVER_FINISHED,
    PTC_STATE_INSTALL_APPLICATION_KEYS,
    PTC_STATE_ESTABLISHED,
    PTC_STATE_ERROR,
    PTC_STATE_CANCELLED
};

struct ptc_impl {
    struct tls13_record_dir tx;
    struct tls13_record_dir rx;
    struct tls13_transcript transcript;
    struct tls13_secrets secrets;

    u8 ephemeral_private[32];
    u8 session_id[32];
    u8 server_name[TLS13_CLIENT_SERVER_NAME_MAX + 1U];
    u32 server_name_len;
    bool has_pin;
    u8 pinned_key[65];
    u8 leaf_key[65];
    u16 peer_record_size_limit;

    u8 tx_wire[PTC_TX_WIRE_MAX];
    u32 tx_len;
    u32 tx_off;
    u32 tx_app_len;

    u8 rx_wire[PTC_WIRE_MAX];
    u32 rx_have;
    u32 rx_need;

    u8 plain[PTC_HANDSHAKE_MAX];
    u32 plain_len;
    u32 plain_off;

    u32 state;
    u32 last_error;
    u32 compat_ccs;
    bool close_sent;
    bool peer_closed;
};

_Static_assert(sizeof(struct ptc_impl) <= TLS13_CLIENT_CONTEXT_BYTES,
               "TLS13_CLIENT_CONTEXT_BYTES is too small");

static struct ptc_impl *impl(tls13_client_t *client)
{
    return (struct ptc_impl *)(void *)client->opaque;
}

static const struct ptc_impl *cimpl(const tls13_client_t *client)
{
    return (const struct ptc_impl *)(const void *)client->opaque;
}

static void secure_zero(void *ptr, u32 len)
{
    volatile u8 *p = (volatile u8 *)ptr;
    while (len--) *p++ = 0;
}

static struct picotls_result result(i32 status, u32 bytes, u32 error)
{
    struct picotls_result out = {
        .status = status,
        .bytes = bytes,
        .error = error,
    };
    return out;
}

static struct picotls_result fail(struct ptc_impl *s, u32 error)
{
    s->last_error = error;
    s->state = PTC_STATE_ERROR;
    secure_zero(s->ephemeral_private, sizeof(s->ephemeral_private));
    secure_zero(&s->secrets, sizeof(s->secrets));
    return result(PICOTLS_STEP_ERROR, 0U, error);
}

static bool io_valid(const struct picotls_io_ops *ops)
{
    return ops && ops->read_some && ops->write_some;
}

static bool handshake_io_valid(const struct picotls_io_ops *ops)
{
    return io_valid(ops) && ops->random_bytes;
}

static i32 flush_output(struct ptc_impl *s,
                        const struct picotls_io_ops *ops,
                        u32 *progress)
{
    *progress = 0U;
    if (s->tx_off == s->tx_len) {
        s->tx_off = 0U;
        s->tx_len = 0U;
        return 1;
    }
    u32 remaining = s->tx_len - s->tx_off;
    i32 written = ops->write_some(ops->ctx, s->tx_wire + s->tx_off,
                                  remaining);
    if (written < 0 || (u32)written > remaining)
        return -1;
    if (written == 0)
        return 0;
    s->tx_off += (u32)written;
    *progress = (u32)written;
    if (s->tx_off != s->tx_len)
        return 0;
    s->tx_off = 0U;
    s->tx_len = 0U;
    return 1;
}

static bool queue_plain_handshake(struct ptc_impl *s,
                                  const u8 *message, u32 message_len)
{
    if (!message || message_len == 0U || s->tx_len != 0U ||
        message_len > sizeof(s->tx_wire) - TLS13_RECORD_HDR_LEN)
        return false;
    u32 first_len = message_len;
    if (message_len > PTC_HANDSHAKE_FRAGMENT &&
        message_len + (2U * TLS13_RECORD_HDR_LEN) <= sizeof(s->tx_wire))
        first_len = PTC_HANDSHAKE_FRAGMENT;
    s->tx_wire[0] = TLS13_CONTENT_HANDSHAKE;
    s->tx_wire[1] = 0x03U;
    s->tx_wire[2] = 0x01U;
    s->tx_wire[3] = (u8)(first_len >> 8);
    s->tx_wire[4] = (u8)first_len;
    simd_memcpy(s->tx_wire + TLS13_RECORD_HDR_LEN, message, first_len);
    s->tx_len = TLS13_RECORD_HDR_LEN + first_len;
    if (first_len != message_len) {
        u32 second_len = message_len - first_len;
        u8 *second = s->tx_wire + s->tx_len;
        second[0] = TLS13_CONTENT_HANDSHAKE;
        second[1] = 0x03U;
        second[2] = 0x01U;
        second[3] = (u8)(second_len >> 8);
        second[4] = (u8)second_len;
        simd_memcpy(second + TLS13_RECORD_HDR_LEN,
                    message + first_len, second_len);
        s->tx_len += TLS13_RECORD_HDR_LEN + second_len;
    }
    s->tx_off = 0U;
    return true;
}

static bool queue_protected(struct ptc_impl *s, u8 content_type,
                            const u8 *plain, u32 plain_len)
{
    if (s->tx_len != 0U ||
        plain_len >= s->peer_record_size_limit)
        return false;
    u32 wire_len = tls13_record_seal(&s->tx, content_type, plain, plain_len,
                                     0U, s->tx_wire,
                                     sizeof(s->tx_wire));
    if (wire_len == 0U) return false;
    s->tx_len = wire_len;
    s->tx_off = 0U;
    return true;
}

static i32 receive_record_step(struct ptc_impl *s,
                               const struct picotls_io_ops *ops,
                               u32 *progress)
{
    *progress = 0U;
    if (s->rx_need != 0U && s->rx_have == s->rx_need)
        return 1;
    u32 target = s->rx_need ? s->rx_need : TLS13_RECORD_HDR_LEN;
    if (s->rx_have < target) {
        i32 got = ops->read_some(ops->ctx, s->rx_wire + s->rx_have,
                                 target - s->rx_have);
        if (got < 0 || (u32)got > target - s->rx_have)
            return -1;
        if (got == 0)
            return 0;
        s->rx_have += (u32)got;
        *progress = (u32)got;
    }
    if (s->rx_need == 0U && s->rx_have == TLS13_RECORD_HDR_LEN) {
        u32 body_len = ((u32)s->rx_wire[3] << 8) | s->rx_wire[4];
        if (body_len == 0U ||
            body_len > sizeof(s->rx_wire) - TLS13_RECORD_HDR_LEN)
            return -2;
        s->rx_need = TLS13_RECORD_HDR_LEN + body_len;
    }
    return s->rx_need != 0U && s->rx_have == s->rx_need ? 1 : 0;
}

static void consume_record(struct ptc_impl *s)
{
    s->rx_have = 0U;
    s->rx_need = 0U;
}

static bool valid_ccs(const struct ptc_impl *s)
{
    return s->rx_need == TLS13_RECORD_HDR_LEN + 1U &&
           s->rx_wire[0] == TLS13_CONTENT_CHANGE_CIPHER_SPEC &&
           s->rx_wire[1] == 0x03U && s->rx_wire[2] == 0x03U &&
           s->rx_wire[5] == 1U;
}

static bool valid_protected_header(const struct ptc_impl *s)
{
    return s->rx_wire[0] == TLS13_CONTENT_APPLICATION_DATA &&
           s->rx_wire[1] == 0x03U && s->rx_wire[2] == 0x03U;
}

static i32 next_handshake(const struct ptc_impl *s, u8 expected,
                          const u8 **message, u32 *message_len,
                          const u8 **body, u32 *body_len)
{
    if (s->plain_len < 4U) return 0;
    u32 len = ((u32)s->plain[1] << 16) |
              ((u32)s->plain[2] << 8) | s->plain[3];
    if (len > PTC_HANDSHAKE_MAX - 4U)
        return -1;
    if (s->plain_len < 4U + len)
        return 0;
    if (s->plain[0] != expected)
        return -1;
    *message = s->plain;
    *message_len = 4U + len;
    *body = s->plain + 4U;
    *body_len = len;
    return 1;
}

static void consume_plain(struct ptc_impl *s, u32 bytes)
{
    if (bytes >= s->plain_len) {
        s->plain_len = 0U;
        s->plain_off = 0U;
        return;
    }
    u32 remain = s->plain_len - bytes;
    for (u32 i = 0; i < remain; i++)
        s->plain[i] = s->plain[bytes + i];
    s->plain_len = remain;
}

static struct picotls_result ingest_plain_handshake(
    struct ptc_impl *s, const struct picotls_io_ops *ops)
{
    u32 progress = 0U;
    i32 rr = receive_record_step(s, ops, &progress);
    if (rr == -1) return fail(s, TLS13_CLIENT_ERR_IO);
    if (rr == -2) return fail(s, TLS13_CLIENT_ERR_RECORD);
    if (rr == 0)
        return result(PICOTLS_STEP_PENDING, progress, TLS13_CLIENT_OK);
    if (s->rx_wire[0] != TLS13_CONTENT_HANDSHAKE ||
        s->rx_wire[1] != 0x03U ||
        (s->rx_wire[2] != 0x01U && s->rx_wire[2] != 0x03U))
        return fail(s, TLS13_CLIENT_ERR_MALFORMED);
    u32 fragment_len = s->rx_need - TLS13_RECORD_HDR_LEN;
    if (fragment_len > sizeof(s->plain) - s->plain_len)
        return fail(s, TLS13_CLIENT_ERR_RECORD);
    simd_memcpy(s->plain + s->plain_len,
                s->rx_wire + TLS13_RECORD_HDR_LEN, fragment_len);
    s->plain_len += fragment_len;
    consume_record(s);
    return result(PICOTLS_STEP_PENDING, progress, TLS13_CLIENT_OK);
}

static struct picotls_result ingest_protected_handshake(
    struct ptc_impl *s, const struct picotls_io_ops *ops)
{
    u32 progress = 0U;
    i32 rr = receive_record_step(s, ops, &progress);
    if (rr == -1) return fail(s, TLS13_CLIENT_ERR_IO);
    if (rr == -2) return fail(s, TLS13_CLIENT_ERR_RECORD);
    if (rr == 0)
        return result(PICOTLS_STEP_PENDING, progress, TLS13_CLIENT_OK);
    if (s->rx_wire[0] == TLS13_CONTENT_CHANGE_CIPHER_SPEC) {
        if (!valid_ccs(s) || ++s->compat_ccs > 4U)
            return fail(s, TLS13_CLIENT_ERR_RECORD);
        consume_record(s);
        return result(PICOTLS_STEP_PENDING, progress, TLS13_CLIENT_OK);
    }
    if (!valid_protected_header(s))
        return fail(s, TLS13_CLIENT_ERR_RECORD);

    u8 content_type = 0U;
    u32 fragment_len = 0U;
    if (!tls13_record_open(&s->rx, s->rx_wire, s->rx_need,
                           &content_type, s->plain + s->plain_len,
                           sizeof(s->plain) - s->plain_len,
                           &fragment_len))
        return fail(s, TLS13_CLIENT_ERR_DECRYPT);
    consume_record(s);
    if (content_type != TLS13_CONTENT_HANDSHAKE)
        return fail(s, TLS13_CLIENT_ERR_RECORD);
    s->plain_len += fragment_len;
    return result(PICOTLS_STEP_PENDING, progress, TLS13_CLIENT_OK);
}

static struct picotls_result queue_client_hello(
    struct ptc_impl *s, const struct picotls_io_ops *ops)
{
    u8 public_key[65];
    bool have_key = false;
    for (u32 attempt = 0; attempt < 8U && !have_key; attempt++) {
        if (!ops->random_bytes(ops->ctx, s->ephemeral_private,
                               sizeof(s->ephemeral_private)))
            break;
        have_key = p256_derive_pubkey(s->ephemeral_private,
                                     public_key) == 0;
    }
    u8 client_random[32];
    if (!have_key ||
        !ops->random_bytes(ops->ctx, client_random,
                           sizeof(client_random)) ||
        !ops->random_bytes(ops->ctx, s->session_id,
                           sizeof(s->session_id)))
        return fail(s, TLS13_CLIENT_ERR_KEY);

    u8 message[512];
    u32 message_len = tls13_build_client_hello(
        s->server_name, s->server_name_len, client_random,
        s->session_id, public_key, message, sizeof(message));
    if (message_len == 0U ||
        !queue_plain_handshake(s, message, message_len))
        return fail(s, TLS13_CLIENT_ERR_RECORD);
    tls13_transcript_init(&s->transcript);
    tls13_transcript_update(&s->transcript, message, message_len);
    s->state = PTC_STATE_WAIT_SERVER_HELLO;
    return result(PICOTLS_STEP_PENDING, 0U, TLS13_CLIENT_OK);
}

static struct picotls_result process_server_hello(
    struct ptc_impl *s, const struct picotls_io_ops *ops)
{
    const u8 *message;
    const u8 *body;
    u32 message_len;
    u32 body_len;
    i32 next = next_handshake(s, TLS13_HS_SERVER_HELLO, &message,
                              &message_len, &body, &body_len);
    if (next < 0) return fail(s, TLS13_CLIENT_ERR_MALFORMED);
    if (next == 0) return ingest_plain_handshake(s, ops);

    struct tls13_server_hello hello;
    if (!tls13_parse_server_hello(body, body_len, &hello) ||
        hello.cipher_suite != TLS13_CIPHER_AES_128_GCM_SHA256 ||
        hello.legacy_session_id_len != sizeof(s->session_id) ||
        !tls13_consttime_eq(hello.legacy_session_id, s->session_id,
                            sizeof(s->session_id)))
        return fail(s, TLS13_CLIENT_ERR_UNSUPPORTED);
    u8 shared[64];
    if (p256_scalar_mul_point(s->ephemeral_private,
                              hello.p256_key_share, shared) != 0)
        return fail(s, TLS13_CLIENT_ERR_KEY);
    secure_zero(s->ephemeral_private, sizeof(s->ephemeral_private));
    tls13_transcript_update(&s->transcript, message, message_len);
    consume_plain(s, message_len);

    u8 hash[32];
    u8 client_key[16], client_iv[12];
    u8 server_key[16], server_iv[12];
    tls13_transcript_snapshot(&s->transcript, hash);
    if (!tls13_derive_handshake_secrets(&s->secrets, shared, hash) ||
        !tls13_derive_traffic_keys(s->secrets.client_hs_traffic,
                                   client_key, client_iv) ||
        !tls13_derive_traffic_keys(s->secrets.server_hs_traffic,
                                   server_key, server_iv))
        return fail(s, TLS13_CLIENT_ERR_KEY);
    secure_zero(shared, sizeof(shared));
    tls13_record_dir_init(&s->tx, client_key, client_iv);
    tls13_record_dir_init(&s->rx, server_key, server_iv);
    s->state = PTC_STATE_WAIT_ENCRYPTED_EXTENSIONS;
    return result(PICOTLS_STEP_PENDING, 0U, TLS13_CLIENT_OK);
}

static struct picotls_result process_encrypted_extensions(
    struct ptc_impl *s, const struct picotls_io_ops *ops)
{
    const u8 *message, *body;
    u32 message_len, body_len;
    i32 next = next_handshake(s, TLS13_HS_ENCRYPTED_EXTENSIONS,
                              &message, &message_len, &body, &body_len);
    if (next < 0) return fail(s, TLS13_CLIENT_ERR_MALFORMED);
    if (next == 0) return ingest_protected_handshake(s, ops);
    u16 record_size_limit = 0U;
    if (!tls13_parse_encrypted_extensions_limit(body, body_len,
                                                &record_size_limit))
        return fail(s, TLS13_CLIENT_ERR_MALFORMED);
    if (record_size_limit != 0U)
        s->peer_record_size_limit = record_size_limit;
    tls13_transcript_update(&s->transcript, message, message_len);
    consume_plain(s, message_len);
    s->state = PTC_STATE_WAIT_CERTIFICATE;
    return result(PICOTLS_STEP_PENDING, 0U, TLS13_CLIENT_OK);
}

static struct picotls_result process_certificate(
    struct ptc_impl *s, const struct picotls_io_ops *ops)
{
    const u8 *message, *body;
    u32 message_len, body_len;
    i32 next = next_handshake(s, TLS13_HS_CERTIFICATE,
                              &message, &message_len, &body, &body_len);
    if (next < 0) return fail(s, TLS13_CLIENT_ERR_MALFORMED);
    if (next == 0) return ingest_protected_handshake(s, ops);
    const u8 *certificate;
    u32 certificate_len;
    if (!tls13_parse_certificate_leaf(body, body_len,
                                      &certificate, &certificate_len) ||
        !tls13_x509_extract_p256_public_key(certificate,
                                           certificate_len,
                                           s->leaf_key))
        return fail(s, TLS13_CLIENT_ERR_CERTIFICATE);
    if (s->has_pin &&
        !tls13_consttime_eq(s->leaf_key, s->pinned_key,
                            sizeof(s->leaf_key)))
        return fail(s, TLS13_CLIENT_ERR_PIN_MISMATCH);
    tls13_transcript_update(&s->transcript, message, message_len);
    consume_plain(s, message_len);
    s->state = PTC_STATE_WAIT_CERTIFICATE_VERIFY;
    return result(PICOTLS_STEP_PENDING, 0U, TLS13_CLIENT_OK);
}

static struct picotls_result process_certificate_verify(
    struct ptc_impl *s, const struct picotls_io_ops *ops)
{
    const u8 *message, *body;
    u32 message_len, body_len;
    i32 next = next_handshake(s, TLS13_HS_CERTIFICATE_VERIFY,
                              &message, &message_len, &body, &body_len);
    if (next < 0) return fail(s, TLS13_CLIENT_ERR_MALFORMED);
    if (next == 0) return ingest_protected_handshake(s, ops);
    u16 scheme;
    const u8 *signature;
    u32 signature_len;
    u8 hash[32];
    tls13_transcript_snapshot(&s->transcript, hash);
    if (!tls13_parse_certificate_verify(body, body_len, &scheme,
                                        &signature, &signature_len) ||
        scheme != TLS13_SIGALG_ECDSA_SECP256R1_SHA256 ||
        !tls13_verify_server_certificate_signature(
            s->leaf_key, hash, signature, signature_len))
        return fail(s, TLS13_CLIENT_ERR_CERTIFICATE);
    tls13_transcript_update(&s->transcript, message, message_len);
    consume_plain(s, message_len);
    s->state = PTC_STATE_WAIT_SERVER_FINISHED;
    return result(PICOTLS_STEP_PENDING, 0U, TLS13_CLIENT_OK);
}

static struct picotls_result process_server_finished(
    struct ptc_impl *s, const struct picotls_io_ops *ops)
{
    const u8 *message, *body;
    u32 message_len, body_len;
    i32 next = next_handshake(s, TLS13_HS_FINISHED,
                              &message, &message_len, &body, &body_len);
    if (next < 0) return fail(s, TLS13_CLIENT_ERR_MALFORMED);
    if (next == 0) return ingest_protected_handshake(s, ops);

    u8 hash_before_finished[32];
    u8 received[32], expected[32];
    tls13_transcript_snapshot(&s->transcript, hash_before_finished);
    if (!tls13_parse_finished(body, body_len, received) ||
        !tls13_compute_finished(s->secrets.server_hs_traffic,
                                hash_before_finished, expected) ||
        !tls13_consttime_eq(received, expected, sizeof(received)))
        return fail(s, TLS13_CLIENT_ERR_FINISHED);
    tls13_transcript_update(&s->transcript, message, message_len);
    consume_plain(s, message_len);

    u8 hash_through_server_finished[32];
    u8 client_verify[32];
    u8 client_finished[64];
    tls13_transcript_snapshot(&s->transcript,
                              hash_through_server_finished);
    if (!tls13_derive_application_secrets(
            &s->secrets, hash_through_server_finished) ||
        !tls13_compute_finished(s->secrets.client_hs_traffic,
                                hash_through_server_finished,
                                client_verify))
        return fail(s, TLS13_CLIENT_ERR_KEY);
    u32 finished_len = tls13_build_finished(
        client_verify, client_finished, sizeof(client_finished));
    if (finished_len == 0U ||
        !queue_protected(s, TLS13_CONTENT_HANDSHAKE,
                         client_finished, finished_len))
        return fail(s, TLS13_CLIENT_ERR_RECORD);
    tls13_transcript_update(&s->transcript, client_finished,
                            finished_len);
    s->state = PTC_STATE_INSTALL_APPLICATION_KEYS;
    return result(PICOTLS_STEP_PENDING, 0U, TLS13_CLIENT_OK);
}

void tls13_client_init(tls13_client_t *client)
{
    if (client) simd_zero(client, sizeof(*client));
}

bool tls13_client_start(tls13_client_t *client,
                        const struct tls13_client_config *config)
{
    if (!client || !config ||
        config->server_name_len > TLS13_CLIENT_SERVER_NAME_MAX ||
        (config->server_name_len && !config->server_name))
        return false;
    if (config->pinned_p256_public_key &&
        p256_pubkey_validate(config->pinned_p256_public_key) != 0)
        return false;
    simd_zero(client, sizeof(*client));
    struct ptc_impl *s = impl(client);
    if (config->server_name_len) {
        simd_memcpy(s->server_name, config->server_name,
                    config->server_name_len);
        s->server_name_len = config->server_name_len;
    }
    if (config->pinned_p256_public_key) {
        simd_memcpy(s->pinned_key, config->pinned_p256_public_key,
                    sizeof(s->pinned_key));
        s->has_pin = true;
    }
    s->state = PTC_STATE_SEND_CLIENT_HELLO;
    s->last_error = TLS13_CLIENT_OK;
    s->peer_record_size_limit = TLS13_MAX_INNER_PLAINTEXT;
    return true;
}

struct picotls_result tls13_client_handshake_step(
    tls13_client_t *client, const struct picotls_io_ops *ops)
{
    if (!client || !handshake_io_valid(ops))
        return result(PICOTLS_STEP_ERROR, 0U,
                      TLS13_CLIENT_ERR_MALFORMED);
    struct ptc_impl *s = impl(client);
    if (s->state == PTC_STATE_ESTABLISHED)
        return result(PICOTLS_STEP_DONE, 0U, TLS13_CLIENT_OK);
    if (s->state == PTC_STATE_ERROR ||
        s->state == PTC_STATE_CANCELLED)
        return result(PICOTLS_STEP_ERROR, 0U, s->last_error);

    if (s->tx_len != 0U) {
        u32 progress;
        i32 flushed = flush_output(s, ops, &progress);
        if (flushed < 0) return fail(s, TLS13_CLIENT_ERR_IO);
        return result(PICOTLS_STEP_PENDING, progress,
                      TLS13_CLIENT_OK);
    }

    switch (s->state) {
    case PTC_STATE_SEND_CLIENT_HELLO:
        return queue_client_hello(s, ops);
    case PTC_STATE_WAIT_SERVER_HELLO:
        return process_server_hello(s, ops);
    case PTC_STATE_WAIT_ENCRYPTED_EXTENSIONS:
        return process_encrypted_extensions(s, ops);
    case PTC_STATE_WAIT_CERTIFICATE:
        return process_certificate(s, ops);
    case PTC_STATE_WAIT_CERTIFICATE_VERIFY:
        return process_certificate_verify(s, ops);
    case PTC_STATE_WAIT_SERVER_FINISHED:
        return process_server_finished(s, ops);
    case PTC_STATE_INSTALL_APPLICATION_KEYS: {
        u8 client_key[16], client_iv[12];
        u8 server_key[16], server_iv[12];
        if (!tls13_derive_traffic_keys(s->secrets.client_ap_traffic,
                                       client_key, client_iv) ||
            !tls13_derive_traffic_keys(s->secrets.server_ap_traffic,
                                       server_key, server_iv))
            return fail(s, TLS13_CLIENT_ERR_KEY);
        tls13_record_dir_init(&s->tx, client_key, client_iv);
        tls13_record_dir_init(&s->rx, server_key, server_iv);
        secure_zero(&s->secrets, sizeof(s->secrets));
        s->state = PTC_STATE_ESTABLISHED;
        s->last_error = TLS13_CLIENT_OK;
        return result(PICOTLS_STEP_DONE, 0U, TLS13_CLIENT_OK);
    }
    default:
        return fail(s, TLS13_CLIENT_ERR_MALFORMED);
    }
}

struct picotls_result tls13_client_write_step(
    tls13_client_t *client, const struct picotls_io_ops *ops,
    const void *data, pts_u32 len)
{
    if (!client || !io_valid(ops))
        return result(PICOTLS_STEP_ERROR, 0U, TLS13_CLIENT_ERR_IO);
    struct ptc_impl *s = impl(client);
    if (s->state != PTC_STATE_ESTABLISHED)
        return result(PICOTLS_STEP_ERROR, 0U,
                      s->last_error ? s->last_error :
                                      TLS13_CLIENT_ERR_BUSY);
    if (s->tx_len != 0U) {
        if (data || len != 0U)
            return result(PICOTLS_STEP_ERROR, 0U,
                          TLS13_CLIENT_ERR_BUSY);
        u32 progress;
        i32 flushed = flush_output(s, ops, &progress);
        if (flushed < 0) return fail(s, TLS13_CLIENT_ERR_IO);
        if (flushed == 0)
            return result(PICOTLS_STEP_PENDING, progress,
                          TLS13_CLIENT_OK);
        u32 complete = s->tx_app_len;
        s->tx_app_len = 0U;
        return result(PICOTLS_STEP_DONE, complete, TLS13_CLIENT_OK);
    }
    if (!data || len == 0U ||
        len > TLS13_MAX_INNER_PLAINTEXT - 1U)
        return result(PICOTLS_STEP_ERROR, 0U,
                      TLS13_CLIENT_ERR_RECORD);
    if (!queue_protected(s, TLS13_CONTENT_APPLICATION_DATA,
                         (const u8 *)data, len))
        return fail(s, TLS13_CLIENT_ERR_RECORD);
    s->tx_app_len = len;
    u32 progress;
    i32 flushed = flush_output(s, ops, &progress);
    if (flushed < 0) return fail(s, TLS13_CLIENT_ERR_IO);
    if (flushed == 0)
        return result(PICOTLS_STEP_PENDING, progress,
                      TLS13_CLIENT_OK);
    s->tx_app_len = 0U;
    return result(PICOTLS_STEP_DONE, len, TLS13_CLIENT_OK);
}

struct picotls_result tls13_client_close_step(
    tls13_client_t *client, const struct picotls_io_ops *ops)
{
    if (!client || !io_valid(ops))
        return result(PICOTLS_STEP_ERROR, 0U, TLS13_CLIENT_ERR_IO);
    struct ptc_impl *s = impl(client);
    if (s->state != PTC_STATE_ESTABLISHED)
        return result(PICOTLS_STEP_ERROR, 0U,
                      s->last_error ? s->last_error :
                                      TLS13_CLIENT_ERR_BUSY);
    if (!s->close_sent) {
        static const u8 close_notify[2] = { 1U, 0U };
        if (!queue_protected(s, TLS13_CONTENT_ALERT,
                             close_notify, sizeof(close_notify)))
            return fail(s, TLS13_CLIENT_ERR_RECORD);
        s->close_sent = true;
    }
    u32 progress;
    i32 flushed = flush_output(s, ops, &progress);
    if (flushed < 0)
        return fail(s, TLS13_CLIENT_ERR_IO);
    if (flushed == 0)
        return result(PICOTLS_STEP_PENDING, progress,
                      TLS13_CLIENT_OK);
    return result(PICOTLS_STEP_DONE, 0U, TLS13_CLIENT_OK);
}

static struct picotls_result deliver_plain(struct ptc_impl *s,
                                            void *out, u32 out_cap)
{
    u32 remaining = s->plain_len - s->plain_off;
    u32 take = remaining < out_cap ? remaining : out_cap;
    if (take) simd_memcpy(out, s->plain + s->plain_off, take);
    s->plain_off += take;
    if (s->plain_off == s->plain_len) {
        s->plain_off = 0U;
        s->plain_len = 0U;
    }
    return result(PICOTLS_STEP_DONE, take, TLS13_CLIENT_OK);
}

struct picotls_result tls13_client_read_step(
    tls13_client_t *client, const struct picotls_io_ops *ops,
    void *out, pts_u32 out_cap)
{
    if (!client || !io_valid(ops) || !out || out_cap == 0U)
        return result(PICOTLS_STEP_ERROR, 0U, TLS13_CLIENT_ERR_IO);
    struct ptc_impl *s = impl(client);
    if (s->state != PTC_STATE_ESTABLISHED)
        return result(PICOTLS_STEP_ERROR, 0U,
                      s->last_error ? s->last_error :
                                      TLS13_CLIENT_ERR_BUSY);
    if (s->plain_len > s->plain_off)
        return deliver_plain(s, out, out_cap);

    u32 progress = 0U;
    i32 rr = receive_record_step(s, ops, &progress);
    if (rr == -1) return fail(s, TLS13_CLIENT_ERR_IO);
    if (rr == -2) return fail(s, TLS13_CLIENT_ERR_RECORD);
    if (rr == 0)
        return result(PICOTLS_STEP_PENDING, progress,
                      TLS13_CLIENT_OK);
    if (s->rx_wire[0] == TLS13_CONTENT_CHANGE_CIPHER_SPEC) {
        if (!valid_ccs(s))
            return fail(s, TLS13_CLIENT_ERR_RECORD);
        consume_record(s);
        return result(PICOTLS_STEP_PENDING, progress,
                      TLS13_CLIENT_OK);
    }
    if (!valid_protected_header(s))
        return fail(s, TLS13_CLIENT_ERR_RECORD);

    u8 content_type = 0U;
    u32 plain_len = 0U;
    if (!tls13_record_open(&s->rx, s->rx_wire, s->rx_need,
                           &content_type, s->plain, sizeof(s->plain),
                           &plain_len))
        return fail(s, TLS13_CLIENT_ERR_DECRYPT);
    consume_record(s);
    if (content_type == TLS13_CONTENT_APPLICATION_DATA) {
        s->plain_len = plain_len;
        s->plain_off = 0U;
        return deliver_plain(s, out, out_cap);
    }
    if (content_type == TLS13_CONTENT_ALERT) {
        if (plain_len == 2U && s->plain[1] == 0U) {
            s->plain_len = 0U;
            s->peer_closed = true;
            return result(PICOTLS_STEP_DONE, 0U, TLS13_CLIENT_OK);
        }
        return fail(s, TLS13_CLIENT_ERR_RECORD);
    }
    if (content_type == TLS13_CONTENT_HANDSHAKE) {
        s->plain_len = 0U;
        return result(PICOTLS_STEP_PENDING, progress,
                      TLS13_CLIENT_OK);
    }
    return fail(s, TLS13_CLIENT_ERR_RECORD);
}

void tls13_client_cancel(tls13_client_t *client)
{
    if (!client) return;
    simd_zero(client, sizeof(*client));
    struct ptc_impl *s = impl(client);
    s->state = PTC_STATE_CANCELLED;
    s->last_error = TLS13_CLIENT_ERR_CANCELLED;
}

bool tls13_client_established(const tls13_client_t *client)
{
    return client && cimpl(client)->state == PTC_STATE_ESTABLISHED &&
           !cimpl(client)->peer_closed;
}

bool tls13_client_peer_closed(const tls13_client_t *client)
{
    return client && cimpl(client)->peer_closed;
}

pts_u32 tls13_client_last_error(const tls13_client_t *client)
{
    return client ? cimpl(client)->last_error :
                    TLS13_CLIENT_ERR_RECORD;
}
