#include "picotlsserver.h"
#include "tls13_handshake.h"
#include "tls13_keysched.h"
#include "tls13_record.h"
#include "p256.h"
#include "simd.h"

#define PTS_WIRE_MAX \
    (TLS13_RECORD_HDR_LEN + TLS13_MAX_INNER_PLAINTEXT + TLS13_TAG_LEN)
#define PTS_TX_WIRE_MAX (PTS_WIRE_MAX + 32U)
#define PTS_HANDSHAKE_MAX TLS13_MAX_INNER_PLAINTEXT
#define PTS_HANDSHAKE_FRAGMENT 96U

enum pts_state {
    PTS_STATE_IDLE = 0,
    PTS_STATE_WAIT_CLIENT_HELLO,
    PTS_STATE_SEND_ENCRYPTED_EXTENSIONS,
    PTS_STATE_SEND_CERTIFICATE,
    PTS_STATE_SEND_CERTIFICATE_VERIFY,
    PTS_STATE_SEND_FINISHED,
    PTS_STATE_WAIT_CLIENT_FINISHED,
    PTS_STATE_ESTABLISHED,
    PTS_STATE_ERROR,
    PTS_STATE_CANCELLED
};

struct pts_impl {
    struct tls13_record_dir tx;
    struct tls13_record_dir rx;
    struct tls13_transcript transcript;
    struct tls13_secrets secrets;

    const u8 *certificate;
    u32 certificate_len;
    u8 signing_key[32];
    u8 server_finished_hash[32];

    u8 tx_wire[PTS_TX_WIRE_MAX];
    u32 tx_len;
    u32 tx_off;
    u32 tx_app_len;

    u8 rx_wire[PTS_WIRE_MAX];
    u32 rx_have;
    u32 rx_need;

    u8 plain[PTS_HANDSHAKE_MAX];
    u32 plain_len;
    u32 plain_off;

    u32 state;
    u32 last_error;
    u32 compat_ccs;
    bool close_sent;
    bool peer_closed;
    u16 peer_record_size_limit;
    bool peer_offered_record_size_limit;
};

_Static_assert(sizeof(struct pts_impl) <= PICOTLSSERVER_CONTEXT_BYTES,
               "PICOTLSSERVER_CONTEXT_BYTES is too small");

static struct pts_impl *impl(picotlsserver_t *server)
{
    return (struct pts_impl *)(void *)server->opaque;
}

static const struct pts_impl *cimpl(const picotlsserver_t *server)
{
    return (const struct pts_impl *)(const void *)server->opaque;
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

static struct picotls_result fail(struct pts_impl *s, u32 error)
{
    s->last_error = error;
    s->state = PTS_STATE_ERROR;
    secure_zero(s->signing_key, sizeof(s->signing_key));
    secure_zero(&s->secrets, sizeof(s->secrets));
    return result(PICOTLS_STEP_ERROR, 0U, error);
}

static bool io_valid(const struct picotls_io_ops *ops)
{
    return ops && ops->read_some && ops->write_some;
}

static bool handshake_io_valid(const struct picotls_io_ops *ops)
{
    return io_valid(ops) && ops->random_bytes && ops->server_identity;
}

static i32 flush_output(struct pts_impl *s,
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

static bool queue_plain_handshake(struct pts_impl *s,
                                  const u8 *message, u32 message_len)
{
    if (!message || message_len == 0U || s->tx_len != 0U ||
        message_len > sizeof(s->tx_wire) - TLS13_RECORD_HDR_LEN)
        return false;
    u32 first_len = message_len;
    if (message_len > PTS_HANDSHAKE_FRAGMENT &&
        message_len + (2U * TLS13_RECORD_HDR_LEN) <= sizeof(s->tx_wire))
        first_len = PTS_HANDSHAKE_FRAGMENT;
    s->tx_wire[0] = TLS13_CONTENT_HANDSHAKE;
    s->tx_wire[1] = 0x03U;
    s->tx_wire[2] = 0x03U;
    s->tx_wire[3] = (u8)(first_len >> 8);
    s->tx_wire[4] = (u8)first_len;
    simd_memcpy(s->tx_wire + TLS13_RECORD_HDR_LEN, message, first_len);
    s->tx_len = TLS13_RECORD_HDR_LEN + first_len;
    if (first_len != message_len) {
        u32 second_len = message_len - first_len;
        u8 *second = s->tx_wire + s->tx_len;
        second[0] = TLS13_CONTENT_HANDSHAKE;
        second[1] = 0x03U;
        second[2] = 0x03U;
        second[3] = (u8)(second_len >> 8);
        second[4] = (u8)second_len;
        simd_memcpy(second + TLS13_RECORD_HDR_LEN,
                    message + first_len, second_len);
        s->tx_len += TLS13_RECORD_HDR_LEN + second_len;
    }
    s->tx_off = 0U;
    return true;
}

static bool queue_protected(struct pts_impl *s, u8 content_type,
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

static bool queue_protected_handshake(struct pts_impl *s,
                                      const u8 *message, u32 message_len)
{
    if (!message || message_len == 0U || s->tx_len != 0U)
        return false;
    if (message_len <= PTS_HANDSHAKE_FRAGMENT)
        return queue_protected(s, TLS13_CONTENT_HANDSHAKE,
                               message, message_len);
    u32 first_len = PTS_HANDSHAKE_FRAGMENT;
    u32 first_wire = tls13_record_seal(
        &s->tx, TLS13_CONTENT_HANDSHAKE, message, first_len, 0U,
        s->tx_wire, sizeof(s->tx_wire));
    if (first_wire == 0U) return false;
    u32 second_wire = tls13_record_seal(
        &s->tx, TLS13_CONTENT_HANDSHAKE,
        message + first_len, message_len - first_len, 0U,
        s->tx_wire + first_wire, sizeof(s->tx_wire) - first_wire);
    if (second_wire == 0U) return false;
    s->tx_len = first_wire + second_wire;
    s->tx_off = 0U;
    return true;
}

/* Returns 1 when a complete record is buffered, 0 for PENDING, -1 on I/O
 * failure, and -2 for malformed framing. It performs at most one transport
 * read per call. */
static i32 receive_record_step(struct pts_impl *s,
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

static void consume_record(struct pts_impl *s)
{
    s->rx_have = 0U;
    s->rx_need = 0U;
}

static bool valid_ccs(const struct pts_impl *s)
{
    return s->rx_need == TLS13_RECORD_HDR_LEN + 1U &&
           s->rx_wire[0] == TLS13_CONTENT_CHANGE_CIPHER_SPEC &&
           s->rx_wire[1] == 0x03U && s->rx_wire[2] == 0x03U &&
           s->rx_wire[5] == 1U;
}

static bool valid_protected_header(const struct pts_impl *s)
{
    return s->rx_wire[0] == TLS13_CONTENT_APPLICATION_DATA &&
           s->rx_wire[1] == 0x03U && s->rx_wire[2] == 0x03U;
}

static i32 next_handshake(const struct pts_impl *s, u8 expected,
                          const u8 **message, u32 *message_len,
                          const u8 **body, u32 *body_len)
{
    if (s->plain_len < 4U) return 0;
    u32 len = ((u32)s->plain[1] << 16) |
              ((u32)s->plain[2] << 8) | s->plain[3];
    if (len > PTS_HANDSHAKE_MAX - 4U)
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

static void consume_plain(struct pts_impl *s, u32 bytes)
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

static struct picotls_result ingest_client_hello(
    struct pts_impl *s, const struct picotls_io_ops *ops)
{
    u32 progress = 0U;
    i32 rr = receive_record_step(s, ops, &progress);
    if (rr == -1) return fail(s, PICOTLSSERVER_ERR_IO);
    if (rr == -2) return fail(s, PICOTLSSERVER_ERR_RECORD);
    if (rr == 0)
        return result(PICOTLS_STEP_PENDING, progress, PICOTLSSERVER_OK);

    if (s->rx_wire[0] != TLS13_CONTENT_HANDSHAKE ||
        (s->rx_wire[1] != 0x03U) ||
        (s->rx_wire[2] != 0x01U && s->rx_wire[2] != 0x03U))
        return fail(s, PICOTLSSERVER_ERR_MALFORMED);
    u32 fragment_len = s->rx_need - TLS13_RECORD_HDR_LEN;
    if (fragment_len > sizeof(s->plain) - s->plain_len)
        return fail(s, PICOTLSSERVER_ERR_RECORD);
    simd_memcpy(s->plain + s->plain_len,
                s->rx_wire + TLS13_RECORD_HDR_LEN, fragment_len);
    s->plain_len += fragment_len;
    consume_record(s);
    return result(PICOTLS_STEP_PENDING, progress, PICOTLSSERVER_OK);
}

static struct picotls_result process_client_hello(
    struct pts_impl *s, const struct picotls_io_ops *ops)
{
    const u8 *message;
    const u8 *body;
    u32 message_len;
    u32 body_len;
    i32 next = next_handshake(s, TLS13_HS_CLIENT_HELLO, &message,
                              &message_len, &body, &body_len);
    if (next < 0) return fail(s, PICOTLSSERVER_ERR_MALFORMED);
    if (next == 0) return ingest_client_hello(s, ops);

    struct tls13_client_hello hello;
    if (!tls13_parse_client_hello(body, body_len, &hello))
        return fail(s, PICOTLSSERVER_ERR_MALFORMED);
    s->peer_record_size_limit = hello.has_record_size_limit
        ? hello.record_size_limit : TLS13_MAX_INNER_PLAINTEXT;
    s->peer_offered_record_size_limit = hello.has_record_size_limit;
    bool cipher_ok = false;
    bool signature_ok = false;
    for (u32 i = 0; i < hello.cipher_suite_count; i++)
        cipher_ok |= hello.cipher_suites[i] ==
                     TLS13_CIPHER_AES_128_GCM_SHA256;
    for (u32 i = 0; i < hello.sig_alg_count; i++)
        signature_ok |= hello.sig_algs[i] ==
                        TLS13_SIGALG_ECDSA_SECP256R1_SHA256;
    if (!hello.offers_tls13 || !cipher_ok || !signature_ok ||
        !hello.has_p256_key_share)
        return fail(s, PICOTLSSERVER_ERR_UNSUPPORTED);

    if (!ops->server_identity(ops->ctx, &s->certificate,
                              &s->certificate_len, s->signing_key) ||
        !s->certificate || s->certificate_len == 0U)
        return fail(s, PICOTLSSERVER_ERR_CERTIFICATE);

    u8 private_key[32];
    u8 public_key[65];
    bool have_key = false;
    for (u32 attempt = 0; attempt < 8U && !have_key; attempt++) {
        if (!ops->random_bytes(ops->ctx, private_key, sizeof(private_key)))
            break;
        have_key = p256_derive_pubkey(private_key, public_key) == 0;
    }
    u8 shared[64];
    if (!have_key ||
        p256_scalar_mul_point(private_key, hello.p256_key_share,
                              shared) != 0) {
        secure_zero(private_key, sizeof(private_key));
        return fail(s, PICOTLSSERVER_ERR_KEY);
    }
    secure_zero(private_key, sizeof(private_key));

    u8 server_random[32];
    if (!ops->random_bytes(ops->ctx, server_random,
                           sizeof(server_random)))
        return fail(s, PICOTLSSERVER_ERR_KEY);

    tls13_transcript_init(&s->transcript);
    tls13_transcript_update(&s->transcript, message, message_len);
    u8 server_hello[256];
    u32 server_hello_len =
        tls13_build_server_hello(hello.legacy_session_id,
                                 hello.legacy_session_id_len,
                                 TLS13_CIPHER_AES_128_GCM_SHA256,
                                 public_key, server_random,
                                 server_hello, sizeof(server_hello));
    if (server_hello_len == 0U ||
        !queue_plain_handshake(s, server_hello, server_hello_len))
        return fail(s, PICOTLSSERVER_ERR_RECORD);
    tls13_transcript_update(&s->transcript, server_hello,
                            server_hello_len);

    u8 transcript_hash[32];
    u8 server_key[16], server_iv[12];
    u8 client_key[16], client_iv[12];
    tls13_transcript_snapshot(&s->transcript, transcript_hash);
    if (!tls13_derive_handshake_secrets(&s->secrets, shared,
                                        transcript_hash) ||
        !tls13_derive_traffic_keys(s->secrets.server_hs_traffic,
                                   server_key, server_iv) ||
        !tls13_derive_traffic_keys(s->secrets.client_hs_traffic,
                                   client_key, client_iv))
        return fail(s, PICOTLSSERVER_ERR_KEY);
    secure_zero(shared, sizeof(shared));
    tls13_record_dir_init(&s->tx, server_key, server_iv);
    tls13_record_dir_init(&s->rx, client_key, client_iv);
    consume_plain(s, message_len);
    s->state = PTS_STATE_SEND_ENCRYPTED_EXTENSIONS;
    return result(PICOTLS_STEP_PENDING, 0U, PICOTLSSERVER_OK);
}

static struct picotls_result queue_server_message(
    struct pts_impl *s, u32 next_state, const u8 *message, u32 message_len)
{
    if (!message || message_len == 0U ||
        !queue_protected_handshake(s, message, message_len))
        return fail(s, PICOTLSSERVER_ERR_RECORD);
    tls13_transcript_update(&s->transcript, message, message_len);
    s->state = next_state;
    return result(PICOTLS_STEP_PENDING, 0U, PICOTLSSERVER_OK);
}

static struct picotls_result ingest_client_finished(
    struct pts_impl *s, const struct picotls_io_ops *ops)
{
    u32 progress = 0U;
    i32 rr = receive_record_step(s, ops, &progress);
    if (rr == -1) return fail(s, PICOTLSSERVER_ERR_IO);
    if (rr == -2) return fail(s, PICOTLSSERVER_ERR_RECORD);
    if (rr == 0)
        return result(PICOTLS_STEP_PENDING, progress, PICOTLSSERVER_OK);

    if (s->rx_wire[0] == TLS13_CONTENT_CHANGE_CIPHER_SPEC) {
        if (!valid_ccs(s) || ++s->compat_ccs > 4U)
            return fail(s, PICOTLSSERVER_ERR_RECORD);
        consume_record(s);
        return result(PICOTLS_STEP_PENDING, progress, PICOTLSSERVER_OK);
    }
    if (!valid_protected_header(s))
        return fail(s, PICOTLSSERVER_ERR_RECORD);

    u8 content_type = 0U;
    u32 fragment_len = 0U;
    if (!tls13_record_open(&s->rx, s->rx_wire, s->rx_need,
                           &content_type, s->plain + s->plain_len,
                           sizeof(s->plain) - s->plain_len,
                           &fragment_len))
        return fail(s, PICOTLSSERVER_ERR_DECRYPT);
    consume_record(s);
    if (content_type != TLS13_CONTENT_HANDSHAKE)
        return fail(s, PICOTLSSERVER_ERR_RECORD);
    s->plain_len += fragment_len;
    s->plain_off = 0U;
    return result(PICOTLS_STEP_PENDING, progress, PICOTLSSERVER_OK);
}

void picotlsserver_init(picotlsserver_t *server)
{
    if (server) simd_zero(server, sizeof(*server));
}

bool picotlsserver_start(picotlsserver_t *server)
{
    if (!server) return false;
    simd_zero(server, sizeof(*server));
    struct pts_impl *s = impl(server);
    s->state = PTS_STATE_WAIT_CLIENT_HELLO;
    s->last_error = PICOTLSSERVER_OK;
    return true;
}

struct picotls_result picotlsserver_handshake_step(
    picotlsserver_t *server, const struct picotls_io_ops *ops)
{
    if (!server || !handshake_io_valid(ops))
        return result(PICOTLS_STEP_ERROR, 0U,
                      PICOTLSSERVER_ERR_MALFORMED);
    struct pts_impl *s = impl(server);
    if (s->state == PTS_STATE_ESTABLISHED)
        return result(PICOTLS_STEP_DONE, 0U, PICOTLSSERVER_OK);
    if (s->state == PTS_STATE_ERROR ||
        s->state == PTS_STATE_CANCELLED)
        return result(PICOTLS_STEP_ERROR, 0U, s->last_error);

    if (s->tx_len != 0U) {
        u32 progress;
        i32 flushed = flush_output(s, ops, &progress);
        if (flushed < 0) return fail(s, PICOTLSSERVER_ERR_IO);
        return result(PICOTLS_STEP_PENDING, progress,
                      PICOTLSSERVER_OK);
    }

    u8 message[PTS_HANDSHAKE_MAX];
    u32 message_len;
    switch (s->state) {
    case PTS_STATE_WAIT_CLIENT_HELLO:
        return process_client_hello(s, ops);
    case PTS_STATE_SEND_ENCRYPTED_EXTENSIONS:
        message_len = tls13_build_encrypted_extensions(
            message, sizeof(message));
        if (s->peer_offered_record_size_limit)
            message_len = tls13_build_encrypted_extensions_with_limit(
                s->peer_record_size_limit, message, sizeof(message));
        return queue_server_message(s, PTS_STATE_SEND_CERTIFICATE,
                                    message, message_len);
    case PTS_STATE_SEND_CERTIFICATE:
        message_len = tls13_build_certificate(
            s->certificate, s->certificate_len,
            message, sizeof(message));
        return queue_server_message(s, PTS_STATE_SEND_CERTIFICATE_VERIFY,
                                    message, message_len);
    case PTS_STATE_SEND_CERTIFICATE_VERIFY: {
        u8 hash[32];
        tls13_transcript_snapshot(&s->transcript, hash);
        message_len = tls13_build_certificate_verify_p256(
            s->signing_key, hash, message, sizeof(message));
        secure_zero(s->signing_key, sizeof(s->signing_key));
        return queue_server_message(s, PTS_STATE_SEND_FINISHED,
                                    message, message_len);
    }
    case PTS_STATE_SEND_FINISHED: {
        u8 hash[32];
        u8 verify_data[32];
        tls13_transcript_snapshot(&s->transcript, hash);
        if (!tls13_compute_finished(s->secrets.server_hs_traffic,
                                    hash, verify_data))
            return fail(s, PICOTLSSERVER_ERR_KEY);
        message_len = tls13_build_finished(verify_data, message,
                                           sizeof(message));
        struct picotls_result queued =
            queue_server_message(s, PTS_STATE_WAIT_CLIENT_FINISHED,
                                 message, message_len);
        if (queued.status == PICOTLS_STEP_ERROR)
            return queued;
        tls13_transcript_snapshot(&s->transcript,
                                  s->server_finished_hash);
        if (!tls13_derive_application_secrets(
                &s->secrets, s->server_finished_hash))
            return fail(s, PICOTLSSERVER_ERR_KEY);
        return queued;
    }
    case PTS_STATE_WAIT_CLIENT_FINISHED: {
        const u8 *finished;
        const u8 *body;
        u32 finished_len;
        u32 body_len;
        i32 next = next_handshake(s, TLS13_HS_FINISHED, &finished,
                                  &finished_len, &body, &body_len);
        if (next < 0) return fail(s, PICOTLSSERVER_ERR_RECORD);
        if (next == 0) return ingest_client_finished(s, ops);
        u8 received[32];
        u8 expected[32];
        if (!tls13_parse_finished(body, body_len, received) ||
            !tls13_compute_finished(s->secrets.client_hs_traffic,
                                    s->server_finished_hash,
                                    expected) ||
            !tls13_consttime_eq(received, expected, sizeof(received)))
            return fail(s, PICOTLSSERVER_ERR_FINISHED);

        u8 server_key[16], server_iv[12];
        u8 client_key[16], client_iv[12];
        if (!tls13_derive_traffic_keys(s->secrets.server_ap_traffic,
                                       server_key, server_iv) ||
            !tls13_derive_traffic_keys(s->secrets.client_ap_traffic,
                                       client_key, client_iv))
            return fail(s, PICOTLSSERVER_ERR_KEY);
        tls13_record_dir_init(&s->tx, server_key, server_iv);
        tls13_record_dir_init(&s->rx, client_key, client_iv);
        consume_plain(s, finished_len);
        secure_zero(&s->secrets, sizeof(s->secrets));
        s->state = PTS_STATE_ESTABLISHED;
        s->last_error = PICOTLSSERVER_OK;
        return result(PICOTLS_STEP_DONE, 0U, PICOTLSSERVER_OK);
    }
    default:
        return fail(s, PICOTLSSERVER_ERR_MALFORMED);
    }
}

struct picotls_result picotlsserver_write_step(
    picotlsserver_t *server, const struct picotls_io_ops *ops,
    const void *data, pts_u32 len)
{
    if (!server || !io_valid(ops))
        return result(PICOTLS_STEP_ERROR, 0U, PICOTLSSERVER_ERR_IO);
    struct pts_impl *s = impl(server);
    if (s->state != PTS_STATE_ESTABLISHED)
        return result(PICOTLS_STEP_ERROR, 0U,
                      s->last_error ? s->last_error :
                                      PICOTLSSERVER_ERR_BUSY);

    if (s->tx_len != 0U) {
        if (data || len != 0U)
            return result(PICOTLS_STEP_ERROR, 0U,
                          PICOTLSSERVER_ERR_BUSY);
        u32 progress;
        i32 flushed = flush_output(s, ops, &progress);
        if (flushed < 0) return fail(s, PICOTLSSERVER_ERR_IO);
        if (flushed == 0)
            return result(PICOTLS_STEP_PENDING, progress,
                          PICOTLSSERVER_OK);
        u32 complete = s->tx_app_len;
        s->tx_app_len = 0U;
        return result(PICOTLS_STEP_DONE, complete,
                      PICOTLSSERVER_OK);
    }

    if (!data || len == 0U ||
        len > TLS13_MAX_INNER_PLAINTEXT - 1U)
        return result(PICOTLS_STEP_ERROR, 0U,
                      PICOTLSSERVER_ERR_RECORD);
    if (!queue_protected(s, TLS13_CONTENT_APPLICATION_DATA,
                         (const u8 *)data, len))
        return fail(s, PICOTLSSERVER_ERR_RECORD);
    s->tx_app_len = len;
    u32 progress;
    i32 flushed = flush_output(s, ops, &progress);
    if (flushed < 0) return fail(s, PICOTLSSERVER_ERR_IO);
    if (flushed == 0)
        return result(PICOTLS_STEP_PENDING, progress,
                      PICOTLSSERVER_OK);
    s->tx_app_len = 0U;
    return result(PICOTLS_STEP_DONE, len, PICOTLSSERVER_OK);
}

struct picotls_result picotlsserver_close_step(
    picotlsserver_t *server, const struct picotls_io_ops *ops)
{
    if (!server || !io_valid(ops))
        return result(PICOTLS_STEP_ERROR, 0U, PICOTLSSERVER_ERR_IO);
    struct pts_impl *s = impl(server);
    if (s->state != PTS_STATE_ESTABLISHED)
        return result(PICOTLS_STEP_ERROR, 0U,
                      s->last_error ? s->last_error :
                                      PICOTLSSERVER_ERR_BUSY);
    if (!s->close_sent) {
        static const u8 close_notify[2] = { 1U, 0U };
        if (!queue_protected(s, TLS13_CONTENT_ALERT,
                             close_notify, sizeof(close_notify)))
            return fail(s, PICOTLSSERVER_ERR_RECORD);
        s->close_sent = true;
    }
    u32 progress;
    i32 flushed = flush_output(s, ops, &progress);
    if (flushed < 0)
        return fail(s, PICOTLSSERVER_ERR_IO);
    if (flushed == 0)
        return result(PICOTLS_STEP_PENDING, progress,
                      PICOTLSSERVER_OK);
    return result(PICOTLS_STEP_DONE, 0U, PICOTLSSERVER_OK);
}

static struct picotls_result deliver_plain(struct pts_impl *s,
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
    return result(PICOTLS_STEP_DONE, take, PICOTLSSERVER_OK);
}

struct picotls_result picotlsserver_read_step(
    picotlsserver_t *server, const struct picotls_io_ops *ops,
    void *out, pts_u32 out_cap)
{
    if (!server || !io_valid(ops) || !out || out_cap == 0U)
        return result(PICOTLS_STEP_ERROR, 0U, PICOTLSSERVER_ERR_IO);
    struct pts_impl *s = impl(server);
    if (s->state != PTS_STATE_ESTABLISHED)
        return result(PICOTLS_STEP_ERROR, 0U,
                      s->last_error ? s->last_error :
                                      PICOTLSSERVER_ERR_BUSY);
    if (s->plain_len > s->plain_off)
        return deliver_plain(s, out, out_cap);

    u32 progress = 0U;
    i32 rr = receive_record_step(s, ops, &progress);
    if (rr == -1) return fail(s, PICOTLSSERVER_ERR_IO);
    if (rr == -2) return fail(s, PICOTLSSERVER_ERR_RECORD);
    if (rr == 0)
        return result(PICOTLS_STEP_PENDING, progress,
                      PICOTLSSERVER_OK);
    if (s->rx_wire[0] == TLS13_CONTENT_CHANGE_CIPHER_SPEC) {
        if (!valid_ccs(s))
            return fail(s, PICOTLSSERVER_ERR_RECORD);
        consume_record(s);
        return result(PICOTLS_STEP_PENDING, progress,
                      PICOTLSSERVER_OK);
    }
    if (!valid_protected_header(s))
        return fail(s, PICOTLSSERVER_ERR_RECORD);

    u8 content_type = 0U;
    u32 plain_len = 0U;
    if (!tls13_record_open(&s->rx, s->rx_wire, s->rx_need,
                           &content_type, s->plain, sizeof(s->plain),
                           &plain_len))
        return fail(s, PICOTLSSERVER_ERR_DECRYPT);
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
            return result(PICOTLS_STEP_DONE, 0U,
                          PICOTLSSERVER_OK);
        }
        return fail(s, PICOTLSSERVER_ERR_RECORD);
    }
    if (content_type == TLS13_CONTENT_HANDSHAKE) {
        s->plain_len = 0U;
        return result(PICOTLS_STEP_PENDING, progress,
                      PICOTLSSERVER_OK);
    }
    return fail(s, PICOTLSSERVER_ERR_RECORD);
}

void picotlsserver_cancel(picotlsserver_t *server)
{
    if (!server) return;
    simd_zero(server, sizeof(*server));
    struct pts_impl *s = impl(server);
    s->state = PTS_STATE_CANCELLED;
    s->last_error = PICOTLSSERVER_ERR_CANCELLED;
}

bool picotlsserver_established(const picotlsserver_t *server)
{
    return server && cimpl(server)->state == PTS_STATE_ESTABLISHED &&
           !cimpl(server)->peer_closed;
}

bool picotlsserver_peer_closed(const picotlsserver_t *server)
{
    return server && cimpl(server)->peer_closed;
}

pts_u32 picotlsserver_last_error(const picotlsserver_t *server)
{
    return server ? cimpl(server)->last_error :
                    PICOTLSSERVER_ERR_RECORD;
}
