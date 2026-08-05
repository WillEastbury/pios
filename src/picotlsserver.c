#include "picotlsserver.h"
#include "tls13_handshake.h"
#include "tls13_keysched.h"
#include "tls13_record.h"
#include "p256.h"
#include "simd.h"

#define PTS_WIRE_MAX (TLS13_RECORD_HDR_LEN + TLS13_MAX_INNER_PLAINTEXT + TLS13_TAG_LEN)

struct pts_impl {
    struct tls13_record_dir tx;
    struct tls13_record_dir rx;
    bool established;
    u32 last_error;
};

struct pts_scratch {
    u8 signing_key[32];
    u8 ephemeral_key[32];
    u8 ephemeral_public[65];
    u8 shared_secret[64];
    struct tls13_secrets secrets;
    u8 server_hs_key[16], server_hs_iv[12];
    u8 client_hs_key[16], client_hs_iv[12];
    u8 server_ap_key[16], server_ap_iv[12];
    u8 client_ap_key[16], client_ap_iv[12];
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
    while (len--)
        *p++ = 0;
}

static bool ops_valid(const struct picotlsserver_ops *ops)
{
    return ops && ops->read_exact && ops->write_all &&
           ops->random_bytes && ops->identity;
}

static bool read_record(const struct picotlsserver_ops *ops,
                        u8 *record, u32 cap, u32 *record_len)
{
    if (cap < TLS13_RECORD_HDR_LEN ||
        !ops->read_exact(ops->ctx, record, TLS13_RECORD_HDR_LEN))
        return false;
    u32 body_len = ((u32)record[3] << 8) | record[4];
    if (body_len == 0 || body_len > cap - TLS13_RECORD_HDR_LEN)
        return false;
    if (!ops->read_exact(ops->ctx, record + TLS13_RECORD_HDR_LEN, body_len))
        return false;
    *record_len = TLS13_RECORD_HDR_LEN + body_len;
    return true;
}

static bool send_handshake(struct pts_impl *s,
                           const struct picotlsserver_ops *ops,
                           const u8 *message, u32 message_len,
                           struct tls13_transcript *transcript)
{
    u8 record[PTS_WIRE_MAX];
    u32 len = tls13_record_seal(&s->tx, TLS13_CONTENT_HANDSHAKE,
                                message, message_len, 0,
                                record, sizeof(record));
    if (len == 0 || !ops->write_all(ops->ctx, record, len))
        return false;
    tls13_transcript_update(transcript, message, message_len);
    return true;
}

void picotlsserver_init(picotlsserver_t *server)
{
    if (server)
        simd_zero(server, sizeof(*server));
}

bool picotlsserver_accept(picotlsserver_t *server,
                          const struct picotlsserver_ops *ops)
{
    if (!server || !ops_valid(ops))
        return false;
    struct pts_impl *s = impl(server);
    struct pts_scratch scratch;
    simd_zero(&scratch, sizeof(scratch));
    s->established = false;
    s->last_error = PICOTLSSERVER_OK;

    const u8 *cert_der = 0;
    u32 cert_len = 0;
    if (!ops->identity(ops->ctx, &cert_der, &cert_len, scratch.signing_key) ||
        !cert_der || cert_len == 0) {
        s->last_error = PICOTLSSERVER_ERR_CERTIFICATE;
        goto fail;
    }

    u8 record[PTS_WIRE_MAX];
    u32 record_len = 0;
    if (!read_record(ops, record, sizeof(record), &record_len)) {
        s->last_error = PICOTLSSERVER_ERR_IO;
        goto fail;
    }
    if (record[0] != TLS13_CONTENT_HANDSHAKE) {
        s->last_error = PICOTLSSERVER_ERR_MALFORMED;
        goto fail;
    }

    const u8 *client_message = record + TLS13_RECORD_HDR_LEN;
    u32 client_message_len = record_len - TLS13_RECORD_HDR_LEN;
    u8 message_type = 0;
    u32 body_len = 0;
    if (!tls13_next_handshake_header(client_message, client_message_len,
                                     &message_type, &body_len) ||
        message_type != TLS13_HS_CLIENT_HELLO ||
        body_len != client_message_len - 4U) {
        s->last_error = PICOTLSSERVER_ERR_MALFORMED;
        goto fail;
    }

    struct tls13_client_hello hello;
    if (!tls13_parse_client_hello(client_message + 4U, body_len, &hello)) {
        s->last_error = PICOTLSSERVER_ERR_RECORD;
        goto fail;
    }
    bool cipher_ok = false;
    for (u32 i = 0; i < hello.cipher_suite_count; i++)
        if (hello.cipher_suites[i] == TLS13_CIPHER_AES_128_GCM_SHA256)
            cipher_ok = true;
    bool signature_ok = false;
    for (u32 i = 0; i < hello.sig_alg_count; i++)
        if (hello.sig_algs[i] == TLS13_SIGALG_ECDSA_SECP256R1_SHA256)
            signature_ok = true;
    if (!hello.offers_tls13 || !cipher_ok ||
        !hello.has_p256_key_share || !signature_ok) {
        s->last_error = PICOTLSSERVER_ERR_UNSUPPORTED;
        goto fail;
    }

    struct tls13_transcript transcript;
    tls13_transcript_init(&transcript);
    tls13_transcript_update(&transcript, client_message, client_message_len);

    bool have_key = false;
    for (u32 attempt = 0; attempt < 8U && !have_key; attempt++) {
        if (!ops->random_bytes(ops->ctx, scratch.ephemeral_key,
                               sizeof(scratch.ephemeral_key)))
            break;
        have_key = p256_derive_pubkey(scratch.ephemeral_key,
                                     scratch.ephemeral_public) == 0;
    }
    if (!have_key ||
        p256_scalar_mul_point(scratch.ephemeral_key, hello.p256_key_share,
                              scratch.shared_secret) != 0) {
        s->last_error = PICOTLSSERVER_ERR_KEY;
        goto fail;
    }

    u8 server_random[32];
    if (!ops->random_bytes(ops->ctx, server_random, sizeof(server_random))) {
        s->last_error = PICOTLSSERVER_ERR_KEY;
        goto fail;
    }
    u8 server_hello[256];
    u32 server_hello_len =
        tls13_build_server_hello(hello.legacy_session_id,
                                 hello.legacy_session_id_len,
                                 TLS13_CIPHER_AES_128_GCM_SHA256,
                                 scratch.ephemeral_public, server_random,
                                 server_hello, sizeof(server_hello));
    if (server_hello_len == 0) {
        s->last_error = PICOTLSSERVER_ERR_RECORD;
        goto fail;
    }
    u8 server_record[300];
    server_record[0] = TLS13_CONTENT_HANDSHAKE;
    server_record[1] = 0x03;
    server_record[2] = 0x03;
    server_record[3] = (u8)(server_hello_len >> 8);
    server_record[4] = (u8)server_hello_len;
    simd_memcpy(server_record + TLS13_RECORD_HDR_LEN,
                server_hello, server_hello_len);
    if (!ops->write_all(ops->ctx, server_record,
                        TLS13_RECORD_HDR_LEN + server_hello_len)) {
        s->last_error = PICOTLSSERVER_ERR_IO;
        goto fail;
    }
    tls13_transcript_update(&transcript, server_hello, server_hello_len);

    u8 transcript_hash[32];
    tls13_transcript_snapshot(&transcript, transcript_hash);
    if (!tls13_derive_handshake_secrets(&scratch.secrets,
                                        scratch.shared_secret,
                                        transcript_hash) ||
        !tls13_derive_traffic_keys(scratch.secrets.server_hs_traffic,
                                   scratch.server_hs_key,
                                   scratch.server_hs_iv) ||
        !tls13_derive_traffic_keys(scratch.secrets.client_hs_traffic,
                                   scratch.client_hs_key,
                                   scratch.client_hs_iv)) {
        s->last_error = PICOTLSSERVER_ERR_KEY;
        goto fail;
    }
    tls13_record_dir_init(&s->tx, scratch.server_hs_key,
                          scratch.server_hs_iv);
    tls13_record_dir_init(&s->rx, scratch.client_hs_key,
                          scratch.client_hs_iv);

    u8 message[4096];
    u32 len = tls13_build_encrypted_extensions(message, sizeof(message));
    if (len == 0 || !send_handshake(s, ops, message, len, &transcript)) {
        s->last_error = PICOTLSSERVER_ERR_IO;
        goto fail;
    }
    len = tls13_build_certificate(cert_der, cert_len,
                                  message, sizeof(message));
    if (len == 0 || !send_handshake(s, ops, message, len, &transcript)) {
        s->last_error = PICOTLSSERVER_ERR_IO;
        goto fail;
    }
    tls13_transcript_snapshot(&transcript, transcript_hash);
    len = tls13_build_certificate_verify_p256(scratch.signing_key,
                                              transcript_hash,
                                              message, sizeof(message));
    if (len == 0 || !send_handshake(s, ops, message, len, &transcript)) {
        s->last_error = PICOTLSSERVER_ERR_IO;
        goto fail;
    }
    tls13_transcript_snapshot(&transcript, transcript_hash);
    u8 verify_data[32];
    if (!tls13_compute_finished(scratch.secrets.server_hs_traffic,
                                transcript_hash, verify_data)) {
        s->last_error = PICOTLSSERVER_ERR_KEY;
        goto fail;
    }
    len = tls13_build_finished(verify_data, message, sizeof(message));
    if (len == 0 || !send_handshake(s, ops, message, len, &transcript)) {
        s->last_error = PICOTLSSERVER_ERR_IO;
        goto fail;
    }

    u8 server_finished_hash[32];
    tls13_transcript_snapshot(&transcript, server_finished_hash);
    if (!tls13_derive_application_secrets(&scratch.secrets,
                                           server_finished_hash)) {
        s->last_error = PICOTLSSERVER_ERR_KEY;
        goto fail;
    }

    bool have_finished_record = false;
    for (u32 compat = 0; compat < 4U; compat++) {
        if (!read_record(ops, record, sizeof(record), &record_len)) {
            s->last_error = PICOTLSSERVER_ERR_IO;
            goto fail;
        }
        if (record[0] != TLS13_CONTENT_CHANGE_CIPHER_SPEC) {
            have_finished_record = true;
            break;
        }
        if (record_len != TLS13_RECORD_HDR_LEN + 1U ||
            record[1] != 0x03U || record[2] != 0x03U ||
            record[3] != 0U || record[4] != 1U || record[5] != 1U) {
            s->last_error = PICOTLSSERVER_ERR_RECORD;
            goto fail;
        }
    }
    if (!have_finished_record) {
        s->last_error = PICOTLSSERVER_ERR_RECORD;
        goto fail;
    }

    u8 content_type = 0;
    u8 client_finished[64];
    u32 client_finished_len = 0;
    if (!tls13_record_open(&s->rx, record, record_len, &content_type,
                           client_finished, sizeof(client_finished),
                           &client_finished_len)) {
        s->last_error = PICOTLSSERVER_ERR_DECRYPT;
        goto fail;
    }
    u8 finished_type = 0;
    u32 finished_body_len = 0;
    u8 client_verify_data[32];
    if (content_type != TLS13_CONTENT_HANDSHAKE ||
        !tls13_next_handshake_header(client_finished, client_finished_len,
                                     &finished_type, &finished_body_len) ||
        finished_type != TLS13_HS_FINISHED ||
        finished_body_len != client_finished_len - 4U ||
        !tls13_parse_finished(client_finished + 4U, finished_body_len,
                              client_verify_data)) {
        s->last_error = PICOTLSSERVER_ERR_RECORD;
        goto fail;
    }
    u8 expected_verify_data[32];
    if (!tls13_compute_finished(scratch.secrets.client_hs_traffic,
                                server_finished_hash,
                                expected_verify_data) ||
        !tls13_consttime_eq(client_verify_data,
                            expected_verify_data, 32U)) {
        s->last_error = PICOTLSSERVER_ERR_FINISHED;
        goto fail;
    }

    if (!tls13_derive_traffic_keys(scratch.secrets.server_ap_traffic,
                                   scratch.server_ap_key,
                                   scratch.server_ap_iv) ||
        !tls13_derive_traffic_keys(scratch.secrets.client_ap_traffic,
                                   scratch.client_ap_key,
                                   scratch.client_ap_iv)) {
        s->last_error = PICOTLSSERVER_ERR_KEY;
        goto fail;
    }
    tls13_record_dir_init(&s->tx, scratch.server_ap_key,
                          scratch.server_ap_iv);
    tls13_record_dir_init(&s->rx, scratch.client_ap_key,
                          scratch.client_ap_iv);
    s->established = true;
    s->last_error = PICOTLSSERVER_OK;
    secure_zero(&scratch, sizeof(scratch));
    return true;

fail:
    secure_zero(&scratch, sizeof(scratch));
    return false;
}

pts_i32 picotlsserver_write(picotlsserver_t *server,
                            const struct picotlsserver_ops *ops,
                            const void *data, pts_u32 len)
{
    if (!server || !ops_valid(ops) || !data || len == 0 ||
        len > TLS13_MAX_INNER_PLAINTEXT)
        return -1;
    struct pts_impl *s = impl(server);
    if (!s->established)
        return -1;
    u8 record[PTS_WIRE_MAX];
    u32 record_len = tls13_record_seal(&s->tx,
                                       TLS13_CONTENT_APPLICATION_DATA,
                                       (const u8 *)data, len, 0,
                                       record, sizeof(record));
    if (record_len == 0 || !ops->write_all(ops->ctx, record, record_len)) {
        s->last_error = PICOTLSSERVER_ERR_IO;
        return -1;
    }
    s->last_error = PICOTLSSERVER_OK;
    return (i32)len;
}

pts_i32 picotlsserver_read(picotlsserver_t *server,
                           const struct picotlsserver_ops *ops,
                           void *out, pts_u32 out_cap)
{
    if (!server || !ops_valid(ops) || !out || out_cap == 0)
        return -1;
    struct pts_impl *s = impl(server);
    if (!s->established)
        return -1;
    u8 record[PTS_WIRE_MAX];
    u32 record_len = 0;
    if (!read_record(ops, record, sizeof(record), &record_len)) {
        s->last_error = PICOTLSSERVER_ERR_IO;
        return -1;
    }
    u8 content_type = 0;
    u32 plain_len = 0;
    if (!tls13_record_open(&s->rx, record, record_len,
                           &content_type, out, out_cap, &plain_len)) {
        s->last_error = PICOTLSSERVER_ERR_DECRYPT;
        return -1;
    }
    if (content_type != TLS13_CONTENT_APPLICATION_DATA) {
        s->last_error = PICOTLSSERVER_OK;
        return 0;
    }
    s->last_error = PICOTLSSERVER_OK;
    return (i32)plain_len;
}

bool picotlsserver_established(const picotlsserver_t *server)
{
    return server && cimpl(server)->established;
}

pts_u32 picotlsserver_last_error(const picotlsserver_t *server)
{
    return server ? cimpl(server)->last_error :
                    PICOTLSSERVER_ERR_RECORD;
}

