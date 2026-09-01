#include "picotlsserver.h"
#include "tls13_client.h"
#include "p256.h"
#include <stdio.h>
#include <string.h>

#define QUEUE_CAP 32768U

struct byte_queue {
    u8 data[QUEUE_CAP];
    u32 read_off;
    u32 write_off;
};

struct endpoint {
    struct byte_queue *in;
    struct byte_queue *out;
    u32 max_read;
    u32 max_write;
    u32 write_calls;
    u32 partial_writes;
    u32 zero_writes;
    u32 force_zero_writes;
    u32 fragmented_reads;
    u32 rng;
    const u8 *certificate;
    u32 certificate_len;
    u8 identity_key[32];
};

static int failures;

#define CHECK(condition, message) do { \
    if (condition) printf("[PASS] %s\n", message); \
    else { printf("[FAIL] %s\n", message); failures++; } \
} while (0)

static i32 test_read_some(void *ctx, u8 *out, u32 cap)
{
    struct endpoint *ep = (struct endpoint *)ctx;
    u32 available = ep->in->write_off - ep->in->read_off;
    if (available == 0U) return 0;
    u32 take = available < cap ? available : cap;
    if (take > ep->max_read) take = ep->max_read;
    if (take < available || take < cap) ep->fragmented_reads++;
    memcpy(out, ep->in->data + ep->in->read_off, take);
    ep->in->read_off += take;
    if (ep->in->read_off == ep->in->write_off) {
        ep->in->read_off = 0U;
        ep->in->write_off = 0U;
    }
    return (i32)take;
}

static i32 test_write_some(void *ctx, const u8 *data, u32 len)
{
    struct endpoint *ep = (struct endpoint *)ctx;
    ep->write_calls++;
    if (ep->force_zero_writes != 0U) {
        ep->force_zero_writes--;
        ep->zero_writes++;
        return 0;
    }
    if ((ep->write_calls % 7U) == 0U) {
        ep->zero_writes++;
        return 0;
    }
    u32 space = QUEUE_CAP - ep->out->write_off;
    if (space == 0U) {
        ep->zero_writes++;
        return 0;
    }
    u32 take = len < space ? len : space;
    if (take > ep->max_write) take = ep->max_write;
    if (take < len) ep->partial_writes++;
    memcpy(ep->out->data + ep->out->write_off, data, take);
    ep->out->write_off += take;
    return (i32)take;
}

static bool test_random(void *ctx, u8 *out, u32 len)
{
    struct endpoint *ep = (struct endpoint *)ctx;
    u32 x = ep->rng;
    for (u32 i = 0; i < len; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        out[i] = (u8)x;
    }
    ep->rng = x;
    return true;
}

static bool test_identity(void *ctx, const u8 **certificate,
                          u32 *certificate_len, u8 private_key[32])
{
    struct endpoint *ep = (struct endpoint *)ctx;
    *certificate = ep->certificate;
    *certificate_len = ep->certificate_len;
    memcpy(private_key, ep->identity_key, 32U);
    return true;
}

static struct picotls_io_ops endpoint_ops(struct endpoint *ep)
{
    struct picotls_io_ops ops = {
        .ctx = ep,
        .read_some = test_read_some,
        .write_some = test_write_some,
        .random_bytes = test_random,
        .server_identity = test_identity,
    };
    return ops;
}

static u32 put_tlv(u8 *out, u8 tag, const u8 *value, u32 value_len)
{
    if (value_len >= 128U) return 0U;
    out[0] = tag;
    out[1] = (u8)value_len;
    if (value_len) memcpy(out + 2U, value, value_len);
    return value_len + 2U;
}

static u32 build_test_certificate(const u8 public_key[65],
                                  u8 *out, u32 out_cap)
{
    static const u8 version[] = { 0x02U, 0x01U, 0x02U };
    static const u8 serial[] = { 0x07U };
    static const u8 oid_ec[] =
        { 0x2a,0x86,0x48,0xce,0x3d,0x02,0x01 };
    static const u8 oid_curve[] =
        { 0x2a,0x86,0x48,0xce,0x3d,0x03,0x01,0x07 };
    u8 algorithm_body[32];
    u32 algorithm_len = 0U;
    algorithm_len += put_tlv(algorithm_body + algorithm_len, 0x06U,
                             oid_ec, sizeof(oid_ec));
    algorithm_len += put_tlv(algorithm_body + algorithm_len, 0x06U,
                             oid_curve, sizeof(oid_curve));
    u8 algorithm[40];
    u32 algorithm_tlv_len = put_tlv(algorithm, 0x30U,
                                    algorithm_body, algorithm_len);

    u8 bit_string_body[66];
    bit_string_body[0] = 0U;
    memcpy(bit_string_body + 1U, public_key, 65U);
    u8 bit_string[72];
    u32 bit_string_len = put_tlv(bit_string, 0x03U,
                                 bit_string_body,
                                 sizeof(bit_string_body));

    u8 spki_body[128];
    u32 spki_body_len = 0U;
    memcpy(spki_body + spki_body_len, algorithm, algorithm_tlv_len);
    spki_body_len += algorithm_tlv_len;
    memcpy(spki_body + spki_body_len, bit_string, bit_string_len);
    spki_body_len += bit_string_len;
    u8 spki[128];
    u32 spki_len = put_tlv(spki, 0x30U, spki_body, spki_body_len);

    u8 tbs_body[128];
    u32 tbs_len = 0U;
    tbs_len += put_tlv(tbs_body + tbs_len, 0xa0U,
                       version, sizeof(version));
    tbs_len += put_tlv(tbs_body + tbs_len, 0x02U,
                       serial, sizeof(serial));
    for (u32 i = 0; i < 4U; i++)
        tbs_len += put_tlv(tbs_body + tbs_len, 0x30U, NULL, 0U);
    memcpy(tbs_body + tbs_len, spki, spki_len);
    tbs_len += spki_len;
    u8 tbs[160];
    u32 tbs_tlv_len = put_tlv(tbs, 0x30U, tbs_body, tbs_len);

    u8 certificate_body[192];
    u32 certificate_body_len = 0U;
    memcpy(certificate_body + certificate_body_len, tbs, tbs_tlv_len);
    certificate_body_len += tbs_tlv_len;
    certificate_body_len += put_tlv(
        certificate_body + certificate_body_len, 0x30U, NULL, 0U);
    static const u8 empty_signature[] = { 0U };
    certificate_body_len += put_tlv(
        certificate_body + certificate_body_len, 0x03U,
        empty_signature, sizeof(empty_signature));
    if (certificate_body_len + 2U > out_cap) return 0U;
    return put_tlv(out, 0x30U, certificate_body,
                   certificate_body_len);
}

static bool drive_handshake(tls13_client_t *client,
                            picotlsserver_t *server,
                            struct picotls_io_ops *client_ops,
                            struct picotls_io_ops *server_ops)
{
    for (u32 i = 0; i < 20000U; i++) {
        if (!tls13_client_established(client)) {
            struct picotls_result cr =
                tls13_client_handshake_step(client, client_ops);
            if (cr.status == PICOTLS_STEP_ERROR) return false;
        }
        if (!picotlsserver_established(server)) {
            struct picotls_result sr =
                picotlsserver_handshake_step(server, server_ops);
            if (sr.status == PICOTLS_STEP_ERROR) return false;
        }
        if (tls13_client_established(client) &&
            picotlsserver_established(server))
            return true;
    }
    return false;
}

static u32 count_complete_records(const struct byte_queue *queue)
{
    u32 off = queue->read_off;
    u32 count = 0U;
    while (queue->write_off - off >= 5U) {
        u32 body_len = ((u32)queue->data[off + 3U] << 8) |
                       queue->data[off + 4U];
        u32 total = 5U + body_len;
        if (total > queue->write_off - off) break;
        count++;
        off += total;
    }
    return count;
}

int main(void)
{
    u8 identity_private[32] = { 0 };
    identity_private[31] = 7U;
    u8 identity_public[65];
    CHECK(p256_derive_pubkey(identity_private, identity_public) == 0,
          "derive fixed server P-256 identity");

    u8 certificate[192];
    u32 certificate_len = build_test_certificate(
        identity_public, certificate, sizeof(certificate));
    CHECK(certificate_len != 0U,
          "build bounded P-256 test certificate");

    struct byte_queue client_to_server = { 0 };
    struct byte_queue server_to_client = { 0 };
    struct endpoint client_ep = {
        .in = &server_to_client,
        .out = &client_to_server,
        .max_read = 3U,
        .max_write = 7U,
        .rng = 0x12345678U,
    };
    struct endpoint server_ep = {
        .in = &client_to_server,
        .out = &server_to_client,
        .max_read = 2U,
        .max_write = 5U,
        .rng = 0x87654321U,
        .certificate = certificate,
        .certificate_len = certificate_len,
    };
    memcpy(server_ep.identity_key, identity_private, 32U);
    struct picotls_io_ops client_ops = endpoint_ops(&client_ep);
    struct picotls_io_ops server_ops = endpoint_ops(&server_ep);

    tls13_client_t client;
    picotlsserver_t server;
    tls13_client_init(&client);
    picotlsserver_init(&server);
    struct tls13_client_config client_config = {
        .server_name = (const u8 *)"tls-event.test",
        .server_name_len = 14U,
        .pinned_p256_public_key = identity_public,
    };
    CHECK(tls13_client_start(&client, &client_config),
          "start event-driven TLS client");
    CHECK(picotlsserver_start(&server),
          "start event-driven TLS server");

    bool client_prefill_ok = true;
    for (u32 i = 0; i < 200U; i++) {
        struct picotls_result step =
            tls13_client_handshake_step(&client, &client_ops);
        if (step.status == PICOTLS_STEP_ERROR) {
            client_prefill_ok = false;
            break;
        }
    }
    CHECK(client_prefill_ok &&
          count_complete_records(&client_to_server) == 2U,
          "ClientHello is accepted as two plaintext TLS fragments");

    bool server_prefill_ok = true;
    for (u32 i = 0; i < 2000U; i++) {
        struct picotls_result step =
            picotlsserver_handshake_step(&server, &server_ops);
        if (step.status == PICOTLS_STEP_ERROR) {
            server_prefill_ok = false;
            break;
        }
    }
    CHECK(server_prefill_ok &&
          count_complete_records(&server_to_client) >= 7U,
          "server handshake messages span multiple TLS records");

    CHECK(drive_handshake(&client, &server, &client_ops, &server_ops),
          "fragmented real TLS 1.3 client/server handshake completes");
    CHECK(client_ep.partial_writes != 0U &&
          server_ep.partial_writes != 0U,
          "handshake preserves partial-write cursors");
    CHECK(client_ep.fragmented_reads != 0U &&
          server_ep.fragmented_reads != 0U,
          "ClientHello and server flight tolerate fragmented reads");
    CHECK(client_ep.zero_writes != 0U &&
          server_ep.zero_writes != 0U,
          "handshake survives transport backpressure");

    static const u8 request[] = "event-driven request";
    client_ep.force_zero_writes = 2U;
    struct picotls_result wr = tls13_client_write_step(
        &client, &client_ops, request, sizeof(request) - 1U);
    CHECK(wr.status == PICOTLS_STEP_PENDING && wr.bytes == 0U,
          "record write reports PENDING under backpressure");
    for (u32 i = 0; i < 1000U && wr.status == PICOTLS_STEP_PENDING; i++)
        wr = tls13_client_write_step(&client, &client_ops, NULL, 0U);
    CHECK(wr.status == PICOTLS_STEP_DONE &&
          wr.bytes == sizeof(request) - 1U,
          "partial record write completes without resealing");

    u8 received[64];
    struct picotls_result rr =
        { .status = PICOTLS_STEP_PENDING, .bytes = 0U, .error = 0U };
    for (u32 i = 0; i < 1000U && rr.status == PICOTLS_STEP_PENDING; i++)
        rr = picotlsserver_read_step(&server, &server_ops,
                                     received, sizeof(received));
    CHECK(rr.status == PICOTLS_STEP_DONE &&
          rr.bytes == sizeof(request) - 1U &&
          memcmp(received, request, sizeof(request) - 1U) == 0,
          "server decrypts exactly one backpressured client record");

    static const u8 response[] = "fragmented response";
    wr = picotlsserver_write_step(&server, &server_ops,
                                  response, sizeof(response) - 1U);
    for (u32 i = 0; i < 1000U && wr.status == PICOTLS_STEP_PENDING; i++)
        wr = picotlsserver_write_step(&server, &server_ops, NULL, 0U);
    CHECK(wr.status == PICOTLS_STEP_DONE,
          "server partial application write completes");

    u8 response_out[64];
    u32 response_have = 0U;
    for (u32 i = 0; i < 1000U &&
                    response_have < sizeof(response) - 1U; i++) {
        rr = tls13_client_read_step(&client, &client_ops,
                                    response_out + response_have, 3U);
        if (rr.status == PICOTLS_STEP_ERROR) break;
        if (rr.status == PICOTLS_STEP_DONE)
            response_have += rr.bytes;
    }
    CHECK(response_have == sizeof(response) - 1U &&
          memcmp(response_out, response, response_have) == 0,
          "client retains decrypted plaintext across small read buffers");

    struct picotls_result close =
        picotlsserver_close_step(&server, &server_ops);
    for (u32 i = 0; i < 1000U && close.status == PICOTLS_STEP_PENDING; i++)
        close = picotlsserver_close_step(&server, &server_ops);
    CHECK(close.status == PICOTLS_STEP_DONE,
          "server queues and flushes TLS close_notify without blocking");

    u8 alert[2];
    struct picotls_result alert_read =
        tls13_client_read_step(&client, &client_ops, alert, sizeof(alert));
    for (u32 i = 0; i < 1000U &&
                    alert_read.status == PICOTLS_STEP_PENDING; i++)
        alert_read = tls13_client_read_step(&client, &client_ops,
                                            alert, sizeof(alert));
    CHECK(alert_read.status == PICOTLS_STEP_DONE && alert_read.bytes == 0U,
          "client consumes close_notify as an orderly TLS EOF");

    tls13_client_cancel(&client);
    picotlsserver_cancel(&server);
    CHECK(!tls13_client_established(&client) &&
          !picotlsserver_established(&server),
          "explicit cancellation clears established state");

    if (failures == 0) {
        printf("test_tls_event: ALL PASS\n");
        return 0;
    }
    printf("test_tls_event: %d FAILURE(S)\n", failures);
    return 1;
}
