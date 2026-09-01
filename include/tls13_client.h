#pragma once

#include "picotlsserver.h"

#define TLS13_CLIENT_CONTEXT_BYTES 32768U
#define TLS13_CLIENT_SERVER_NAME_MAX 127U

typedef union tls13_client {
    unsigned long long align;
    pts_u8 opaque[TLS13_CLIENT_CONTEXT_BYTES];
} tls13_client_t;

enum tls13_client_error {
    TLS13_CLIENT_OK = 0,
    TLS13_CLIENT_ERR_IO = 2,
    TLS13_CLIENT_ERR_MALFORMED = 3,
    TLS13_CLIENT_ERR_KEY = 4,
    TLS13_CLIENT_ERR_DECRYPT = 5,
    TLS13_CLIENT_ERR_RECORD = 6,
    TLS13_CLIENT_ERR_UNSUPPORTED = 7,
    TLS13_CLIENT_ERR_CERTIFICATE = 8,
    TLS13_CLIENT_ERR_FINISHED = 9,
    TLS13_CLIENT_ERR_BUSY = 10,
    TLS13_CLIENT_ERR_CANCELLED = 11,
    TLS13_CLIENT_ERR_PIN_MISMATCH = 12
};

struct tls13_client_config {
    /* Optional SNI. This engine does not provide a CA root store. */
    const pts_u8 *server_name;
    pts_u32 server_name_len;
    /* Optional trust pin; CertificateVerify is always checked. */
    const pts_u8 *pinned_p256_public_key;
};

void tls13_client_init(tls13_client_t *client);
bool tls13_client_start(tls13_client_t *client,
                        const struct tls13_client_config *config);
struct picotls_result tls13_client_handshake_step(
    tls13_client_t *client, const struct picotls_io_ops *ops);
struct picotls_result tls13_client_write_step(
    tls13_client_t *client, const struct picotls_io_ops *ops,
    const void *data, pts_u32 len);
struct picotls_result tls13_client_close_step(
    tls13_client_t *client, const struct picotls_io_ops *ops);
struct picotls_result tls13_client_read_step(
    tls13_client_t *client, const struct picotls_io_ops *ops,
    void *out, pts_u32 out_cap);
void tls13_client_cancel(tls13_client_t *client);
bool tls13_client_established(const tls13_client_t *client);
bool tls13_client_peer_closed(const tls13_client_t *client);
pts_u32 tls13_client_last_error(const tls13_client_t *client);
