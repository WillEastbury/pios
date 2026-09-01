#pragma once

#include "types.h"
typedef u8 pts_u8;
typedef u32 pts_u32;
typedef i32 pts_i32;

#define PICOTLSSERVER_CONTEXT_BYTES 32768U

typedef union picotlsserver {
    unsigned long long align;
    pts_u8 opaque[PICOTLSSERVER_CONTEXT_BYTES];
} picotlsserver_t;

enum picotls_step_status {
    PICOTLS_STEP_ERROR = -1,
    PICOTLS_STEP_PENDING = 0,
    PICOTLS_STEP_DONE = 1
};

struct picotls_result {
    pts_i32 status;
    pts_u32 bytes;
    pts_u32 error;
};

enum picotlsserver_error {
    PICOTLSSERVER_OK = 0,
    PICOTLSSERVER_ERR_IO = 2,
    PICOTLSSERVER_ERR_MALFORMED = 3,
    PICOTLSSERVER_ERR_KEY = 4,
    PICOTLSSERVER_ERR_DECRYPT = 5,
    PICOTLSSERVER_ERR_RECORD = 6,
    PICOTLSSERVER_ERR_UNSUPPORTED = 7,
    PICOTLSSERVER_ERR_CERTIFICATE = 8,
    PICOTLSSERVER_ERR_FINISHED = 9,
    PICOTLSSERVER_ERR_BUSY = 10,
    PICOTLSSERVER_ERR_CANCELLED = 11
};

/*
 * read_some/write_some are non-blocking transport operations:
 *   >0  bytes consumed/queued now
 *    0  no progress is currently possible
 *   <0  terminal transport failure
 * They must never wait for data or buffer space.
 */
struct picotls_io_ops {
    void *ctx;
    pts_i32 (*read_some)(void *ctx, pts_u8 *out, pts_u32 cap);
    pts_i32 (*write_some)(void *ctx, const pts_u8 *data, pts_u32 len);
    bool (*random_bytes)(void *ctx, pts_u8 *out, pts_u32 len);
    bool (*server_identity)(void *ctx, const pts_u8 **cert_der,
                            pts_u32 *cert_len,
                            pts_u8 p256_private_scalar[32]);
};

void picotlsserver_init(picotlsserver_t *server);
bool picotlsserver_start(picotlsserver_t *server);
struct picotls_result picotlsserver_handshake_step(
    picotlsserver_t *server, const struct picotls_io_ops *ops);
struct picotls_result picotlsserver_write_step(
    picotlsserver_t *server, const struct picotls_io_ops *ops,
    const void *data, pts_u32 len);
struct picotls_result picotlsserver_close_step(
    picotlsserver_t *server, const struct picotls_io_ops *ops);
struct picotls_result picotlsserver_read_step(
    picotlsserver_t *server, const struct picotls_io_ops *ops,
    void *out, pts_u32 out_cap);
void picotlsserver_cancel(picotlsserver_t *server);
bool picotlsserver_established(const picotlsserver_t *server);
bool picotlsserver_peer_closed(const picotlsserver_t *server);
pts_u32 picotlsserver_last_error(const picotlsserver_t *server);
