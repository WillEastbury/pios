#pragma once

#if defined(__STDC_HOSTED__) && __STDC_HOSTED__
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef uint8_t pts_u8;
typedef uint32_t pts_u32;
typedef int32_t pts_i32;
#else
#include "types.h"
typedef u8 pts_u8;
typedef u32 pts_u32;
typedef i32 pts_i32;
#endif

#define PICOTLSSERVER_CONTEXT_BYTES 32768U

typedef union picotlsserver {
    unsigned long long align;
    pts_u8 opaque[PICOTLSSERVER_CONTEXT_BYTES];
} picotlsserver_t;

enum picotlsserver_error {
    PICOTLSSERVER_OK = 0,
    PICOTLSSERVER_ERR_IO = 2,
    PICOTLSSERVER_ERR_MALFORMED = 3,
    PICOTLSSERVER_ERR_KEY = 4,
    PICOTLSSERVER_ERR_DECRYPT = 5,
    PICOTLSSERVER_ERR_RECORD = 6,
    PICOTLSSERVER_ERR_UNSUPPORTED = 7,
    PICOTLSSERVER_ERR_CERTIFICATE = 8,
    PICOTLSSERVER_ERR_FINISHED = 9
};

struct picotlsserver_ops {
    void *ctx;
    bool (*read_exact)(void *ctx, pts_u8 *out, pts_u32 len);
    bool (*write_all)(void *ctx, const pts_u8 *data, pts_u32 len);
    bool (*random_bytes)(void *ctx, pts_u8 *out, pts_u32 len);
    bool (*identity)(void *ctx, const pts_u8 **cert_der, pts_u32 *cert_len,
                     pts_u8 p256_private_scalar[32]);
};

void picotlsserver_init(picotlsserver_t *server);
bool picotlsserver_accept(picotlsserver_t *server,
                          const struct picotlsserver_ops *ops);
pts_i32 picotlsserver_write(picotlsserver_t *server,
                            const struct picotlsserver_ops *ops,
                            const void *data, pts_u32 len);
pts_i32 picotlsserver_read(picotlsserver_t *server,
                           const struct picotlsserver_ops *ops,
                           void *out, pts_u32 out_cap);
bool picotlsserver_established(const picotlsserver_t *server);
pts_u32 picotlsserver_last_error(const picotlsserver_t *server);

