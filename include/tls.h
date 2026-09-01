/*
 * tls.h - Fixed-table, event-driven RFC 8446 TLS 1.3 client/server API.
 *
 * The caller owns scheduling. Every step consumes only currently available
 * TCP bytes or queues currently writable bytes, then returns. No TLS API
 * polls the network, sleeps, allocates, or waits for buffer space.
 */

#pragma once
#include "types.h"
#include "tcp.h"

typedef i32 tls_conn_t;

#define TLS_MAX_CONNECTIONS 4U
#define TLS_CONN_INVALID ((tls_conn_t)-1)
#define TLS_DEFAULT_HANDSHAKE_TIMEOUT_MS 5000ULL
#define TLS_CLIENT_SERVER_NAME_MAX 127U

enum tls_step_status {
    TLS_STEP_ERROR = -1,
    TLS_STEP_PENDING = 0,
    TLS_STEP_DONE = 1
};

enum tls_connection_state {
    TLS_STATE_FREE = 0,
    TLS_STATE_HANDSHAKE = 1,
    TLS_STATE_ESTABLISHED = 2,
    TLS_STATE_ERROR = 3,
    TLS_STATE_CANCELLED = 4,
    TLS_STATE_CLOSED = 5
};

enum tls_error {
    TLS_ERR_NONE = 0,
    TLS_ERR_ALLOC = 1,
    TLS_ERR_IO = 2,
    TLS_ERR_MALFORMED = 3,
    TLS_ERR_KEY = 4,
    TLS_ERR_DECRYPT = 5,
    TLS_ERR_RECORD = 6,
    TLS_ERR_UNSUPPORTED = 7,
    TLS_ERR_CERTIFICATE = 8,
    TLS_ERR_FINISHED = 9,
    TLS_ERR_BUSY = 10,
    TLS_ERR_CANCELLED = 11,
    TLS_ERR_PIN_MISMATCH = 12,
    TLS_ERR_TIMEOUT = 13,
    TLS_ERR_STALE_HANDLE = 14
};

struct tls_result {
    i32 status;
    u32 bytes;
    u32 error;
};

struct tls_client_start_config {
    /* SNI only; PIOS does not currently carry a CA root store. */
    const u8 *server_name;
    u32 server_name_len;
    /*
     * Optional SEC1 uncompressed P-256 public-key pin (65 bytes). The client
     * always verifies CertificateVerify against the leaf key. Without a pin
     * this proves possession but does not establish CA/hostname trust.
     */
    const u8 *pinned_p256_public_key;
    /* Absolute caller clock deadline passed back to tls_handshake_step(). */
    u64 deadline_ms;
};

struct tls_diag_snapshot {
    u32 active;
    u32 established;
    u64 connect_attempts;
    u64 accept_attempts;
    u64 handshakes_ok;
    u64 handshake_failures;
    u64 records_tx;
    u64 records_rx;
    u64 decrypt_failures;
    u64 closes;
    u64 bridge_parse_ok;
    u64 bridge_parse_need_more;
    u64 bridge_parse_error;
    u32 selftests;
    u32 selftest_failures;
    u32 last_error;
    u64 pending_steps;
    u64 timeouts;
    u64 cancels;
    u64 backpressure;
    u64 stale_handles;
} PACKED;

struct tls_bridge_request {
    char method[8];
    char path[96];
    u32 header_bytes;
} PACKED;

#define TLS_BRIDGE_OK          1
#define TLS_BRIDGE_NEED_MORE   0
#define TLS_BRIDGE_ERROR      -1

/* Start over an already-established TCP connection. No transport I/O occurs. */
tls_conn_t tls_client_start(tcp_conn_t tcp,
                            const struct tls_client_start_config *config);
tls_conn_t tls_server_start(tcp_conn_t tcp, u64 deadline_ms);

/* Drive one bounded handshake step. now_ms uses the caller's monotonic clock. */
struct tls_result tls_handshake_step(tls_conn_t conn, u64 now_ms);

/*
 * write_step seals/copies data into persistent TLS state before returning.
 * If it returns PENDING, continue with data=NULL,len=0 until DONE/ERROR.
 * Starting a second write while one is pending returns TLS_ERR_BUSY.
 */
struct tls_result tls_write_step(tls_conn_t conn,
                                 const void *data, u32 len);
struct tls_result tls_close_step(tls_conn_t conn);

/* Reads/decrypts at most one currently available record per call. */
struct tls_result tls_read_step(tls_conn_t conn, void *buf, u32 len);

bool tls_established(tls_conn_t conn);
bool tls_peer_closed(tls_conn_t conn);
u32 tls_state(tls_conn_t conn);
u32 tls_last_error(tls_conn_t conn);
void tls_cancel(tls_conn_t conn, bool close_tcp);

/*
 * Compile-compatibility shims for callers being migrated by the runtime pass.
 * They are non-blocking: start returns a handle immediately; read/write return
 * 0 for PENDING, bytes for DONE, and -1 for ERROR.
 */
tls_conn_t tls_connect(tcp_conn_t tcp);
tls_conn_t tls_accept(tcp_conn_t tcp);
i32 tls_write(tls_conn_t conn, const void *data, u32 len);
i32 tls_read(tls_conn_t conn, void *buf, u32 len);
void tls_close(tls_conn_t conn);

void tls_init(void);
void tls_diag_snapshot(struct tls_diag_snapshot *out);
bool tls_selftest(void);

i32 tls_bridge_parse_request(const u8 *plain, u32 len,
                             struct tls_bridge_request *out);
