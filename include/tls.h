/*
 * tls.h - Kernel TLS-style record wrapper and plaintext bridge
 *
 * Wraps a tcp_conn_t into an authenticated encrypted tls_conn_t.
 * Uses hardware-accelerated AES-GCM and SHA-256 from crypto.h.
 * The bridge API mirrors PicoWeb's plaintext HTTP handoff shape so
 * TLS termination can remain in kernel space.
 */

#pragma once
#include "types.h"
#include "tcp.h"

typedef i32 tls_conn_t;

#define TLS_MAX_CONNECTIONS 4

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
} PACKED;

struct tls_bridge_request {
    char method[8];
    char path[96];
    u32 header_bytes;
} PACKED;

#define TLS_BRIDGE_OK          1
#define TLS_BRIDGE_NEED_MORE   0
#define TLS_BRIDGE_ERROR      -1

/* Client: connect + kernel TLS-style handshake */
tls_conn_t tls_connect(tcp_conn_t tcp);

/* Server: accept + kernel TLS-style handshake */
tls_conn_t tls_accept(tcp_conn_t tcp);

/* Encrypted read/write */
i32 tls_write(tls_conn_t conn, const void *data, u32 len);
i32 tls_read(tls_conn_t conn, void *buf, u32 len);

/* Close TLS session + underlying TCP */
void tls_close(tls_conn_t conn);

/* Init TLS subsystem */
void tls_init(void);

void tls_diag_snapshot(struct tls_diag_snapshot *out);
bool tls_selftest(void);

/* PicoWeb-style bridge: parse one plaintext HTTP request in-place.
 * Returns 1 complete, 0 need more, -1 parse error. */
i32 tls_bridge_parse_request(const u8 *plain, u32 len, struct tls_bridge_request *out);
