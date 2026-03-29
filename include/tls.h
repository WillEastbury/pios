/*
 * tls.h - Minimal TLS 1.3 (RFC 8446)
 *
 * Single cipher suite: TLS_AES_128_GCM_SHA256 (0x1301)
 * Wraps a tcp_conn_t into an encrypted tls_conn_t.
 * Uses hardware-accelerated AES-GCM and SHA-256 from crypto.h.
 */

#pragma once
#include "types.h"
#include "tcp.h"

typedef i32 tls_conn_t;

#define TLS_MAX_CONNECTIONS 4

/* Client: connect + TLS 1.3 handshake */
tls_conn_t tls_connect(tcp_conn_t tcp);

/* Server: accept + TLS 1.3 handshake */
tls_conn_t tls_accept(tcp_conn_t tcp);

/* Encrypted read/write */
i32 tls_write(tls_conn_t conn, const void *data, u32 len);
i32 tls_read(tls_conn_t conn, void *buf, u32 len);

/* Close TLS session + underlying TCP */
void tls_close(tls_conn_t conn);

/* Init TLS subsystem */
void tls_init(void);
