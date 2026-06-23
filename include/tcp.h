/*
 * tcp.h - Hardened TCP stack
 *
 * Full TCP with reliable delivery, congestion control, and hardening.
 * Max 8 simultaneous connections. SYN cookies for server sockets.
 *
 * Reference: RFC 793 (TCP), RFC 5681 (Congestion Control),
 *            RFC 5961 (RST validation), RFC 6528 (ISN randomization)
 */

#pragma once
#include "types.h"

#define TCP_MAX_CONNECTIONS 128
#define TCP_BUF_SIZE        4096
#define TCP_MSS             1460
#define TCP_DEFAULT_WINDOW  TCP_BUF_SIZE

/* TCP connection states (RFC 793) */
#define TCP_CLOSED          0
#define TCP_LISTEN          1
#define TCP_SYN_SENT        2
#define TCP_SYN_RECEIVED    3
#define TCP_ESTABLISHED     4
#define TCP_FIN_WAIT_1      5
#define TCP_FIN_WAIT_2      6
#define TCP_CLOSE_WAIT      7
#define TCP_CLOSING         8
#define TCP_LAST_ACK        9
#define TCP_TIME_WAIT       10

/* Opaque connection handle */
typedef i32 tcp_conn_t;

/* Init TCP subsystem */
void tcp_init(void);

/* Process incoming TCP segment (called from net.c IP dispatch) */
void tcp_input(const u8 *frame, u32 len, u32 src_ip, u32 dst_ip,
               const u8 *payload, u32 payload_len, bool checksum_trusted);

/* Timer tick (call once per ~100ms from net_poll or timer) */
void tcp_tick(void);

/* ---- Client API ---- */

/* Open an active connection (SYN_SENT) */
tcp_conn_t tcp_connect(u32 dst_ip, u16 dst_port);

/* ---- Server API ---- */

/* Open a passive (listening) socket */
tcp_conn_t tcp_listen(u16 port);

/* Accept a pending connection on a listening socket. Returns new conn or -1. */
tcp_conn_t tcp_accept(tcp_conn_t listen_conn);

/* ---- Data Transfer ---- */

/* Write data to send buffer. Returns bytes written (may be < len). */
u32 tcp_write(tcp_conn_t conn, const void *data, u32 len);

/* Read data from receive buffer. Returns bytes read. */
u32 tcp_read(tcp_conn_t conn, void *data, u32 len);

/* ---- Connection Management ---- */

/* Initiate graceful close (FIN) */
void tcp_close(tcp_conn_t conn);

/* Immediately discard a connection control block. */
void tcp_abort(tcp_conn_t conn);

/* Drop non-listener TCBs and queued SYN-cookie accepts for a local port. */
void tcp_purge_port(u16 local_port);

/* Get connection state */
u32 tcp_state(tcp_conn_t conn);

/* Check if data is available to read */
u32 tcp_readable(tcp_conn_t conn);

/* Check how much space is in send buffer */
u32 tcp_writable(tcp_conn_t conn);

/* Bytes still queued in the send buffer (unsent + unacknowledged). 0 means all
 * written data has been transmitted AND acknowledged by the peer — safe to FIN
 * without truncating the tail. */
u32 tcp_tx_pending(tcp_conn_t conn);

/* Count non-listener active TCP control blocks. */
u32 tcp_active_count(void);

typedef struct {
    i32 conn;
    u32 state;
    u32 local_ip;
    u32 remote_ip;
    u16 local_port;
    u16 remote_port;
    u32 pending_count;
    u32 rx_used;
    u32 tx_used;
    u32 retries;
} tcp_snapshot_entry_t;

/* Snapshot TCP listeners and sessions. Returns number written. */
u32 tcp_snapshot(tcp_snapshot_entry_t *out, u32 max);

typedef struct {
    u64 in_short;
    u64 bad_checksum;
    u64 bad_header;
    u64 no_listener;
    u64 syn_seen;
    u64 synack_sent;
    u64 ack_cookie_seen;
    u64 ack_cookie_bad;
    u64 pending_queued;
    u64 pending_full;
    u64 accepted;
    u64 tx_segments;
    u64 tx_no_mac;
    u64 tx_send_fail;
    u64 active_syn_sent;
    u64 active_synack_seen;
    u64 active_established;
    u64 active_rst;
    u64 active_bad_ack;
    u64 active_timeout;
    u32 active_last_local_ip;
    u32 active_last_remote_ip;
    u16 active_last_local_port;
    u16 active_last_remote_port;
    u32 active_last_state;
    u32 active_last_retries;
} tcp_diag_t;

const tcp_diag_t *tcp_diag(void);

/* Live connection-table capacity + in-use count (network-first scaling diag). */
void tcp_table_stats(u32 *capacity, u32 *inuse, u32 *listeners);
