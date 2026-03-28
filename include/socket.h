/*
 * socket.h - BSD-like socket API for user cores
 *
 * Runs on user cores (2-3), talks to Core 0 (network) via FIFO.
 * Supports TCP (SOCK_STREAM) and UDP (SOCK_DGRAM).
 *
 * All blocking calls use wfe() while waiting for FIFO responses.
 */

#pragma once
#include "types.h"

/* Address family */
#define AF_INET         2

/* Socket types */
#define SOCK_STREAM     1   /* TCP */
#define SOCK_DGRAM      2   /* UDP */

/* Max sockets per core */
#define SOCKET_MAX      16

/* Error codes (negative return values) */
#define SOCK_OK         0
#define SOCK_ERR        (-1)
#define SOCK_EAGAIN     (-11)
#define SOCK_ECONNREF   (-111)
#define SOCK_ETIMEDOUT  (-110)
#define SOCK_EADDRINUSE (-98)
#define SOCK_ENOTCONN   (-107)
#define SOCK_ENOMEM     (-12)

/* Socket address */
struct sockaddr_in {
    u32 ip;
    u16 port;
};

/* FIFO message types for socket operations */
#define MSG_SOCK_BIND       20
#define MSG_SOCK_CONNECT    21
#define MSG_SOCK_ACCEPT     22
#define MSG_SOCK_SEND       23
#define MSG_SOCK_RECV       24
#define MSG_SOCK_CLOSE      25
#define MSG_SOCK_STATUS     26
#define MSG_SOCK_LISTEN     27
#define MSG_SOCK_RESULT     28

/* ---- Socket API (called from user cores) ---- */

/* Create a socket. Returns fd >= 0 or SOCK_ERR. */
i32 sock_socket(u32 type);

/* Bind to a local address/port. */
i32 sock_bind(i32 fd, const struct sockaddr_in *addr);

/* Connect to a remote address (TCP: 3-way handshake, UDP: set default dest). */
i32 sock_connect(i32 fd, const struct sockaddr_in *addr);

/* Listen for incoming connections (TCP only). */
i32 sock_listen(i32 fd, u32 backlog);

/* Accept an incoming connection. Returns new fd or blocks. */
i32 sock_accept(i32 fd, struct sockaddr_in *client_addr);

/* Send data. Returns bytes sent or error. */
i32 sock_send(i32 fd, const void *data, u32 len);

/* Receive data. Returns bytes received or error. */
i32 sock_recv(i32 fd, void *buf, u32 len);

/* Send UDP to specific address. */
i32 sock_sendto(i32 fd, const void *data, u32 len, const struct sockaddr_in *dest);

/* Receive UDP with sender info. */
i32 sock_recvfrom(i32 fd, void *buf, u32 len, struct sockaddr_in *src);

/* Close a socket. */
i32 sock_close(i32 fd);

/* Non-blocking variants */
i32 sock_send_nb(i32 fd, const void *data, u32 len);
i32 sock_recv_nb(i32 fd, void *buf, u32 len);

/* Process socket FIFO messages on Core 0 (called from net_poll) */
void socket_handle_fifo(u32 from_core);

/* Init socket subsystem on Core 0 */
void socket_init(void);
