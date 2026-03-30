/*
 * socket.c - BSD-like socket API
 *
 * Two parts:
 *   1. User-core side: sock_*() functions that send FIFO messages to Core 0
 *      and block (wfe) waiting for replies.
 *   2. Core 0 side: socket_handle_fifo() processes requests using the TCP/UDP
 *      stack and sends results back.
 *
 * Socket state is kept on the user core (socket table).
 * TCP connections are managed by Core 0's TCP stack; the socket layer
 * just maps fd → tcp_conn_t and shuttles data via FIFO.
 */

#include "socket.h"
#include "tcp.h"
#include "net.h"
#include "fifo.h"
#include "core.h"
#include "uart.h"
#include "simd.h"
#include "timer.h"

/* ---- Socket Descriptor ---- */

struct socket_desc {
    bool    used;
    u32     type;           /* SOCK_STREAM or SOCK_DGRAM */
    u32     state;          /* 0=new, 1=bound, 2=listening, 3=connected, 4=closed */
    struct  sockaddr_in local;
    struct  sockaddr_in remote;
    i32     tcp_conn;       /* tcp_conn_t handle (for SOCK_STREAM) */
    /* Small recv buffer for UDP */
    u8      udp_buf[512];
    u32     udp_buf_len;
    u32     udp_src_ip;
    u16     udp_src_port;
    bool    udp_pending;
};

#define SOCK_STATE_NEW      0
#define SOCK_STATE_BOUND    1
#define SOCK_STATE_LISTEN   2
#define SOCK_STATE_CONN     3
#define SOCK_STATE_CLOSED   4

/* Per-core socket tables (cores 2 and 3) */
static struct socket_desc sockets[2][SOCKET_MAX];

static struct socket_desc *get_sock(i32 fd) {
    u32 core = core_id();
    u32 ci = (core >= CORE_USER0) ? core - CORE_USER0 : 0;
    if (fd < 0 || fd >= SOCKET_MAX) return NULL;
    if (!sockets[ci][fd].used) return NULL;
    return &sockets[ci][fd];
}

/* ---- FIFO helpers ---- */

static void send_to_net(const struct fifo_msg *msg) {
    u32 core = core_id();
    fifo_push(core, CORE_NET, msg);
}

static bool recv_from_net(struct fifo_msg *msg) {
    u32 core = core_id();
    return fifo_pop(core, CORE_NET, msg);
}

static void wait_reply(struct fifo_msg *reply) {
    while (!recv_from_net(reply))
        wfe();
}

/* ---- User-core socket API ---- */

i32 sock_socket(u32 type) {
    u32 core = core_id();
    u32 ci = (core >= CORE_USER0) ? core - CORE_USER0 : 0;

    for (i32 i = 0; i < SOCKET_MAX; i++) {
        if (!sockets[ci][i].used) {
            struct socket_desc *s = &sockets[ci][i];
            simd_zero(s, sizeof(*s));
            s->used = true;
            s->type = type;
            s->state = SOCK_STATE_NEW;
            s->tcp_conn = -1;
            return i;
        }
    }
    return SOCK_ENOMEM;
}

i32 sock_bind(i32 fd, const struct sockaddr_in *addr) {
    struct socket_desc *s = get_sock(fd);
    if (!s) return SOCK_ERR;

    s->local = *addr;
    s->state = SOCK_STATE_BOUND;

    /* Notify Core 0 for UDP port registration */
    if (s->type == SOCK_DGRAM) {
        struct fifo_msg msg = {0};
        msg.type = MSG_SOCK_BIND;
        msg.param = addr->ip;
        msg.tag = addr->port;
        msg.length = fd;
        send_to_net(&msg);
    }
    return SOCK_OK;
}

i32 sock_connect(i32 fd, const struct sockaddr_in *addr) {
    struct socket_desc *s = get_sock(fd);
    if (!s) return SOCK_ERR;

    s->remote = *addr;

    if (s->type == SOCK_DGRAM) {
        s->state = SOCK_STATE_CONN;
        return SOCK_OK;
    }

    /* TCP: send connect request to Core 0 */
    struct fifo_msg msg = {0};
    msg.type = MSG_SOCK_CONNECT;
    msg.param = addr->ip;
    msg.tag = ((u64)s->local.port << 16) | addr->port;
    msg.length = fd;
    send_to_net(&msg);

    /* Wait for result */
    struct fifo_msg reply;
    wait_reply(&reply);

    if (reply.status != 0)
        return SOCK_ECONNREF;

    s->tcp_conn = (i32)reply.param;
    s->state = SOCK_STATE_CONN;
    return SOCK_OK;
}

i32 sock_listen(i32 fd, u32 backlog) {
    struct socket_desc *s = get_sock(fd);
    if (!s || s->type != SOCK_STREAM) return SOCK_ERR;
    (void)backlog;

    struct fifo_msg msg = {0};
    msg.type = MSG_SOCK_LISTEN;
    msg.param = s->local.port;
    msg.length = fd;
    send_to_net(&msg);

    struct fifo_msg reply;
    wait_reply(&reply);

    if (reply.status != 0)
        return SOCK_ERR;

    s->tcp_conn = (i32)reply.param;
    s->state = SOCK_STATE_LISTEN;
    return SOCK_OK;
}

i32 sock_accept(i32 fd, struct sockaddr_in *client_addr) {
    struct socket_desc *s = get_sock(fd);
    if (!s || s->state != SOCK_STATE_LISTEN) return SOCK_ERR;

    struct fifo_msg msg = {0};
    msg.type = MSG_SOCK_ACCEPT;
    msg.param = (u32)s->tcp_conn;
    msg.length = fd;
    send_to_net(&msg);

    struct fifo_msg reply;
    wait_reply(&reply);

    if (reply.status != 0)
        return SOCK_ERR;

    /* Allocate new socket for accepted connection */
    i32 new_fd = sock_socket(SOCK_STREAM);
    if (new_fd < 0) return SOCK_ENOMEM;

    struct socket_desc *ns = get_sock(new_fd);
    ns->tcp_conn = (i32)reply.param;
    ns->state = SOCK_STATE_CONN;
    ns->remote.ip = (u32)(reply.tag >> 16);
    ns->remote.port = (u16)(reply.tag & 0xFFFF);

    if (client_addr) {
        client_addr->ip = ns->remote.ip;
        client_addr->port = ns->remote.port;
    }

    return new_fd;
}

i32 sock_send(i32 fd, const void *data, u32 len) {
    struct socket_desc *s = get_sock(fd);
    if (!s || s->state != SOCK_STATE_CONN) return SOCK_ENOTCONN;

    if (s->type == SOCK_DGRAM) {
        struct sockaddr_in dest = s->remote;
        return sock_sendto(fd, data, len, &dest);
    }

    /* TCP: send data via Core 0 */
    struct fifo_msg msg = {0};
    msg.type = MSG_SOCK_SEND;
    msg.param = (u32)s->tcp_conn;
    msg.buffer = (u64)(usize)data;
    msg.length = len;
    send_to_net(&msg);

    struct fifo_msg reply;
    wait_reply(&reply);

    return (i32)reply.param; /* bytes written */
}

i32 sock_recv(i32 fd, void *buf, u32 len) {
    struct socket_desc *s = get_sock(fd);
    if (!s) return SOCK_ERR;

    if (s->type == SOCK_DGRAM)
        return sock_recvfrom(fd, buf, len, NULL);

    if (s->state != SOCK_STATE_CONN) return SOCK_ENOTCONN;

    /* TCP: request read from Core 0 */
    struct fifo_msg msg = {0};
    msg.type = MSG_SOCK_RECV;
    msg.param = (u32)s->tcp_conn;
    msg.buffer = (u64)(usize)buf;
    msg.length = len;
    send_to_net(&msg);

    struct fifo_msg reply;
    wait_reply(&reply);

    return (i32)reply.param; /* bytes read */
}

i32 sock_sendto(i32 fd, const void *data, u32 len, const struct sockaddr_in *dest) {
    struct socket_desc *s = get_sock(fd);
    if (!s || s->type != SOCK_DGRAM) return SOCK_ERR;

    struct fifo_msg msg = {0};
    msg.type = MSG_NET_UDP_SEND;
    msg.param = dest->ip;
    msg.buffer = (u64)(usize)data;
    msg.length = len;
    msg.tag = ((u64)s->local.port << 16) | dest->port;
    send_to_net(&msg);

    struct fifo_msg reply;
    wait_reply(&reply);

    return (reply.status == 0) ? (i32)len : SOCK_ERR;
}

i32 sock_recvfrom(i32 fd, void *buf, u32 len, struct sockaddr_in *src) {
    struct socket_desc *s = get_sock(fd);
    if (!s || s->type != SOCK_DGRAM) return SOCK_ERR;

    /* Wait for incoming UDP data via FIFO */
    struct fifo_msg msg;
    while (!recv_from_net(&msg))
        wfe();

    if (msg.type != MSG_NET_UDP_RECV || !msg.buffer)
        return SOCK_ERR;

    u32 copy = (msg.length < len) ? msg.length : len;
    if (copy > 1472) copy = 1472; /* cap to max UDP payload */
    simd_memcpy(buf, (void *)(usize)msg.buffer, copy);

    if (src) {
        src->ip = msg.param;
        src->port = (u16)(msg.tag & 0xFFFF);
    }

    return (i32)copy;
}

i32 sock_close(i32 fd) {
    struct socket_desc *s = get_sock(fd);
    if (!s) return SOCK_ERR;

    if (s->type == SOCK_STREAM && s->tcp_conn >= 0) {
        struct fifo_msg msg = {0};
        msg.type = MSG_SOCK_CLOSE;
        msg.param = (u32)s->tcp_conn;
        send_to_net(&msg);

        struct fifo_msg reply;
        wait_reply(&reply);
    }

    s->used = false;
    s->state = SOCK_STATE_CLOSED;
    return SOCK_OK;
}

bool sock_local_port(i32 fd, u16 *port_out)
{
    struct socket_desc *s = get_sock(fd);
    if (!s || !port_out) return false;
    if (s->state == SOCK_STATE_NEW || s->state == SOCK_STATE_CLOSED) return false;
    *port_out = s->local.port;
    return true;
}

/* Non-blocking variants */
i32 sock_send_nb(i32 fd, const void *data, u32 len) {
    struct socket_desc *s = get_sock(fd);
    if (!s || s->state != SOCK_STATE_CONN) return SOCK_ENOTCONN;

    if (s->type == SOCK_STREAM) {
        dmb(); /* ensure visibility of Core 0's ring buffer updates */
        u32 avail = tcp_writable(s->tcp_conn);
        if (avail == 0) return SOCK_EAGAIN;
        if (len > avail) len = avail;
    }
    return sock_send(fd, data, len);
}

i32 sock_recv_nb(i32 fd, void *buf, u32 len) {
    struct socket_desc *s = get_sock(fd);
    if (!s) return SOCK_ERR;

    if (s->type == SOCK_STREAM && s->state == SOCK_STATE_CONN) {
        dmb(); /* ensure visibility of Core 0's ring buffer updates */
        if (tcp_readable(s->tcp_conn) == 0)
            return SOCK_EAGAIN;
    }

    struct fifo_msg msg;
    if (!recv_from_net(&msg))
        return SOCK_EAGAIN;

    /* Process the received message */
    if (s->type == SOCK_DGRAM && msg.type == MSG_NET_UDP_RECV) {
        u32 copy = (msg.length < len) ? msg.length : len;
        simd_memcpy(buf, (void *)(usize)msg.buffer, copy);
        return (i32)copy;
    }

    return SOCK_EAGAIN;
}

/* ================================================================== */
/*  Core 0 side: process socket FIFO requests from user cores          */
/* ================================================================== */

void socket_handle_fifo(u32 from_core) {
    struct fifo_msg msg;
    if (!fifo_pop(CORE_NET, from_core, &msg))
        return;

    struct fifo_msg reply = {0};
    reply.tag = msg.tag;
    reply.type = MSG_SOCK_RESULT;

    switch (msg.type) {
    case MSG_SOCK_CONNECT: {
        u32 dst_ip = msg.param;
        u16 dst_port = (u16)(msg.tag & 0xFFFF);
        tcp_conn_t conn = tcp_connect(dst_ip, dst_port);
        if (conn >= 0) {
            /* Poll until established or failed */
            for (u32 i = 0; i < 5000; i++) {
                net_poll();
                u32 st = tcp_state(conn);
                if (st == TCP_ESTABLISHED) {
                    reply.status = 0;
                    reply.param = (u32)conn;
                    fifo_push(CORE_NET, from_core, &reply);
                    return;
                }
                if (st == TCP_CLOSED) break;
                timer_delay_ms(1);
            }
            reply.status = 1;
            fifo_push(CORE_NET, from_core, &reply);
        } else {
            reply.status = 1;
            fifo_push(CORE_NET, from_core, &reply);
        }
        break;
    }

    case MSG_SOCK_LISTEN: {
        u16 port = (u16)msg.param;
        tcp_conn_t conn = tcp_listen(port);
        reply.status = (conn >= 0) ? 0 : 1;
        reply.param = (u32)conn;
        fifo_push(CORE_NET, from_core, &reply);
        break;
    }

    case MSG_SOCK_ACCEPT: {
        tcp_conn_t listen_conn = (tcp_conn_t)msg.param;
        /* Poll until a connection is accepted */
        for (u32 i = 0; i < 30000; i++) {
            net_poll();
            tcp_conn_t ac = tcp_accept(listen_conn);
            if (ac >= 0) {
                reply.status = 0;
                reply.param = (u32)ac;
                fifo_push(CORE_NET, from_core, &reply);
                return;
            }
            timer_delay_ms(1);
        }
        reply.status = 1;
        fifo_push(CORE_NET, from_core, &reply);
        break;
    }

    case MSG_SOCK_SEND: {
        tcp_conn_t conn = (tcp_conn_t)msg.param;
        u32 written = tcp_write(conn, (void *)(usize)msg.buffer, msg.length);
        reply.param = written;
        reply.status = 0;
        fifo_push(CORE_NET, from_core, &reply);
        break;
    }

    case MSG_SOCK_RECV: {
        tcp_conn_t conn = (tcp_conn_t)msg.param;
        /* Poll until data available or timeout */
        for (u32 i = 0; i < 5000; i++) {
            net_poll();
            u32 avail = tcp_readable(conn);
            if (avail > 0) {
                u32 rd = tcp_read(conn, (void *)(usize)msg.buffer, msg.length);
                reply.param = rd;
                reply.status = 0;
                fifo_push(CORE_NET, from_core, &reply);
                return;
            }
            u32 st = tcp_state(conn);
            if (st == TCP_CLOSED || st == TCP_CLOSE_WAIT) {
                reply.param = 0;
                reply.status = 0;
                fifo_push(CORE_NET, from_core, &reply);
                return;
            }
            timer_delay_ms(1);
        }
        reply.param = 0;
        reply.status = 1;
        fifo_push(CORE_NET, from_core, &reply);
        break;
    }

    case MSG_SOCK_CLOSE: {
        tcp_conn_t conn = (tcp_conn_t)msg.param;
        tcp_close(conn);
        reply.status = 0;
        fifo_push(CORE_NET, from_core, &reply);
        break;
    }

    default:
        break;
    }
}

void socket_init(void) {
    for (u32 c = 0; c < 2; c++)
        for (u32 i = 0; i < SOCKET_MAX; i++)
            sockets[c][i].used = false;
}
