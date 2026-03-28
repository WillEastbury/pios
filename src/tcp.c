/*
 * tcp.c - Hardened TCP stack for PIOS
 *
 * Full TCP state machine (all 11 states), SYN cookies, ISN randomization,
 * Reno congestion control, RFC 5961 RST hardening, retransmit with
 * exponential backoff, fast retransmit on 3 dup ACKs.
 *
 * Max 8 simultaneous connections. Runs on Core 0 alongside net_poll().
 *
 * References: RFC 793, RFC 5681, RFC 5961, RFC 6528
 */

#include "tcp.h"
#include "net.h"
#include "genet.h"
#include "arp.h"
#include "simd.h"
#include "uart.h"
#include "timer.h"

/* Provided by net.c */
extern u32 net_get_our_ip(void);

/* ================================================================== */
/*  Constants                                                          */
/* ================================================================== */

#define IP_PROTO_TCP        6
#define ETH_HDR_SIZE        14
#define IP_HDR_SIZE         20
#define TCP_HDR_SIZE        20
#define TCP_OVERHEAD        (ETH_HDR_SIZE + IP_HDR_SIZE + TCP_HDR_SIZE)
#define MAX_FRAME           1514
#define MIN_FRAME           60

#define TCP_FIN  0x01
#define TCP_SYN  0x02
#define TCP_RST  0x04
#define TCP_PSH  0x08
#define TCP_ACK  0x10

#define RTO_INIT_MS         1000
#define RTO_MAX_MS          60000
#define TIME_WAIT_MS        120000   /* 2 * MSL (60s each) */
#define MAX_RETRIES         8

#define LISTEN_BACKLOG      4

/* ================================================================== */
/*  TCP header (wire format)                                           */
/* ================================================================== */

struct tcp_hdr {
    u16 src_port;
    u16 dst_port;
    u32 seq;
    u32 ack;
    u8  data_off;       /* upper 4 bits = offset in 32-bit words */
    u8  flags;
    u16 window;
    u16 checksum;
    u16 urgent;
} PACKED;

/* Pseudo-header for TCP checksum */
struct tcp_pseudo {
    u32 src_ip;
    u32 dst_ip;
    u8  zero;
    u8  protocol;
    u16 tcp_len;
} PACKED;

/* ================================================================== */
/*  Circular buffer                                                    */
/* ================================================================== */

struct ring_buf {
    u8  data[TCP_BUF_SIZE];
    u32 head;       /* write position */
    u32 tail;       /* read position */
};

static u32 ring_used(const struct ring_buf *r) {
    return (r->head - r->tail) & (TCP_BUF_SIZE - 1);
}

static u32 ring_free(const struct ring_buf *r) {
    return (TCP_BUF_SIZE - 1) - ring_used(r);
}

static u32 ring_write(struct ring_buf *r, const void *src, u32 len) {
    u32 avail = ring_free(r);
    if (len > avail) len = avail;
    const u8 *s = (const u8 *)src;
    u32 idx = r->head & (TCP_BUF_SIZE - 1);
    u32 first = TCP_BUF_SIZE - idx;
    if (first > len) first = len;
    simd_memcpy(r->data + idx, s, first);
    if (len > first)
        simd_memcpy(r->data, s + first, len - first);
    r->head += len;
    return len;
}

static u32 ring_read(struct ring_buf *r, void *dst, u32 len) {
    u32 avail = ring_used(r);
    if (len > avail) len = avail;
    u8 *d = (u8 *)dst;
    u32 idx = r->tail & (TCP_BUF_SIZE - 1);
    u32 first = TCP_BUF_SIZE - idx;
    if (first > len) first = len;
    simd_memcpy(d, r->data + idx, first);
    if (len > first)
        simd_memcpy(d + first, r->data, len - first);
    r->tail += len;
    return len;
}

/* Peek at data at offset from tail without consuming */
static u32 ring_peek_at(const struct ring_buf *r, u32 offset, void *dst, u32 len) {
    u32 avail = ring_used(r);
    if (offset >= avail) return 0;
    if (len > avail - offset) len = avail - offset;
    u8 *d = (u8 *)dst;
    u32 pos = r->tail + offset;
    for (u32 i = 0; i < len; i++) {
        d[i] = r->data[pos & (TCP_BUF_SIZE - 1)];
        pos++;
    }
    return len;
}

/* Discard n bytes from front of buffer */
static void ring_consume(struct ring_buf *r, u32 n) {
    u32 avail = ring_used(r);
    if (n > avail) n = avail;
    r->tail += n;
}

/* ================================================================== */
/*  Transmission Control Block (TCB)                                   */
/* ================================================================== */

struct tcb {
    /* Connection 4-tuple */
    u32 local_ip;
    u32 remote_ip;
    u16 local_port;
    u16 remote_port;

    u32 state;

    /* Send sequence space */
    u32 snd_una;        /* oldest unACKed */
    u32 snd_nxt;        /* next to send */
    u32 snd_wnd;        /* peer's advertised window */
    u32 iss;            /* initial send sequence */

    /* Receive sequence space */
    u32 rcv_nxt;        /* next expected */
    u32 rcv_wnd;        /* our advertised window */
    u32 irs;            /* initial receive sequence */

    /* Buffers */
    struct ring_buf tx_buf;
    struct ring_buf rx_buf;

    /* Retransmit */
    u64 rto_ms;
    u64 rto_deadline;   /* timer_ticks() when we should retransmit */
    u32 retries;

    /* RTT estimation (Jacobson/Karn) */
    u64 rtt_seq;        /* seq# being timed */
    u64 rtt_start;      /* ticks when that segment was sent */
    bool rtt_active;
    i32 srtt;           /* smoothed RTT in ms (fixed-point /8) */
    i32 rttvar;         /* RTT variance in ms (fixed-point /4) */

    /* Congestion control (Reno) */
    u32 cwnd;
    u32 ssthresh;
    u32 dup_ack_cnt;
    u32 recover;        /* snd_nxt at time of fast retransmit */

    /* FIN tracking */
    u32 fin_seq;        /* seq of our FIN */
    bool fin_sent;

    /* TIME_WAIT */
    u64 tw_expiry;

    /* LISTEN backlog: pending connections completed via SYN cookie */
    u32 pending_count;
    struct {
        u32 remote_ip;
        u16 remote_port;
        u32 irs;        /* client ISN */
        u32 iss;        /* our ISN (from SYN cookie) */
    } pending[LISTEN_BACKLOG];
};

static struct tcb tcbs[TCP_MAX_CONNECTIONS];
static u32 tcp_local_ip;
static u8  tcp_local_mac[6];
static u16 tcp_ip_id;

/* SYN cookie secret, initialized once */
static u32 syn_secret;

/* TX frame buffer */
static u8 tx_frame[1600] ALIGNED(64);

/* ================================================================== */
/*  Helpers                                                            */
/* ================================================================== */

static u32 min32(u32 a, u32 b) { return a < b ? a : b; }

/* Sequence number comparison: a < b in modular arithmetic */
static bool seq_lt(u32 a, u32 b)  { return (i32)(a - b) < 0; }
static bool seq_le(u32 a, u32 b)  { return (i32)(a - b) <= 0; }
static bool seq_gt(u32 a, u32 b)  { return (i32)(a - b) > 0; }

static struct tcb *tcb_find(u32 local_port, u32 remote_ip, u16 remote_port) {
    for (u32 i = 0; i < TCP_MAX_CONNECTIONS; i++) {
        struct tcb *t = &tcbs[i];
        if (t->state == TCP_CLOSED) continue;
        if (t->local_port == local_port &&
            t->remote_ip == remote_ip &&
            t->remote_port == remote_port)
            return t;
    }
    return NULL;
}

static struct tcb *tcb_find_listen(u16 port) {
    for (u32 i = 0; i < TCP_MAX_CONNECTIONS; i++) {
        struct tcb *t = &tcbs[i];
        if (t->state == TCP_LISTEN && t->local_port == port)
            return t;
    }
    return NULL;
}

static struct tcb *tcb_alloc(void) {
    for (u32 i = 0; i < TCP_MAX_CONNECTIONS; i++)
        if (tcbs[i].state == TCP_CLOSED)
            return &tcbs[i];
    return NULL;
}

static i32 tcb_index(const struct tcb *t) {
    return (i32)(t - tcbs);
}

static bool tcb_valid(tcp_conn_t c) {
    return c >= 0 && c < TCP_MAX_CONNECTIONS && tcbs[c].state != TCP_CLOSED;
}

/* Ephemeral port allocator */
static u16 next_ephemeral = 49152;
static u16 alloc_port(void) {
    u16 p = next_ephemeral++;
    if (next_ephemeral == 0) next_ephemeral = 49152;
    return p;
}

/* ================================================================== */
/*  ISN generation (RFC 6528)                                          */
/* ================================================================== */

static u32 generate_isn(u32 local_ip, u16 local_port,
                        u32 remote_ip, u16 remote_port) {
    struct {
        u32 lip;
        u16 lp;
        u32 rip;
        u16 rp;
        u32 secret;
        u32 ts;
    } PACKED seed;
    seed.lip    = local_ip;
    seed.lp     = local_port;
    seed.rip    = remote_ip;
    seed.rp     = remote_port;
    seed.secret = syn_secret;
    seed.ts     = (u32)timer_ticks();
    return hw_crc32c(&seed, sizeof(seed));
}

/* ================================================================== */
/*  SYN Cookies                                                        */
/* ================================================================== */

static u32 make_syn_cookie(u32 local_port, u32 remote_ip, u16 remote_port,
                           u32 their_seq) {
    struct {
        u32 rip;
        u16 rp;
        u16 lp;
        u32 seq;
        u32 secret;
    } PACKED blob;
    blob.rip    = remote_ip;
    blob.rp     = remote_port;
    blob.lp     = (u16)local_port;
    blob.seq    = their_seq;
    blob.secret = syn_secret;
    return hw_crc32c(&blob, sizeof(blob));
}

static bool validate_syn_cookie(u32 local_port, u32 remote_ip, u16 remote_port,
                                u32 their_seq, u32 our_ack_minus1) {
    u32 expected = make_syn_cookie(local_port, remote_ip, remote_port, their_seq);
    return expected == our_ack_minus1;
}

/* ================================================================== */
/*  TCP checksum                                                       */
/* ================================================================== */

static u16 tcp_checksum(u32 src_ip, u32 dst_ip,
                        const void *tcp_data, u32 tcp_len) {
    /* Two-pass checksum: pseudo-header on stack (12 bytes), then TCP data in-place */
    struct tcp_pseudo ph;
    ph.src_ip   = htonl(src_ip);
    ph.dst_ip   = htonl(dst_ip);
    ph.zero     = 0;
    ph.protocol = IP_PROTO_TCP;
    ph.tcp_len  = htons((u16)tcp_len);

    /* Accumulate checksum manually: pseudo-header then TCP data */
    u32 sum = 0;
    const u16 *p = (const u16 *)&ph;
    for (u32 i = 0; i < sizeof(ph) / 2; i++)
        sum += p[i];
    p = (const u16 *)tcp_data;
    u32 words = tcp_len / 2;
    for (u32 i = 0; i < words; i++)
        sum += p[i];
    if (tcp_len & 1)
        sum += ((const u8 *)tcp_data)[tcp_len - 1];
    while (sum >> 16)
        sum = (sum & 0xFFFF) + (sum >> 16);
    return (u16)~sum;
}

/* ================================================================== */
/*  TX path: build Ethernet + IP + TCP and send                        */
/* ================================================================== */

static void tcp_send_segment(struct tcb *t, u8 flags,
                             const void *data, u32 data_len) {
    /* Resolve destination MAC */
    const u8 *dst_mac = arp_resolve(t->remote_ip);
    if (unlikely(!dst_mac)) return;

    u16 tcp_len = TCP_HDR_SIZE + data_len;
    u16 ip_total = IP_HDR_SIZE + tcp_len;
    u32 frame_len = ETH_HDR_SIZE + ip_total;
    if (frame_len > MAX_FRAME) return;

    /* Ethernet */
    struct eth_hdr *eth = (struct eth_hdr *)tx_frame;
    simd_memcpy(eth->dst, dst_mac, 6);
    simd_memcpy(eth->src, tcp_local_mac, 6);
    eth->ethertype = htons(ETH_P_IP);

    /* IP */
    struct ip_hdr *ip = (struct ip_hdr *)(tx_frame + ETH_HDR_SIZE);
    ip->ver_ihl    = 0x45;
    ip->tos        = 0;
    ip->total_len  = htons(ip_total);
    ip->id         = htons(tcp_ip_id++);
    ip->flags_frag = htons(0x4000);     /* DF */
    ip->ttl        = 64;
    ip->protocol   = IP_PROTO_TCP;
    ip->checksum   = 0;
    ip->src_ip     = htonl(t->local_ip);
    ip->dst_ip     = htonl(t->remote_ip);
    ip->checksum   = simd_checksum(ip, IP_HDR_SIZE);

    /* TCP */
    struct tcp_hdr *tcp = (struct tcp_hdr *)(tx_frame + ETH_HDR_SIZE + IP_HDR_SIZE);
    tcp->src_port  = htons(t->local_port);
    tcp->dst_port  = htons(t->remote_port);
    tcp->seq       = htonl(t->snd_nxt);
    tcp->ack       = htonl(t->rcv_nxt);
    tcp->data_off  = (TCP_HDR_SIZE / 4) << 4;
    tcp->flags     = flags;
    tcp->window    = htons((u16)min32(t->rcv_wnd, 0xFFFF));
    tcp->checksum  = 0;
    tcp->urgent    = 0;

    if (data_len > 0)
        simd_memcpy(tx_frame + TCP_OVERHEAD, data, data_len);

    tcp->checksum = tcp_checksum(t->local_ip, t->remote_ip, tcp, tcp_len);

    if (frame_len < MIN_FRAME) frame_len = MIN_FRAME;
    genet_send(tx_frame, frame_len);
}

/* Send a raw RST/ACK without a TCB (for rejecting unexpected segments) */
static void tcp_send_rst(u32 src_ip, u32 dst_ip, u16 src_port, u16 dst_port,
                         u32 seq, u32 ack) {
    const u8 *dst_mac = arp_resolve(dst_ip);
    if (!dst_mac) return;

    u16 ip_total = IP_HDR_SIZE + TCP_HDR_SIZE;
    u32 frame_len = ETH_HDR_SIZE + ip_total;

    struct eth_hdr *eth = (struct eth_hdr *)tx_frame;
    simd_memcpy(eth->dst, dst_mac, 6);
    simd_memcpy(eth->src, tcp_local_mac, 6);
    eth->ethertype = htons(ETH_P_IP);

    struct ip_hdr *ip = (struct ip_hdr *)(tx_frame + ETH_HDR_SIZE);
    ip->ver_ihl    = 0x45;
    ip->tos        = 0;
    ip->total_len  = htons(ip_total);
    ip->id         = htons(tcp_ip_id++);
    ip->flags_frag = htons(0x4000);
    ip->ttl        = 64;
    ip->protocol   = IP_PROTO_TCP;
    ip->checksum   = 0;
    ip->src_ip     = htonl(src_ip);
    ip->dst_ip     = htonl(dst_ip);
    ip->checksum   = simd_checksum(ip, IP_HDR_SIZE);

    struct tcp_hdr *tcp = (struct tcp_hdr *)(tx_frame + ETH_HDR_SIZE + IP_HDR_SIZE);
    tcp->src_port  = htons(src_port);
    tcp->dst_port  = htons(dst_port);
    tcp->seq       = htonl(seq);
    tcp->ack       = htonl(ack);
    tcp->data_off  = (TCP_HDR_SIZE / 4) << 4;
    tcp->flags     = TCP_RST | TCP_ACK;
    tcp->window    = 0;
    tcp->checksum  = 0;
    tcp->urgent    = 0;

    tcp->checksum = tcp_checksum(src_ip, dst_ip, tcp, TCP_HDR_SIZE);

    if (frame_len < MIN_FRAME) frame_len = MIN_FRAME;
    genet_send(tx_frame, frame_len);
}

/* Send a SYN-ACK with SYN cookie ISN (no TCB needed) */
static void tcp_send_synack_cookie(u32 remote_ip, u16 remote_port,
                                   u16 local_port, u32 their_seq,
                                   u32 cookie_isn) {
    const u8 *dst_mac = arp_resolve(remote_ip);
    if (!dst_mac) return;

    u16 ip_total = IP_HDR_SIZE + TCP_HDR_SIZE;
    u32 frame_len = ETH_HDR_SIZE + ip_total;

    struct eth_hdr *eth = (struct eth_hdr *)tx_frame;
    simd_memcpy(eth->dst, dst_mac, 6);
    simd_memcpy(eth->src, tcp_local_mac, 6);
    eth->ethertype = htons(ETH_P_IP);

    struct ip_hdr *ip = (struct ip_hdr *)(tx_frame + ETH_HDR_SIZE);
    ip->ver_ihl    = 0x45;
    ip->tos        = 0;
    ip->total_len  = htons(ip_total);
    ip->id         = htons(tcp_ip_id++);
    ip->flags_frag = htons(0x4000);
    ip->ttl        = 64;
    ip->protocol   = IP_PROTO_TCP;
    ip->checksum   = 0;
    ip->src_ip     = htonl(tcp_local_ip);
    ip->dst_ip     = htonl(remote_ip);
    ip->checksum   = simd_checksum(ip, IP_HDR_SIZE);

    struct tcp_hdr *tcp = (struct tcp_hdr *)(tx_frame + ETH_HDR_SIZE + IP_HDR_SIZE);
    tcp->src_port  = htons(local_port);
    tcp->dst_port  = htons(remote_port);
    tcp->seq       = htonl(cookie_isn);
    tcp->ack       = htonl(their_seq + 1);
    tcp->data_off  = (TCP_HDR_SIZE / 4) << 4;
    tcp->flags     = TCP_SYN | TCP_ACK;
    tcp->window    = htons(TCP_DEFAULT_WINDOW);
    tcp->checksum  = 0;
    tcp->urgent    = 0;

    tcp->checksum = tcp_checksum(tcp_local_ip, remote_ip, tcp, TCP_HDR_SIZE);

    if (frame_len < MIN_FRAME) frame_len = MIN_FRAME;
    genet_send(tx_frame, frame_len);
}

/* ================================================================== */
/*  RTT estimation (Jacobson/Karn, RFC 6298)                           */
/* ================================================================== */

static void rtt_init(struct tcb *t) {
    t->srtt       = 0;
    t->rttvar     = 0;
    t->rtt_active = false;
    t->rto_ms     = RTO_INIT_MS;
}

static void rtt_update(struct tcb *t, u32 rtt_ms) {
    if (t->srtt == 0) {
        /* First measurement */
        t->srtt   = (i32)(rtt_ms << 3);    /* SRTT = R << 3 */
        t->rttvar = (i32)(rtt_ms << 1);    /* RTTVAR = R/2 << 2 */
    } else {
        i32 delta = (i32)rtt_ms - (t->srtt >> 3);
        if (delta < 0) delta = -delta;
        t->rttvar = t->rttvar - (t->rttvar >> 2) + (delta);
        t->srtt   = t->srtt - (t->srtt >> 3) + (i32)rtt_ms;
    }
    i32 rto = (t->srtt >> 3) + (t->rttvar);
    if (rto < 200) rto = 200;
    if (rto > (i32)RTO_MAX_MS) rto = (i32)RTO_MAX_MS;
    t->rto_ms = (u64)rto;
}

/* ================================================================== */
/*  Retransmit & data sending                                          */
/* ================================================================== */

static void tcp_arm_rto(struct tcb *t) {
    t->rto_deadline = timer_ticks() + t->rto_ms;
}

static void tcp_output(struct tcb *t) {
    if (t->state != TCP_ESTABLISHED &&
        t->state != TCP_CLOSE_WAIT)
        return;

    /* Effective send window */
    u32 eff_wnd = min32(t->snd_wnd, t->cwnd);
    u32 in_flight = t->snd_nxt - t->snd_una;
    if (in_flight >= eff_wnd) return;
    u32 can_send = eff_wnd - in_flight;

    u32 buffered = ring_used(&t->tx_buf);
    /* Data in buffer that hasn't been sent yet */
    u32 unsent_off = t->snd_nxt - t->snd_una;
    if (unsent_off >= buffered) return;
    u32 unsent = buffered - unsent_off;

    u32 to_send = min32(unsent, can_send);
    to_send = min32(to_send, TCP_MSS);
    if (to_send == 0) return;

    u8 seg_data[TCP_MSS];
    u32 got = ring_peek_at(&t->tx_buf, unsent_off, seg_data, to_send);
    if (got == 0) return;

    /* Start RTT measurement on this segment if not already timing one */
    if (!t->rtt_active) {
        t->rtt_seq    = t->snd_nxt;
        t->rtt_start  = timer_ticks();
        t->rtt_active = true;
    }

    u8 flags = TCP_ACK;
    if (got < unsent && ring_free(&t->tx_buf) > 0) {
        /* More data buffered, don't push yet */
    } else {
        flags |= TCP_PSH;
    }

    tcp_send_segment(t, flags, seg_data, got);
    t->snd_nxt += got;
    tcp_arm_rto(t);
}

static void tcp_retransmit(struct tcb *t) {
    u32 buffered = ring_used(&t->tx_buf);
    if (buffered == 0) {
        /* Maybe retransmit SYN or FIN */
        if (t->state == TCP_SYN_SENT) {
            tcp_send_segment(t, TCP_SYN, NULL, 0);
            tcp_arm_rto(t);
        } else if (t->state == TCP_FIN_WAIT_1 || t->state == TCP_LAST_ACK) {
            u8 flags = TCP_FIN | TCP_ACK;
            t->snd_nxt = t->fin_seq;
            tcp_send_segment(t, flags, NULL, 0);
            t->snd_nxt = t->fin_seq + 1;
            tcp_arm_rto(t);
        }
        return;
    }

    /* Retransmit from snd_una */
    u32 len = min32(buffered, TCP_MSS);
    u8 seg_data[TCP_MSS];
    ring_peek_at(&t->tx_buf, 0, seg_data, len);

    u32 saved_nxt = t->snd_nxt;
    t->snd_nxt = t->snd_una;
    tcp_send_segment(t, TCP_ACK | TCP_PSH, seg_data, len);
    t->snd_nxt = saved_nxt;
    if (seq_lt(t->snd_una + len, t->snd_nxt))
        ; /* snd_nxt stays */
    else
        t->snd_nxt = t->snd_una + len;

    /* Karn's algorithm: don't use retransmitted segment for RTT */
    t->rtt_active = false;
    tcp_arm_rto(t);
}

/* Send a pure ACK (no data) */
static void tcp_send_ack(struct tcb *t) {
    tcp_send_segment(t, TCP_ACK, NULL, 0);
}

/* Send FIN */
static void tcp_send_fin(struct tcb *t) {
    t->fin_seq  = t->snd_nxt;
    t->fin_sent = true;
    tcp_send_segment(t, TCP_FIN | TCP_ACK, NULL, 0);
    t->snd_nxt++;
    tcp_arm_rto(t);
}

/* ================================================================== */
/*  Congestion control (Reno)                                          */
/* ================================================================== */

static void cc_init(struct tcb *t) {
    t->cwnd      = TCP_MSS;
    t->ssthresh  = 65535;
    t->dup_ack_cnt = 0;
    t->recover   = t->iss;
}

static void cc_on_ack(struct tcb *t, u32 acked_bytes) {
    t->dup_ack_cnt = 0;
    if (t->cwnd < t->ssthresh) {
        /* Slow start */
        t->cwnd += min32(acked_bytes, TCP_MSS);
    } else {
        /* Congestion avoidance */
        t->cwnd += TCP_MSS * TCP_MSS / t->cwnd;
        if (t->cwnd < TCP_MSS) t->cwnd = TCP_MSS;
    }
}

static void cc_on_dup_ack(struct tcb *t) {
    t->dup_ack_cnt++;
    if (t->dup_ack_cnt == 3) {
        /* Fast retransmit */
        t->ssthresh = t->cwnd / 2;
        if (t->ssthresh < 2 * TCP_MSS) t->ssthresh = 2 * TCP_MSS;
        t->cwnd = t->ssthresh + 3 * TCP_MSS;
        t->recover = t->snd_nxt;
        tcp_retransmit(t);
    } else if (t->dup_ack_cnt > 3) {
        /* Inflate window */
        t->cwnd += TCP_MSS;
    }
}

static void cc_on_timeout(struct tcb *t) {
    t->ssthresh = t->cwnd / 2;
    if (t->ssthresh < 2 * TCP_MSS) t->ssthresh = 2 * TCP_MSS;
    t->cwnd = TCP_MSS;
    t->dup_ack_cnt = 0;
}

/* ================================================================== */
/*  TCB reset helper                                                   */
/* ================================================================== */

static void tcb_reset(struct tcb *t) {
    memset(t, 0, sizeof(struct tcb));
    t->state = TCP_CLOSED;
}

/* ================================================================== */
/*  RX path: process incoming TCP segment                              */
/* ================================================================== */

/* Check if sequence number is acceptable (RFC 793 segment acceptance) */
static bool seq_acceptable(struct tcb *t, u32 seg_seq, u32 seg_len) {
    u32 rcv_wnd = t->rcv_wnd;
    if (seg_len == 0 && rcv_wnd == 0)
        return seg_seq == t->rcv_nxt;
    if (seg_len == 0 && rcv_wnd > 0)
        return seq_le(t->rcv_nxt, seg_seq) &&
               seq_lt(seg_seq, t->rcv_nxt + rcv_wnd);
    if (seg_len > 0 && rcv_wnd > 0)
        return (seq_le(t->rcv_nxt, seg_seq) &&
                seq_lt(seg_seq, t->rcv_nxt + rcv_wnd)) ||
               (seq_le(t->rcv_nxt, seg_seq + seg_len - 1) &&
                seq_lt(seg_seq + seg_len - 1, t->rcv_nxt + rcv_wnd));
    /* seg_len > 0, rcv_wnd == 0 → not acceptable */
    return false;
}

static void handle_established(struct tcb *t, u32 seg_seq, u32 seg_ack,
                               u8 flags, u16 seg_wnd,
                               const u8 *data, u32 data_len) {
    /* Process ACK */
    if (flags & TCP_ACK) {
        if (seq_gt(seg_ack, t->snd_nxt)) {
            /* ACK for unsent data — send ACK and drop */
            tcp_send_ack(t);
            return;
        }

        if (seq_gt(seg_ack, t->snd_una)) {
            /* New data ACKed */
            u32 acked = seg_ack - t->snd_una;
            ring_consume(&t->tx_buf, acked);
            t->snd_una = seg_ack;
            t->retries = 0;

            /* RTT measurement */
            if (t->rtt_active && seq_le(t->rtt_seq + 1, seg_ack)) {
                u64 rtt = timer_ticks() - t->rtt_start;
                rtt_update(t, (u32)rtt);
                t->rtt_active = false;
            }

            cc_on_ack(t, acked);

            /* Reset RTO if more data outstanding */
            if (t->snd_una != t->snd_nxt)
                tcp_arm_rto(t);
        } else if (seg_ack == t->snd_una && data_len == 0 &&
                   t->snd_una != t->snd_nxt) {
            /* Duplicate ACK */
            cc_on_dup_ack(t);
        }
    }

    /* Update send window */
    t->snd_wnd = seg_wnd;

    /* Process data */
    if (data_len > 0 && seg_seq == t->rcv_nxt) {
        u32 written = ring_write(&t->rx_buf, data, data_len);
        t->rcv_nxt += written;
        t->rcv_wnd = ring_free(&t->rx_buf);
        tcp_send_ack(t);
    }

    /* FIN processing */
    if (flags & TCP_FIN) {
        if (seg_seq + data_len == t->rcv_nxt ||
            (data_len == 0 && seg_seq == t->rcv_nxt)) {
            t->rcv_nxt++;
            t->state = TCP_CLOSE_WAIT;
            tcp_send_ack(t);
        }
    }

    /* Try to push more data */
    tcp_output(t);
}

void tcp_input(const u8 *frame UNUSED, u32 len UNUSED, u32 src_ip, u32 dst_ip,
               const u8 *payload, u32 payload_len) {
    if (unlikely(payload_len < TCP_HDR_SIZE))
        return;

    const struct tcp_hdr *tcp = (const struct tcp_hdr *)payload;

    /* Verify TCP checksum */
    if (unlikely(tcp_checksum(src_ip, dst_ip, payload, payload_len) != 0))
        return;

    u16 src_port = ntohs(tcp->src_port);
    u16 dst_port = ntohs(tcp->dst_port);
    u32 seg_seq  = ntohl(tcp->seq);
    u32 seg_ack  = ntohl(tcp->ack);
    u8  flags    = tcp->flags;
    u16 seg_wnd  = ntohs(tcp->window);

    u32 hdr_len = (tcp->data_off >> 4) * 4;
    if (unlikely(hdr_len < TCP_HDR_SIZE || hdr_len > payload_len))
        return;

    const u8 *seg_data = payload + hdr_len;
    u32 data_len = payload_len - hdr_len;

    /* Look up existing connection */
    struct tcb *t = tcb_find(dst_port, src_ip, src_port);

    /* ---- LISTEN handling (SYN cookies) ---- */
    if (!t) {
        struct tcb *listen = tcb_find_listen(dst_port);
        if (!listen) {
            /* No matching socket — send RST */
            if (!(flags & TCP_RST)) {
                if (flags & TCP_ACK) {
                    tcp_send_rst(dst_ip, src_ip, dst_port, src_port,
                                 seg_ack, 0);
                } else {
                    tcp_send_rst(dst_ip, src_ip, dst_port, src_port,
                                 0, seg_seq + data_len +
                                 ((flags & TCP_SYN) ? 1 : 0) +
                                 ((flags & TCP_FIN) ? 1 : 0));
                }
            }
            return;
        }

        if (flags & TCP_SYN) {
            /* Respond with SYN-ACK using SYN cookie ISN — no TCB allocated */
            u32 cookie = make_syn_cookie(dst_port, src_ip, src_port, seg_seq);
            tcp_send_synack_cookie(src_ip, src_port, dst_port, seg_seq, cookie);
            return;
        }

        if (flags & TCP_ACK) {
            /* Validate SYN cookie: the ACK should be cookie+1 */
            u32 their_seq = seg_ack - 1;  /* this was our ISN (cookie) */
            u32 their_iss = seg_seq - 1;  /* their ISN was seg_seq - 1 */
            if (!validate_syn_cookie(dst_port, src_ip, src_port,
                                     their_iss, their_seq)) {
                /* Bad cookie — send RST */
                tcp_send_rst(dst_ip, src_ip, dst_port, src_port, seg_ack, 0);
                return;
            }

            /* Valid cookie — queue in listen backlog */
            if (listen->pending_count < LISTEN_BACKLOG) {
                u32 idx = listen->pending_count++;
                listen->pending[idx].remote_ip   = src_ip;
                listen->pending[idx].remote_port  = src_port;
                listen->pending[idx].irs          = their_iss;
                listen->pending[idx].iss          = their_seq;
            }
            return;
        }

        /* Any other packet to LISTEN socket: drop */
        return;
    }

    /* ---- SYN_SENT state ---- */
    if (t->state == TCP_SYN_SENT) {
        if (flags & TCP_ACK) {
            if (seg_ack != t->iss + 1) {
                if (!(flags & TCP_RST))
                    tcp_send_rst(t->local_ip, t->remote_ip,
                                 t->local_port, t->remote_port, seg_ack, 0);
                return;
            }
        }
        if (flags & TCP_RST) {
            if (flags & TCP_ACK) {
                tcb_reset(t);
            }
            return;
        }
        if (flags & TCP_SYN) {
            t->irs     = seg_seq;
            t->rcv_nxt = seg_seq + 1;
            t->snd_wnd = seg_wnd;
            if (flags & TCP_ACK) {
                t->snd_una = seg_ack;
                t->state   = TCP_ESTABLISHED;
                tcp_send_ack(t);
                cc_init(t);
            } else {
                /* Simultaneous open */
                t->state = TCP_SYN_RECEIVED;
                tcp_send_segment(t, TCP_SYN | TCP_ACK, NULL, 0);
            }
        }
        return;
    }

    /* ---- SYN_RECEIVED state ---- */
    if (t->state == TCP_SYN_RECEIVED) {
        if (!seq_acceptable(t, seg_seq, data_len))
            return;

        if (flags & TCP_RST) {
            tcb_reset(t);
            return;
        }

        if (flags & TCP_ACK) {
            if (seq_le(t->snd_una, seg_ack) && seq_le(seg_ack, t->snd_nxt)) {
                t->snd_una = seg_ack;
                t->snd_wnd = seg_wnd;
                t->state   = TCP_ESTABLISHED;
                cc_init(t);
            } else {
                tcp_send_rst(t->local_ip, t->remote_ip,
                             t->local_port, t->remote_port, seg_ack, 0);
                return;
            }
        }
        return;
    }

    /* ---- All other ESTABLISHED+ states ---- */

    /* Sequence number check */
    u32 seg_len = data_len + ((flags & TCP_SYN) ? 1 : 0) + ((flags & TCP_FIN) ? 1 : 0);
    if (!seq_acceptable(t, seg_seq, seg_len)) {
        if (!(flags & TCP_RST))
            tcp_send_ack(t);
        return;
    }

    /* RST validation (RFC 5961): only accept if SEQ == RCV.NXT */
    if (flags & TCP_RST) {
        if (seg_seq == t->rcv_nxt) {
            tcb_reset(t);
        }
        /* Else: silently drop (blind RST attack prevention) */
        return;
    }

    /* SYN in window — RFC 5961: send ACK (challenge ACK) */
    if (flags & TCP_SYN) {
        tcp_send_ack(t);
        return;
    }

    if (!(flags & TCP_ACK))
        return;

    /* Update send window from every valid ACK */
    t->snd_wnd = seg_wnd;

    /* State-specific ACK processing */
    switch (t->state) {
    case TCP_ESTABLISHED:
        handle_established(t, seg_seq, seg_ack, flags, seg_wnd, seg_data, data_len);
        break;

    case TCP_FIN_WAIT_1:
        /* Process data + ACK like ESTABLISHED */
        if (flags & TCP_ACK) {
            if (seq_gt(seg_ack, t->snd_una)) {
                u32 acked = seg_ack - t->snd_una;
                ring_consume(&t->tx_buf, acked);
                t->snd_una = seg_ack;
            }
        }
        /* Accept incoming data */
        if (data_len > 0 && seg_seq == t->rcv_nxt) {
            u32 written = ring_write(&t->rx_buf, seg_data, data_len);
            t->rcv_nxt += written;
            t->rcv_wnd = ring_free(&t->rx_buf);
        }
        /* Check if our FIN has been ACKed */
        if (t->fin_sent && seg_ack == t->fin_seq + 1) {
            t->state = TCP_FIN_WAIT_2;
        }
        if (flags & TCP_FIN) {
            t->rcv_nxt++;
            tcp_send_ack(t);
            if (t->state == TCP_FIN_WAIT_2) {
                t->state     = TCP_TIME_WAIT;
                t->tw_expiry = timer_ticks() + TIME_WAIT_MS;
            } else {
                /* Simultaneous close: FIN_WAIT_1 → CLOSING */
                t->state = TCP_CLOSING;
            }
        } else {
            tcp_send_ack(t);
        }
        break;

    case TCP_FIN_WAIT_2:
        /* Accept data */
        if (data_len > 0 && seg_seq == t->rcv_nxt) {
            u32 written = ring_write(&t->rx_buf, seg_data, data_len);
            t->rcv_nxt += written;
            t->rcv_wnd = ring_free(&t->rx_buf);
        }
        if (flags & TCP_FIN) {
            t->rcv_nxt++;
            t->state     = TCP_TIME_WAIT;
            t->tw_expiry = timer_ticks() + TIME_WAIT_MS;
            tcp_send_ack(t);
        } else if (data_len > 0) {
            tcp_send_ack(t);
        }
        break;

    case TCP_CLOSE_WAIT:
        /* ACK processing only (we're waiting for user to call tcp_close) */
        if (flags & TCP_ACK) {
            if (seq_gt(seg_ack, t->snd_una)) {
                u32 acked = seg_ack - t->snd_una;
                ring_consume(&t->tx_buf, acked);
                t->snd_una = seg_ack;
            }
        }
        break;

    case TCP_CLOSING:
        /* Waiting for ACK of our FIN */
        if (flags & TCP_ACK) {
            if (t->fin_sent && seg_ack == t->fin_seq + 1) {
                t->state     = TCP_TIME_WAIT;
                t->tw_expiry = timer_ticks() + TIME_WAIT_MS;
            }
        }
        break;

    case TCP_LAST_ACK:
        if (flags & TCP_ACK) {
            if (t->fin_sent && seg_ack == t->fin_seq + 1) {
                tcb_reset(t);
            }
        }
        break;

    case TCP_TIME_WAIT:
        /* Retransmit ACK if FIN received again */
        if (flags & TCP_FIN) {
            tcp_send_ack(t);
            t->tw_expiry = timer_ticks() + TIME_WAIT_MS;
        }
        break;

    default:
        break;
    }
}

/* ================================================================== */
/*  Timer tick (~100ms)                                                */
/* ================================================================== */

void tcp_tick(void) {
    u64 now = timer_ticks();

    for (u32 i = 0; i < TCP_MAX_CONNECTIONS; i++) {
        struct tcb *t = &tcbs[i];
        if (t->state == TCP_CLOSED || t->state == TCP_LISTEN)
            continue;

        /* TIME_WAIT expiry */
        if (t->state == TCP_TIME_WAIT) {
            if (now >= t->tw_expiry) {
                tcb_reset(t);
            }
            continue;
        }

        /* Retransmit timer */
        if (t->rto_deadline != 0 && now >= t->rto_deadline) {
            t->retries++;
            if (t->retries > MAX_RETRIES) {
                /* Connection failed */
                uart_puts("[tcp] conn timeout\n");
                tcb_reset(t);
                continue;
            }

            cc_on_timeout(t);

            /* Exponential backoff */
            t->rto_ms *= 2;
            if (t->rto_ms > RTO_MAX_MS) t->rto_ms = RTO_MAX_MS;

            tcp_retransmit(t);
        }

        /* Zero-window probe: if peer advertised window=0, periodically poke */
        if (t->snd_wnd == 0 && t->state == TCP_ESTABLISHED &&
            ring_used(&t->tx_buf) > 0) {
            if (t->rto_deadline == 0 || now >= t->rto_deadline) {
                /* Send 1-byte probe */
                u8 probe;
                if (ring_peek_at(&t->tx_buf, 0, &probe, 1) == 1) {
                    u32 saved = t->snd_nxt;
                    t->snd_nxt = t->snd_una;
                    tcp_send_segment(t, TCP_ACK, &probe, 1);
                    t->snd_nxt = saved;
                }
                tcp_arm_rto(t);
            }
        }
    }
}

/* ================================================================== */
/*  Public API                                                         */
/* ================================================================== */

void tcp_init(void) {
    for (u32 i = 0; i < TCP_MAX_CONNECTIONS; i++)
        tcb_reset(&tcbs[i]);

    genet_get_mac(tcp_local_mac);
    tcp_local_ip = net_get_our_ip();

    /* Generate SYN cookie secret from timer jitter */
    u32 t0 = (u32)timer_ticks();
    syn_secret = hw_crc32c(&t0, sizeof(t0));
    syn_secret ^= 0xA5C39E17;  /* mix in constant */

    tcp_ip_id = (u16)(syn_secret & 0xFFFF);
    next_ephemeral = 49152 + (u16)(syn_secret >> 16);

    uart_puts("[tcp] init ok\n");
}

tcp_conn_t tcp_connect(u32 dst_ip, u16 dst_port) {
    struct tcb *t = tcb_alloc();
    if (!t) return -1;

    u16 src_port = alloc_port();

    memset(t, 0, sizeof(struct tcb));
    t->local_ip    = tcp_local_ip;
    t->remote_ip   = dst_ip;
    t->local_port  = src_port;
    t->remote_port = dst_port;
    t->rcv_wnd     = TCP_DEFAULT_WINDOW;

    t->iss     = generate_isn(tcp_local_ip, src_port, dst_ip, dst_port);
    t->snd_una = t->iss;
    t->snd_nxt = t->iss;

    rtt_init(t);
    cc_init(t);

    /* Send SYN */
    t->state = TCP_SYN_SENT;
    tcp_send_segment(t, TCP_SYN, NULL, 0);
    t->snd_nxt = t->iss + 1;
    tcp_arm_rto(t);

    return tcb_index(t);
}

tcp_conn_t tcp_listen(u16 port) {
    struct tcb *t = tcb_alloc();
    if (!t) return -1;

    memset(t, 0, sizeof(struct tcb));
    t->local_ip   = tcp_local_ip;
    t->local_port = port;
    t->state      = TCP_LISTEN;
    t->rcv_wnd    = TCP_DEFAULT_WINDOW;

    return tcb_index(t);
}

tcp_conn_t tcp_accept(tcp_conn_t listen_conn) {
    if (!tcb_valid(listen_conn)) return -1;
    struct tcb *lt = &tcbs[listen_conn];
    if (lt->state != TCP_LISTEN || lt->pending_count == 0) return -1;

    /* Pop oldest pending connection */
    u32 remote_ip   = lt->pending[0].remote_ip;
    u16 remote_port = lt->pending[0].remote_port;
    u32 irs         = lt->pending[0].irs;
    u32 iss         = lt->pending[0].iss;

    /* Shift backlog */
    for (u32 i = 1; i < lt->pending_count; i++)
        lt->pending[i - 1] = lt->pending[i];
    lt->pending_count--;

    /* Allocate new TCB for the accepted connection */
    struct tcb *t = tcb_alloc();
    if (!t) return -1;

    memset(t, 0, sizeof(struct tcb));
    t->local_ip    = tcp_local_ip;
    t->remote_ip   = remote_ip;
    t->local_port  = lt->local_port;
    t->remote_port = remote_port;
    t->rcv_wnd     = TCP_DEFAULT_WINDOW;

    t->iss     = iss;
    t->snd_una = iss + 1;
    t->snd_nxt = iss + 1;
    t->irs     = irs;
    t->rcv_nxt = irs + 1;
    t->snd_wnd = TCP_DEFAULT_WINDOW;

    rtt_init(t);
    cc_init(t);

    t->state = TCP_ESTABLISHED;
    return tcb_index(t);
}

u32 tcp_write(tcp_conn_t conn, const void *data, u32 len) {
    if (!tcb_valid(conn)) return 0;
    struct tcb *t = &tcbs[conn];
    if (t->state != TCP_ESTABLISHED && t->state != TCP_CLOSE_WAIT) return 0;

    u32 written = ring_write(&t->tx_buf, data, len);
    tcp_output(t);
    return written;
}

u32 tcp_read(tcp_conn_t conn, void *data, u32 len) {
    if (!tcb_valid(conn)) return 0;
    struct tcb *t = &tcbs[conn];
    u32 n = ring_read(&t->rx_buf, data, len);
    t->rcv_wnd = ring_free(&t->rx_buf);
    return n;
}

void tcp_close(tcp_conn_t conn) {
    if (!tcb_valid(conn)) return;
    struct tcb *t = &tcbs[conn];

    switch (t->state) {
    case TCP_LISTEN:
    case TCP_SYN_SENT:
        tcb_reset(t);
        break;
    case TCP_ESTABLISHED:
        tcp_send_fin(t);
        t->state = TCP_FIN_WAIT_1;
        break;
    case TCP_CLOSE_WAIT:
        tcp_send_fin(t);
        t->state = TCP_LAST_ACK;
        break;
    case TCP_SYN_RECEIVED:
        tcp_send_fin(t);
        t->state = TCP_FIN_WAIT_1;
        break;
    default:
        break;
    }
}

u32 tcp_state(tcp_conn_t conn) {
    if (conn < 0 || conn >= TCP_MAX_CONNECTIONS)
        return TCP_CLOSED;
    return tcbs[conn].state;
}

u32 tcp_readable(tcp_conn_t conn) {
    if (!tcb_valid(conn)) return 0;
    return ring_used(&tcbs[conn].rx_buf);
}

u32 tcp_writable(tcp_conn_t conn) {
    if (!tcb_valid(conn)) return 0;
    return ring_free(&tcbs[conn].tx_buf);
}
