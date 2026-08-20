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
#include "nic.h"
#include "dma.h"
#include "simd.h"
#include "uart.h"
#include "timer.h"
#include "highmem.h"
#include "dtrace.h"

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
#define TIME_WAIT_MS        1500
#define CLOSE_STATE_MS      1500
#define MAX_RETRIES         8

#define LISTEN_BACKLOG      64
#define TCP_DMA_COPY_THRESHOLD 256U
#define TCP_UART_DIAG_VERBOSE 0

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

static inline void tcp_memcpy_accel(void *dst, const void *src, u32 len)
{
    /* TCP ring data may be consumed in the same call chain by tcp_output().
     * Keep this copy synchronous and CPU-visible; the DMA path still needs a
     * standalone self-test before it can be trusted for hot TX buffers. */
    simd_memcpy(dst, src, len);
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
    tcp_memcpy_accel(r->data + idx, s, first);
    if (len > first)
        tcp_memcpy_accel(r->data, s + first, len - first);
    dmb(); /* payload visible before producer index update */
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
    dmb(); /* consume producer index before reading payload */
    tcp_memcpy_accel(d, r->data + idx, first);
    if (len > first)
        tcp_memcpy_accel(d + first, r->data, len - first);
    dmb(); /* payload consumed before consumer index update */
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
    u32 idx = pos & (TCP_BUF_SIZE - 1);
    u32 first = TCP_BUF_SIZE - idx;
    if (first > len) first = len;
    dmb(); /* observe producer writes before peeking */
    tcp_memcpy_accel(d, r->data + idx, first);
    if (len > first)
        tcp_memcpy_accel(d + first, r->data, len - first);
    return len;
}

/* Copy from ring at offset without temporary segment buffer. */
static u32 ring_copy_from_offset(const struct ring_buf *r, u32 offset, void *dst, u32 len) {
    u32 avail = ring_used(r);
    if (offset >= avail) return 0;
    if (len > avail - offset) len = avail - offset;
    u8 *d = (u8 *)dst;
    u32 pos = r->tail + offset;
    u32 idx = pos & (TCP_BUF_SIZE - 1);
    u32 first = TCP_BUF_SIZE - idx;
    if (first > len) first = len;
    dmb();
    tcp_memcpy_accel(d, r->data + idx, first);
    if (len > first)
        tcp_memcpy_accel(d + first, r->data, len - first);
    return len;
}

static const u8 *ring_linear_ptr_at(const struct ring_buf *r, u32 offset, u32 *contig_out)
{
    u32 avail = ring_used(r);
    if (offset >= avail) return NULL;
    u32 pos = r->tail + offset;
    u32 idx = pos & (TCP_BUF_SIZE - 1);
    u32 contig = TCP_BUF_SIZE - idx;
    u32 rem = avail - offset;
    if (contig > rem) contig = rem;
    if (contig_out) *contig_out = contig;
    dmb();
    return &r->data[idx];
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
    nic_iface_t iface;
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
    u64 rto_deadline;   /* monotonic ms when we should retransmit */
    u32 retries;

    /* RTT estimation (Jacobson/Karn) */
    u64 rtt_seq;        /* seq# being timed */
    u64 rtt_start;      /* monotonic ms when that segment was sent */
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
        nic_iface_t iface;
        u32 irs;        /* client ISN */
        u32 iss;        /* our ISN (from SYN cookie) */
    } pending[LISTEN_BACKLOG];

    /* Intrusive links for the O(1) connection table (see below). */
    i32 hash_next;      /* next tcb index in this hash bucket, -1 = end */
    i32 free_next;      /* next free tcb index when on the free list, -1 = end */
};

/* ── Scaled connection table ──────────────────────────────────────────
 * PIOS is network-first: the connection table is sized for tens of thousands
 * of concurrent connections and lives in high RAM (highmem_alloc), NOT .bss.
 * The per-packet hot path (tcb_find) is O(1) via an intrusive hash on the
 * 4-tuple; allocation is O(1) via a free list; listeners are tracked in a tiny
 * separate registry so tcb_find_listen stays O(listeners). If highmem is
 * unavailable (e.g. QEMU/probe-fail) we fall back to a small static table so
 * the board still boots. TCP_MAX_CONNECTIONS (tcp.h) is unchanged — it remains
 * the *snapshot/display* cap used by external stack arrays. */
#define TCP_TABLE_TARGET    16384U   /* highmem table capacity (network-first) */
#define TCP_TABLE_FALLBACK  128U     /* static .bss table when highmem is
                                      * unavailable (QEMU has no highmem, so this
                                      * IS the live table there). Must be large
                                      * enough for all listeners (:80/:443/:2323/
                                      * admin :8081/:8082...) PLUS concurrent
                                      * connections, or QEMU can't accept HTTP.
                                      * .bss is NOLOAD so this costs ZERO binary
                                      * size — only runtime RAM. */
#define TCP_MAX_LISTENERS   32U

static struct tcb  tcbs_fallback[TCP_TABLE_FALLBACK];
static i32         hash_fallback[256];
static struct tcb *tcbs;             /* points at fallback or highmem array */
static i32        *tcb_hash;         /* hash bucket heads (index, -1 = empty) */
static u32         tcb_capacity;     /* live table capacity */
static u32         tcb_hash_mask;    /* bucket count - 1 (power of two) */
static i32         tcb_free_head = -1;
static u32         tcb_inuse;        /* live (non-free) tcb count, diag */
static i32         tcp_listeners[TCP_MAX_LISTENERS];
static u32         tcp_listener_count;

static inline u32 tcb_hash_key(u16 local_port, u32 remote_ip, u16 remote_port,
                               nic_iface_t iface) {
    u32 h = (u32)local_port * 2654435761U;
    h ^= remote_ip * 2246822519U;
    h ^= ((u32)remote_port << 16) * 3266489917U;
    h ^= (u32)iface * 0x9E3779B9U;
    h ^= h >> 15;
    return h & tcb_hash_mask;
}
static u16 tcp_ip_id;
static tcp_diag_t tcp_diag_counts;

static u64 tcp_now_ms(void)
{
    return timer_monotonic_ms();
}

static void tcp_diag_active_tuple(const struct tcb *t)
{
    if (!t)
        return;
    tcp_diag_counts.active_last_local_ip = t->local_ip;
    tcp_diag_counts.active_last_remote_ip = t->remote_ip;
    tcp_diag_counts.active_last_local_port = t->local_port;
    tcp_diag_counts.active_last_remote_port = t->remote_port;
    tcp_diag_counts.active_last_state = t->state;
    tcp_diag_counts.active_last_retries = t->retries;
}

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

static i32 tcb_index(const struct tcb *t) {
    return (i32)(t - tcbs);
}

/* Insert an active (4-tuple-bearing) tcb into the hash. Call after the 4-tuple
 * + state are set in connect/accept. Listeners do NOT go in this hash. */
static void tcb_hash_insert(struct tcb *t) {
    u32 b = tcb_hash_key(t->local_port, t->remote_ip, t->remote_port,
                        t->iface);
    t->hash_next = tcb_hash[b];
    tcb_hash[b] = tcb_index(t);
}

/* Remove a tcb from the hash (uses its current 4-tuple to find the bucket). */
static void tcb_hash_remove(struct tcb *t) {
    u32 b = tcb_hash_key(t->local_port, t->remote_ip, t->remote_port,
                        t->iface);
    i32 idx = tcb_index(t);
    i32 cur = tcb_hash[b];
    i32 prev = -1;
    while (cur >= 0) {
        if (cur == idx) {
            if (prev < 0) tcb_hash[b] = t->hash_next;
            else          tcbs[prev].hash_next = t->hash_next;
            t->hash_next = -1;
            return;
        }
        prev = cur;
        cur = tcbs[cur].hash_next;
    }
}

static struct tcb *tcb_find(u32 local_port, u32 remote_ip, u16 remote_port,
                            nic_iface_t iface) {
    u32 b = tcb_hash_key((u16)local_port, remote_ip, remote_port, iface);
    for (i32 cur = tcb_hash[b]; cur >= 0; cur = tcbs[cur].hash_next) {
        struct tcb *t = &tcbs[cur];
        if (t->local_port == local_port &&
            t->remote_ip == remote_ip &&
            t->remote_port == remote_port &&
            t->iface == iface)
            return t;
    }
    return NULL;
}

static void tcp_listener_register(i32 idx) {
    if (tcp_listener_count < TCP_MAX_LISTENERS)
        tcp_listeners[tcp_listener_count++] = idx;
}

static void tcp_listener_unregister(i32 idx) {
    for (u32 i = 0; i < tcp_listener_count; i++) {
        if (tcp_listeners[i] == idx) {
            tcp_listeners[i] = tcp_listeners[--tcp_listener_count];
            return;
        }
    }
}

static struct tcb *tcb_find_listen(u16 port) {
    for (u32 i = 0; i < tcp_listener_count; i++) {
        struct tcb *t = &tcbs[tcp_listeners[i]];
        if (t->state == TCP_LISTEN && t->local_port == port)
            return t;
    }
    return NULL;
}

static struct tcb *tcb_alloc(void) {
    if (tcb_free_head < 0)
        return NULL;
    i32 idx = tcb_free_head;
    struct tcb *t = &tcbs[idx];
    tcb_free_head = t->free_next;
    t->free_next = -1;
    tcb_inuse++;
    return t;
}

static bool tcb_valid(tcp_conn_t c) {
    return c >= 0 && (u32)c < tcb_capacity && tcbs[c].state != TCP_CLOSED;
}

#if TCP_UART_DIAG_VERBOSE
static void tcp_log_ip(u32 ip)
{
    uart_hex((ip >> 24) & 0xFF); uart_putc('.');
    uart_hex((ip >> 16) & 0xFF); uart_putc('.');
    uart_hex((ip >> 8) & 0xFF);  uart_putc('.');
    uart_hex(ip & 0xFF);
}
#endif

static void tcp_log_established(const struct tcb *t, const char *kind)
{
#if TCP_UART_DIAG_VERBOSE
    uart_puts("[tcp] ESTABLISHED ");
    uart_puts(kind);
    uart_puts(" local=");
    tcp_log_ip(t->local_ip);
    uart_putc(':');
    uart_hex(t->local_port);
    uart_puts(" remote=");
    tcp_log_ip(t->remote_ip);
    uart_putc(':');
    uart_hex(t->remote_port);
    uart_putc('\n');
#else
    (void)t;
    (void)kind;
#endif
}

/* Ephemeral port allocator */
static u16 next_ephemeral = 49152;
static u16 alloc_port(void) {
    u16 p = next_ephemeral++;
    if (next_ephemeral < 49152) next_ephemeral = 49152;
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
    seed.ts     = (u32)tcp_now_ms();
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

static u32 csum_add_bytes(u32 sum, const void *data, u32 len)
{
    const u8 *p = (const u8 *)data;
    while (len >= 2) {
        sum += ((u16)p[0] << 8) | p[1];
        p += 2;
        len -= 2;
    }
    if (len)
        sum += ((u16)p[0] << 8);
    return sum;
}

static u16 csum_fold(u32 sum)
{
    while (sum >> 16)
        sum = (sum & 0xFFFF) + (sum >> 16);
    return (u16)~sum;
}

static u32 tcp_pseudo_sum(u32 src_ip, u32 dst_ip, u32 tcp_len)
{
    u32 sum = 0;
    sum += (src_ip >> 16) & 0xFFFF;
    sum += src_ip & 0xFFFF;
    sum += (dst_ip >> 16) & 0xFFFF;
    sum += dst_ip & 0xFFFF;
    sum += IP_PROTO_TCP;
    sum += (u16)tcp_len;
    return sum;
}

static u16 tcp_checksum(u32 src_ip, u32 dst_ip,
                        const void *tcp_data, u32 tcp_len) {
    u32 sum = tcp_pseudo_sum(src_ip, dst_ip, tcp_len);
    sum = csum_add_bytes(sum, tcp_data, tcp_len);
    return htons(csum_fold(sum));
}

static u16 tcp_checksum_split(u32 src_ip, u32 dst_ip,
                              const void *tcp_hdr, u32 tcp_hdr_len,
                              const void *payload, u32 payload_len)
{
    u32 sum = tcp_pseudo_sum(src_ip, dst_ip, tcp_hdr_len + payload_len);
    sum = csum_add_bytes(sum, tcp_hdr, tcp_hdr_len);
    sum = csum_add_bytes(sum, payload, payload_len);
    return htons(csum_fold(sum));
}

/* ================================================================== */
/*  TX path: build Ethernet + IP + TCP and send                        */
/* ================================================================== */

static void tcp_send_segment(struct tcb *t, u8 flags,
                             const void *data, u32 data_len) {
    /* Guard against u16 overflow: max frame 2048 minus IP+TCP headers */
    if (data_len > 2048 - 54) return;

    /* Resolve destination MAC */
    const u8 *dst_mac = net_resolve_mac_on(t->iface, t->remote_ip);
    if (unlikely(!dst_mac)) {
        tcp_diag_counts.tx_no_mac++;
        return;
    }

    u16 tcp_len = TCP_HDR_SIZE + data_len;
    u16 ip_total = IP_HDR_SIZE + tcp_len;
    u32 frame_len = ETH_HDR_SIZE + ip_total;
    if (frame_len > MAX_FRAME) return;

    /* Ethernet */
    struct eth_hdr *eth = (struct eth_hdr *)tx_frame;
    simd_memcpy(eth->dst, dst_mac, 6);
    u8 local_mac[6];
    net_get_mac_for(t->iface, local_mac);
    simd_memcpy(eth->src, local_mac, 6);
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

    bool tx_offload_window = nic_tx_checksum_offload_enabled_for(t->iface) &&
                             data_len > 0;
    bool need_pad = frame_len < MIN_FRAME;
    if (need_pad) frame_len = MIN_FRAME;

    if (!tx_offload_window) {
        if (data_len > 0)
            tcp->checksum = tcp_checksum_split(t->local_ip, t->remote_ip,
                                               tcp, TCP_HDR_SIZE, data, data_len);
        else
            tcp->checksum = tcp_checksum(t->local_ip, t->remote_ip, tcp, tcp_len);
    }

    if (data_len == 0) {
        tcp_diag_counts.tx_segments++;
        if (!nic_send_on(t->iface, tx_frame, frame_len)) tcp_diag_counts.tx_send_fail++;
        return;
    }
    if (need_pad) {
        tcp_memcpy_accel(tx_frame + TCP_OVERHEAD, data, data_len);
        tcp_diag_counts.tx_segments++;
        if (!nic_send_on(t->iface, tx_frame, frame_len)) tcp_diag_counts.tx_send_fail++;
        return;
    }
    tcp_diag_counts.tx_segments++;
    if (!nic_send_parts_on(t->iface, tx_frame, TCP_OVERHEAD, data, data_len))
        tcp_diag_counts.tx_send_fail++;
}

static void tcp_send_segment_from_txbuf(struct tcb *t, u8 flags,
                                        u32 tx_off, u32 data_len) {
    /* Guard against u16 overflow: max frame 2048 minus IP+TCP headers */
    if (data_len > 2048 - 54) return;

    const u8 *dst_mac = net_resolve_mac_on(t->iface, t->remote_ip);
    if (unlikely(!dst_mac)) {
        tcp_diag_counts.tx_no_mac++;
        return;
    }

    u16 tcp_len = TCP_HDR_SIZE + data_len;
    u16 ip_total = IP_HDR_SIZE + tcp_len;
    u32 frame_len = ETH_HDR_SIZE + ip_total;
    if (frame_len > MAX_FRAME) return;

    struct eth_hdr *eth = (struct eth_hdr *)tx_frame;
    simd_memcpy(eth->dst, dst_mac, 6);
    u8 local_mac[6];
    net_get_mac_for(t->iface, local_mac);
    simd_memcpy(eth->src, local_mac, 6);
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
    ip->src_ip     = htonl(t->local_ip);
    ip->dst_ip     = htonl(t->remote_ip);
    ip->checksum   = simd_checksum(ip, IP_HDR_SIZE);

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

    bool tx_offload_window = nic_tx_checksum_offload_enabled_for(t->iface) &&
                             data_len > 0;
    bool need_pad = frame_len < MIN_FRAME;
    if (need_pad) frame_len = MIN_FRAME;

    if (data_len == 0) {
        if (!tx_offload_window)
            tcp->checksum = tcp_checksum(t->local_ip, t->remote_ip, tcp, tcp_len);
        tcp_diag_counts.tx_segments++;
        if (!nic_send_on(t->iface, tx_frame, frame_len)) tcp_diag_counts.tx_send_fail++;
        return;
    }

    u32 contig = 0;
    const u8 *lin = ring_linear_ptr_at(&t->tx_buf, tx_off, &contig);
    if (!need_pad && lin && contig >= data_len) {
        if (!tx_offload_window)
            tcp->checksum = tcp_checksum_split(t->local_ip, t->remote_ip,
                                               tcp, TCP_HDR_SIZE, lin, data_len);
        tcp_diag_counts.tx_segments++;
        if (!nic_send_parts_on(t->iface, tx_frame, TCP_OVERHEAD, lin, data_len))
            tcp_diag_counts.tx_send_fail++;
        return;
    }

    ring_copy_from_offset(&t->tx_buf, tx_off, tx_frame + TCP_OVERHEAD, data_len);
    if (!tx_offload_window)
        tcp->checksum = tcp_checksum(t->local_ip, t->remote_ip, tcp, tcp_len);
    tcp_diag_counts.tx_segments++;
    if (!nic_send_on(t->iface, tx_frame, frame_len)) tcp_diag_counts.tx_send_fail++;
}

/* Send a raw RST/ACK without a TCB (for rejecting unexpected segments) */
static void tcp_send_rst(nic_iface_t iface, u32 src_ip, u32 dst_ip,
                         u16 src_port, u16 dst_port, u32 seq, u32 ack) {
    const u8 *dst_mac = net_resolve_mac_on(iface, dst_ip);
    if (!dst_mac) { tcp_diag_counts.tx_no_mac++; return; }

    u16 ip_total = IP_HDR_SIZE + TCP_HDR_SIZE;
    u32 frame_len = ETH_HDR_SIZE + ip_total;

    struct eth_hdr *eth = (struct eth_hdr *)tx_frame;
    simd_memcpy(eth->dst, dst_mac, 6);
    u8 local_mac[6];
    net_get_mac_for(iface, local_mac);
    simd_memcpy(eth->src, local_mac, 6);
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
    tcp_diag_counts.tx_segments++;
    if (!nic_send_on(iface, tx_frame, frame_len)) tcp_diag_counts.tx_send_fail++;
}

/* Send a SYN-ACK with SYN cookie ISN (no TCB needed) */
static void tcp_send_synack_cookie(nic_iface_t iface, u32 remote_ip, u16 remote_port,
                                   u16 local_port, u32 their_seq,
                                   u32 cookie_isn) {
    const u8 *dst_mac = net_resolve_mac_on(iface, remote_ip);
    if (!dst_mac) { tcp_diag_counts.tx_no_mac++; return; }

    u16 ip_total = IP_HDR_SIZE + TCP_HDR_SIZE;
    u32 frame_len = ETH_HDR_SIZE + ip_total;

    struct eth_hdr *eth = (struct eth_hdr *)tx_frame;
    simd_memcpy(eth->dst, dst_mac, 6);
    u8 local_mac[6];
    net_get_mac_for(iface, local_mac);
    simd_memcpy(eth->src, local_mac, 6);
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
    ip->src_ip     = htonl(net_get_our_ip_for(iface));
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

    tcp->checksum = tcp_checksum(net_get_our_ip_for(iface), remote_ip,
                                 tcp, TCP_HDR_SIZE);

    if (frame_len < MIN_FRAME) frame_len = MIN_FRAME;
    tcp_diag_counts.tx_segments++;
    if (nic_send_on(iface, tx_frame, frame_len))
        tcp_diag_counts.synack_sent++;
    else {
        tcp_diag_counts.tx_send_fail++;
#if TCP_UART_DIAG_VERBOSE
        uart_puts("[tcp] SYNACK nic_send failed\n");
#endif
    }
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
    t->rto_deadline = tcp_now_ms() + t->rto_ms;
}

static void tcp_output(struct tcb *t) {
    if (t->state != TCP_ESTABLISHED &&
        t->state != TCP_CLOSE_WAIT)
        return;

    bool sent_any = false;
    for (u32 burst = 0; burst < 4; burst++) {
        u32 eff_wnd = min32(t->snd_wnd, t->cwnd);
        u32 in_flight = t->snd_nxt - t->snd_una;
        if (in_flight >= eff_wnd) break;
        u32 can_send = eff_wnd - in_flight;

        u32 buffered = ring_used(&t->tx_buf);
        u32 unsent_off = t->snd_nxt - t->snd_una;
        if (unsent_off >= buffered) break;
        u32 unsent = buffered - unsent_off;

        u32 to_send = min32(unsent, can_send);
        to_send = min32(to_send, TCP_MSS);
        if (to_send == 0) break;

        if (!t->rtt_active) {
            t->rtt_seq    = t->snd_nxt;
            t->rtt_start  = tcp_now_ms();
            t->rtt_active = true;
        }

        u8 flags = TCP_ACK;
        if (to_send >= unsent)
            flags |= TCP_PSH;

        tcp_send_segment_from_txbuf(t, flags, unsent_off, to_send);
        t->snd_nxt += to_send;
        sent_any = true;

        if (to_send < TCP_MSS)
            break;
    }
    if (sent_any)
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

    u32 saved_nxt = t->snd_nxt;
    t->snd_nxt = t->snd_una;
    tcp_send_segment_from_txbuf(t, TCP_ACK | TCP_PSH, 0, len);
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
    u32 st = t->state;
    if (st == TCP_CLOSED)
        return;   /* already free — never double-link onto the free list */
    i32 idx = tcb_index(t);
    if (st == TCP_LISTEN)
        tcp_listener_unregister(idx);
    else
        tcb_hash_remove(t);   /* active conn: unlink from the 4-tuple hash */
    simd_zero(t, sizeof(struct tcb));   /* state -> TCP_CLOSED (0) */
    t->hash_next = -1;
    t->free_next = tcb_free_head;
    tcb_free_head = idx;
    if (tcb_inuse)
        tcb_inuse--;
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

static u32 tcp_rx_ingest_in_order(struct tcb *t, u32 seg_seq, const u8 *data, u32 data_len)
{
    if (data_len == 0)
        return 0;
    if (seg_seq != t->rcv_nxt) {
        if (seq_gt(seg_seq, t->rcv_nxt))
            tcp_diag_counts.rx_out_of_order++;
        return 0;
    }
    /* Atomic accept: never PARTIALLY buffer a segment. If the whole segment
     * does not fit in the receive ring, drop it entirely and leave rcv_nxt on
     * the segment boundary. Advancing rcv_nxt by a partial amount (what a
     * truncating ring_write would do) desyncs us from the sender's MSS
     * boundaries: the sender keeps retransmitting full segments at the old
     * boundary, every one now has seg_seq != rcv_nxt, so none are ever ingested
     * and the transfer freezes forever mid-segment (observed: a 2.5MB OTA froze
     * at rcv_nxt=18224 = 12*1448 + 848, i.e. a 1448B segment truncated to 848).
     * The advertised window (rcv_wnd = ring_free, sent on every ACK) throttles
     * the sender; this full-fit check is the safety net for the lag between
     * advertising the window and the sender's in-flight data arriving. The
     * dropped segment is dup-ACKed by handle_established, so the sender
     * retransmits it whole once the window reopens. */
    if (ring_free(&t->rx_buf) < data_len) {
        tcp_diag_counts.rx_no_space++;
        return 0;
    }
    u32 written = ring_write(&t->rx_buf, data, data_len);
    t->rcv_nxt += written;
    t->rcv_wnd = ring_free(&t->rx_buf);
    return written;
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
                u64 rtt = tcp_now_ms() - t->rtt_start;
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

    /* Process data. ALWAYS ACK a data-bearing segment: an in-order segment is
     * ingested and ACKed (advancing rcv_nxt); a retransmit whose tail overlaps
     * the window (seg_seq < rcv_nxt <= seg_seq+seg_len-1) passes seq_acceptable
     * but is NOT ingested (seg_seq != rcv_nxt) — it MUST still get a duplicate
     * ACK so the sender learns our current rcv_nxt + window and resyncs. Without
     * this, the peer retransmits forever with exponential backoff and a bulk
     * inbound transfer deadlocks (OTA stream froze ~18KB: MAC delivered 561
     * frames but only 94 were ACKed). The ACK also re-advertises the receive
     * window, so a window that reopened after the app drained the ring is
     * promptly communicated. */
    if (data_len > 0) {
        tcp_rx_ingest_in_order(t, seg_seq, data, data_len);
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
               const u8 *payload, u32 payload_len, bool checksum_trusted,
               nic_iface_t ingress_iface) {
    if ((ingress_iface != NIC_IFACE_WIRED &&
         ingress_iface != NIC_IFACE_WIFI) ||
        !net_interface_configured(ingress_iface))
        return;
    if (unlikely(payload_len < TCP_HDR_SIZE)) {
        tcp_diag_counts.in_short++;
        return;
    }

    const struct tcp_hdr *tcp = (const struct tcp_hdr *)payload;

    /* Verify TCP checksum unless explicitly trusted by RX offload window. */
    if (!checksum_trusted &&
        unlikely(tcp_checksum(src_ip, dst_ip, payload, payload_len) != 0)) {
        tcp_diag_counts.bad_checksum++;
        if ((tcp_diag_counts.bad_checksum & 0x0F) == 1) {
#if TCP_UART_DIAG_VERBOSE
            uart_puts("[tcp] drop bad checksum\n");
#endif
        }
        return;
    }

    u16 src_port = ntohs(tcp->src_port);
    u16 dst_port = ntohs(tcp->dst_port);
    u32 seg_seq  = ntohl(tcp->seq);
    u32 seg_ack  = ntohl(tcp->ack);
    u8  flags    = tcp->flags;
    u16 seg_wnd  = ntohs(tcp->window);

    u32 hdr_len = (tcp->data_off >> 4) * 4;
    if (unlikely(hdr_len < TCP_HDR_SIZE || hdr_len > payload_len)) {
        tcp_diag_counts.bad_header++;
        return;
    }

    const u8 *seg_data = payload + hdr_len;
    u32 data_len = payload_len - hdr_len;

    /* Look up existing connection */
    struct tcb *t = tcb_find(dst_port, src_ip, src_port, ingress_iface);

    /* ---- LISTEN handling (SYN cookies) ---- */
    if (!t) {
        struct tcb *listen = tcb_find_listen(dst_port);
        if (!listen) {
            tcp_diag_counts.no_listener++;
            /* No matching socket — send RST */
            if (!(flags & TCP_RST)) {
                if (flags & TCP_ACK) {
                    tcp_send_rst(ingress_iface, dst_ip, src_ip, dst_port, src_port,
                                 seg_ack, 0);
                } else {
                    tcp_send_rst(ingress_iface, dst_ip, src_ip, dst_port, src_port,
                                 0, seg_seq + data_len +
                                 ((flags & TCP_SYN) ? 1 : 0) +
                                 ((flags & TCP_FIN) ? 1 : 0));
                }
            }
            return;
        }

        if (flags & TCP_SYN) {
            tcp_diag_counts.syn_seen++;
            /* Respond with SYN-ACK using SYN cookie ISN — no TCB allocated */
            u32 cookie = make_syn_cookie(dst_port, src_ip, src_port, seg_seq);
            tcp_send_synack_cookie(ingress_iface, src_ip, src_port, dst_port,
                                   seg_seq, cookie);
            return;
        }

        if (flags & TCP_ACK) {
            tcp_diag_counts.ack_cookie_seen++;
            /* Validate SYN cookie: the ACK should be cookie+1 */
            u32 their_seq = seg_ack - 1;  /* this was our ISN (cookie) */
            u32 their_iss = seg_seq - 1;  /* their ISN was seg_seq - 1 */
            if (!validate_syn_cookie(dst_port, src_ip, src_port,
                                     their_iss, their_seq)) {
                /* Ignore stale/stray ACKs in LISTEN. Browsers can retransmit
                 * final ACK+HTTP data while core0 has not accepted yet; an RST
                 * here kills an otherwise recoverable connection attempt.
                 * Do not count these as bad cookies: on noisy networks they
                 * are expected background churn and are intentionally ignored. */
                return;
            }

            /* Valid cookie — queue in listen backlog */
            for (u32 i = 0; i < listen->pending_count; i++) {
                if (listen->pending[i].remote_ip == src_ip &&
                    listen->pending[i].remote_port == src_port &&
                    listen->pending[i].iface == ingress_iface)
                    return;
            }
            if (listen->pending_count < LISTEN_BACKLOG) {
                tcp_diag_counts.pending_queued++;
                u32 idx = listen->pending_count++;
                listen->pending[idx].remote_ip   = src_ip;
                listen->pending[idx].remote_port  = src_port;
                listen->pending[idx].iface        = ingress_iface;
                listen->pending[idx].irs          = their_iss;
                listen->pending[idx].iss          = their_seq;
            } else {
                tcp_diag_counts.pending_full++;
            }
            return;
        }

        /* Any other packet to LISTEN socket: drop */
        return;
    }

    /* ---- SYN_SENT state ---- */
    if (t->state == TCP_SYN_SENT) {
        tcp_diag_active_tuple(t);
        if (flags & TCP_ACK) {
            if (seg_ack != t->iss + 1) {
                tcp_diag_counts.active_bad_ack++;
                if (!(flags & TCP_RST))
                    tcp_send_rst(t->iface, t->local_ip, t->remote_ip,
                                 t->local_port, t->remote_port, seg_ack, 0);
                return;
            }
        }
        if (flags & TCP_RST) {
            tcp_diag_counts.active_rst++;
            if (flags & TCP_ACK) {
                tcb_reset(t);
            }
            return;
        }
        if (flags & TCP_SYN) {
            if (flags & TCP_ACK)
                tcp_diag_counts.active_synack_seen++;
            t->irs     = seg_seq;
            t->rcv_nxt = seg_seq + 1;
            t->snd_wnd = seg_wnd;
            if (flags & TCP_ACK) {
                t->snd_una = seg_ack;
                t->state   = TCP_ESTABLISHED;
                tcp_send_ack(t);
                cc_init(t);
                tcp_diag_counts.active_established++;
                tcp_diag_active_tuple(t);
                tcp_log_established(t, "active");
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
                tcp_log_established(t, "passive");
            } else {
                tcp_send_rst(t->iface, t->local_ip, t->remote_ip,
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
        /* Accept incoming data (direct ingest fast path) */
        (void)tcp_rx_ingest_in_order(t, seg_seq, seg_data, data_len);
        /* Check if our FIN has been ACKed */
        if (t->fin_sent && seg_ack == t->fin_seq + 1) {
            t->state = TCP_FIN_WAIT_2;
        }
        if (flags & TCP_FIN) {
            t->rcv_nxt++;
            tcp_send_ack(t);
            if (t->state == TCP_FIN_WAIT_2) {
                t->state     = TCP_TIME_WAIT;
                t->tw_expiry = tcp_now_ms() + TIME_WAIT_MS;
            } else {
                /* Simultaneous close: FIN_WAIT_1 → CLOSING */
                t->state = TCP_CLOSING;
            }
        } else {
            tcp_send_ack(t);
        }
        break;

    case TCP_FIN_WAIT_2:
        /* Accept data (direct ingest fast path) */
        (void)tcp_rx_ingest_in_order(t, seg_seq, seg_data, data_len);
        if (flags & TCP_FIN) {
            t->rcv_nxt++;
            t->state     = TCP_TIME_WAIT;
            t->tw_expiry = tcp_now_ms() + TIME_WAIT_MS;
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
                t->tw_expiry = tcp_now_ms() + TIME_WAIT_MS;
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
            t->tw_expiry = tcp_now_ms() + TIME_WAIT_MS;
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
    u64 now = tcp_now_ms();

    for (u32 i = 0; i < tcb_capacity; i++) {
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

        if ((t->state == TCP_FIN_WAIT_1 || t->state == TCP_FIN_WAIT_2 ||
             t->state == TCP_CLOSING || t->state == TCP_LAST_ACK) &&
            t->tw_expiry != 0 && now >= t->tw_expiry) {
            tcb_reset(t);
            continue;
        }

        /* Retransmit timer */
        if (t->rto_deadline != 0 && now >= t->rto_deadline) {
            t->retries++;
            if (t->retries > MAX_RETRIES) {
                /* Connection failed */
                if (t->state == TCP_SYN_SENT) {
                    tcp_diag_counts.active_timeout++;
                    tcp_diag_active_tuple(t);
                }
#if TCP_UART_DIAG_VERBOSE
                uart_puts("[tcp] conn timeout\n");
#endif
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
    /* Pick the connection table. Network-first: prefer a large highmem table
     * (tens of thousands of connections) with an O(1) hash; fall back to a
     * small static .bss table if highmem is unavailable so the board always
     * boots and serves. The hot path (tcb_find per packet) is O(1) regardless. */
    struct highmem_status hm;
    highmem_status(&hm);
    struct tcb *table = tcbs_fallback;
    i32 *hash = hash_fallback;
    u32 cap = TCP_TABLE_FALLBACK;
    u32 nbuckets = (u32)(sizeof(hash_fallback) / sizeof(hash_fallback[0]));
    if (tcbs != NULL) {
        /* Re-init (e.g. network reconfigure): reuse the already-allocated table
         * — highmem_alloc never frees, so do not leak a fresh ~150MB table. */
        table = tcbs;
        hash = tcb_hash;
        cap = tcb_capacity;
        nbuckets = tcb_hash_mask + 1U;
    } else if (hm.ready) {
        u32 nb = 1;
        while (nb < TCP_TABLE_TARGET * 2U)
            nb <<= 1;
        /* Allocate the SMALL hash first; only then the large table. highmem is a
         * bump allocator that never frees, so if the (hundreds-of-MB) table
         * allocation failed AFTER a successful table-then-hash order, that whole
         * table would be permanently leaked. Allocating the tiny hash first
         * bounds the worst-case leak to the hash itself (and if the hash fails we
         * never attempt the table) before falling back to the static table. */
        i32 *hh = (i32 *)highmem_alloc((u64)nb * sizeof(i32), 64);
        struct tcb *ht = hh
            ? (struct tcb *)highmem_alloc((u64)TCP_TABLE_TARGET * sizeof(struct tcb), 64)
            : NULL;
        if (ht && hh) {
            table = ht;
            hash = hh;
            cap = TCP_TABLE_TARGET;
            nbuckets = nb;
        }
    }
    tcbs = table;
    tcb_hash = hash;
    tcb_capacity = cap;
    tcb_hash_mask = nbuckets - 1U;

    /* Empty all hash buckets. */
    for (u32 i = 0; i < nbuckets; i++)
        hash[i] = -1;

    /* Mark every slot CLOSED and thread the free list 0 -> 1 -> ... -> -1.
     * Other tcb fields stay garbage until tcb_alloc's caller simd_zero()s the
     * slot, so we avoid zeroing the whole (up to ~134MB) highmem table. */
    for (u32 i = 0; i < cap; i++) {
        table[i].state     = TCP_CLOSED;
        table[i].hash_next = -1;
        table[i].free_next = (i + 1U < cap) ? (i32)(i + 1U) : -1;
    }
    tcb_free_head = 0;
    tcb_inuse = 0;
    tcp_listener_count = 0;


    /* Generate SYN cookie secret from timer jitter */
    u32 t0 = (u32)tcp_now_ms();
    syn_secret = hw_crc32c(&t0, sizeof(t0));
    syn_secret ^= 0xA5C39E17;  /* mix in constant */

    tcp_ip_id = (u16)(syn_secret & 0xFFFF);
    next_ephemeral = 49152 + (u16)((syn_secret >> 16) & 0x3FFFU);

#if TCP_UART_DIAG_VERBOSE
    uart_puts("[tcp] init ok\n");
#endif
}

tcp_conn_t tcp_connect_on(nic_iface_t iface, u32 dst_ip, u16 dst_port) {
    if (iface == NIC_IFACE_ANY)
        iface = nic_default_iface();
    if (!net_interface_configured(iface))
        return -1;
    struct tcb *t = tcb_alloc();
    if (!t) return -1;

    u16 src_port = alloc_port();

    simd_zero(t, sizeof(struct tcb));
    t->iface       = iface;
    t->local_ip    = net_get_our_ip_for(t->iface);
    t->remote_ip   = dst_ip;
    t->local_port  = src_port;
    t->remote_port = dst_port;
    t->rcv_wnd     = TCP_DEFAULT_WINDOW;

    t->iss     = generate_isn(t->local_ip, src_port, dst_ip, dst_port);
    t->snd_una = t->iss;
    t->snd_nxt = t->iss;

    rtt_init(t);
    cc_init(t);

    /* Send SYN */
    t->state = TCP_SYN_SENT;
    tcb_hash_insert(t);
    tcp_send_segment(t, TCP_SYN, NULL, 0);
    tcp_diag_counts.active_syn_sent++;
    tcp_diag_active_tuple(t);
    t->snd_nxt = t->iss + 1;
    tcp_arm_rto(t);

    return tcb_index(t);
}

tcp_conn_t tcp_connect(u32 dst_ip, u16 dst_port)
{
    return tcp_connect_on(nic_default_iface(), dst_ip, dst_port);
}

nic_iface_t tcp_iface(tcp_conn_t conn)
{
    return (tcbs && tcb_valid(conn)) ? tcbs[conn].iface : NIC_IFACE_ANY;
}

tcp_conn_t tcp_listen(u16 port) {
    struct tcb *t = tcb_alloc();
    if (!t) return -1;

    simd_zero(t, sizeof(struct tcb));
    t->iface      = NIC_IFACE_ANY;
    t->local_ip   = 0;
    t->local_port = port;
    t->state      = TCP_LISTEN;
    t->rcv_wnd    = TCP_DEFAULT_WINDOW;

    tcp_listener_register(tcb_index(t));
    return tcb_index(t);
}

tcp_conn_t tcp_accept(tcp_conn_t listen_conn) {
    if (!tcb_valid(listen_conn)) return -1;
    struct tcb *lt = &tcbs[listen_conn];
    if (lt->state != TCP_LISTEN || lt->pending_count == 0) return -1;

    /* Do not consume backlog entries if no TCB is available yet. */
    struct tcb *t = tcb_alloc();
    if (!t) return -1;

    /* Pop oldest pending connection */
    u32 remote_ip   = lt->pending[0].remote_ip;
    u16 remote_port = lt->pending[0].remote_port;
    nic_iface_t iface = lt->pending[0].iface;
    u32 irs         = lt->pending[0].irs;
    u32 iss         = lt->pending[0].iss;

    /* Shift backlog */
    for (u32 i = 1; i < lt->pending_count; i++)
        lt->pending[i - 1] = lt->pending[i];
    lt->pending_count--;

    simd_zero(t, sizeof(struct tcb));
    t->iface       = iface;
    t->local_ip    = net_get_our_ip_for(iface);
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
    tcb_hash_insert(t);
    tcp_diag_counts.accepted++;
    tcp_log_established(t, "accepted");
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

/* Re-send an ACK that re-advertises our current receive window. Used to
 * self-heal a lost window-update during a bulk inbound transfer: if the single
 * window-update ACK sent from tcp_read is dropped, the peer waits forever for a
 * window it will never hear about and the (now-empty) rx ring gives tcp_read
 * nothing to drain — so nothing re-triggers the ACK. A caller stalled waiting
 * for more inbound data can call this to keep the window advertised. */
void tcp_advertise_window(tcp_conn_t conn) {
    if (!tcb_valid(conn)) return;
    struct tcb *t = &tcbs[conn];
    if (t->state != TCP_ESTABLISHED) return;
    u32 old_wnd = t->rcv_wnd;
    t->rcv_wnd = ring_free(&t->rx_buf);
    tcp_send_ack(t);
    DTRACE(DTRACE_CAT_TCP, DT_TCP_WNDUPD, (u64)conn, old_wnd, t->rcv_wnd, 0xFFFFFFFFU);
}

u32 tcp_read(tcp_conn_t conn, void *data, u32 len) {
    if (!tcb_valid(conn)) return 0;
    struct tcb *t = &tcbs[conn];
    u32 old_wnd = t->rcv_wnd;
    u32 n = ring_read(&t->rx_buf, data, len);
    t->rcv_wnd = ring_free(&t->rx_buf);
    /* Window-update ACK: when the application drains the receive buffer and the
     * advertised window reopens past one MSS after having been (near-)closed,
     * proactively ACK so the peer resumes immediately. Without this, a bulk
     * inbound transfer (e.g. an OTA upload into the 4KB rx ring) stalls: the
     * window hits 0, the peer enters exponentially-backed-off zero-window
     * probing, and once a probe gap exceeds the admin stall watchdog the board
     * resets the connection (host sees WinError 10054 mid-upload). This is the
     * standard receiver-side window-update every TCP must send. */
    if (n > 0 && t->state == TCP_ESTABLISHED &&
        old_wnd < TCP_MSS && t->rcv_wnd >= TCP_MSS) {
        tcp_send_ack(t);
        DTRACE(DTRACE_CAT_TCP, DT_TCP_WNDUPD, (u64)conn, old_wnd, t->rcv_wnd, n);
    }
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
        t->tw_expiry = tcp_now_ms() + CLOSE_STATE_MS;
        break;
    case TCP_CLOSE_WAIT:
        tcp_send_fin(t);
        t->state = TCP_LAST_ACK;
        t->tw_expiry = tcp_now_ms() + CLOSE_STATE_MS;
        break;
    case TCP_SYN_RECEIVED:
        tcp_send_fin(t);
        t->state = TCP_FIN_WAIT_1;
        t->tw_expiry = tcp_now_ms() + CLOSE_STATE_MS;
        break;
    default:
        break;
    }
}

void tcp_abort(tcp_conn_t conn)
{
    if (conn < 0 || (u32)conn >= tcb_capacity)
        return;
    tcb_reset(&tcbs[conn]);
}

void tcp_purge_port(u16 local_port)
{
    for (u32 i = 0; i < tcb_capacity; i++) {
        struct tcb *t = &tcbs[i];
        if (t->state == TCP_CLOSED)
            continue;
        if (t->local_port != local_port)
            continue;
        if (t->state == TCP_LISTEN) {
            t->pending_count = 0;
        } else {
            tcb_reset(t);
        }
    }
}

u32 tcp_state(tcp_conn_t conn) {
    if (conn < 0 || (u32)conn >= tcb_capacity)
        return TCP_CLOSED;
    return tcbs[conn].state;
}

u32 tcp_readable(tcp_conn_t conn) {
    if (!tcb_valid(conn)) return 0;
    dmb();
    u32 n = ring_used(&tcbs[conn].rx_buf);
    dmb();
    return n;
}

u32 tcp_writable(tcp_conn_t conn) {
    if (!tcb_valid(conn)) return 0;
    dmb();
    u32 n = ring_free(&tcbs[conn].tx_buf);
    dmb();
    return n;
}

u32 tcp_tx_pending(tcp_conn_t conn) {
    if (!tcb_valid(conn)) return 0;
    dmb();
    u32 n = ring_used(&tcbs[conn].tx_buf);
    dmb();
    return n;
}

u32 tcp_active_count(void)
{
    u32 n = 0;
    for (u32 i = 0; i < tcb_capacity; i++) {
        u32 st = tcbs[i].state;
        if (st != TCP_CLOSED && st != TCP_LISTEN)
            n++;
    }
    return n;
}

u32 tcp_snapshot(tcp_snapshot_entry_t *out, u32 max)
{
    if (!out || max == 0)
        return 0;
    u32 n = 0;
    for (u32 i = 0; i < tcb_capacity && n < max; i++) {
        struct tcb *t = &tcbs[i];
        if (t->state == TCP_CLOSED)
            continue;
        dmb();
        out[n].conn = (i32)i;
        out[n].state = t->state;
        out[n].local_ip = t->local_ip;
        out[n].remote_ip = t->remote_ip;
        out[n].local_port = t->local_port;
        out[n].remote_port = t->remote_port;
        out[n].pending_count = t->state == TCP_LISTEN ? t->pending_count : 0;
        out[n].rx_used = ring_used(&t->rx_buf);
        out[n].tx_used = ring_used(&t->tx_buf);
        out[n].retries = t->retries;
        out[n].iface = t->iface;
        n++;
    }
    return n;
}

const tcp_diag_t *tcp_diag(void)
{
    return &tcp_diag_counts;
}

void tcp_table_stats(u32 *capacity, u32 *inuse, u32 *listeners)
{
    if (capacity)  *capacity = tcb_capacity;
    if (inuse)     *inuse = tcb_inuse;
    if (listeners) *listeners = tcp_listener_count;
}
