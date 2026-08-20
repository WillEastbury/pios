/*
 * net.c - Hardened network stack: IPv4 + ICMP + UDP + TCP + ARP
 *
 * Hardened ARP with anti-spoofing. NEON checksums. Rate-limited ICMP.
 * Every ingress byte is validated before processing.
 */

#include "net.h"
#include "arp.h"
#include "tcp.h"
#include "socket.h"
#include "tls.h"
#include "nic.h"
#include "simd.h"
#include "core_env.h"
#include "fifo.h"
#include "core.h"
#include "uart.h"
#include "timer.h"
#include "workq.h"
#include "fb.h"
#include "pcie.h"
#include "mmio.h"
#include "macb.h"
#include "dns.h"

#define NET_ICMP_DIAG_VERBOSE 0

/* ---- Network state ---- */

static u32 our_ip;
static u32 our_gw;
static u32 our_mask;
static u8  our_mac[6];
static nic_iface_t net_current_iface = NIC_IFACE_WIRED;

#define UDP_SUBSCRIBER_MAX 4U
struct udp_subscriber_slot {
    udp_recv_cb cb;
    u64 _pad[7];
} ALIGNED(64);

static udp_recv_cb udp_callback;
static struct udp_subscriber_slot udp_subscribers[UDP_SUBSCRIBER_MAX];
_Static_assert(sizeof(struct udp_subscriber_slot) == 64,
               "UDP subscriber slots must be one cache line");
static net_stats_t stats;

static bool mac_is_zero_6(const u8 *m) {
    return m[0]==0 && m[1]==0 && m[2]==0 && m[3]==0 && m[4]==0 && m[5]==0;
}

/* ---- Static neighbor table (replaces ARP) ---- */

static struct neighbor_entry neighbors[MAX_NEIGHBORS];
static u32 neighbor_count;
static struct net_route_entry routes[NET_ROUTE_MAX];
static u32 route_count;
static u8  gw_mac[6];
static bool gw_mac_set;
static struct net_egress_snapshot egress_trace;

/* ---- ICMP rate limiter ---- */

static u64 icmp_last_tick;
static u64 icmp_min_interval;

/* ---- Frame buffers ---- */

static u8 rx_frame[2048] ALIGNED(64);
static u8 tx_frame[2048] ALIGNED(64);
/* Drain up to a full RX ring (NUM_RX=32) per net_poll call. The MACB RX ring
 * is 32 descriptors deep; draining only 1 frame/poll (the old value) could not
 * keep pace with bursty load, so the ring backed up (observed rx_owned->21/32)
 * and frames waited seconds before being processed -> multi-second ICMP/HTTP
 * latency that looked like a wedge. Draining the whole ring per poll keeps the
 * ring empty under load while staying bounded by the ring depth. Must be >=
 * NUM_RX (macb.c) so one poll can clear a full ring; the deeper ring (896,
 * bumped from 512 on 2026-07-17 for a wrap-boundary diagnostic -- see macb.c
 * NUM_RX comment) plus the macb_rx_recover() overrun safety net is what keeps
 * concurrent-connection bursts from wedging the NIC. */
#define NET_RX_BURST_MAX 896U
#define NET_FIFO_BURST_MAX 4U
#define NET_MAX_MULTICAST_MACS 8U

static u8 multicast_macs[NET_MAX_MULTICAST_MACS][6];
static u32 multicast_count;

struct net_iface_state {
    bool configured;
    u32 ip;
    u32 gw;
    u32 mask;
    u8 mac[6];
    struct neighbor_entry neighbors[MAX_NEIGHBORS];
    u32 neighbor_count;
    struct net_route_entry routes[NET_ROUTE_MAX];
    u32 route_count;
    u8 gw_mac[6];
    bool gw_mac_set;
    u8 multicast_macs[NET_MAX_MULTICAST_MACS][6];
    u32 multicast_count;
};
static struct net_iface_state net_ifaces[NIC_IFACE_MAX];
static u8 resolve_mac_copy[NIC_IFACE_MAX][6];

static void net_save_iface(void)
{
    if (net_current_iface >= NIC_IFACE_MAX)
        return;
    struct net_iface_state *s = &net_ifaces[net_current_iface];
    s->configured = our_ip != 0U;
    s->ip = our_ip;
    s->gw = our_gw;
    s->mask = our_mask;
    simd_memcpy(s->mac, our_mac, 6);
    simd_memcpy(s->neighbors, neighbors, sizeof(neighbors));
    s->neighbor_count = neighbor_count;
    simd_memcpy(s->routes, routes, sizeof(routes));
    s->route_count = route_count;
    simd_memcpy(s->gw_mac, gw_mac, 6);
    s->gw_mac_set = gw_mac_set;
    simd_memcpy(s->multicast_macs, multicast_macs, sizeof(multicast_macs));
    s->multicast_count = multicast_count;
}

static bool net_load_iface(nic_iface_t iface)
{
    if (iface >= NIC_IFACE_MAX || !net_ifaces[iface].configured)
        return false;
    struct net_iface_state *s = &net_ifaces[iface];
    net_current_iface = iface;
    our_ip = s->ip;
    our_gw = s->gw;
    our_mask = s->mask;
    simd_memcpy(our_mac, s->mac, 6);
    simd_memcpy(neighbors, s->neighbors, sizeof(neighbors));
    neighbor_count = s->neighbor_count;
    simd_memcpy(routes, s->routes, sizeof(routes));
    route_count = s->route_count;
    simd_memcpy(gw_mac, s->gw_mac, 6);
    gw_mac_set = s->gw_mac_set;
    simd_memcpy(multicast_macs, s->multicast_macs, sizeof(multicast_macs));
    multicast_count = s->multicast_count;
    nic_set_local_ipv4_for(iface, our_ip);
    arp_set_interface(iface);
    return true;
}

static u16 ip_id_counter;

static bool mac_eq_6(const u8 *a, const u8 *b) {
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2] &&
           a[3] == b[3] && a[4] == b[4] && a[5] == b[5];
}

static bool mac_is_broadcast_6(const u8 *m) {
    return m[0] == 0xFF && m[1] == 0xFF && m[2] == 0xFF &&
           m[3] == 0xFF && m[4] == 0xFF && m[5] == 0xFF;
}

static bool mac_is_multicast_6(const u8 *m) {
    return (m[0] & 0x01) != 0;
}

static bool net_multicast_joined(const u8 *mac) {
    for (u32 i = 0; i < multicast_count; i++)
        if (mac_eq_6(multicast_macs[i], mac))
            return true;
    return false;
}

static bool net_accept_eth_dst(const u8 *dst) {
    if (mac_eq_6(dst, our_mac))
        return true;
    if (mac_is_broadcast_6(dst))
        return true;
    if (mac_is_multicast_6(dst) && net_multicast_joined(dst))
        return true;
    return false;
}

static void egress_set_mac(const u8 *mac)
{
    if (mac)
        simd_memcpy(egress_trace.last_mac, mac, 6);
    else
        simd_zero(egress_trace.last_mac, sizeof(egress_trace.last_mac));
}

static void egress_record_route(u32 dst_ip, const struct net_route_entry *route,
                                u32 next_hop, u8 source, const u8 *mac)
{
    egress_trace.last_dst_ip = dst_ip;
    egress_trace.last_next_hop = next_hop;
    egress_trace.last_mac_source = source;
    if (route) {
        egress_trace.last_route_dst = route->dst;
        egress_trace.last_route_mask = route->mask;
        egress_trace.last_route_gateway = route->gateway;
        egress_trace.last_route_flags = route->flags;
        egress_trace.last_route_prefix = route->prefix_len;
    } else {
        egress_trace.last_route_dst = 0;
        egress_trace.last_route_mask = 0;
        egress_trace.last_route_gateway = 0;
        egress_trace.last_route_flags = 0;
        egress_trace.last_route_prefix = 0;
    }
    egress_set_mac(mac);
}

static void egress_record_udp_route(u32 dst_ip)
{
    egress_trace.last_udp_dst_ip = dst_ip;
    egress_trace.last_udp_next_hop = egress_trace.last_next_hop;
    egress_trace.last_udp_mac_source = egress_trace.last_mac_source;
    simd_memcpy(egress_trace.last_udp_mac, egress_trace.last_mac,
                sizeof(egress_trace.last_udp_mac));
}

void net_egress_snapshot(struct net_egress_snapshot *out)
{
    if (!out)
        return;
    *out = egress_trace;
}

static u32 net_read_be32(const u8 *p) {
    return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

static u32 csum_add_bytes(u32 sum, const void *data, u32 len)
{
    const u8 *p = (const u8 *)data;
    while (len >= 2U) {
        sum += ((u16)p[0] << 8) | p[1];
        p += 2;
        len -= 2U;
    }
    if (len)
        sum += ((u16)p[0] << 8);
    return sum;
}

static u16 csum_fold(u32 sum)
{
    while (sum >> 16)
        sum = (sum & 0xFFFFU) + (sum >> 16);
    return (u16)~sum;
}

static u16 l4_checksum(u32 src_ip, u32 dst_ip, u8 proto,
                       const void *l4_data, u32 l4_len)
{
    u32 sum = 0;
    sum += (src_ip >> 16) & 0xFFFFU;
    sum += src_ip & 0xFFFFU;
    sum += (dst_ip >> 16) & 0xFFFFU;
    sum += dst_ip & 0xFFFFU;
    sum += proto;
    sum += (u16)l4_len;
    sum = csum_add_bytes(sum, l4_data, l4_len);
    return htons(csum_fold(sum));
}

static bool net_drop_arp_not_for_us(const u8 *frame, u32 len) {
    if (len < sizeof(struct eth_hdr) + 28)
        return false;
    const struct eth_hdr *eth = (const struct eth_hdr *)frame;
    if (ntohs(eth->ethertype) != ETH_P_ARP)
        return false;
    const u8 *arp = frame + sizeof(struct eth_hdr);
    u16 opcode = ((u16)arp[6] << 8) | arp[7];
    if (opcode != 1 && opcode != 2)
        return false;
    return net_read_be32(arp + 24) != our_ip;
}

void net_firewall_install_defaults(void) {
    nic_filter_rule_t rule;

    nic_filter_clear();
    nic_filter_set_default(false, true);

    for (u32 fi = NIC_IFACE_WIRED; fi <= NIC_IFACE_WIFI; fi++) {
        if (!net_ifaces[fi].configured)
            continue;
        (void)net_load_iface((nic_iface_t)fi);
    simd_zero(&rule, sizeof(rule));
    rule.direction = NIC_FILTER_DIR_IN;
    rule.action = NIC_FILTER_ALLOW;
    rule.iface = (u8)fi;
    rule.flags = NIC_FILTER_ETHERTYPE | NIC_FILTER_IP_TO;
    rule.ethertype = ETH_P_ARP;
    rule.ip_to = our_ip;
    (void)nic_filter_add(&rule);

    simd_zero(&rule, sizeof(rule));
    rule.direction = NIC_FILTER_DIR_IN;
    rule.action = NIC_FILTER_ALLOW;
    rule.flags = NIC_FILTER_ETHERTYPE | NIC_FILTER_IP_TO |
                 NIC_FILTER_IP_PROTO | NIC_FILTER_TCP_PORT_TO;
    rule.ethertype = ETH_P_IP;
    rule.ip_to = our_ip;
    rule.ip_proto = IP_PROTO_TCP;
    rule.tcp_port_to = 80;
    (void)nic_filter_add(&rule);

    rule.tcp_port_to = 443;
    (void)nic_filter_add(&rule);
    rule.tcp_port_to = 81;          /* legacy userland HTTP */
    (void)nic_filter_add(&rule);
    rule.tcp_port_to = 82;          /* benchmark: EL0 PicoScript VM HTTP */
    (void)nic_filter_add(&rule);
    rule.tcp_port_to = 83;          /* benchmark: EL0 native C HTTP */
    (void)nic_filter_add(&rule);
    rule.tcp_port_to = 8080;
    (void)nic_filter_add(&rule);
    rule.tcp_port_to = 8081;
    (void)nic_filter_add(&rule);
    rule.tcp_port_to = 8082;
    (void)nic_filter_add(&rule);
    rule.tcp_port_to = 2323;
    (void)nic_filter_add(&rule);
    rule.tcp_port_to = 8090;        /* capsvc.c generic capsule-service dispatcher (see kernel.c CAPSVC_ADMIN_PORT) */
    (void)nic_filter_add(&rule);

    /* Replies to outbound TCP clients target our ephemeral source ports.
     * The NIC firewall is stateless, so allow the client port range while
     * keeping privileged inbound services deny-by-default. */
    simd_zero(&rule, sizeof(rule));
    rule.direction = NIC_FILTER_DIR_IN;
    rule.action = NIC_FILTER_ALLOW;
    rule.flags = NIC_FILTER_ETHERTYPE | NIC_FILTER_IP_TO |
                 NIC_FILTER_IP_PROTO | NIC_FILTER_TCP_PORT_TO_RANGE;
    rule.ethertype = ETH_P_IP;
    rule.ip_to = our_ip;
    rule.ip_proto = IP_PROTO_TCP;
    rule.tcp_port_to = 49152;
    rule.tcp_port_to_end = 65535;
    (void)nic_filter_add(&rule);

    simd_zero(&rule, sizeof(rule));
    rule.direction = NIC_FILTER_DIR_IN;
    rule.action = NIC_FILTER_ALLOW;
    rule.flags = NIC_FILTER_ETHERTYPE | NIC_FILTER_IP_TO |
                 NIC_FILTER_IP_PROTO | NIC_FILTER_UDP_PORT_TO_RANGE;
    rule.ethertype = ETH_P_IP;
    rule.ip_to = our_ip;
    rule.ip_proto = IP_PROTO_UDP;
    rule.udp_port_to = 49152;
    rule.udp_port_to_end = 65535;
    (void)nic_filter_add(&rule);

    simd_zero(&rule, sizeof(rule));
    rule.direction = NIC_FILTER_DIR_IN;
    rule.action = NIC_FILTER_ALLOW;
    rule.flags = NIC_FILTER_ETHERTYPE | NIC_FILTER_IP_TO |
                 NIC_FILTER_IP_PROTO;
    rule.ethertype = ETH_P_IP;
    rule.ip_to = our_ip;
    rule.ip_proto = IP_PROTO_ICMP;
    (void)nic_filter_add(&rule);
    }
    (void)net_load_iface(NIC_IFACE_WIRED);
}

/* ================================================================== */
/*  Static neighbor table                                              */
/* ================================================================== */

void net_add_neighbor(u32 ip, const u8 *mac) {
    arp_set_interface(net_current_iface);
    for (u32 i = 0; i < neighbor_count; i++) {
        if (neighbors[i].ip == ip) {
            simd_memcpy(neighbors[i].mac, mac, 6);
            arp_add_static(ip, mac);
            return;
        }
    }
    if (neighbor_count < MAX_NEIGHBORS) {
        neighbors[neighbor_count].ip = ip;
        simd_memcpy(neighbors[neighbor_count].mac, mac, 6);
        neighbor_count++;
    }
    arp_add_static(ip, mac);
}

static const u8 *neighbor_lookup(u32 ip) {
    for (u32 i = 0; i < neighbor_count; i++)
        if (neighbors[i].ip == ip)
            return neighbors[i].mac;
    return NULL;
}

static u8 prefix_len_from_mask(u32 mask)
{
    u8 n = 0;
    bool zero_seen = false;
    for (i32 i = 31; i >= 0; i--) {
        bool bit = ((mask >> (u32)i) & 1U) != 0;
        if (bit && zero_seen)
            return 0xFFU;
        if (bit) n++;
        else zero_seen = true;
    }
    return n;
}

bool net_route_add(u32 dst, u32 mask, u32 gateway, u8 flags)
{
    u8 prefix = prefix_len_from_mask(mask);
    if (prefix == 0xFFU)
        return false;
    dst &= mask;
    for (u32 i = 0; i < route_count; i++) {
        if (routes[i].dst == dst && routes[i].mask == mask) {
            routes[i].gateway = gateway;
            routes[i].prefix_len = prefix;
            routes[i].flags = flags;
            return true;
        }
    }
    if (route_count >= NET_ROUTE_MAX)
        return false;
    routes[route_count].dst = dst;
    routes[route_count].mask = mask;
    routes[route_count].gateway = gateway;
    routes[route_count].prefix_len = prefix;
    routes[route_count].flags = flags;
    route_count++;
    return true;
}

u32 net_route_snapshot(struct net_route_entry *out, u32 max)
{
    u32 n = 0;
    if (!out || max == 0)
        return 0;
    for (u32 i = 0; i < route_count && n < max; i++)
        out[n++] = routes[i];
    return n;
}

bool net_route_lookup(u32 dst_ip, struct net_route_entry *out)
{
    i32 best = -1;
    u8 best_prefix = 0;
    for (u32 i = 0; i < route_count; i++) {
        if ((dst_ip & routes[i].mask) != routes[i].dst)
            continue;
        if (best < 0 || routes[i].prefix_len >= best_prefix) {
            best = (i32)i;
            best_prefix = routes[i].prefix_len;
        }
    }
    if (best < 0)
        return false;
    if (out)
        *out = routes[best];
    return true;
}

bool net_join_multicast_mac(const u8 *mac) {
    if (!mac || !mac_is_multicast_6(mac) || mac_is_broadcast_6(mac))
        return false;
    for (u32 i = 0; i < multicast_count; i++) {
        if (mac_eq_6(multicast_macs[i], mac))
            return true;
    }
    if (multicast_count >= NET_MAX_MULTICAST_MACS)
        return false;
    simd_memcpy(multicast_macs[multicast_count], mac, 6);
    multicast_count++;
    return true;
}

bool net_leave_multicast_mac(const u8 *mac) {
    if (!mac)
        return false;
    for (u32 i = 0; i < multicast_count; i++) {
        if (!mac_eq_6(multicast_macs[i], mac))
            continue;
        multicast_count--;
        if (i != multicast_count)
            simd_memcpy(multicast_macs[i], multicast_macs[multicast_count], 6);
        return true;
    }
    return false;
}

/* ================================================================== */
/*  ICMP - echo reply only, rate-limited                               */
/* ================================================================== */

static bool icmp_rate_ok(void) {
    u64 now = read_cntvct();
    if ((now - icmp_last_tick) < icmp_min_interval)
        return false;
    icmp_last_tick = now;
    return true;
}

/* Pending outbound ping/traceroute probe (single-outstanding, core-0-only). */
static struct {
    bool waiting;
    u16  ident;
    u16  seq;
    u64  sent_ms;
    struct net_ping_result result;
} g_ping;

static void handle_icmp(const u8 *frame, u32 len,
                        struct ip_hdr *ip, u32 payload_off) {
    (void)len;
    u16 ipt = ntohs(ip->total_len);
    if (ipt < 20 + sizeof(struct icmp_hdr)) return;
    u32 icmp_len = ipt - 20;
    /* Cap to tx_frame capacity (prevent buffer overflow on large ICMP) */
    if (sizeof(struct eth_hdr) + ipt > sizeof(tx_frame)) return;

    struct icmp_hdr *icmp = (struct icmp_hdr *)(frame + payload_off);

    /* Client-side matching for our own outstanding ping/traceroute probe. */
    if (icmp->type == 0 && g_ping.waiting) {
        if (ntohs(icmp->id) == g_ping.ident && ntohs(icmp->seq) == g_ping.seq) {
            g_ping.result.got_reply = true;
            g_ping.result.from_ip   = ntohl(ip->src_ip);
            g_ping.result.reply_ttl = ip->ttl;
            g_ping.result.rtt_ms    = (u32)(timer_monotonic_ms() - g_ping.sent_ms);
        }
        return;
    }
    if (icmp->type == 11 && g_ping.waiting) {
        /* Time Exceeded (traceroute hop). Only one probe is ever outstanding
         * on this path, so accept any Time Exceeded while waiting rather
         * than parsing the embedded original datagram back out. */
        g_ping.result.got_ttl_exceeded = true;
        g_ping.result.from_ip          = ntohl(ip->src_ip);
        g_ping.result.reply_ttl        = ip->ttl;
        g_ping.result.rtt_ms           = (u32)(timer_monotonic_ms() - g_ping.sent_ms);
        return;
    }

    if (icmp->type != 8 || icmp->code != 0)
        return;

    if (unlikely(!icmp_rate_ok())) {
        stats.drop_icmp_ratelimit++;
        nic_record_rate_limited();
        return;
    }

    struct eth_hdr *eth_in  = (struct eth_hdr *)frame;
    struct eth_hdr *eth_out = (struct eth_hdr *)tx_frame;

    simd_memcpy(eth_out->dst, eth_in->src, 6);
    simd_memcpy(eth_out->src, our_mac, 6);
    eth_out->ethertype = htons(ETH_P_IP);

    u16 ip_total = ntohs(ip->total_len);

    struct ip_hdr *ip_out = (struct ip_hdr *)(tx_frame + sizeof(struct eth_hdr));
    ip_out->ver_ihl    = 0x45;
    ip_out->tos        = 0;
    ip_out->total_len  = htons(ip_total);
    ip_out->id         = htons(ip_id_counter++);
    ip_out->flags_frag = htons(0x4000);   /* DF bit */
    ip_out->ttl        = 64;
    ip_out->protocol   = IP_PROTO_ICMP;
    ip_out->checksum   = 0;
    ip_out->src_ip     = htonl(our_ip);
    ip_out->dst_ip     = ip->src_ip;
    ip_out->checksum   = simd_checksum(ip_out, 20);

    u32 icmp_out_off = sizeof(struct eth_hdr) + 20;
    simd_memcpy(tx_frame + icmp_out_off, frame + payload_off, icmp_len);
    struct icmp_hdr *icmp_out = (struct icmp_hdr *)(tx_frame + icmp_out_off);
    icmp_out->type     = 0;
    icmp_out->checksum = 0;
    icmp_out->checksum = simd_checksum(icmp_out, icmp_len);

    u32 frame_len = sizeof(struct eth_hdr) + ip_total;
#if NET_ICMP_DIAG_VERBOSE
    static u32 icmp_reply_count = 0;
    if (icmp_reply_count < 10) {
        uart_puts("[icmp] reply #");
        uart_hex(icmp_reply_count++);
        uart_puts(" dst=");
        for (u32 i = 0; i < 6; i++) { uart_hex(eth_out->dst[i]); if (i<5) uart_puts(":"); }
        uart_puts(" src=");
        for (u32 i = 0; i < 6; i++) { uart_hex(eth_out->src[i]); if (i<5) uart_puts(":"); }
        uart_puts("\n");
    }
#endif
    if (!nic_send_on(net_current_iface, tx_frame, frame_len)) {
#if NET_ICMP_DIAG_VERBOSE
        static u32 send_fail_count = 0;
        if (send_fail_count++ < 5) {
            uart_puts("[icmp] TX FAILED #");
            uart_hex(send_fail_count);
            uart_puts("\n");
        }
#endif
    }
    stats.icmp_echo_replies++;
    stats.tx_packets++;
    stats.tx_bytes += frame_len;
}

/* ================================================================== */
/*  UDP                                                                */
/* ================================================================== */

static void handle_udp(const u8 *frame, u32 len,
                       struct ip_hdr *ip, u32 payload_off,
                       bool checksum_trusted) {
    if (unlikely(payload_off + sizeof(struct udp_hdr) > len)) {
        stats.drop_udp_malformed++;
        return;
    }

    struct udp_hdr *udp = (struct udp_hdr *)(frame + payload_off);
    u16 udp_len = ntohs(udp->length);

    if (unlikely(udp_len < sizeof(struct udp_hdr))) {
        stats.drop_udp_malformed++;
        return;
    }
    if (unlikely(payload_off + udp_len > len)) {
        stats.drop_udp_malformed++;
        return;
    }

    u16 data_len = udp_len - sizeof(struct udp_hdr);
    const u8 *data = frame + payload_off + sizeof(struct udp_hdr);
    if (udp->checksum != 0 && !checksum_trusted &&
        unlikely(l4_checksum(ntohl(ip->src_ip), ntohl(ip->dst_ip),
                             IP_PROTO_UDP, udp, udp_len) != 0)) {
        stats.drop_udp_bad_cksum++;
        return;
    }

    stats.udp_recv++;

    if (udp_callback) {
        udp_callback(ntohl(ip->src_ip),
                     ntohs(udp->src_port),
                     ntohs(udp->dst_port),
                     data, data_len);
    }
    for (u32 i = 0; i < UDP_SUBSCRIBER_MAX; i++) {
        udp_recv_cb cb = udp_subscribers[i].cb;
        if (cb) {
            cb(ntohl(ip->src_ip),
               ntohs(udp->src_port),
               ntohs(udp->dst_port),
               data, data_len);
        }
    }
}

/* ================================================================== */
/*  IP - hardened ingress validation                                   */
/* ================================================================== */

static void UNUSED handle_ip(const u8 *frame, u32 len, bool checksum_trusted) {
    if (unlikely(len < sizeof(struct eth_hdr) + 20)) {
        stats.drop_runt++;
        return;
    }

    struct ip_hdr *ip = (struct ip_hdr *)(frame + sizeof(struct eth_hdr));

    /* Must be IPv4 */
    if (unlikely((ip->ver_ihl >> 4) != 4)) {
        stats.drop_runt++;
        return;
    }

    /* REJECT IP options (IHL must be exactly 5 = 20 bytes) */
    if (unlikely((ip->ver_ihl & 0x0F) != 5)) {
        stats.drop_ip_options++;
        return;
    }

    /* Validate total_length vs actual frame */
    u16 ip_total = ntohs(ip->total_len);
    if (unlikely(ip_total < 20 || sizeof(struct eth_hdr) + ip_total > len)) {
        stats.drop_runt++;
        return;
    }

    /* Verify IP header checksum (NEON-accelerated) */
    if (unlikely(simd_checksum(ip, 20) != 0)) {
        stats.drop_bad_cksum++;
        return;
    }

    /* DROP all IP fragments (MF flag or frag offset != 0) */
    u16 flags_frag = ntohs(ip->flags_frag);
    if (unlikely((flags_frag & 0x2000) || (flags_frag & 0x1FFF))) {
        stats.drop_fragment++;
        return;
    }

    /* Validate source IP */
    u32 src = ntohl(ip->src_ip);
    if (unlikely(src == 0 ||
                 src == 0xFFFFFFFF ||
                 src == our_ip ||
                 (src >> 24) == 127 ||
                 (src >> 28) == 0xE)) {
        stats.drop_bad_src++;
        return;
    }

    /* TTL check */
    if (unlikely(ip->ttl == 0)) {
        stats.drop_runt++;
        return;
    }

    /* Is this for us? */
    u32 dst = ntohl(ip->dst_ip);
    if (dst != our_ip && dst != 0xFFFFFFFF) {
        stats.drop_not_for_us++;
        return;
    }

    u32 payload_off = sizeof(struct eth_hdr) + 20;

    switch (ip->protocol) {
    case IP_PROTO_ICMP: handle_icmp(frame, len, ip, payload_off); break;
    case IP_PROTO_TCP: {
        u16 ipt = ntohs(ip->total_len);
        if (ipt > 20)
            tcp_input(frame, len, ntohl(ip->src_ip), ntohl(ip->dst_ip),
                      frame + payload_off, ipt - 20,
                      checksum_trusted, net_current_iface);
        break;
    }
    case IP_PROTO_UDP:  handle_udp(frame, len, ip, payload_off, checksum_trusted);  break;
    default: stats.drop_bad_proto++; break;
    }
}

/* ================================================================== */
/*  TX path - neighbor resolution (static table + ARP)                 */
/* ================================================================== */

static bool resolve_mac(u32 dst_ip, const u8 **mac_out) {
    arp_set_interface(net_current_iface);
    struct net_route_entry route;
    u32 next_hop = 0;
    egress_trace.resolve_calls++;
    if (!net_route_lookup(dst_ip, &route)) {
        egress_trace.no_route++;
        egress_record_route(dst_ip, NULL, 0, NET_EGRESS_MAC_NO_ROUTE, NULL);
        return false;
    }
    next_hop = (route.flags & NET_ROUTE_F_CONNECTED) ? dst_ip : route.gateway;

    /* Broadcast */
    if (dst_ip == 0xFFFFFFFF) {
        static const u8 bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
        *mac_out = bcast;
        egress_record_route(dst_ip, &route, next_hop, NET_EGRESS_MAC_BCAST, bcast);
        return true;
    }

    /* Try static gateway MAC first */
    if (next_hop == our_gw && gw_mac_set) {
        *mac_out = gw_mac;
        egress_record_route(dst_ip, &route, next_hop, NET_EGRESS_MAC_GW_STATIC, gw_mac);
        return true;
    }

    /* Try static neighbor table */
    const u8 *mac = neighbor_lookup(next_hop);
    if (mac) {
        *mac_out = mac;
        egress_record_route(dst_ip, &route, next_hop, NET_EGRESS_MAC_NEIGHBOR, mac);
        return true;
    }

    /* Fall through to ARP */
    mac = arp_resolve(next_hop);
    if (mac) {
        *mac_out = mac;
        egress_record_route(dst_ip, &route, next_hop, NET_EGRESS_MAC_ARP, mac);
        return true;
    }

    stats.drop_no_neighbor++;
    egress_trace.no_mac++;
    egress_record_route(dst_ip, &route, next_hop, NET_EGRESS_MAC_NO_MAC, NULL);
    return false;
}

const u8 *net_resolve_mac(u32 dst_ip)
{
    const u8 *mac = NULL;
    if (!resolve_mac(dst_ip, &mac))
        return NULL;
    return mac;
}

const u8 *net_resolve_mac_on(nic_iface_t iface, u32 dst_ip)
{
    if (iface == NIC_IFACE_ANY)
        iface = nic_default_iface();
    if (!net_interface_configured(iface) || !nic_iface_active(iface))
        return NULL;
    if (iface == net_current_iface)
        return net_resolve_mac(dst_ip);
    nic_iface_t previous = net_current_iface;
    net_save_iface();
    if (!net_load_iface(iface))
        return NULL;
    const u8 *mac = NULL;
    bool ok = resolve_mac(dst_ip, &mac);
    if (ok)
        simd_memcpy(resolve_mac_copy[iface], mac, 6);
    net_save_iface();
    (void)net_load_iface(previous);
    return ok ? resolve_mac_copy[iface] : NULL;
}

static bool net_send_udp_current(u32 dst_ip, u16 src_port, u16 dst_port,
                                 const u8 *data, u16 len) {
    egress_trace.udp_attempts++;
    egress_trace.last_udp_src_port = src_port;
    egress_trace.last_udp_dst_port = dst_port;
    egress_trace.last_udp_len = len;
    egress_trace.last_udp_ok = 0;

    /* Guard against u16 overflow: max frame 2048 minus IP+UDP headers */
    if (len > 2048 - 42) {
        egress_trace.udp_fail++;
        return false;
    }

    const u8 *dst_mac;
    if (unlikely(!resolve_mac(dst_ip, &dst_mac))) {
        egress_record_udp_route(dst_ip);
        egress_trace.udp_fail++;
        return false;
    }
    egress_record_udp_route(dst_ip);

    u16 udp_len  = sizeof(struct udp_hdr) + len;
    u16 ip_total = 20 + udp_len;

    if (unlikely(sizeof(struct eth_hdr) + ip_total > 1514)) {
        egress_trace.udp_fail++;
        return false;
    }

    struct eth_hdr *eth = (struct eth_hdr *)tx_frame;
    simd_memcpy(eth->dst, dst_mac, 6);
    simd_memcpy(eth->src, our_mac, 6);
    eth->ethertype = htons(ETH_P_IP);

    struct ip_hdr *ip = (struct ip_hdr *)(tx_frame + sizeof(struct eth_hdr));
    ip->ver_ihl    = 0x45;
    ip->tos        = 0;
    ip->total_len  = htons(ip_total);
    ip->id         = htons(ip_id_counter++);
    ip->flags_frag = htons(0x4000);    /* DF */
    ip->ttl        = 64;
    ip->protocol   = IP_PROTO_UDP;
    ip->checksum   = 0;
    ip->src_ip     = htonl(our_ip);
    ip->dst_ip     = htonl(dst_ip);
    ip->checksum   = simd_checksum(ip, 20);

    struct udp_hdr *udp = (struct udp_hdr *)(tx_frame + sizeof(struct eth_hdr) + 20);
    udp->src_port  = htons(src_port);
    udp->dst_port  = htons(dst_port);
    udp->length    = htons(udp_len);
    udp->checksum  = 0;

    u32 frame_len = sizeof(struct eth_hdr) + ip_total;
    bool need_pad = frame_len < 60;
    if (need_pad) frame_len = 60;

    stats.tx_packets++;
    stats.tx_bytes += frame_len;
    stats.udp_sent++;

    bool tx_csum_offload =
        nic_tx_checksum_offload_enabled_for(net_current_iface);
    if (!tx_csum_offload) {
        if (len != 0)
            simd_memcpy(tx_frame + sizeof(struct eth_hdr) + 20 + sizeof(struct udp_hdr),
                        data, len);
        udp->checksum = l4_checksum(our_ip, dst_ip, IP_PROTO_UDP, udp, udp_len);
        if (udp->checksum == 0)
            udp->checksum = 0xFFFFU;
    }

    bool ok = false;
    if (len == 0) {
        ok = nic_send_on(net_current_iface, tx_frame, frame_len);
        egress_trace.last_udp_ok = ok ? 1U : 0U;
        if (ok) egress_trace.udp_ok++;
        else egress_trace.udp_fail++;
        return ok;
    }

    if (need_pad || !tx_csum_offload) {
        simd_memcpy(tx_frame + sizeof(struct eth_hdr) + 20 + sizeof(struct udp_hdr),
                    data, len);
        ok = nic_send_on(net_current_iface, tx_frame, frame_len);
        egress_trace.last_udp_ok = ok ? 1U : 0U;
        if (ok) egress_trace.udp_ok++;
        else egress_trace.udp_fail++;
        return ok;
    }

    u32 head_len = sizeof(struct eth_hdr) + 20 + sizeof(struct udp_hdr);
    ok = nic_send_parts_on(net_current_iface, tx_frame, head_len, data, len);
    egress_trace.last_udp_ok = ok ? 1U : 0U;
    if (ok) egress_trace.udp_ok++;
    else egress_trace.udp_fail++;
    return ok;
}

bool net_send_udp_on(nic_iface_t iface, u32 dst_ip, u16 src_port, u16 dst_port,
                     const u8 *data, u16 len)
{
    if (iface == NIC_IFACE_ANY)
        iface = nic_default_iface();
    if (!net_interface_configured(iface) || !nic_iface_active(iface))
        return false;
    if (iface == net_current_iface)
        return net_send_udp_current(dst_ip, src_port, dst_port, data, len);
    nic_iface_t previous = net_current_iface;
    net_save_iface();
    if (!net_load_iface(iface))
        return false;
    bool ok = net_send_udp_current(dst_ip, src_port, dst_port, data, len);
    net_save_iface();
    (void)net_load_iface(previous);
    return ok;
}

bool net_send_udp(u32 dst_ip, u16 src_port, u16 dst_port,
                  const u8 *data, u16 len)
{
    /* Core-0 callbacks invoked by an ingress packet reply on that same
     * interface. Outside ingress dispatch net_current_iface is the wired
     * default, preserving the historical single-NIC behaviour. */
    return net_send_udp_on(net_current_iface, dst_ip, src_port, dst_port,
                           data, len);
}

/* ================================================================== */
/*  ICMP echo client (ping / traceroute)                               */
/* ================================================================== */

bool net_icmp_echo_send(u32 dst_ip, u16 ident, u16 seq, u8 ttl) {
    const u8 *dst_mac = net_resolve_mac(dst_ip);
    if (!dst_mac) return false;

    static const u8 payload[8] = { 'P','I','O','S','P','I','N','G' };
    u16 icmp_len = (u16)(sizeof(struct icmp_hdr) + sizeof(payload));
    u16 ip_total = (u16)(20 + icmp_len);
    u32 frame_len = sizeof(struct eth_hdr) + ip_total;
    if (frame_len > sizeof(tx_frame)) return false;

    struct eth_hdr *eth = (struct eth_hdr *)tx_frame;
    simd_memcpy(eth->dst, dst_mac, 6);
    simd_memcpy(eth->src, our_mac, 6);
    eth->ethertype = htons(ETH_P_IP);

    struct ip_hdr *ip = (struct ip_hdr *)(tx_frame + sizeof(struct eth_hdr));
    ip->ver_ihl    = 0x45;
    ip->tos        = 0;
    ip->total_len  = htons(ip_total);
    ip->id         = htons(ip_id_counter++);
    ip->flags_frag = htons(0x4000);
    ip->ttl        = ttl;
    ip->protocol   = IP_PROTO_ICMP;
    ip->checksum   = 0;
    ip->src_ip     = htonl(our_ip);
    ip->dst_ip     = htonl(dst_ip);
    ip->checksum   = simd_checksum(ip, 20);

    struct icmp_hdr *icmp = (struct icmp_hdr *)(tx_frame + sizeof(struct eth_hdr) + 20);
    icmp->type     = 8;
    icmp->code     = 0;
    icmp->checksum = 0;
    icmp->id       = htons(ident);
    icmp->seq      = htons(seq);
    simd_memcpy((u8 *)icmp + sizeof(struct icmp_hdr), payload, sizeof(payload));
    icmp->checksum = simd_checksum(icmp, icmp_len);

    if (frame_len < 60) frame_len = 60;

    g_ping.waiting  = true;
    g_ping.ident    = ident;
    g_ping.seq      = seq;
    g_ping.sent_ms  = timer_monotonic_ms();
    g_ping.result.got_reply        = false;
    g_ping.result.got_ttl_exceeded = false;
    g_ping.result.from_ip          = 0;
    g_ping.result.rtt_ms           = 0;
    g_ping.result.reply_ttl        = 0;

    bool ok = nic_send_on(net_current_iface, tx_frame, frame_len);
    if (ok) { stats.tx_packets++; stats.tx_bytes += frame_len; }
    return ok;
}

bool net_icmp_echo_poll_result(struct net_ping_result *out) {
    if (!g_ping.waiting) return false;
    if (!g_ping.result.got_reply && !g_ping.result.got_ttl_exceeded) return false;
    if (out) *out = g_ping.result;
    g_ping.waiting = false;
    return true;
}

/* ================================================================== */
/*  DNS FIFO integration - user cores must never call dns_resolve() or */
/*  net_poll() directly: dns.c/net.c/macb.c state is owned exclusively */
/*  by Core 0. A user core that touched it directly would race Core 0  */
/*  on the same non-coherent NIC descriptor rings/registers. Route     */
/*  resolves through a small pending queue drained on Core 0.          */
/* ================================================================== */

#define DNS_FIFO_QUEUE_MAX 4U
struct dns_fifo_req {
    bool used;
    u32  requester_core;
    u64  tag;
    char hostname[DNS_HOST_MAX];
};
static struct dns_fifo_req dns_fifo_queue[DNS_FIFO_QUEUE_MAX];
static bool dns_fifo_active;
static u32  dns_fifo_active_core;
static u64  dns_fifo_active_tag;

static void dns_fifo_reply(u32 requester_core, u64 tag, u32 status, u32 ip) {
    struct fifo_msg reply;
    reply.type   = MSG_DNS_RESOLVE_DONE;
    reply.status = status;
    reply.param  = ip;
    reply.tag    = tag;
    fifo_push(CORE_NET, requester_core, &reply);
}

static void dns_fifo_enqueue(u32 requester_core, u64 tag, const u8 *hostbuf, u32 len) {
    for (u32 i = 0; i < DNS_FIFO_QUEUE_MAX; i++) {
        if (dns_fifo_queue[i].used)
            continue;
        struct dns_fifo_req *r = &dns_fifo_queue[i];
        u32 n = (len < DNS_HOST_MAX - 1U) ? len : DNS_HOST_MAX - 1U;
        u32 j = 0;
        for (; j < n && hostbuf[j] != 0; j++)
            r->hostname[j] = (char)hostbuf[j];
        r->hostname[j] = 0;
        r->requester_core = requester_core;
        r->tag = tag;
        r->used = true;
        return;
    }
    /* Queue full: reply busy immediately so the caller can retry rather
     * than block forever waiting for a slot that will never open. */
    dns_fifo_reply(requester_core, tag, 2U, 0U);
}

/* Advance at most one in-flight resolve + one queued start per call.
 * Never blocks: dns_resolve_async_start()/dns_async_status() are the
 * non-blocking async DNS primitives, unlike dns_resolve(). */
static void dns_fifo_poll(void) {
    if (dns_fifo_active) {
        struct dns_async_status st;
        dns_async_status(&st);
        if (st.state == DNS_ASYNC_DONE || st.state == DNS_ASYNC_FAILED) {
            dns_fifo_reply(dns_fifo_active_core, dns_fifo_active_tag,
                           (st.state == DNS_ASYNC_DONE) ? 0U : 1U, st.result_ip);
            dns_fifo_active = false;
        } else {
            return; /* still resolving; revisit next poll */
        }
    }

    for (u32 i = 0; i < DNS_FIFO_QUEUE_MAX; i++) {
        if (!dns_fifo_queue[i].used)
            continue;
        dns_fifo_active_core = dns_fifo_queue[i].requester_core;
        dns_fifo_active_tag  = dns_fifo_queue[i].tag;
        dns_fifo_queue[i].used = false;
        if (dns_resolve_async_start(dns_fifo_queue[i].hostname))
            dns_fifo_active = true;
        else
            dns_fifo_reply(dns_fifo_active_core, dns_fifo_active_tag, 1U, 0U);
        break;
    }
}

/* ================================================================== */
/*  FIFO integration - user cores send UDP via messages                */
/* ================================================================== */

void net_handle_fifo_request(void) {
    struct fifo_msg msgs[16];
    struct fifo_msg reply;
    u32 n;

    n = fifo_pop_batch(CORE_NET, CORE_USER0, msgs, 16);
    for (u32 i = 0; i < n; i++) {
        struct fifo_msg msg = msgs[i];
        simd_zero(&reply, sizeof(reply));
        if (msg.type == MSG_NET_UDP_SEND && msg.buffer && msg.length <= 1472) {
            if (!ptr_in_core_ram(CORE_USER0, msg.buffer, msg.length))
                continue;
            u16 sp = (u16)(msg.tag >> 16);
            u16 dp = (u16)(msg.tag & 0xFFFF);
            nic_iface_t iface = (msg.iface == NIC_IFACE_ANY)
                ? nic_default_iface() : (nic_iface_t)msg.iface;
            bool ok = net_send_udp_on(iface, msg.param, sp, dp,
                                      (const u8 *)(usize)msg.buffer,
                                      (u16)msg.length);
            reply.type   = MSG_NET_UDP_DONE;
            reply.status = ok ? 0 : 1;
            reply.tag    = msg.tag;
            reply.iface  = iface;
            fifo_push(CORE_NET, CORE_USER0, &reply);
        } else if (msg.type == MSG_DNS_RESOLVE && msg.buffer && msg.length <= 256U) {
            if (!ptr_in_core_ram(CORE_USER0, msg.buffer, msg.length)) {
                dns_fifo_reply(CORE_USER0, msg.tag, 1U, 0U);
                continue;
            }
            dns_fifo_enqueue(CORE_USER0, msg.tag,
                             (const u8 *)(usize)msg.buffer, msg.length);
        }
    }

    n = fifo_pop_batch(CORE_NET, CORE_USER1, msgs, 16);
    for (u32 i = 0; i < n; i++) {
        struct fifo_msg msg = msgs[i];
        simd_zero(&reply, sizeof(reply));
        if (msg.type == MSG_NET_UDP_SEND && msg.buffer && msg.length <= 1472) {
            if (!ptr_in_core_ram(CORE_USER1, msg.buffer, msg.length))
                continue;
            u16 sp = (u16)(msg.tag >> 16);
            u16 dp = (u16)(msg.tag & 0xFFFF);
            nic_iface_t iface = (msg.iface == NIC_IFACE_ANY)
                ? nic_default_iface() : (nic_iface_t)msg.iface;
            bool ok = net_send_udp_on(iface, msg.param, sp, dp,
                                      (const u8 *)(usize)msg.buffer,
                                      (u16)msg.length);
            reply.type   = MSG_NET_UDP_DONE;
            reply.status = ok ? 0 : 1;
            reply.tag    = msg.tag;
            reply.iface  = iface;
            fifo_push(CORE_NET, CORE_USER1, &reply);
        } else if (msg.type == MSG_DNS_RESOLVE && msg.buffer && msg.length <= 256U) {
            if (!ptr_in_core_ram(CORE_USER1, msg.buffer, msg.length)) {
                dns_fifo_reply(CORE_USER1, msg.tag, 1U, 0U);
                continue;
            }
            dns_fifo_enqueue(CORE_USER1, msg.tag,
                             (const u8 *)(usize)msg.buffer, msg.length);
        }
    }

    /* Drain socket-layer FIFO requests in small bursts per poll pass. */
    for (u32 i = 0; i < NET_FIFO_BURST_MAX; i++)
        socket_handle_fifo(CORE_USER0);
    for (u32 i = 0; i < NET_FIFO_BURST_MAX; i++)
        socket_handle_fifo(CORE_USER1);

    dns_fifo_poll();
}

/* ================================================================== */
/*  Poll + Init                                                        */
/* ================================================================== */

bool net_ingress_receive(nic_iface_t iface, u8 *frame, u32 frame_max,
                         u32 *len, bool *checksum_trusted)
{
    if (!frame || !len || frame_max < ETH_FRAME_MAX ||
        !net_interface_configured(iface) || !nic_iface_active(iface))
        return false;
    *len = 0U;
    if (checksum_trusted)
        *checksum_trusted = false;
    return nic_recv_on(iface, frame, len, checksum_trusted);
}

static bool net_ingress_select_iface(nic_iface_t iface, nic_iface_t *previous_out)
{
    if (!net_interface_configured(iface) || !nic_iface_active(iface))
        return false;
    nic_iface_t previous = net_current_iface;
    if (previous != iface) {
        net_save_iface();
        if (!net_load_iface(iface))
            return false;
    }
    if (previous_out)
        *previous_out = previous;
    arp_set_interface(iface);
    return true;
}

static void net_ingress_restore_iface(nic_iface_t previous)
{
    if (net_current_iface != previous) {
        net_save_iface();
        (void)net_load_iface(previous);
    }
}

bool net_ingress_mac_process(nic_iface_t iface, const u8 *frame, u32 len,
                             u16 *ethertype_out)
{
    nic_iface_t previous;
    if (!frame || !net_ingress_select_iface(iface, &previous))
        return false;
    stats.rx_packets++;
    stats.rx_bytes += len;

    if (unlikely(len < sizeof(struct eth_hdr))) {
        stats.drop_runt++;
        net_ingress_restore_iface(previous);
        return false;
    }
    if (unlikely(len > 1518U)) {
        stats.drop_oversized++;
        net_ingress_restore_iface(previous);
        return false;
    }

    const struct eth_hdr *eth = (const struct eth_hdr *)frame;
    if (unlikely(net_drop_arp_not_for_us(frame, len))) {
        stats.drop_not_for_us++;
        net_ingress_restore_iface(previous);
        return false;
    }
    if (unlikely(!net_accept_eth_dst(eth->dst))) {
        stats.drop_not_for_us++;
        net_ingress_restore_iface(previous);
        return false;
    }
    u16 etype = ntohs(eth->ethertype);
    if (etype == ETH_P_ARP) {
        arp_input_iface(iface, frame, len);
        stats.rx_dispatched++;
        net_ingress_restore_iface(previous);
        return false;
    }
    if (etype != ETH_P_IP) {
        stats.rx_unsupported++;
        net_ingress_restore_iface(previous);
        return false;
    }
    if (ethertype_out)
        *ethertype_out = etype;
    net_ingress_restore_iface(previous);
    return true;
}

bool net_ingress_ip_process(nic_iface_t iface, const u8 *frame, u32 len,
                            u8 *protocol_out)
{
    nic_iface_t previous;
    if (!frame || !net_ingress_select_iface(iface, &previous))
        return false;
    if (unlikely(len < sizeof(struct eth_hdr) + 20U)) {
        stats.drop_runt++;
        goto reject;
    }
    const struct ip_hdr *ip = (const struct ip_hdr *)(frame + sizeof(struct eth_hdr));
    if (unlikely((ip->ver_ihl >> 4) != 4U)) {
        stats.drop_runt++;
        goto reject;
    }
    if (unlikely((ip->ver_ihl & 0x0FU) != 5U)) {
        stats.drop_ip_options++;
        goto reject;
    }
    u16 ip_total = ntohs(ip->total_len);
    if (unlikely(ip_total < 20U || sizeof(struct eth_hdr) + ip_total > len)) {
        stats.drop_runt++;
        goto reject;
    }
    if (unlikely(simd_checksum(ip, 20) != 0U)) {
        stats.drop_bad_cksum++;
        goto reject;
    }
    u16 flags_frag = ntohs(ip->flags_frag);
    if (unlikely((flags_frag & 0x2000U) || (flags_frag & 0x1FFFU))) {
        stats.drop_fragment++;
        goto reject;
    }
    u32 src = ntohl(ip->src_ip);
    if (unlikely(src == 0U || src == 0xFFFFFFFFU || src == our_ip ||
                 (src >> 24) == 127U || (src >> 28) == 0xEU)) {
        stats.drop_bad_src++;
        goto reject;
    }
    if (unlikely(ip->ttl == 0U)) {
        stats.drop_runt++;
        goto reject;
    }
    u32 dst = ntohl(ip->dst_ip);
    if (dst != our_ip && dst != 0xFFFFFFFFU) {
        stats.drop_not_for_us++;
        goto reject;
    }
    if (protocol_out)
        *protocol_out = ip->protocol;
    net_ingress_restore_iface(previous);
    return true;
reject:
    net_ingress_restore_iface(previous);
    return false;
}

void net_ingress_l4_process(nic_iface_t iface, const u8 *frame, u32 len,
                            bool checksum_trusted, u8 protocol)
{
    nic_iface_t previous;
    if (!frame || !net_ingress_select_iface(iface, &previous))
        return;
    const struct ip_hdr *ip = (const struct ip_hdr *)(frame + sizeof(struct eth_hdr));
    u32 payload_off = sizeof(struct eth_hdr) + 20U;
    switch (protocol) {
    case IP_PROTO_ICMP:
        handle_icmp(frame, len, (struct ip_hdr *)ip, payload_off);
        break;
    case IP_PROTO_TCP: {
        u16 ipt = ntohs(ip->total_len);
        if (ipt > 20U)
            tcp_input(frame, len, ntohl(ip->src_ip), ntohl(ip->dst_ip),
                      frame + payload_off, ipt - 20U, checksum_trusted, iface);
        stats.rx_dispatched++;
        break;
    }
    case IP_PROTO_UDP:
        handle_udp(frame, len, (struct ip_hdr *)ip, payload_off, checksum_trusted);
        stats.rx_dispatched++;
        break;
    default:
        stats.rx_unsupported++;
        stats.drop_bad_proto++;
        break;
    }
    net_ingress_restore_iface(previous);
}

void net_ingress_process(nic_iface_t iface, const u8 *frame, u32 len,
                         bool checksum_trusted)
{
    u16 etype;
    u8 protocol;
    if (!net_ingress_mac_process(iface, frame, len, &etype))
        return;
    if (etype == ETH_P_IP && net_ingress_ip_process(iface, frame, len, &protocol))
        net_ingress_l4_process(iface, frame, len, checksum_trusted, protocol);
}

void net_service_step(void)
{
    net_handle_fifo_request();
    socket_service_step();
    workq_drain(4);
}

u32 net_poll(void)
{
    /*
     * Compatibility entry point for legacy synchronous console diagnostics.
     * ADR-033 normal ingress never calls this: hardware/timer/reactor paths
     * publish descriptors to net_dispatch instead.
     */
    u32 got = 0U;
    stats.poll_calls++;
    prefetch_r(rx_frame);
    for (u32 fi = NIC_IFACE_WIRED; fi <= NIC_IFACE_WIFI; fi++) {
        for (u32 burst = 0U; burst < NET_RX_BURST_MAX; burst++) {
            u32 len = 0U;
            bool checksum_trusted = false;
            if (!net_ingress_receive((nic_iface_t)fi, rx_frame, sizeof(rx_frame),
                                     &len, &checksum_trusted))
                break;
            net_ingress_process((nic_iface_t)fi, rx_frame, len, checksum_trusted);
            got++;
        }
    }
    stats.poll_last_frames = got;
    if (got == 0U)
        stats.poll_empty++;
    if (got == NET_RX_BURST_MAX)
        stats.poll_budget_hits++;
    net_service_step();
    return got;
}

void net_init(u32 ip, u32 gateway, u32 netmask, const u8 *gateway_mac) {
    simd_zero(net_ifaces, sizeof(net_ifaces));
    net_current_iface = NIC_IFACE_WIRED;
    our_ip   = ip;
    our_gw   = gateway;
    our_mask = netmask;
    nic_set_local_ipv4_for(NIC_IFACE_WIRED, ip);
    nic_get_mac_for(NIC_IFACE_WIRED, our_mac);

    gw_mac_set = false;
    simd_zero(gw_mac, sizeof(gw_mac));
    if (gateway_mac && !mac_is_zero_6(gateway_mac)) {
        simd_memcpy(gw_mac, gateway_mac, 6);
        gw_mac_set = true;
    }

    simd_zero(&stats, sizeof(stats));
    simd_zero(neighbors, sizeof(neighbors));
    simd_zero(routes, sizeof(routes));
    simd_zero(multicast_macs, sizeof(multicast_macs));
    neighbor_count = 0;
    route_count = 0;
    multicast_count = 0;
    udp_callback   = NULL;
    for (u32 i = 0; i < UDP_SUBSCRIBER_MAX; i++)
        udp_subscribers[i].cb = NULL;
    ip_id_counter  = 1;

    icmp_min_interval = 0;  /* Disable ICMP rate limiting for testing */
    udp_callback = NULL;
    icmp_last_tick    = 0;

    /* Init ARP subsystem */
    arp_init_iface(NIC_IFACE_WIRED, ip, netmask, our_mac);
    (void)net_route_add(ip & netmask, netmask, 0, NET_ROUTE_F_CONNECTED);
    if (gateway != 0)
        (void)net_route_add(0, 0, gateway, 0);

    /* Init TCP subsystem */
    tcp_init();

    /* Init socket layer */
    socket_init();

    /* MACB/GEM TX checksum generation enabled via DMACFG.TXCOEN. */
    nic_set_rx_checksum_offload(true);
    nic_set_tx_checksum_offload(true);
    nic_set_tso(false);

    /* Init TLS wrapper subsystem */
    tls_init();

    /* Add gateway as static ARP entry if MAC provided */
    if (gateway_mac && !mac_is_zero_6(gateway_mac)) {
        arp_add_static(gateway, gateway_mac);
    }

    net_save_iface();
    net_firewall_install_defaults();

    uart_puts("[net] IP=");
    uart_hex(ip);
    uart_puts(" GW=");
    uart_hex(gateway);
    uart_puts(" (ARP+TCP/UDP)\n");

    /* Announce our presence on the network */
    pcie_aer_dump("pre-ARP");
    arp_announce();
    pcie_aer_dump("post-ARP");

    uart_puts("[net] post-announce:\n");
    uart_puts("[net] link="); uart_hex(nic_link_up() ? 1 : 0); uart_puts("\n");
}

bool net_add_interface(nic_iface_t iface, u32 ip, u32 gateway, u32 netmask,
                       const u8 *gateway_mac)
{
    if (iface != NIC_IFACE_WIRED && iface != NIC_IFACE_WIFI)
        return false;
    if (!nic_iface_active(iface) || ip == 0U)
        return false;

    nic_iface_t previous = net_current_iface;
    net_save_iface();
    net_current_iface = iface;
    our_ip = ip;
    our_gw = gateway;
    our_mask = netmask;
    nic_get_mac_for(iface, our_mac);
    nic_set_local_ipv4_for(iface, ip);
    simd_zero(neighbors, sizeof(neighbors));
    simd_zero(routes, sizeof(routes));
    simd_zero(multicast_macs, sizeof(multicast_macs));
    neighbor_count = 0;
    route_count = 0;
    multicast_count = 0;
    simd_zero(gw_mac, sizeof(gw_mac));
    gw_mac_set = false;
    if (gateway_mac && !mac_is_zero_6(gateway_mac)) {
        simd_memcpy(gw_mac, gateway_mac, 6);
        gw_mac_set = true;
    }
    arp_init_iface(iface, ip, netmask, our_mac);
    (void)net_route_add(ip & netmask, netmask, 0, NET_ROUTE_F_CONNECTED);
    if (gateway != 0U)
        (void)net_route_add(0, 0, gateway, 0);
    if (gateway_mac && !mac_is_zero_6(gateway_mac))
        arp_add_static_iface(iface, gateway, gateway_mac);
    /*
     * Make the newly configured address visible without the blocking
     * multi-probe boot announcement. A single gratuitous ARP is bounded and
     * lets peers refresh their neighbor entry before the first service SYN.
     */
    arp_probe();
    net_save_iface();
    (void)net_load_iface(previous);
    net_firewall_install_defaults();
    return true;
}

bool net_interface_configured(nic_iface_t iface)
{
    return iface < NIC_IFACE_MAX && net_ifaces[iface].configured;
}

nic_iface_t net_current_interface(void)
{
    return net_current_iface;
}

u32 net_get_our_ip_for(nic_iface_t iface)
{
    return iface < NIC_IFACE_MAX ? net_ifaces[iface].ip : 0U;
}

u32 net_get_netmask_for(nic_iface_t iface)
{
    return iface < NIC_IFACE_MAX ? net_ifaces[iface].mask : 0U;
}

void net_get_mac_for(nic_iface_t iface, u8 *mac)
{
    if (mac)
        simd_zero(mac, 6);
    if (iface < NIC_IFACE_MAX && net_ifaces[iface].configured && mac)
        simd_memcpy(mac, net_ifaces[iface].mac, 6);
}

void net_set_udp_callback(udp_recv_cb cb) {
    udp_callback = cb;
}

udp_recv_cb net_swap_udp_callback(udp_recv_cb cb) {
    udp_recv_cb prev = udp_callback;
    udp_callback = cb;
    return prev;
}

bool net_udp_subscribe(udp_recv_cb cb)
{
    if (!cb) return false;
    for (u32 i = 0; i < UDP_SUBSCRIBER_MAX; i++) {
        if (udp_subscribers[i].cb == cb)
            return true;
    }
    for (u32 i = 0; i < UDP_SUBSCRIBER_MAX; i++) {
        if (!udp_subscribers[i].cb) {
            udp_subscribers[i].cb = cb;
            return true;
        }
    }
    return false;
}

bool net_udp_unsubscribe(udp_recv_cb cb)
{
    if (!cb) return false;
    for (u32 i = 0; i < UDP_SUBSCRIBER_MAX; i++) {
        if (udp_subscribers[i].cb == cb) {
            udp_subscribers[i].cb = NULL;
            return true;
        }
    }
    return false;
}

u32 net_get_our_ip(void) {
    return our_ip;
}

u32 net_get_netmask(void) {
    return our_mask;
}

const net_stats_t *net_get_stats(void) {
    return &stats;
}
