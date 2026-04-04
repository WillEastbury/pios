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

/* ---- Network state ---- */

static u32 our_ip;
static u32 our_gw;
static u32 our_mask;
static u8  our_mac[6];

static udp_recv_cb udp_callback;
static udp_recv_cb udp_subscribers[4];
static net_stats_t stats;

static bool mac_is_zero_6(const u8 *m) {
    return m[0]==0 && m[1]==0 && m[2]==0 && m[3]==0 && m[4]==0 && m[5]==0;
}

/* ---- Static neighbor table (replaces ARP) ---- */

static struct neighbor_entry neighbors[MAX_NEIGHBORS];
static u32 neighbor_count;
static u8  gw_mac[6];
static bool gw_mac_set;

/* ---- ICMP rate limiter ---- */

static u64 icmp_last_tick;
static u64 icmp_min_interval;

/* ---- Frame buffers ---- */

static u8 rx_frame[2048] ALIGNED(64);
static u8 tx_frame[2048] ALIGNED(64);
#define NET_RX_BURST_MAX 4U
#define NET_FIFO_BURST_MAX 4U

static u16 ip_id_counter;
static volatile bool net_maint_queued;

static void net_maintenance_work(void *ctx)
{
    (void)ctx;
    arp_tick();
    tcp_tick();
    net_maint_queued = false;
}

static void net_tick_hook(u32 core, u64 tick)
{
    if (core != CORE_NET)
        return;
    if ((tick % 100U) != 0)
        return;
    if (net_maint_queued)
        return;
    if (workq_enqueue(CORE_NET, net_maintenance_work, NULL))
        net_maint_queued = true;
}

/* ================================================================== */
/*  Static neighbor table                                              */
/* ================================================================== */

void net_add_neighbor(u32 ip, const u8 *mac) {
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

static void handle_icmp(const u8 *frame, u32 len,
                        struct ip_hdr *ip, u32 payload_off) {
    (void)len;
    u16 ipt = ntohs(ip->total_len);
    if (ipt < 20 + sizeof(struct icmp_hdr)) return;
    u32 icmp_len = ipt - 20;
    /* Cap to tx_frame capacity (prevent buffer overflow on large ICMP) */
    if (sizeof(struct eth_hdr) + ipt > sizeof(tx_frame)) return;

    struct icmp_hdr *icmp = (struct icmp_hdr *)(frame + payload_off);

    if (icmp->type != 8 || icmp->code != 0)
        return;

    if (unlikely(!icmp_rate_ok())) {
        stats.drop_icmp_ratelimit++;
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
    nic_send(tx_frame, frame_len);
    stats.icmp_echo_replies++;
    stats.tx_packets++;
    stats.tx_bytes += frame_len;
}

/* ================================================================== */
/*  UDP                                                                */
/* ================================================================== */

static void handle_udp(const u8 *frame, u32 len,
                       struct ip_hdr *ip, u32 payload_off) {
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

    stats.udp_recv++;

    if (udp_callback) {
        udp_callback(ntohl(ip->src_ip),
                     ntohs(udp->src_port),
                     ntohs(udp->dst_port),
                     data, data_len);
    }
    for (u32 i = 0; i < (u32)(sizeof(udp_subscribers) / sizeof(udp_subscribers[0])); i++) {
        if (udp_subscribers[i]) {
            udp_subscribers[i](ntohl(ip->src_ip),
                               ntohs(udp->src_port),
                               ntohs(udp->dst_port),
                               data, data_len);
        }
    }
}

/* ================================================================== */
/*  IP - hardened ingress validation                                   */
/* ================================================================== */

static void handle_ip(const u8 *frame, u32 len, bool checksum_trusted) {
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
                      checksum_trusted);
        break;
    }
    case IP_PROTO_UDP:  handle_udp(frame, len, ip, payload_off);  break;
    default: stats.drop_bad_proto++; break;
    }
}

/* ================================================================== */
/*  TX path - neighbor resolution (static table + ARP)                 */
/* ================================================================== */

static bool resolve_mac(u32 dst_ip, const u8 **mac_out) {
    u32 next_hop = ((dst_ip & our_mask) == (our_ip & our_mask))
                    ? dst_ip : our_gw;

    /* Broadcast */
    if (dst_ip == 0xFFFFFFFF) {
        static const u8 bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
        *mac_out = bcast;
        return true;
    }

    /* Try static gateway MAC first */
    if (next_hop == our_gw && gw_mac_set) {
        *mac_out = gw_mac;
        return true;
    }

    /* Try static neighbor table */
    const u8 *mac = neighbor_lookup(next_hop);
    if (mac) {
        *mac_out = mac;
        return true;
    }

    /* Fall through to ARP */
    mac = arp_resolve(next_hop);
    if (mac) {
        *mac_out = mac;
        return true;
    }

    stats.drop_no_neighbor++;
    return false;
}

bool net_send_udp(u32 dst_ip, u16 src_port, u16 dst_port,
                  const u8 *data, u16 len) {
    const u8 *dst_mac;
    if (unlikely(!resolve_mac(dst_ip, &dst_mac)))
        return false;

    u16 udp_len  = sizeof(struct udp_hdr) + len;
    u16 ip_total = 20 + udp_len;

    if (unlikely(sizeof(struct eth_hdr) + ip_total > 1514))
        return false;

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
    udp->checksum  = 0;    /* optional in IPv4 */

    u32 frame_len = sizeof(struct eth_hdr) + ip_total;
    bool need_pad = frame_len < 60;
    if (need_pad) frame_len = 60;

    stats.tx_packets++;
    stats.tx_bytes += frame_len;
    stats.udp_sent++;

    if (len == 0)
        return nic_send(tx_frame, frame_len);

    if (need_pad) {
        simd_memcpy(tx_frame + sizeof(struct eth_hdr) + 20 + sizeof(struct udp_hdr),
                    data, len);
        return nic_send(tx_frame, frame_len);
    }

    u32 head_len = sizeof(struct eth_hdr) + 20 + sizeof(struct udp_hdr);
    return nic_send_parts(tx_frame, head_len, data, len);
}

/* ================================================================== */
/*  FIFO integration - user cores send UDP via messages                */
/* ================================================================== */

void net_handle_fifo_request(void) {
    struct fifo_msg msg;
    struct fifo_msg reply;

    for (u32 i = 0; i < NET_FIFO_BURST_MAX; i++) {
        if (!fifo_pop(CORE_NET, CORE_USER0, &msg))
            break;
        if (msg.type == MSG_NET_UDP_SEND && msg.buffer && msg.length <= 1472) {
            u16 sp = (u16)(msg.tag >> 16);
            u16 dp = (u16)(msg.tag & 0xFFFF);
            bool ok = net_send_udp(msg.param, sp, dp,
                                   (const u8 *)(usize)msg.buffer, (u16)msg.length);
            reply.type   = MSG_NET_UDP_DONE;
            reply.status = ok ? 0 : 1;
            reply.tag    = msg.tag;
            fifo_push(CORE_NET, CORE_USER0, &reply);
        }
    }

    for (u32 i = 0; i < NET_FIFO_BURST_MAX; i++) {
        if (!fifo_pop(CORE_NET, CORE_USER1, &msg))
            break;
        if (msg.type == MSG_NET_UDP_SEND && msg.buffer && msg.length <= 1472) {
            u16 sp = (u16)(msg.tag >> 16);
            u16 dp = (u16)(msg.tag & 0xFFFF);
            bool ok = net_send_udp(msg.param, sp, dp,
                                   (const u8 *)(usize)msg.buffer, (u16)msg.length);
            reply.type   = MSG_NET_UDP_DONE;
            reply.status = ok ? 0 : 1;
            reply.tag    = msg.tag;
            fifo_push(CORE_NET, CORE_USER1, &reply);
        }
    }

    /* Drain socket-layer FIFO requests in small bursts per poll pass. */
    for (u32 i = 0; i < NET_FIFO_BURST_MAX; i++)
        socket_handle_fifo(CORE_USER0);
    for (u32 i = 0; i < NET_FIFO_BURST_MAX; i++)
        socket_handle_fifo(CORE_USER1);
}

/* ================================================================== */
/*  Poll + Init                                                        */
/* ================================================================== */

void net_poll(void) {
    u32 len;
    bool checksum_trusted;
    static u32 poll_count;
    static u32 rx_count;

    /* Log occasionally to prove polling is running */
    if ((poll_count & 0xFFFFF) == 0 && poll_count > 0) {
        uart_puts("[net] poll=");
        uart_hex(poll_count);
        uart_puts(" rx=");
        uart_hex(rx_count);
        uart_puts("\n");
    }
    poll_count++;

    prefetch_r(rx_frame);

    for (u32 burst = 0; burst < NET_RX_BURST_MAX; burst++) {
        checksum_trusted = false;
        if (!likely(nic_recv(rx_frame, &len, &checksum_trusted)))
            break;

        rx_count++;
        stats.rx_packets++;
        stats.rx_bytes += len;

        /* Log first few received frames */
        if (rx_count <= 5) {
            uart_puts("[net] RX#");
            uart_hex(rx_count);
            uart_puts(" len=");
            uart_hex(len);
            uart_puts(" eth=");
            for (u32 i = 0; i < 14 && i < len; i++) {
                static const char hex[] = "0123456789ABCDEF";
                uart_putc(hex[rx_frame[i] >> 4]);
                uart_putc(hex[rx_frame[i] & 0xF]);
            }
            uart_puts("\n");
        }

        if (unlikely(len < sizeof(struct eth_hdr))) {
            stats.drop_runt++;
            return;
        }
        if (unlikely(len > 1518)) {
            stats.drop_oversized++;
            return;
        }

        struct eth_hdr *eth = (struct eth_hdr *)rx_frame;
        u16 etype = ntohs(eth->ethertype);

        /* Dispatch by EtherType */
        if (likely(etype == ETH_P_IP))
            handle_ip(rx_frame, len, checksum_trusted);
        else if (etype == ETH_P_ARP)
            arp_input(rx_frame, len);
    }

    net_handle_fifo_request();

    workq_drain(4);
}

void net_init(u32 ip, u32 gateway, u32 netmask, const u8 *gateway_mac) {
    our_ip   = ip;
    our_gw   = gateway;
    our_mask = netmask;
    nic_get_mac(our_mac);

    if (gateway_mac) {
        simd_memcpy(gw_mac, gateway_mac, 6);
        gw_mac_set = true;
    }

    simd_zero(&stats, sizeof(stats));
    simd_zero(neighbors, sizeof(neighbors));
    neighbor_count = 0;
    udp_callback   = NULL;
    for (u32 i = 0; i < (u32)(sizeof(udp_subscribers) / sizeof(udp_subscribers[0])); i++)
        udp_subscribers[i] = NULL;
    ip_id_counter  = 1;

    u64 freq = read_cntfrq();
    icmp_min_interval = freq / 10;
    icmp_last_tick    = 0;

    /* Init ARP subsystem */
    arp_init(ip, netmask, our_mac);

    /* Init TCP subsystem */
    tcp_init();

    /* Init socket layer */
    socket_init();

    /* Enable safe NIC checksum assist paths by default. */
    nic_set_rx_checksum_offload(true);
    nic_set_tx_checksum_offload(true);
    nic_set_tso(true);

    net_maint_queued = false;
    timer_set_tick_hook(net_tick_hook);

    /* Init TLS wrapper subsystem */
    tls_init();

    /* Add gateway as static ARP entry if MAC provided */
    if (gateway_mac && !mac_is_zero_6(gateway_mac)) {
        arp_add_static(gateway, gateway_mac);
    }

    uart_puts("[net] Hardened stack: IP=");
    uart_hex(ip);
    uart_puts(" GW=");
    uart_hex(gateway);
    uart_puts(" (ARP hardened, TCP/UDP, NO DHCP)\n");

    /* Announce our presence on the network */
    pcie_aer_dump("pre-ARP");
    arp_announce();
    pcie_aer_dump("post-ARP");

    uart_puts("[net] Post-announce NIC state:\n");
    uart_puts("[net] nic_link="); uart_hex(nic_link_up() ? 1 : 0); uart_puts("\n");
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
    for (u32 i = 0; i < (u32)(sizeof(udp_subscribers) / sizeof(udp_subscribers[0])); i++) {
        if (udp_subscribers[i] == cb)
            return true;
    }
    for (u32 i = 0; i < (u32)(sizeof(udp_subscribers) / sizeof(udp_subscribers[0])); i++) {
        if (!udp_subscribers[i]) {
            udp_subscribers[i] = cb;
            return true;
        }
    }
    return false;
}

bool net_udp_unsubscribe(udp_recv_cb cb)
{
    if (!cb) return false;
    for (u32 i = 0; i < (u32)(sizeof(udp_subscribers) / sizeof(udp_subscribers[0])); i++) {
        if (udp_subscribers[i] == cb) {
            udp_subscribers[i] = NULL;
            return true;
        }
    }
    return false;
}

u32 net_get_our_ip(void) {
    return our_ip;
}

const net_stats_t *net_get_stats(void) {
    return &stats;
}
