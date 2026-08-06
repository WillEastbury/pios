#pragma once
#include "types.h"

/*
 * Hardened minimal network stack: IPv4 + ICMP + UDP + TCP + ARP.
 * NO DHCP. NO DNS. NO fragmentation.
 * Static IP + static-neighbor model with strict ingress validation.
 *
 * Security posture:
 *   - ARP hardened: rate-limited, anti-gratuitous, MAC consistency checks
 *   - No DHCP = immune to rogue DHCP / starvation
 *   - TCP hardened with SYN cookies and strict sequence/RST validation
 *   - No fragments = immune to teardrop / overlap / ping-of-death
 *   - No IP options = immune to source-routing attacks
 *   - ICMP rate-limited = resistant to ping floods
 *   - Strict validation = drops malformed packets early
 */

#define ETH_P_IP        0x0800
#define ETH_P_ARP       0x0806
#define IP_PROTO_ICMP   1
#define IP_PROTO_TCP    6
#define IP_PROTO_UDP    17

/* Network byte order helpers */
static inline u16 htons(u16 v) { return (v >> 8) | (v << 8); }
static inline u16 ntohs(u16 v) { return htons(v); }
static inline u32 htonl(u32 v) {
    return ((v >> 24) & 0xFF) | ((v >> 8) & 0xFF00) |
           ((v << 8) & 0xFF0000) | ((v << 24) & 0xFF000000);
}
static inline u32 ntohl(u32 v) { return htonl(v); }

#define IP4(a,b,c,d) (((u32)(a)<<24)|((u32)(b)<<16)|((u32)(c)<<8)|(u32)(d))

/* Ethernet header */
struct eth_hdr {
    u8  dst[6];
    u8  src[6];
    u16 ethertype;
} PACKED;

/* IPv4 header (fixed 20 bytes - we reject options) */
struct ip_hdr {
    u8  ver_ihl;
    u8  tos;
    u16 total_len;
    u16 id;
    u16 flags_frag;
    u8  ttl;
    u8  protocol;
    u16 checksum;
    u32 src_ip;
    u32 dst_ip;
} PACKED;

/* ICMP header */
struct icmp_hdr {
    u8  type;
    u8  code;
    u16 checksum;
    u16 id;
    u16 seq;
} PACKED;

/* UDP header */
struct udp_hdr {
    u16 src_port;
    u16 dst_port;
    u16 length;
    u16 checksum;
} PACKED;

/* Static neighbor entry (replaces ARP entirely) */
#define MAX_NEIGHBORS   16
#define NET_ROUTE_MAX   16

struct neighbor_entry {
    u32 ip;
    u8  mac[6];
    u8  _pad[2];
};

struct net_route_entry {
    u32 dst;
    u32 mask;
    u32 gateway;
    u8  prefix_len;
    u8  flags;
    u16 _pad;
} PACKED;

#define NET_ROUTE_F_CONNECTED 1U
bool net_route_add(u32 dst, u32 mask, u32 gateway, u8 flags);
u32 net_route_snapshot(struct net_route_entry *out, u32 max);
bool net_route_lookup(u32 dst_ip, struct net_route_entry *out);

#define NET_EGRESS_MAC_NONE       0U
#define NET_EGRESS_MAC_BCAST      1U
#define NET_EGRESS_MAC_GW_STATIC  2U
#define NET_EGRESS_MAC_NEIGHBOR   3U
#define NET_EGRESS_MAC_ARP        4U
#define NET_EGRESS_MAC_NO_ROUTE   5U
#define NET_EGRESS_MAC_NO_MAC     6U

struct net_egress_snapshot {
    u64 resolve_calls;
    u64 no_route;
    u64 no_mac;
    u64 udp_attempts;
    u64 udp_ok;
    u64 udp_fail;
    u32 last_dst_ip;
    u32 last_next_hop;
    u32 last_route_dst;
    u32 last_route_mask;
    u32 last_route_gateway;
    u32 last_udp_dst_ip;
    u32 last_udp_next_hop;
    u8  last_route_flags;
    u8  last_route_prefix;
    u8  last_mac_source;
    u8  last_udp_ok;
    u8  last_udp_mac_source;
    u8  _pad0;
    u16 last_udp_src_port;
    u16 last_udp_dst_port;
    u16 last_udp_len;
    u8  last_mac[6];
    u8  last_udp_mac[6];
} PACKED;

void net_egress_snapshot(struct net_egress_snapshot *out);

/* UDP receive callback */
typedef void (*udp_recv_cb)(u32 src_ip, u16 src_port, u16 dst_port,
                            const u8 *data, u16 len);

/* Init with static IP, gateway IP, and gateway MAC */
void net_init(u32 ip, u32 gateway, u32 netmask, const u8 *gateway_mac);

/* Restore default NIC firewall: inbound deny, outbound allow, and service allows. */
void net_firewall_install_defaults(void);

/* Add a static neighbor (IP → MAC mapping). No ARP. */
void net_add_neighbor(u32 ip, const u8 *mac);

/* Resolve destination IP to next-hop Ethernet MAC using route/gateway policy,
 * static neighbors, and dynamic ARP. Returns NULL while ARP is unresolved. */
const u8 *net_resolve_mac(u32 dst_ip);

/* Join/leave an Ethernet multicast destination MAC. Frames for multicast
 * groups not in this table are dropped before protocol dispatch. */
bool net_join_multicast_mac(const u8 *mac);
bool net_leave_multicast_mac(const u8 *mac);

/* Poll: process one incoming frame (call in tight loop on core 0) */
u32 net_poll(void);

/* Send a UDP datagram */
bool net_send_udp(u32 dst_ip, u16 src_port, u16 dst_port,
                  const u8 *data, u16 len);

/* ---- ICMP echo client (ping/traceroute) -------------------------------
 * Single-outstanding-probe, core-0-only client API (mirrors net_send_udp).
 * Caller sends one probe, then polls net_poll() + net_icmp_echo_poll_result()
 * in a bounded loop (same pattern as ui_http_fetch()) until a reply/timeout. */
struct net_ping_result {
    bool got_reply;         /* true: genuine echo reply from dst_ip */
    bool got_ttl_exceeded;  /* true: ICMP Time Exceeded from an intermediate hop */
    u32  from_ip;           /* IP that actually replied */
    u32  rtt_ms;
    u8   reply_ttl;
} PACKED;

/* Send one ICMP echo request with the given identifier/sequence/TTL. */
bool net_icmp_echo_send(u32 dst_ip, u16 ident, u16 seq, u8 ttl);

/* Non-blocking: true + fills *out and clears the pending state once a
 * matching reply (or Time Exceeded) has arrived since the last send. */
bool net_icmp_echo_poll_result(struct net_ping_result *out);

/* Process a UDP send request from another core via FIFO */
void net_handle_fifo_request(void);

void net_set_udp_callback(udp_recv_cb cb);
udp_recv_cb net_swap_udp_callback(udp_recv_cb cb);
bool net_udp_subscribe(udp_recv_cb cb);
bool net_udp_unsubscribe(udp_recv_cb cb);

/* Detailed stats with per-drop-reason counters */
typedef struct {
    u64 tx_packets;
    u64 rx_packets;
    u64 tx_bytes;
    u64 rx_bytes;
    u64 icmp_echo_replies;
    u64 udp_recv;
    u64 udp_sent;
    /* Drop counters - attack diagnostics */
    u64 drop_runt;          /* frame too short */
    u64 drop_bad_cksum;     /* IP checksum failed */
    u64 drop_fragment;      /* IP fragment (MF/offset) */
    u64 drop_ip_options;    /* IHL > 5 (IP options present) */
    u64 drop_bad_src;       /* invalid source IP */
    u64 drop_not_for_us;    /* dst IP not ours */
    u64 drop_bad_proto;     /* unsupported IP protocol */
    u64 drop_icmp_ratelimit;/* ICMP throttled */
    u64 drop_no_neighbor;   /* no static MAC for dst */
    u64 drop_udp_malformed; /* UDP length mismatch */
    u64 drop_oversized;     /* frame > MTU */
    u64 drop_udp_bad_cksum;  /* UDP checksum failed */
    u64 rx_dispatched;      /* accepted frames whose protocol handler returned */
    u64 rx_unsupported;     /* accepted EtherTypes with no protocol handler */
    u64 poll_calls;         /* lifetime net_poll() invocations */
    u64 poll_empty;         /* net_poll() calls that received no frames */
    u64 poll_budget_hits;   /* calls that consumed the full RX burst budget */
    u32 poll_last_frames;   /* frames received by the most recent net_poll() */
} net_stats_t;

const net_stats_t *net_get_stats(void);

/* Get our configured IP address (for use by TCP, etc.) */
u32 net_get_our_ip(void);
u32 net_get_netmask(void);
