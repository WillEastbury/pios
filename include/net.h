#pragma once
#include "types.h"

/*
 * Hardened minimal network stack: IPv4 + ICMP + UDP.
 * NO TCP. NO ARP. NO DHCP. NO DNS. NO fragmentation.
 * Static IP. Static neighbor table. Maximum attack resistance.
 *
 * Security posture:
 *   - No ARP = immune to ARP spoofing/cache poisoning
 *   - No DHCP = immune to rogue DHCP / starvation
 *   - No TCP = immune to SYN floods / RST / seq prediction
 *   - No fragments = immune to teardrop / overlap / ping-of-death
 *   - No IP options = immune to source-routing attacks
 *   - ICMP rate-limited = resistant to ping floods
 *   - Strict validation = drops malformed packets early
 *   - Static neighbors = no dynamic state to exhaust
 */

#define ETH_P_IP        0x0800
#define IP_PROTO_ICMP   1
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

struct neighbor_entry {
    u32 ip;
    u8  mac[6];
    u8  _pad[2];
};

/* UDP receive callback */
typedef void (*udp_recv_cb)(u32 src_ip, u16 src_port, u16 dst_port,
                            const u8 *data, u16 len);

/* Init with static IP, gateway IP, and gateway MAC */
void net_init(u32 ip, u32 gateway, u32 netmask, const u8 *gateway_mac);

/* Add a static neighbor (IP → MAC mapping). No ARP. */
void net_add_neighbor(u32 ip, const u8 *mac);

/* Poll: process one incoming frame (call in tight loop on core 0) */
void net_poll(void);

/* Send a UDP datagram */
bool net_send_udp(u32 dst_ip, u16 src_port, u16 dst_port,
                  const u8 *data, u16 len);

/* Process a UDP send request from another core via FIFO */
void net_handle_fifo_request(void);

void net_set_udp_callback(udp_recv_cb cb);

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
} net_stats_t;

const net_stats_t *net_get_stats(void);
