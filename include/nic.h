#pragma once
#include "types.h"

#define ETH_FRAME_MAX   1518
#define ETH_ALEN        6

bool nic_init(void);
bool nic_init_wifi(void);
bool nic_is_wifi(void);
bool nic_send(const u8 *frame, u32 len);
bool nic_send_parts(const void *head, u32 head_len, const void *tail, u32 tail_len);
bool nic_recv(u8 *frame, u32 *len, bool *checksum_trusted);
void nic_get_mac(u8 *mac);
bool nic_link_up(void);

void nic_set_tx_checksum_offload(bool enable);
void nic_set_rx_checksum_offload(bool enable);
void nic_set_tso(bool enable);
bool nic_tx_checksum_offload_enabled(void);
bool nic_rx_checksum_offload_enabled(void);
bool nic_tso_enabled(void);

/* Configure our host-order IPv4 address for the earliest ARP RX filter.
 * Broadcast ARP frames whose target protocol address is not this IP are
 * dropped inside nic_recv(), before framebuffer dumps or protocol dispatch. */
void nic_set_local_ipv4(u32 ip);
void nic_set_packet_dump(bool enable);

typedef struct {
    u64 rx_bytes, tx_bytes;
    u64 rx_total, tx_total;
    u64 rx_arp, tx_arp;
    u64 rx_arp_req, tx_arp_req;
    u64 rx_arp_rep, tx_arp_rep;
    u64 rx_ip, tx_ip;
    u64 rx_icmp, tx_icmp;
    u64 rx_tcp, tx_tcp;
    u64 rx_udp, tx_udp;
    u64 rx_other, tx_other;
    u64 rx_arp_not_us;
    u64 rx_filter_drop, tx_filter_drop;
    u64 processed;
    u64 dropped;
    u64 firewalled;
    u64 flood_blocked;
    u64 rate_limited;
} nic_packet_counters_t;

void nic_packet_counters(nic_packet_counters_t *out);
void nic_record_rate_limited(void);

/* Packet firewall at the NIC boundary.
 *
 * Rules are evaluated in insertion order; first match wins. If no rule
 * matches, the per-direction default policy is used (default: allow).
 *
 * "to" means destination, "from" means source. IP addresses are host-order
 * IPv4 values (use IP4(a,b,c,d) from net.h when constructing rules).
 */
#define NIC_FILTER_MAX_RULES 32U

#define NIC_FILTER_DIR_IN     0x01U
#define NIC_FILTER_DIR_OUT    0x02U
#define NIC_FILTER_DIR_BOTH   (NIC_FILTER_DIR_IN | NIC_FILTER_DIR_OUT)

#define NIC_FILTER_ALLOW      0U
#define NIC_FILTER_DROP       1U

#define NIC_FILTER_MAC_TO        (1U << 0)
#define NIC_FILTER_MAC_FROM      (1U << 1)
#define NIC_FILTER_ETHERTYPE     (1U << 2)  /* L2 protocol id, e.g. 0x0800/0x0806 */
#define NIC_FILTER_IP_PROTO      (1U << 3)  /* L3 protocol id, e.g. 1/6/17 */
#define NIC_FILTER_IP_TO         (1U << 4)
#define NIC_FILTER_IP_FROM       (1U << 5)
#define NIC_FILTER_TCP_PORT_TO   (1U << 6)
#define NIC_FILTER_TCP_PORT_FROM (1U << 7)
#define NIC_FILTER_UDP_PORT_TO   (1U << 8)
#define NIC_FILTER_UDP_PORT_FROM (1U << 9)
#define NIC_FILTER_IP_TO_RANGE   (1U << 10)
#define NIC_FILTER_IP_FROM_RANGE (1U << 11)
#define NIC_FILTER_TCP_PORT_TO_RANGE   (1U << 12)
#define NIC_FILTER_TCP_PORT_FROM_RANGE (1U << 13)
#define NIC_FILTER_UDP_PORT_TO_RANGE   (1U << 14)
#define NIC_FILTER_UDP_PORT_FROM_RANGE (1U << 15)

typedef struct {
    u32 flags;
    u8  direction;   /* NIC_FILTER_DIR_* */
    u8  action;      /* NIC_FILTER_ALLOW or NIC_FILTER_DROP */
    u8  mac_to[6];
    u8  mac_from[6];
    u16 ethertype;
    u8  ip_proto;
    u8  _pad;
    u32 ip_to;
    u32 ip_from;
    u32 ip_to_mask;      /* optional with IP_TO; 0 means exact match */
    u32 ip_from_mask;    /* optional with IP_FROM; 0 means exact match */
    u32 ip_to_end;       /* optional with IP_TO_RANGE */
    u32 ip_from_end;     /* optional with IP_FROM_RANGE */
    u16 tcp_port_to;
    u16 tcp_port_from;
    u16 udp_port_to;
    u16 udp_port_from;
    u16 tcp_port_to_end;
    u16 tcp_port_from_end;
    u16 udp_port_to_end;
    u16 udp_port_from_end;
} nic_filter_rule_t;

void nic_filter_clear(void);
void nic_filter_set_default(bool allow_in, bool allow_out);
bool nic_filter_add(const nic_filter_rule_t *rule);
bool nic_filter_add_front(const nic_filter_rule_t *rule);
bool nic_filter_remove(u32 index);
bool nic_filter_get(u32 index, nic_filter_rule_t *out);
u32  nic_filter_count(void);
void nic_filter_stats(u64 *rx_dropped, u64 *tx_dropped);
