/*
 * nic.c - Active NIC backend for Raspberry Pi 5
 *
 * Pi 5 networking is provided by RP1 Cadence MACB/GEM (Ethernet).
 * WiFi support (CYW43455 via SDIO) is parked in spike/wifi/ — see
 * GitHub issue. nic_init_wifi() is retained as a stub returning false.
 */

#include "nic.h"
#include "macb.h"
#include "virtio_net.h"
#include "tcp.h"
#include "socket.h"
#include "simd.h"
#include "fb.h"
#include "timer.h"
#include "core_env.h"
#include "platform.h"
#include "pioscap.h"

static bool tx_checksum_offload;
static bool rx_checksum_offload;
static bool tso_enabled;
static bool packet_dump_enabled;
static u32 local_ipv4;
static nic_packet_counters_t pkt_counts;

/* ── Pluggable NIC backend registry ───────────────────────────────────────
 * Backends are probed in array order; the first whose hardware is detected
 * is activated. macb (Pi5 wired GEM) takes priority over virtio-net (QEMU),
 * which takes priority over any future software/wireless backend. Symbols for
 * every backend exist in all builds (the inactive ones link to stubs), so the
 * table needs no per-platform #if. */

static bool macb_backend_probe(void)
{
    /* The MACB/GEM register window is only mapped on platforms that have it;
     * gate on the platform capability so we never touch unmapped MMIO. */
    return PIOS_HAS_GENET ? true : false;
}

static bool virtio_backend_recv(u8 *frame, u32 *len, bool *checksum_trusted)
{
    if (checksum_trusted)
        *checksum_trusted = false;   /* virtio-net carries no csum-trust signal here */
    return virtio_net_recv(frame, len);
}

static u32 virtio_backend_link_mbps(void)
{
    return virtio_net_link_up() ? 1000U : 0U;
}

static const struct nic_ops nic_backend_macb = {
    .name        = "macb",
    .probe       = macb_backend_probe,
    .init        = macb_init,
    .send        = macb_send,
    .recv        = macb_recv,
    .get_mac     = macb_get_mac,
    .link_up     = macb_link_up,
    .link_mbps   = macb_link_mbps,
    .full_duplex = macb_link_full_duplex,
    .set_tx_csum = macb_set_tx_checksum_offload,
    .set_rx_csum = macb_set_rx_checksum_offload,
    .set_tso     = macb_set_tso,
    .tx_csum_enabled = macb_tx_checksum_offload_enabled,
    .rx_csum_enabled = macb_rx_checksum_offload_enabled,
    .tso_enabled = macb_tso_enabled,
    .offload_regs = macb_offload_regs,
};

static const struct nic_ops nic_backend_virtio = {
    .name        = "virtio-net",
    .probe       = virtio_net_probe,
    .init        = virtio_net_init,
    .send        = virtio_net_send,
    .recv        = virtio_backend_recv,
    .get_mac     = virtio_net_get_mac,
    .link_up     = virtio_net_link_up,
    .link_mbps   = virtio_backend_link_mbps,
    .full_duplex = virtio_net_link_up,
    /* no offload hooks */
};

static const struct nic_ops *const nic_backends[] = {
    &nic_backend_macb,
    &nic_backend_virtio,
};

static const struct nic_ops *g_nic;     /* active backend, or NULL */

bool nic_active(void) { return g_nic != 0; }
const char *nic_active_name(void) { return g_nic ? g_nic->name : "none"; }

#define NIC_HDMI_TEXT_PANELS 0

#define FLOW_PROTO_ARP 0xFEU
#define NIC_FLOW_MAX 24U
#define FLOW_SEC_BUCKETS 60U
#define FLOW_5S_BUCKETS 60U
#define FLOW_MIN_BUCKETS 60U
#define FLOW_HOUR_BUCKETS 24U

struct flow_bucket {
    u32 slot;
    u32 packets;
};

struct flow_key {
    u8 dir;        /* NIC_FILTER_DIR_IN / OUT */
    u8 proto;      /* ARP pseudo-proto, ICMP/TCP/UDP IP proto */
    u16 subtype;   /* ARP op or ICMP type; 0 for TCP/UDP */
    u32 ip_from;
    u32 ip_to;
    u16 port_from;
    u16 port_to;
};

struct flow_entry {
    bool used;
    struct flow_key key;
    u64 all_packets;
    struct flow_bucket sec[FLOW_SEC_BUCKETS];   /* 1m: 60 x 1s */
    struct flow_bucket five[FLOW_5S_BUCKETS];   /* 5m: 60 x 5s */
    struct flow_bucket min[FLOW_MIN_BUCKETS];   /* 1h: 60 x 1m */
    struct flow_bucket hour[FLOW_HOUR_BUCKETS]; /* today-ish: 24 x 1h */
};

static struct flow_entry flow_table[NIC_FLOW_MAX];

static nic_filter_rule_t filter_rules[NIC_FILTER_MAX_RULES];
static u32 filter_rule_count;
static bool filter_default_in = true;
static bool filter_default_out = true;
static u64 filter_rx_dropped;
static u64 filter_tx_dropped;

static void fb_hex2(u8 v) {
    static const char hex[] = "0123456789ABCDEF";
    fb_putc(hex[v >> 4]);
    fb_putc(hex[v & 0xF]);
}

static void fb_hex3(u32 v) {
    static const char hex[] = "0123456789ABCDEF";
    fb_putc(hex[(v >> 8) & 0xF]);
    fb_putc(hex[(v >> 4) & 0xF]);
    fb_putc(hex[v & 0xF]);
}

static void fb_hex8(u32 v) {
    fb_hex2((u8)(v >> 24));
    fb_hex2((u8)(v >> 16));
    fb_hex2((u8)(v >> 8));
    fb_hex2((u8)v);
}

static void fb_dec8(u8 v) {
    if (v >= 100) {
        fb_putc('0' + v / 100);
        v %= 100;
        fb_putc('0' + v / 10);
        fb_putc('0' + v % 10);
    } else if (v >= 10) {
        fb_putc('0' + v / 10);
        fb_putc('0' + v % 10);
    } else {
        fb_putc('0' + v);
    }
}

static void fb_dec16(u16 v) {
    u16 div = 10000;
    bool started = false;
    while (div) {
        u8 d = (u8)(v / div);
        if (d || started || div == 1) {
            fb_putc('0' + d);
            started = true;
        }
        v %= div;
        div /= 10;
    }
}

static void fb_ip(const u8 *p) {
    fb_dec8(p[0]); fb_putc('.');
    fb_dec8(p[1]); fb_putc('.');
    fb_dec8(p[2]); fb_putc('.');
    fb_dec8(p[3]);
}

static void fb_mac(const u8 *p) {
    for (u32 i = 0; i < 6; i++) {
        if (i) fb_putc(':');
        fb_hex2(p[i]);
    }
}

static u16 be16(const u8 *p) {
    return ((u16)p[0] << 8) | p[1];
}

static u32 be32(const u8 *p) {
    return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

static bool mac_eq(const u8 *a, const u8 *b) {
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2] &&
           a[3] == b[3] && a[4] == b[4] && a[5] == b[5];
}

static bool ip_match_mask(u32 packet_ip, u32 rule_ip, u32 mask)
{
    if (mask == 0)
        return packet_ip == rule_ip;
    return (packet_ip & mask) == (rule_ip & mask);
}

static bool ip_match_range(u32 packet_ip, u32 start, u32 end)
{
    if (start <= end)
        return packet_ip >= start && packet_ip <= end;
    return packet_ip >= end && packet_ip <= start;
}

static bool mac_is_broadcast(const u8 *p) {
    return p[0] == 0xFF && p[1] == 0xFF && p[2] == 0xFF &&
           p[3] == 0xFF && p[4] == 0xFF && p[5] == 0xFF;
}

static bool ip_is_our_debug_addr(const u8 *p) {
    return local_ipv4 && be32(p) == local_ipv4;
}

static void fb_tcp_flags(u8 flags) {
    if (flags & 0x02) fb_puts(" SYN");
    if (flags & 0x10) fb_puts(" ACK");
    if (flags & 0x01) fb_puts(" FIN");
    if (flags & 0x04) fb_puts(" RST");
    if (flags & 0x08) fb_puts(" PSH");
    if (flags & 0x20) fb_puts(" URG");
}

struct filter_packet {
    const u8 *mac_to;
    const u8 *mac_from;
    u16 ethertype;
    bool has_ip;
    u8 ip_proto;
    u32 ip_to;
    u32 ip_from;
    bool has_tcp;
    bool has_udp;
    u16 port_to;
    u16 port_from;
};

static void filter_parse_packet(const u8 *frame, u32 len, struct filter_packet *pkt) {
    pkt->mac_to = NULL;
    pkt->mac_from = NULL;
    pkt->ethertype = 0;
    pkt->has_ip = false;
    pkt->ip_proto = 0;
    pkt->ip_to = 0;
    pkt->ip_from = 0;
    pkt->has_tcp = false;
    pkt->has_udp = false;
    pkt->port_to = 0;
    pkt->port_from = 0;

    if (len < 14)
        return;

    pkt->mac_to = frame;
    pkt->mac_from = frame + 6;
    pkt->ethertype = be16(frame + 12);

    if (pkt->ethertype == 0x0806 && len >= 42) {
        const u8 *arp = frame + 14;
        pkt->has_ip = true;
        pkt->ip_from = be32(arp + 14);
        pkt->ip_to = be32(arp + 24);
        return;
    }

    if (pkt->ethertype != 0x0800 || len < 34)
        return;

    const u8 *ip = frame + 14;
    if ((ip[0] >> 4) != 4)
        return;
    u32 ihl = (u32)(ip[0] & 0x0F) * 4;
    if (ihl < 20 || len < 14 + ihl)
        return;

    pkt->has_ip = true;
    pkt->ip_proto = ip[9];
    pkt->ip_from = be32(ip + 12);
    pkt->ip_to = be32(ip + 16);

    const u8 *l4 = ip + ihl;
    u32 l4_len = len - 14 - ihl;
    if (pkt->ip_proto == 6 && l4_len >= 4) {
        pkt->has_tcp = true;
        pkt->port_from = be16(l4);
        pkt->port_to = be16(l4 + 2);
    } else if (pkt->ip_proto == 17 && l4_len >= 4) {
        pkt->has_udp = true;
        pkt->port_from = be16(l4);
        pkt->port_to = be16(l4 + 2);
    }
}

static bool filter_rule_matches(const nic_filter_rule_t *rule, const struct filter_packet *pkt) {
    if ((rule->flags & NIC_FILTER_MAC_TO) && (!pkt->mac_to || !mac_eq(pkt->mac_to, rule->mac_to)))
        return false;
    if ((rule->flags & NIC_FILTER_MAC_FROM) && (!pkt->mac_from || !mac_eq(pkt->mac_from, rule->mac_from)))
        return false;
    if ((rule->flags & NIC_FILTER_ETHERTYPE) && pkt->ethertype != rule->ethertype)
        return false;
    if ((rule->flags & NIC_FILTER_IP_PROTO) && (!pkt->has_ip || pkt->ip_proto != rule->ip_proto))
        return false;
    if ((rule->flags & NIC_FILTER_IP_TO) &&
        (!pkt->has_ip || !ip_match_mask(pkt->ip_to, rule->ip_to, rule->ip_to_mask)))
        return false;
    if ((rule->flags & NIC_FILTER_IP_FROM) &&
        (!pkt->has_ip || !ip_match_mask(pkt->ip_from, rule->ip_from, rule->ip_from_mask)))
        return false;
    if ((rule->flags & NIC_FILTER_IP_TO_RANGE) &&
        (!pkt->has_ip || !ip_match_range(pkt->ip_to, rule->ip_to, rule->ip_to_end)))
        return false;
    if ((rule->flags & NIC_FILTER_IP_FROM_RANGE) &&
        (!pkt->has_ip || !ip_match_range(pkt->ip_from, rule->ip_from, rule->ip_from_end)))
        return false;
    if ((rule->flags & NIC_FILTER_TCP_PORT_TO) && (!pkt->has_tcp || pkt->port_to != rule->tcp_port_to))
        return false;
    if ((rule->flags & NIC_FILTER_TCP_PORT_FROM) && (!pkt->has_tcp || pkt->port_from != rule->tcp_port_from))
        return false;
    if ((rule->flags & NIC_FILTER_UDP_PORT_TO) && (!pkt->has_udp || pkt->port_to != rule->udp_port_to))
        return false;
    if ((rule->flags & NIC_FILTER_UDP_PORT_FROM) && (!pkt->has_udp || pkt->port_from != rule->udp_port_from))
        return false;
    if ((rule->flags & NIC_FILTER_TCP_PORT_TO_RANGE) &&
        (!pkt->has_tcp || pkt->port_to < rule->tcp_port_to || pkt->port_to > rule->tcp_port_to_end))
        return false;
    if ((rule->flags & NIC_FILTER_TCP_PORT_FROM_RANGE) &&
        (!pkt->has_tcp || pkt->port_from < rule->tcp_port_from || pkt->port_from > rule->tcp_port_from_end))
        return false;
    if ((rule->flags & NIC_FILTER_UDP_PORT_TO_RANGE) &&
        (!pkt->has_udp || pkt->port_to < rule->udp_port_to || pkt->port_to > rule->udp_port_to_end))
        return false;
    if ((rule->flags & NIC_FILTER_UDP_PORT_FROM_RANGE) &&
        (!pkt->has_udp || pkt->port_from < rule->udp_port_from || pkt->port_from > rule->udp_port_from_end))
        return false;
    return true;
}

static bool nic_filter_allows(u8 direction, const u8 *frame, u32 len) {
    struct filter_packet pkt;
    filter_parse_packet(frame, len, &pkt);

    for (u32 i = 0; i < filter_rule_count; i++) {
        const nic_filter_rule_t *rule = &filter_rules[i];
        if (!(rule->direction & direction))
            continue;
        if (!filter_rule_matches(rule, &pkt))
            continue;
        return rule->action == NIC_FILTER_ALLOW;
    }

    return direction == NIC_FILTER_DIR_IN ? filter_default_in : filter_default_out;
}

static void fb_filter_drop(char tag, u64 count) {
#if NIC_HDMI_TEXT_PANELS
    if ((count & 0x3F) != 1)
        return;
    fb_set_color(0x00FF4040, 0x00000000);
    fb_putc(tag);
    fb_puts(" FILTER_DROP count=0x");
    fb_hex8((u32)count);
    fb_putc('\n');
#else
    (void)tag;
    (void)count;
#endif
}

static void fb_count32(u64 v)
{
    fb_printf("%u", (u32)v);
}

static void fb_ip32(u32 ip)
{
    fb_dec8((u8)(ip >> 24)); fb_putc('.');
    fb_dec8((u8)(ip >> 16)); fb_putc('.');
    fb_dec8((u8)(ip >> 8));  fb_putc('.');
    fb_dec8((u8)ip);
}

static bool flow_key_eq(const struct flow_key *a, const struct flow_key *b)
{
    return a->dir == b->dir && a->proto == b->proto && a->subtype == b->subtype &&
           a->ip_from == b->ip_from && a->ip_to == b->ip_to &&
           a->port_from == b->port_from && a->port_to == b->port_to;
}

static void flow_bucket_add(struct flow_bucket *b, u32 count, u32 slot)
{
    struct flow_bucket *e = &b[slot % count];
    if (e->slot != slot) {
        e->slot = slot;
        e->packets = 0;
    }
    e->packets++;
}

static u32 flow_bucket_sum(const struct flow_bucket *b, u32 count, u32 now_slot)
{
    u32 total = 0;
    for (u32 i = 0; i < count; i++) {
        if (now_slot >= b[i].slot && (now_slot - b[i].slot) < count)
            total += b[i].packets;
    }
    return total;
}

static struct flow_entry *flow_find_or_alloc(const struct flow_key *key)
{
    static u32 next_evict;
    for (u32 i = 0; i < NIC_FLOW_MAX; i++) {
        if (flow_table[i].used && flow_key_eq(&flow_table[i].key, key))
            return &flow_table[i];
    }
    for (u32 i = 0; i < NIC_FLOW_MAX; i++) {
        if (!flow_table[i].used) {
            flow_table[i].used = true;
            flow_table[i].key = *key;
            flow_table[i].all_packets = 0;
            simd_zero(flow_table[i].sec, sizeof(flow_table[i].sec));
            simd_zero(flow_table[i].five, sizeof(flow_table[i].five));
            simd_zero(flow_table[i].min, sizeof(flow_table[i].min));
            simd_zero(flow_table[i].hour, sizeof(flow_table[i].hour));
            return &flow_table[i];
        }
    }

    struct flow_entry *e = &flow_table[next_evict++ % NIC_FLOW_MAX];
    e->used = true;
    e->key = *key;
    e->all_packets = 0;
    simd_zero(e->sec, sizeof(e->sec));
    simd_zero(e->five, sizeof(e->five));
    simd_zero(e->min, sizeof(e->min));
    simd_zero(e->hour, sizeof(e->hour));
    return e;
}

static void flow_record_key(const struct flow_key *key)
{
    u64 now = timer_ticks();
    u32 sec_slot = (u32)(now / 1000ULL);
    u32 five_slot = (u32)(now / 5000ULL);
    u32 min_slot = (u32)(now / 60000ULL);
    u32 hour_slot = (u32)(now / 3600000ULL);

    struct flow_entry *e = flow_find_or_alloc(key);
    e->all_packets++;
    flow_bucket_add(e->sec, FLOW_SEC_BUCKETS, sec_slot);
    flow_bucket_add(e->five, FLOW_5S_BUCKETS, five_slot);
    flow_bucket_add(e->min, FLOW_MIN_BUCKETS, min_slot);
    flow_bucket_add(e->hour, FLOW_HOUR_BUCKETS, hour_slot);
}

static bool nic_ip_l4_checksum_field_zero(const u8 *frame, u32 len, bool *is_l4_out)
{
    if (is_l4_out)
        *is_l4_out = false;
    if (len < 34 || be16(frame + 12) != 0x0800)
        return false;
    const u8 *ip = frame + 14;
    u32 ihl = (u32)(ip[0] & 0x0F) * 4U;
    if ((ip[0] >> 4) != 4 || ihl < 20U || len < 14U + ihl)
        return false;
    u8 proto = ip[9];
    const u8 *l4 = ip + ihl;
    u32 l4_len = len - 14U - ihl;
    if (proto == 6 && l4_len >= 18U) {
        if (is_l4_out)
            *is_l4_out = true;
        return be16(l4 + 16U) == 0U;
    }
    if (proto == 17 && l4_len >= 8U) {
        if (is_l4_out)
            *is_l4_out = true;
        return be16(l4 + 6U) == 0U;
    }
    return false;
}

static void nic_record_tx_checksum_path(const u8 *frame, u32 len)
{
    bool is_l4 = false;
    bool csum_zero = nic_ip_l4_checksum_field_zero(frame, len, &is_l4);
    if (!is_l4)
        return;
    if (tx_checksum_offload && csum_zero)
        pkt_counts.tx_csum_offloaded++;
    else
        pkt_counts.tx_csum_software++;
}

static void flow_record_packet(bool tx, const u8 *frame, u32 len)
{
    if (len < 14)
        return;

    struct flow_key key;
    simd_zero(&key, sizeof(key));
    key.dir = tx ? NIC_FILTER_DIR_OUT : NIC_FILTER_DIR_IN;

    u16 et = be16(frame + 12);
    if (et == 0x0806 && len >= 42) {
        const u8 *arp = frame + 14;
        if (!tx && be16(arp + 6) == 1 && local_ipv4 && be32(arp + 24) == local_ipv4)
            return;
        key.proto = FLOW_PROTO_ARP;
        key.subtype = be16(arp + 6);
        key.ip_from = be32(arp + 14);
        key.ip_to = be32(arp + 24);
        flow_record_key(&key);
        return;
    }

    if (et != 0x0800 || len < 34)
        return;

    const u8 *ip = frame + 14;
    u32 ihl = (u32)(ip[0] & 0x0F) * 4;
    if ((ip[0] >> 4) != 4 || ihl < 20 || len < 14 + ihl)
        return;

    key.proto = ip[9];
    key.ip_from = be32(ip + 12);
    key.ip_to = be32(ip + 16);
    const u8 *l4 = ip + ihl;
    u32 l4_len = len - 14 - ihl;
    if ((key.proto == 6 || key.proto == 17) && l4_len >= 4) {
        key.port_from = be16(l4);
        key.port_to = be16(l4 + 2);
    } else if (key.proto == 1 && l4_len >= 1) {
        key.subtype = l4[0];
    }
    flow_record_key(&key);
}

static void fb_flow_name(const struct flow_key *key)
{
    fb_putc(key->dir == NIC_FILTER_DIR_OUT ? 'T' : 'R');
    fb_putc(' ');
    if (key->proto == FLOW_PROTO_ARP) {
        fb_puts("ARP");
        if (key->subtype == 1) fb_puts("REQ ");
        else if (key->subtype == 2) fb_puts("REP ");
        else fb_puts("??? ");
        fb_ip32(key->ip_from);
        fb_puts(">");
        fb_ip32(key->ip_to);
    } else if (key->proto == 1) {
        fb_puts("ICMP");
        fb_dec16(key->subtype);
        fb_putc(' ');
        fb_ip32(key->ip_from);
        fb_puts(">");
        fb_ip32(key->ip_to);
    } else if (key->proto == 6 || key->proto == 17) {
        fb_puts(key->proto == 6 ? "TCP " : "UDP ");
        fb_ip32(key->ip_from);
        fb_putc(':');
        fb_dec16(key->port_from);
        fb_puts(">");
        fb_ip32(key->ip_to);
        fb_putc(':');
        fb_dec16(key->port_to);
    } else {
        fb_puts("IP");
        fb_dec8(key->proto);
        fb_putc(' ');
        fb_ip32(key->ip_from);
        fb_puts(">");
        fb_ip32(key->ip_to);
    }
}

static u32 pct_u64(u64 part, u64 total)
{
    if (total == 0)
        return 0;
    return (u32)((part * 100ULL) / total);
}

static void nic_render_system_panel(void)
{
#if NIC_HDMI_TEXT_PANELS
    struct nic_panel_core_sample {
        u64 last_ticks;
        u64 last_poll;
        u32 last_cpu;
        u32 _pad0;
        u64 _pad[4];
    } ALIGNED(64);
    static u64 last_ms;
    static struct nic_panel_core_sample samples[4];
    static u32 heartbeat;
    _Static_assert(sizeof(struct nic_panel_core_sample) == 64,
                   "NIC panel per-core samples must be one cache line");

    u64 now_ms = timer_monotonic_ms();
    if (now_ms < last_ms + 1000ULL)
        return;

    if (last_ms != 0) {
        for (u32 i = 0; i < 4; i++) {
            struct core_env *e = core_env_of(i);
            u64 dt = timer_ticks_core(i) - samples[i].last_ticks;
            u64 dp = e->poll_count - samples[i].last_poll;
            samples[i].last_cpu = dp ? 100 : (dt ? 1 : 0);
            samples[i].last_ticks = timer_ticks_core(i);
            samples[i].last_poll = e->poll_count;
        }
    } else {
        for (u32 i = 0; i < 4; i++) {
            struct core_env *e = core_env_of(i);
            samples[i].last_ticks = timer_ticks_core(i);
            samples[i].last_poll = e->poll_count;
        }
    }
    last_ms = now_ms;
    heartbeat++;

    u64 used = 0;
    for (u32 i = 0; i < 4; i++) {
        struct core_env *e = core_env_of(i);
        if (e->id == i && e->ram_base == (u8 *)(usize)core_ram_bases[i] &&
            e->ram_end == e->ram_base + CORE_PRIV_SIZE &&
            e->heap_ptr >= e->ram_base && e->heap_ptr <= e->ram_end)
            used += (u64)(usize)(e->heap_ptr - e->ram_base);
    }
    u64 total = CORE_PRIV_SIZE * 4ULL;

    fb_set_color(0x00FF80FF, 0x00000000);
    fb_set_cursor(0, 0);
    fb_puts("SYS up=");
    fb_count32(now_ms / 1000ULL);
    fb_puts("s hb=");
    fb_count32(heartbeat);
    fb_puts(" tcp=");
    fb_count32(tcp_active_count());
    fb_puts(" udp=");
    fb_count32(socket_udp_active_count());
    fb_puts(" ACT% c0=");
    fb_count32(samples[0].last_cpu);
    fb_puts(" c1=");
    fb_count32(samples[1].last_cpu);
    fb_puts(" c2=");
    fb_count32(samples[2].last_cpu);
    fb_puts(" c3=");
    fb_count32(samples[3].last_cpu);
    fb_puts(" RAM=");
    fb_count32(used / 1024ULL);
    fb_puts("K/");
    fb_count32(total / 1024ULL);
    fb_puts("K ");
    fb_count32(pct_u64(used, total));
    fb_puts("%                              ");
#endif
}

static void nic_render_counter_panel(void)
{
#if NIC_HDMI_TEXT_PANELS
    static u64 last_rendered;
    u64 total = pkt_counts.rx_total + pkt_counts.tx_total + pkt_counts.rx_filter_drop +
                pkt_counts.tx_filter_drop + pkt_counts.rx_arp_not_us +
                pkt_counts.processed + pkt_counts.dropped + pkt_counts.firewalled +
                pkt_counts.flood_blocked + pkt_counts.rate_limited;
    if (total == last_rendered)
        return;
    if (total > 16 && (total & 0x1F) != 0)
        return;
    last_rendered = total;

    fb_set_color(0x0000FF80, 0x00000000);
    fb_set_cursor(0, 1);
    fb_puts("+--------------------+----------+----------+----------+----------+----------+----------+----------+\n");
    fb_puts("| PIOS NETWORK STATS | RX       | TX       | PROC     | DROP     | FW       | FLOOD    | RATE     |\n");
    fb_puts("+--------------------+----------+----------+----------+----------+----------+----------+----------+\n");
    fb_puts("| TOTAL              | ");
    fb_count32(pkt_counts.rx_total); fb_puts(" | ");
    fb_count32(pkt_counts.tx_total); fb_puts(" | ");
    fb_count32(pkt_counts.processed); fb_puts(" | ");
    fb_count32(pkt_counts.dropped); fb_puts(" | ");
    fb_count32(pkt_counts.firewalled); fb_puts(" | ");
    fb_count32(pkt_counts.flood_blocked); fb_puts(" | ");
    fb_count32(pkt_counts.rate_limited); fb_puts(" |                    \n");

    fb_set_color(0x00C8C8FF, 0x00000000);
    fb_puts("| RX TYPES           | ARP=");
    fb_count32(pkt_counts.rx_arp);
    fb_puts(" REQ=");
    fb_count32(pkt_counts.rx_arp_req);
    fb_puts(" REP=");
    fb_count32(pkt_counts.rx_arp_rep);
    fb_puts(" IP=");
    fb_count32(pkt_counts.rx_ip);
    fb_puts(" ICMP=");
    fb_count32(pkt_counts.rx_icmp);
    fb_puts(" TCP=");
    fb_count32(pkt_counts.rx_tcp);
    fb_puts(" UDP=");
    fb_count32(pkt_counts.rx_udp);
    fb_puts(" OTH=");
    fb_count32(pkt_counts.rx_other);
    fb_puts("                                  \n");

    fb_set_color(0x00FFFF80, 0x00000000);
    fb_puts("| TX TYPES           | ARP=");
    fb_count32(pkt_counts.tx_arp);
    fb_puts(" REQ=");
    fb_count32(pkt_counts.tx_arp_req);
    fb_puts(" REP=");
    fb_count32(pkt_counts.tx_arp_rep);
    fb_puts(" IP=");
    fb_count32(pkt_counts.tx_ip);
    fb_puts(" ICMP=");
    fb_count32(pkt_counts.tx_icmp);
    fb_puts(" TCP=");
    fb_count32(pkt_counts.tx_tcp);
    fb_puts(" UDP=");
    fb_count32(pkt_counts.tx_udp);
    fb_puts(" OTH=");
    fb_count32(pkt_counts.tx_other);
    fb_puts("                                  \n");

    const tcp_diag_t *td = tcp_diag();
    fb_set_color(0x00FFAA00, 0x00000000);
    fb_puts("| TCP DIAG           | syn=");
    fb_count32(td->syn_seen);
    fb_puts(" synack=");
    fb_count32(td->synack_sent);
    fb_puts(" ack=");
    fb_count32(td->ack_cookie_seen);
    fb_puts(" badck=");
    fb_count32(td->bad_checksum);
    fb_puts(" badcookie=");
    fb_count32(td->ack_cookie_bad);
    fb_puts(" pend=");
    fb_count32(td->pending_queued);
    fb_puts(" acc=");
    fb_count32(td->accepted);
    fb_puts(" nolst=");
    fb_count32(td->no_listener);
    fb_puts("                \n");

    u64 now = timer_ticks();
    u32 sec_slot = (u32)(now / 1000ULL);
    u32 five_slot = (u32)(now / 5000ULL);
    u32 min_slot = (u32)(now / 60000ULL);
    u32 hour_slot = (u32)(now / 3600000ULL);
    fb_set_color(0x00FFFFFF, 0x00000000);
    fb_puts("+------+-----+-----+-----+-----+--------------------------------------------------------------+\n");
    fb_puts("| ALL  | 24H | 1H  | 5M  | 1M  | FLOW                                                         |\n");
    fb_puts("+------+-----+-----+-----+-----+--------------------------------------------------------------+\n");
    u32 row = 10;
    u32 max_row = fb_reserved_rows() > 5 ? fb_reserved_rows() - 5 : 18;
    for (u32 i = 0; i < NIC_FLOW_MAX && row < max_row; i++) {
        if (!flow_table[i].used)
            continue;
        fb_set_cursor(0, row++);
        fb_putc('|');
        fb_putc(' ');
        fb_count32(flow_table[i].all_packets);
        fb_puts(" | ");
        fb_count32(flow_bucket_sum(flow_table[i].hour, FLOW_HOUR_BUCKETS, hour_slot));
        fb_puts(" | ");
        fb_count32(flow_bucket_sum(flow_table[i].min, FLOW_MIN_BUCKETS, min_slot));
        fb_puts(" | ");
        fb_count32(flow_bucket_sum(flow_table[i].five, FLOW_5S_BUCKETS, five_slot));
        fb_puts(" | ");
        fb_count32(flow_bucket_sum(flow_table[i].sec, FLOW_SEC_BUCKETS, sec_slot));
        fb_puts(" | ");
        fb_flow_name(&flow_table[i].key);
        fb_puts("                                                |");
    }
    nic_render_system_panel();
#endif
}

static void nic_count_packet(bool tx, const u8 *frame, u32 len)
{
    if (tx) {
        pkt_counts.tx_total++;
        pkt_counts.tx_bytes += len;
    } else {
        pkt_counts.rx_total++;
        pkt_counts.rx_bytes += len;
    }
    flow_record_packet(tx, frame, len);

    if (len < 14) {
        if (tx) pkt_counts.tx_other++;
        else    pkt_counts.rx_other++;
        nic_render_counter_panel();
        return;
    }

    u16 et = be16(frame + 12);
    if (et == 0x0806 && len >= 42) {
        const u8 *arp = frame + 14;
        u16 op = be16(arp + 6);
        if (tx) {
            pkt_counts.tx_arp++;
            if (op == 1) pkt_counts.tx_arp_req++;
            else if (op == 2) pkt_counts.tx_arp_rep++;
        } else {
            pkt_counts.rx_arp++;
            if (op == 1) pkt_counts.rx_arp_req++;
            else if (op == 2) pkt_counts.rx_arp_rep++;
        }
    } else if (et == 0x0800 && len >= 34) {
        const u8 *ip = frame + 14;
        if (tx) pkt_counts.tx_ip++;
        else    pkt_counts.rx_ip++;
        if (ip[9] == 1) {
            if (tx) pkt_counts.tx_icmp++;
            else    pkt_counts.rx_icmp++;
        } else if (ip[9] == 6) {
            if (tx) pkt_counts.tx_tcp++;
            else    pkt_counts.rx_tcp++;
        } else if (ip[9] == 17) {
            if (tx) pkt_counts.tx_udp++;
            else    pkt_counts.rx_udp++;
        }
    } else {
        if (tx) pkt_counts.tx_other++;
        else    pkt_counts.rx_other++;
    }

    nic_render_counter_panel();
}

static bool nic_drop_arp_broadcast_not_for_us(const u8 *frame, u32 len) {
    if (!local_ipv4 || len < 42)
        return false;
    if (!mac_is_broadcast(frame) || be16(frame + 12) != 0x0806)
        return false;

    const u8 *arp = frame + 14;
    if (be16(arp + 6) != 1)  /* ARP request */
        return false;

    return be32(arp + 24) != local_ipv4;
}

/* Diagnostic: dump every packet to the framebuffer. The first line decodes
 * ethertype plus ARP/IP fields; the second line shows the first 42 raw bytes. */
static void fb_pkt_dump(char tag, u32 color, const u8 *frame, u32 len) {
    static u32 skipped_bcast_arp;
    static u32 dump_count;

    if (tag == 'R' && len >= 42 && mac_is_broadcast(frame) &&
        be16(frame + 12) == 0x0806) {
        const u8 *arp = frame + 14;
        if (be16(arp + 6) == 1 && !ip_is_our_debug_addr(arp + 24)) {
            skipped_bcast_arp++;
            if ((skipped_bcast_arp & 0x3F) != 0)
                return;
            fb_set_color(0x00666666, 0x00000000);
            fb_puts("R len=0x03C/60 proto=ARP/IPv4 broadcast spam skipped=");
            fb_hex8(skipped_bcast_arp);
            fb_puts(" last ");
            fb_ip(arp + 14);
            fb_puts(" -> ");
            fb_ip(arp + 24);
            fb_putc('\n');
            return;
        }
    }

    dump_count++;
    if (tag == 'R' && dump_count > 32 && (dump_count & 0x0F) != 0)
        return;

    fb_set_color(color, 0x00000000);
    fb_putc(tag);
    fb_puts(" len=0x");
    fb_hex3(len);
    fb_putc('/');
    fb_dec16((u16)len);
    fb_puts(" proto=");

    if (len >= 14) {
        u16 et = be16(frame + 12);
        if (et == 0x0806 && len >= 42) {
            const u8 *arp = frame + 14;
            u16 op = be16(arp + 6);
            fb_puts("ARP/IPv4 ");
            fb_puts(op == 1 ? "REQ " : (op == 2 ? "REP " : "OP? "));
            fb_ip(arp + 14);
            fb_puts(" -> ");
            fb_ip(arp + 24);
            fb_puts(" sm=");
            fb_mac(arp + 8);
            fb_puts(" tm=");
            fb_mac(arp + 18);
        } else if (et == 0x0800 && len >= 34) {
            const u8 *ip = frame + 14;
            u8 proto = ip[9];
            u32 ihl = (u32)(ip[0] & 0x0F) * 4;
            fb_puts("IP ");
            if (proto == 1) {
                fb_puts("ICMP ");
                if (ihl >= 20 && len >= 14 + ihl + 4) {
                    const u8 *icmp = ip + ihl;
                    if (icmp[0] == 8) fb_puts("ECHO_REQ ");
                    else if (icmp[0] == 0) fb_puts("ECHO_REP ");
                    else { fb_puts("T"); fb_dec8(icmp[0]); fb_puts(" "); }
                    fb_puts("c=");
                    fb_dec8(icmp[1]);
                    fb_putc(' ');
                }
            } else if (proto == 6) {
                fb_puts("TCP ");
                if (ihl >= 20 && len >= 14 + ihl + 14) {
                    const u8 *tcp = ip + ihl;
                    fb_dec16(be16(tcp));
                    fb_puts("->");
                    fb_dec16(be16(tcp + 2));
                    fb_tcp_flags(tcp[13]);
                    fb_putc(' ');
                }
            } else if (proto == 17) {
                fb_puts("UDP ");
                if (ihl >= 20 && len >= 14 + ihl + 8) {
                    const u8 *udp = ip + ihl;
                    fb_dec16(be16(udp));
                    fb_puts("->");
                    fb_dec16(be16(udp + 2));
                    fb_putc(' ');
                }
            } else {
                fb_puts("PROTO=");
                fb_dec8(proto);
                fb_putc(' ');
            }
            fb_ip(ip + 12);
            fb_puts(" -> ");
            fb_ip(ip + 16);
        } else {
            fb_puts("ETH ");
            fb_hex2((u8)(et >> 8));
            fb_hex2((u8)et);
            fb_puts(" dst=");
            fb_mac(frame);
            fb_puts(" src=");
            fb_mac(frame + 6);
        }
    } else {
        fb_puts("SHORT");
    }
    fb_putc('\n');

    fb_puts("  H [dst ");
    u32 n = (len > 42) ? 42 : len;  /* eth(14) + arp(28) or eth(14) + ipv4(20) + icmp/tcp(8) */
    for (u32 i = 0; i < n; i++) {
        if (i == 6) fb_puts("] [src ");
        else if (i == 12) fb_puts("] [type ");
        else if (i == 14) fb_puts("] [payload ");
        else if (i > 0) fb_putc(' ');
        fb_hex2(frame[i]);
    }
    if (n <= 6) fb_putc(']');
    else if (n <= 12) fb_putc(']');
    else if (n <= 14) fb_putc(']');
    else fb_putc(']');
    fb_putc('\n');
}

bool nic_init(void)
{
    tx_checksum_offload = false;
    rx_checksum_offload = false;
    tso_enabled = false;
    packet_dump_enabled = false;
    simd_zero(&pkt_counts, sizeof(pkt_counts));
    simd_zero(flow_table, sizeof(flow_table));
    nic_filter_clear();
    pioscap_init();

    g_nic = 0;
    for (u32 i = 0; i < sizeof(nic_backends) / sizeof(nic_backends[0]); i++) {
        const struct nic_ops *be = nic_backends[i];
        if (!be->probe || !be->probe())
            continue;
        if (be->init && be->init()) {
            g_nic = be;
            return true;
        }
        /* probe matched but init failed: keep trying lower-priority backends. */
    }
    return false;
}

bool nic_init_wifi(void)
{
    /* WiFi backend parked in spike/wifi/. Always fail. */
    return false;
}

bool nic_is_wifi(void)
{
    return false;
}

bool nic_send(const u8 *frame, u32 len)
{
    if (!nic_filter_allows(NIC_FILTER_DIR_OUT, frame, len)) {
        filter_tx_dropped++;
        pkt_counts.tx_filter_drop++;
        pkt_counts.firewalled++;
        nic_render_counter_panel();
        fb_filter_drop('T', filter_tx_dropped);
        return false;
    }
    nic_record_tx_checksum_path(frame, len);
    nic_count_packet(true, frame, len);
    if (packet_dump_enabled)
        fb_pkt_dump('T', 0x00FFFF80, frame, len);
    pioscap_tx(frame, len);
    bool ok = (g_nic && g_nic->send) ? g_nic->send(frame, len) : false;
    if (ok) pkt_counts.processed++;
    else    pkt_counts.dropped++;
    nic_render_counter_panel();
    return ok;
}

bool nic_send_parts(const void *head, u32 head_len, const void *tail, u32 tail_len)
{
    static u8 tx_frame[2048] ALIGNED(64);
    u32 total = head_len + tail_len;
    if (total > sizeof(tx_frame))
        return false;
    if (head_len)
        simd_memcpy(tx_frame, head, head_len);
    if (tail_len)
        simd_memcpy(tx_frame + head_len, tail, tail_len);
    if (!nic_filter_allows(NIC_FILTER_DIR_OUT, tx_frame, total)) {
        filter_tx_dropped++;
        pkt_counts.tx_filter_drop++;
        pkt_counts.firewalled++;
        nic_render_counter_panel();
        fb_filter_drop('T', filter_tx_dropped);
        return false;
    }
    nic_record_tx_checksum_path(tx_frame, total);
    nic_count_packet(true, tx_frame, total);
    if (packet_dump_enabled)
        fb_pkt_dump('T', 0x00FFFF80, tx_frame, total);
    pioscap_tx(tx_frame, total);
    bool ok = (g_nic && g_nic->send) ? g_nic->send(tx_frame, total) : false;
    if (ok) pkt_counts.processed++;
    else    pkt_counts.dropped++;
    nic_render_counter_panel();
    return ok;
}

bool nic_recv(u8 *frame, u32 *len, bool *checksum_trusted)
{
    if (checksum_trusted)
        *checksum_trusted = false;
    if (!g_nic || !g_nic->recv) {
        (void)frame;
        (void)len;
        return false;
    }

    for (u32 attempt = 0; attempt < 16; attempt++) {
        bool rx_trusted = false;
        bool ok = g_nic->recv(frame, len, &rx_trusted);
        if (!ok)
            return false;
        if (checksum_trusted)
            *checksum_trusted = rx_trusted;

        if (nic_drop_arp_broadcast_not_for_us(frame, *len)) {
            filter_rx_dropped++;
            pkt_counts.rx_arp_not_us++;
            pkt_counts.dropped++;
            pkt_counts.flood_blocked++;
            nic_render_counter_panel();
            continue;
        }

        if (!nic_filter_allows(NIC_FILTER_DIR_IN, frame, *len)) {
            filter_rx_dropped++;
            pkt_counts.rx_filter_drop++;
            pkt_counts.firewalled++;
            nic_render_counter_panel();
            fb_filter_drop('R', filter_rx_dropped);
            continue;
        }

        nic_count_packet(false, frame, *len);
        if (rx_trusted)
            pkt_counts.rx_csum_trusted++;
        else
            pkt_counts.rx_csum_untrusted++;

        if (packet_dump_enabled)
            fb_pkt_dump('R', 0x0080C8FF, frame, *len);
        pioscap_rx(frame, *len);
        pkt_counts.processed++;
        nic_render_counter_panel();
        return true;
    }

    return false;
}

void nic_get_mac(u8 *mac)
{
    if (g_nic && g_nic->get_mac)
        g_nic->get_mac(mac);
    else if (mac)
        simd_zero(mac, 6);
}

bool nic_link_up(void)
{
    return (g_nic && g_nic->link_up) ? g_nic->link_up() : false;
}

u32 nic_link_mbps(void)
{
    return (g_nic && g_nic->link_mbps) ? g_nic->link_mbps() : 0U;
}

bool nic_link_full_duplex(void)
{
    return (g_nic && g_nic->full_duplex) ? g_nic->full_duplex() : false;
}

void nic_set_tx_checksum_offload(bool enable)
{
    /* Only honour the request if the active backend can actually offload TX
     * checksums in hardware; otherwise force it off so the stack always emits
     * software-computed checksums. Without this, a backend with no offload
     * (e.g. virtio-net) would ship payload segments with an unfilled checksum
     * that the peer/host silently drops. */
    if (g_nic && g_nic->set_tx_csum) {
        tx_checksum_offload = enable;
        g_nic->set_tx_csum(enable);
    } else {
        tx_checksum_offload = false;
    }
}

void nic_set_rx_checksum_offload(bool enable)
{
    /* Same rule: only trust hardware RX checksum validation if the active
     * backend actually performs it; otherwise verify in software. */
    if (g_nic && g_nic->set_rx_csum) {
        rx_checksum_offload = enable;
        g_nic->set_rx_csum(enable);
    } else {
        rx_checksum_offload = false;
    }
}

void nic_set_tso(bool enable)
{
    if (g_nic && g_nic->set_tso) {
        g_nic->set_tso(enable);
        tso_enabled = g_nic->tso_enabled ? g_nic->tso_enabled() : enable;
    } else {
        tso_enabled = false;
    }
}

bool nic_tx_checksum_offload_enabled(void)
{
    if (g_nic && g_nic->tx_csum_enabled)
        tx_checksum_offload = tx_checksum_offload && g_nic->tx_csum_enabled();
    return tx_checksum_offload;
}

bool nic_rx_checksum_offload_enabled(void)
{
    if (g_nic && g_nic->rx_csum_enabled)
        rx_checksum_offload = rx_checksum_offload && g_nic->rx_csum_enabled();
    return rx_checksum_offload;
}

bool nic_tso_enabled(void)
{
    if (g_nic && g_nic->tso_enabled)
        tso_enabled = g_nic->tso_enabled();
    return tso_enabled;
}

void nic_set_local_ipv4(u32 ip)
{
    local_ipv4 = ip;
}

void nic_set_packet_dump(bool enable)
{
    packet_dump_enabled = enable;
}

void nic_packet_counters(nic_packet_counters_t *out)
{
    if (out)
        *out = pkt_counts;
}

void nic_offload_status(nic_offload_status_t *out)
{
    if (!out)
        return;
    simd_zero(out, sizeof(*out));
    if (g_nic && g_nic->set_tx_csum) {
        out->tx_checksum_capable = true;
        out->rx_checksum_capable = (g_nic->set_rx_csum != 0);
        out->tso_capable = (g_nic->set_tso != 0);
        out->tx_checksum_enabled = nic_tx_checksum_offload_enabled();
        out->rx_checksum_enabled = nic_rx_checksum_offload_enabled();
        out->tso_enabled = nic_tso_enabled();
        if (g_nic->offload_regs)
            g_nic->offload_regs(&out->mac_ncfgr, &out->mac_dmacfg);
    }
    out->tx_csum_offloaded = pkt_counts.tx_csum_offloaded;
    out->tx_csum_software = pkt_counts.tx_csum_software;
    out->rx_csum_trusted = pkt_counts.rx_csum_trusted;
    out->rx_csum_untrusted = pkt_counts.rx_csum_untrusted;
}

void nic_record_rate_limited(void)
{
    pkt_counts.rate_limited++;
    nic_render_counter_panel();
}

void nic_filter_clear(void)
{
    filter_rule_count = 0;
    filter_default_in = true;
    filter_default_out = true;
    filter_rx_dropped = 0;
    filter_tx_dropped = 0;
}

void nic_filter_set_default(bool allow_in, bool allow_out)
{
    filter_default_in = allow_in;
    filter_default_out = allow_out;
}

bool nic_filter_add(const nic_filter_rule_t *rule)
{
    if (!rule || filter_rule_count >= NIC_FILTER_MAX_RULES)
        return false;
    if ((rule->direction & NIC_FILTER_DIR_BOTH) == 0)
        return false;
    if (rule->action != NIC_FILTER_ALLOW && rule->action != NIC_FILTER_DROP)
        return false;

    filter_rules[filter_rule_count++] = *rule;
    return true;
}

bool nic_filter_add_front(const nic_filter_rule_t *rule)
{
    if (!rule || filter_rule_count >= NIC_FILTER_MAX_RULES)
        return false;
    if ((rule->direction & NIC_FILTER_DIR_BOTH) == 0)
        return false;
    if (rule->action != NIC_FILTER_ALLOW && rule->action != NIC_FILTER_DROP)
        return false;
    for (u32 i = filter_rule_count; i > 0; i--)
        filter_rules[i] = filter_rules[i - 1];
    filter_rules[0] = *rule;
    filter_rule_count++;
    return true;
}

bool nic_filter_remove(u32 index)
{
    if (index >= filter_rule_count)
        return false;
    filter_rule_count--;
    if (index != filter_rule_count)
        filter_rules[index] = filter_rules[filter_rule_count];
    return true;
}

bool nic_filter_get(u32 index, nic_filter_rule_t *out)
{
    if (!out || index >= filter_rule_count)
        return false;
    *out = filter_rules[index];
    return true;
}

u32 nic_filter_count(void)
{
    return filter_rule_count;
}

void nic_filter_stats(u64 *rx_dropped, u64 *tx_dropped)
{
    if (rx_dropped)
        *rx_dropped = filter_rx_dropped;
    if (tx_dropped)
        *tx_dropped = filter_tx_dropped;
}
