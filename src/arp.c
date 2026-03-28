/*
 * arp.c - Hardened ARP implementation
 *
 * Fixed 32-entry table, anti-spoofing, rate limiting, conflict detection.
 *
 * ARP frame format (Ethernet + ARP):
 *   [0..13]  Ethernet header (dst, src, type=0x0806)
 *   [14..15] hw_type = 1 (Ethernet)
 *   [16..17] proto_type = 0x0800 (IPv4)
 *   [18]     hw_len = 6
 *   [19]     proto_len = 4
 *   [20..21] opcode: 1=request, 2=reply
 *   [22..27] sender MAC
 *   [28..31] sender IP
 *   [32..37] target MAC
 *   [38..41] target IP
 *
 * Reference: RFC 826, RFC 5227 (conflict detection)
 */

#include "arp.h"
#include "net.h"
#include "genet.h"
#include "simd.h"
#include "uart.h"
#include "timer.h"

/* ---- ARP opcodes ---- */

#define ARP_OP_REQUEST  1
#define ARP_OP_REPLY    2

/* ---- ARP frame offsets from start of Ethernet payload (offset 14) ---- */

#define ARP_HW_TYPE     0
#define ARP_PROTO_TYPE  2
#define ARP_HW_LEN      4
#define ARP_PROTO_LEN   5
#define ARP_OPCODE      6
#define ARP_SENDER_MAC  8
#define ARP_SENDER_IP   14
#define ARP_TARGET_MAC  18
#define ARP_TARGET_IP   24
#define ARP_FRAME_LEN   28  /* ARP payload = 28 bytes */

/* ---- Hardening constants ---- */

#define ARP_REACHABLE_MS    (60 * 1000)     /* 60s reachable TTL */
#define ARP_STALE_MS        (300 * 1000)    /* 300s stale before eviction */
#define ARP_REQ_INTERVAL_MS 1000            /* max 1 request/sec per IP */
#define ARP_REPLY_INTERVAL_MS 100           /* max 10 replies/sec global */
#define ARP_CONSISTENCY_COUNT 2             /* require N consistent MACs */
#define ARP_ANNOUNCE_COUNT  3               /* 3 announcements at boot */
#define ARP_MAX_RETRIES     3               /* retries for incomplete entries */

/* ---- Table entry ---- */

struct arp_entry {
    u32 ip;
    u8  mac[6];
    u8  state;
    u8  retries;
    u64 timestamp;          /* last confirmed (ms) */
    u64 last_request;       /* last request sent (ms) */
    u8  pending_mac[6];     /* MAC from unconfirmed reply */
    u8  consistency;        /* count of consistent replies with pending_mac */
    u8  _pad;
};

/* ---- State ---- */

static struct arp_entry table[ARP_TABLE_SIZE];
static u32 my_ip, my_mask;
static u8  my_mac[6];
static arp_stats_t arp_stats;

static u64 last_reply_time;   /* global reply rate limiter */

static u8 arp_tx[60] ALIGNED(64);  /* min Ethernet frame */

/* ---- Helpers ---- */

static u64 now_ms(void) {
    return timer_ticks();  /* timer_ticks returns ms-resolution ticks at 1kHz */
}

static bool mac_is_zero(const u8 *m) {
    return m[0]==0 && m[1]==0 && m[2]==0 && m[3]==0 && m[4]==0 && m[5]==0;
}

static bool mac_is_broadcast(const u8 *m) {
    return m[0]==0xFF && m[1]==0xFF && m[2]==0xFF && m[3]==0xFF && m[4]==0xFF && m[5]==0xFF;
}

static bool mac_eq(const u8 *a, const u8 *b) {
    return a[0]==b[0] && a[1]==b[1] && a[2]==b[2] &&
           a[3]==b[3] && a[4]==b[4] && a[5]==b[5];
}

static u32 read_ip(const u8 *p) {
    return ((u32)p[0]<<24) | ((u32)p[1]<<16) | ((u32)p[2]<<8) | p[3];
}

static void write_ip(u8 *p, u32 ip) {
    p[0] = (u8)(ip >> 24); p[1] = (u8)(ip >> 16);
    p[2] = (u8)(ip >> 8);  p[3] = (u8)ip;
}

static bool ip_valid_sender(u32 ip) {
    if (ip == 0) return false;                  /* 0.0.0.0 */
    if (ip == 0xFFFFFFFF) return false;         /* broadcast */
    if (ip == my_ip) return false;              /* conflict (handled separately) */
    if ((ip >> 24) == 127) return false;        /* loopback */
    if ((ip >> 28) == 0xE) return false;        /* multicast */
    return true;
}

/* ---- Table operations ---- */

static struct arp_entry *find_entry(u32 ip) {
    for (u32 i = 0; i < ARP_TABLE_SIZE; i++)
        if (table[i].state != ARP_FREE && table[i].ip == ip)
            return &table[i];
    return NULL;
}

static struct arp_entry *alloc_entry(void) {
    /* First try free slot */
    for (u32 i = 0; i < ARP_TABLE_SIZE; i++)
        if (table[i].state == ARP_FREE)
            return &table[i];

    /* Evict oldest non-static stale entry */
    struct arp_entry *oldest = NULL;
    u64 oldest_ts = ~0ULL;
    for (u32 i = 0; i < ARP_TABLE_SIZE; i++) {
        if (table[i].state == ARP_STATIC) continue;
        if (table[i].timestamp < oldest_ts) {
            oldest_ts = table[i].timestamp;
            oldest = &table[i];
        }
    }
    return oldest; /* may be NULL if all static */
}

/* ---- ARP frame construction ---- */

static void send_arp(u16 opcode, const u8 *target_mac, u32 target_ip) {
    struct eth_hdr *eth = (struct eth_hdr *)arp_tx;

    if (opcode == ARP_OP_REQUEST) {
        /* Request → broadcast */
        u8 bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
        simd_memcpy(eth->dst, bcast, 6);
    } else {
        simd_memcpy(eth->dst, target_mac, 6);
    }
    simd_memcpy(eth->src, my_mac, 6);
    eth->ethertype = htons(ETH_P_ARP);

    u8 *arp = arp_tx + sizeof(struct eth_hdr);
    arp[ARP_HW_TYPE]     = 0; arp[ARP_HW_TYPE+1] = 1;     /* Ethernet */
    arp[ARP_PROTO_TYPE]  = 0x08; arp[ARP_PROTO_TYPE+1] = 0; /* IPv4 */
    arp[ARP_HW_LEN]     = 6;
    arp[ARP_PROTO_LEN]  = 4;
    arp[ARP_OPCODE]      = 0; arp[ARP_OPCODE+1] = (u8)opcode;

    simd_memcpy(arp + ARP_SENDER_MAC, my_mac, 6);
    write_ip(arp + ARP_SENDER_IP, my_ip);

    if (opcode == ARP_OP_REQUEST) {
        u8 zeros[6] = {0,0,0,0,0,0};
        simd_memcpy(arp + ARP_TARGET_MAC, zeros, 6);
    } else {
        simd_memcpy(arp + ARP_TARGET_MAC, target_mac, 6);
    }
    write_ip(arp + ARP_TARGET_IP, target_ip);

    /* Pad to minimum Ethernet frame (60 bytes) */
    for (u32 i = sizeof(struct eth_hdr) + ARP_FRAME_LEN; i < 60; i++)
        arp_tx[i] = 0;

    genet_send(arp_tx, 60);
}

/* ---- Public API ---- */

void arp_init(u32 ip, u32 mask, const u8 *mac) {
    my_ip = ip;
    my_mask = mask;
    simd_memcpy(my_mac, mac, 6);
    for (u32 i = 0; i < ARP_TABLE_SIZE; i++)
        table[i].state = ARP_FREE;
    simd_zero(&arp_stats, sizeof(arp_stats));
    last_reply_time = 0;
}

void arp_add_static(u32 ip, const u8 *mac) {
    struct arp_entry *e = find_entry(ip);
    if (!e) {
        e = alloc_entry();
        if (!e) return;
    }
    e->ip = ip;
    simd_memcpy(e->mac, mac, 6);
    e->state = ARP_STATIC;
    e->timestamp = now_ms();
    e->retries = 0;
}

const u8 *arp_resolve(u32 ip) {
    struct arp_entry *e = find_entry(ip);

    if (e && (e->state == ARP_REACHABLE || e->state == ARP_STALE || e->state == ARP_STATIC))
        return e->mac;

    /* No entry or incomplete: send request (rate-limited) */
    u64 now = now_ms();

    if (e && e->state == ARP_INCOMPLETE) {
        if (now - e->last_request < ARP_REQ_INTERVAL_MS)
            return NULL;
        if (e->retries >= ARP_MAX_RETRIES) {
            e->state = ARP_FREE;
            return NULL;
        }
        e->retries++;
    } else {
        e = alloc_entry();
        if (!e) return NULL;
        e->ip = ip;
        e->state = ARP_INCOMPLETE;
        e->retries = 0;
        e->consistency = 0;
    }

    e->last_request = now;
    send_arp(ARP_OP_REQUEST, NULL, ip);
    arp_stats.requests_sent++;
    return NULL;
}

void arp_input(const u8 *frame, u32 len) {
    if (len < sizeof(struct eth_hdr) + ARP_FRAME_LEN)
        return;

    const u8 *arp = frame + sizeof(struct eth_hdr);

    /* Validate hardware type = Ethernet, protocol = IPv4 */
    if (arp[ARP_HW_TYPE] != 0 || arp[ARP_HW_TYPE+1] != 1) return;
    if (arp[ARP_PROTO_TYPE] != 0x08 || arp[ARP_PROTO_TYPE+1] != 0) return;
    if (arp[ARP_HW_LEN] != 6 || arp[ARP_PROTO_LEN] != 4) return;

    u16 opcode = ((u16)arp[ARP_OPCODE] << 8) | arp[ARP_OPCODE+1];
    const u8 *sender_mac = arp + ARP_SENDER_MAC;
    u32 sender_ip = read_ip(arp + ARP_SENDER_IP);
    u32 target_ip = read_ip(arp + ARP_TARGET_IP);

    /* ---- Validate sender MAC ---- */
    if (mac_is_zero(sender_mac) || mac_is_broadcast(sender_mac)) {
        arp_stats.drop_spoof++;
        return;
    }

    /* ---- Conflict detection: someone else claims our IP ---- */
    if (sender_ip == my_ip && !mac_eq(sender_mac, my_mac)) {
        arp_stats.conflicts++;
        uart_puts("[arp] CONFLICT: our IP from different MAC!\n");
        return;
    }

    /* ---- Validate sender IP ---- */
    if (!ip_valid_sender(sender_ip)) {
        arp_stats.drop_spoof++;
        return;
    }

    /* ---- Process based on opcode ---- */

    if (opcode == ARP_OP_REQUEST) {
        /* Someone is asking for our MAC */
        if (target_ip != my_ip)
            return; /* not for us */

        /* Rate-limit replies */
        u64 now = now_ms();
        if (now - last_reply_time < ARP_REPLY_INTERVAL_MS) {
            arp_stats.drop_ratelimit++;
            return;
        }
        last_reply_time = now;

        /* Learn sender (they're talking to us, so they're real) */
        struct arp_entry *e = find_entry(sender_ip);
        if (!e) {
            e = alloc_entry();
            if (e) {
                e->ip = sender_ip;
                simd_memcpy(e->mac, sender_mac, 6);
                e->state = ARP_REACHABLE;
                e->timestamp = now;
                e->retries = 0;
                arp_stats.learned++;
            }
        } else if (e->state != ARP_STATIC) {
            simd_memcpy(e->mac, sender_mac, 6);
            e->state = ARP_REACHABLE;
            e->timestamp = now;
        }

        /* Send reply */
        send_arp(ARP_OP_REPLY, sender_mac, sender_ip);
        arp_stats.replies_sent++;

    } else if (opcode == ARP_OP_REPLY) {
        /* Only accept replies to our outstanding requests (anti-gratuitous) */
        struct arp_entry *e = find_entry(sender_ip);
        if (!e || e->state == ARP_STATIC)
            return;

        if (e->state == ARP_INCOMPLETE) {
            /* First reply: record and start consistency check */
            simd_memcpy(e->pending_mac, sender_mac, 6);
            e->consistency = 1;

            if (e->consistency >= ARP_CONSISTENCY_COUNT) {
                simd_memcpy(e->mac, sender_mac, 6);
                e->state = ARP_REACHABLE;
                e->timestamp = now_ms();
                arp_stats.learned++;
            }
        } else if (e->state == ARP_REACHABLE || e->state == ARP_STALE) {
            if (mac_eq(sender_mac, e->mac)) {
                /* Same MAC — refresh */
                e->state = ARP_REACHABLE;
                e->timestamp = now_ms();
            } else {
                /* Different MAC — consistency check (anti-spoofing) */
                if (mac_eq(sender_mac, e->pending_mac)) {
                    e->consistency++;
                    if (e->consistency >= ARP_CONSISTENCY_COUNT) {
                        simd_memcpy(e->mac, sender_mac, 6);
                        e->state = ARP_REACHABLE;
                        e->timestamp = now_ms();
                        arp_stats.learned++;
                    }
                } else {
                    simd_memcpy(e->pending_mac, sender_mac, 6);
                    e->consistency = 1;
                    arp_stats.drop_spoof++;
                }
            }
        }
    }
}

void arp_announce(void) {
    u64 delay = 500;
    for (u32 i = 0; i < ARP_ANNOUNCE_COUNT; i++) {
        send_arp(ARP_OP_REQUEST, NULL, my_ip); /* ARP probe: who-has our IP */
        arp_stats.requests_sent++;
        timer_delay_ms(delay);
        delay *= 2; /* exponential backoff */
    }
}

void arp_tick(void) {
    u64 now = now_ms();
    for (u32 i = 0; i < ARP_TABLE_SIZE; i++) {
        struct arp_entry *e = &table[i];
        if (e->state == ARP_FREE || e->state == ARP_STATIC)
            continue;
        if (e->state == ARP_REACHABLE && now - e->timestamp > ARP_REACHABLE_MS)
            e->state = ARP_STALE;
        if (e->state == ARP_STALE && now - e->timestamp > ARP_STALE_MS)
            e->state = ARP_FREE;
        if (e->state == ARP_INCOMPLETE && now - e->last_request > ARP_REQ_INTERVAL_MS * ARP_MAX_RETRIES)
            e->state = ARP_FREE;
    }
}

const arp_stats_t *arp_get_stats(void) {
    return &arp_stats;
}
