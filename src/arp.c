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
#include "nic.h"
#include "simd.h"
#include "uart.h"
#include "timer.h"
#include "lru.h"
#include "fb.h"

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

struct arp_context {
    struct arp_entry table[ARP_TABLE_SIZE];
    u32 my_ip, my_mask;
    u8 my_mac[6];
    arp_stats_t stats;
    struct lru_cache index;
    u64 last_reply_time;
};

static struct arp_context arp_contexts[3];
static u32 arp_current_iface = NIC_IFACE_WIRED;

static u8 arp_tx[60] ALIGNED(64);  /* min Ethernet frame */

/* Only core 0 mutates these sessions; IRQs only publish ingress descriptors. */
struct arp_claim_session {
    u64 next_send_ms;
    u64 deadline_ms;
    u32 first, last, cursor, generation, round;
    enum arp_claim_result result;
    u8 occupied[ARP_CLAIM_MAX_HOSTS / 8U];
    bool active;
    bool conflict;
} ALIGNED(64);
static struct arp_claim_session claims[NIC_IFACE_MAX];
_Static_assert(sizeof(struct arp_claim_session) % 64U == 0U, "claim stride");

static struct arp_context *arp_current(void)
{
    return &arp_contexts[arp_current_iface < 3U ? arp_current_iface : 0U];
}

#define table           (arp_current()->table)
#define my_ip           (arp_current()->my_ip)
#define my_mask         (arp_current()->my_mask)
#define my_mac          (arp_current()->my_mac)
#define arp_stats       (arp_current()->stats)
#define arp_index       (arp_current()->index)
#define last_reply_time (arp_current()->last_reply_time)

/* ---- Helpers ---- */

static u64 now_ms(void) {
    return timer_monotonic_ms();
}

static bool mac_is_zero(const u8 *m) {
    return m[0]==0 && m[1]==0 && m[2]==0 && m[3]==0 && m[4]==0 && m[5]==0;
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

/* ---- Table operations (LRU-accelerated) ---- */

static void arp_index_update(u32 ip, u32 table_idx) {
    lru_put(&arp_index, (const u8 *)&ip, sizeof(ip), &table_idx, sizeof(table_idx));
}

static void arp_index_remove(u32 ip) {
    lru_remove(&arp_index, (const u8 *)&ip, sizeof(ip));
}

static struct arp_entry *find_entry(u32 ip) {
    /* Fast path: LRU index lookup */
    struct lru_entry *le = lru_get(&arp_index, (const u8 *)&ip, sizeof(ip));
    if (le) {
        u32 idx = *(u32 *)le->value;
        if (idx < ARP_TABLE_SIZE && table[idx].state != ARP_FREE && table[idx].ip == ip)
            return &table[idx];
        /* Stale index entry — remove and fall through */
        arp_index_remove(ip);
    }
    /* Slow path: linear scan (shouldn't happen often) */
    for (u32 i = 0; i < ARP_TABLE_SIZE; i++) {
        if (table[i].state != ARP_FREE && table[i].ip == ip) {
            arp_index_update(ip, i);
            return &table[i];
        }
    }
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

static bool send_arp_from(u16 opcode, const u8 *target_mac, u32 target_ip,
                          u32 source_ip) {
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
    write_ip(arp + ARP_SENDER_IP, source_ip);

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

    /* Diagnostic: dump the first few outgoing ARP frames so we can verify
     * exactly what we put on the wire. Catches malformed replies that the
     * host's NIC would silently filter. */
    static u32 send_count;
    if (source_ip != 0U)
        send_count++;
    if (source_ip != 0U && (send_count <= 10 || (send_count & 0x1F) == 0)) {
        uart_puts("[arp] tx#");
        uart_hex(send_count);
        uart_puts(" op=");
        uart_hex(opcode);
        uart_puts(" frame=");
        for (u32 i = 0; i < 42; i++) {
            static const char hex[] = "0123456789ABCDEF";
            uart_putc(hex[arp_tx[i] >> 4]);
            uart_putc(hex[arp_tx[i] & 0xF]);
        }
        uart_puts("\n");
    }

    return nic_send_on((nic_iface_t)arp_current_iface, arp_tx, 60);
}

static void send_arp(u16 opcode, const u8 *target_mac, u32 target_ip)
{
    (void)send_arp_from(opcode, target_mac, target_ip, my_ip);
}

/* ---- Public API ---- */

static struct arp_claim_session *claim_session(struct arp_claim_handle handle)
{
    if (handle.iface != NIC_IFACE_WIRED && handle.iface != NIC_IFACE_WIFI)
        return NULL;
    struct arp_claim_session *s = &claims[handle.iface];
    return s->active && s->generation == handle.generation ? s : NULL;
}

static bool claim_contains(const struct arp_claim_session *s, u32 ip)
{
    return ip >= s->first && ip <= s->last;
}

bool arp_claim_begin(nic_iface_t iface, u32 first, u32 last,
                     struct arp_claim_handle *out)
{
    if (!out || (iface != NIC_IFACE_WIRED && iface != NIC_IFACE_WIFI) ||
        !nic_iface_active(iface) || first == 0U || first > last ||
        last - first >= ARP_CLAIM_MAX_HOSTS || (first >> 28) >= 14U ||
        (last >> 28) >= 14U || (first >> 24) == 0U ||
        (last >> 24) == 0U || (first >> 24) == 127U ||
        (last >> 24) == 127U || claims[iface].active)
        return false;
    struct arp_claim_session *s = &claims[iface];
    u32 generation = s->generation + 1U;
    if (!generation)
        generation = 1U;
    simd_zero(s, sizeof(*s));
    s->generation = generation;
    s->first = first;
    s->last = last;
    s->cursor = first;
    s->deadline_ms = now_ms() + 120000U;
    s->result = ARP_CLAIM_PENDING;
    s->active = true;
    *out = (struct arp_claim_handle){ .generation = generation, .iface = iface };
    return true;
}

enum arp_claim_result arp_claim_poll(struct arp_claim_handle handle)
{
    struct arp_claim_session *s = claim_session(handle);
    if (!s)
        return ARP_CLAIM_ERROR;
    if (s->result != ARP_CLAIM_PENDING)
        return s->conflict && s->result == ARP_CLAIM_AVAILABLE ?
               ARP_CLAIM_CONFLICT : s->result;
    u64 now = now_ms();
    if (now >= s->deadline_ms) {
        uart_puts("[arp] claim deadline expired\n");
        return s->result = ARP_CLAIM_ERROR;
    }
    if (now < s->next_send_ms)
        return ARP_CLAIM_PENDING;
    if (s->round == 3U)
        return s->result = s->conflict ? ARP_CLAIM_CONFLICT : ARP_CLAIM_AVAILABLE;
    u32 previous = arp_current_iface;
    arp_set_interface(handle.iface);
    for (u32 budget = 0; budget < 8U; budget++) {
        if (!send_arp_from(ARP_OP_REQUEST, NULL, s->cursor, 0U)) {
            uart_puts("[arp] claim TX rejected\n");
            s->result = ARP_CLAIM_ERROR;
            break;
        }
        arp_stats.requests_sent++;
        if (s->cursor == s->last) {
            s->cursor = s->first;
            s->round++;
            s->next_send_ms = now + 1000U;
            break;
        }
        s->cursor++;
        s->next_send_ms = now + 100U;
    }
    arp_current_iface = previous;
    return s->result;
}

bool arp_claim_conflicted(struct arp_claim_handle handle, u32 ip)
{
    struct arp_claim_session *s = claim_session(handle);
    if (!s || s->result == ARP_CLAIM_ERROR || !claim_contains(s, ip))
        return true;
    u32 index = ip - s->first;
    return (s->occupied[index >> 3] & (1U << (index & 7U))) != 0U;
}

void arp_claim_cancel(struct arp_claim_handle handle)
{
    struct arp_claim_session *s = claim_session(handle);
    if (s) {
        s->active = false;
        s->generation++;
        if (!s->generation)
            s->generation = 1U;
        s->first = s->last = s->cursor = 0U;
        s->result = ARP_CLAIM_ERROR;
        simd_memset(s->occupied, 0xff, sizeof(s->occupied));
    }
}

bool arp_claim_accepts(nic_iface_t iface, const u8 *frame, u32 len)
{
    if ((iface != NIC_IFACE_WIRED && iface != NIC_IFACE_WIFI) ||
        !frame || len < sizeof(struct eth_hdr) + ARP_FRAME_LEN ||
        frame[12] != 0x08U || frame[13] != 0x06U)
        return false;
    const struct arp_claim_session *s = &claims[iface];
    const u8 *arp = frame + sizeof(struct eth_hdr);
    return s->active && s->result != ARP_CLAIM_ERROR &&
           (claim_contains(s, read_ip(arp + ARP_SENDER_IP)) ||
            (read_ip(arp + ARP_SENDER_IP) == 0U &&
             claim_contains(s, read_ip(arp + ARP_TARGET_IP))));
}

void arp_set_interface(nic_iface_t iface)
{
    if (iface == NIC_IFACE_WIRED || iface == NIC_IFACE_WIFI)
        arp_current_iface = iface;
}

void arp_init_iface(nic_iface_t iface, u32 ip, u32 mask, const u8 *mac) {
    arp_set_interface(iface);
    arp_claim_cancel((struct arp_claim_handle){
        .iface = iface, .generation = claims[arp_current_iface].generation });
    my_ip = ip;
    my_mask = mask;
    simd_memcpy(my_mac, mac, 6);
    for (u32 i = 0; i < ARP_TABLE_SIZE; i++)
        table[i].state = ARP_FREE;
    simd_zero(&arp_stats, sizeof(arp_stats));
    lru_init(&arp_index, NULL, 0); /* no TTL — managed by arp_tick */
    last_reply_time = 0;
}

void arp_init(u32 ip, u32 mask, const u8 *mac)
{
    arp_init_iface(NIC_IFACE_WIRED, ip, mask, mac);
}

void arp_add_static_iface(nic_iface_t iface, u32 ip, const u8 *mac)
{
    arp_set_interface(iface);
    arp_add_static(ip, mac);
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
    arp_index_update(ip, (u32)(e - table));
}

const u8 *arp_resolve_iface(nic_iface_t iface, u32 ip)
{
    arp_set_interface(iface);
    return arp_resolve(ip);
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

void arp_input_iface(nic_iface_t iface, const u8 *frame, u32 len)
{
    arp_set_interface(iface);
    arp_input(frame, len);
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

    if (opcode != ARP_OP_REQUEST && opcode != ARP_OP_REPLY)
        return;

    /* ---- Validate sender MAC ---- */
    if (mac_is_zero(sender_mac) || (sender_mac[0] & 1U) ||
        !mac_eq(sender_mac, frame + 6)) {
        arp_stats.drop_spoof++;
        return;
    }

    if (!mac_eq(sender_mac, my_mac) &&
        arp_claim_accepts((nic_iface_t)arp_current_iface, frame, len) &&
        (sender_ip != 0U || opcode == ARP_OP_REQUEST)) {
        struct arp_claim_session *s = &claims[arp_current_iface];
        u32 ip = sender_ip ? sender_ip : target_ip;
        u32 index = ip - s->first;
        s->occupied[index >> 3] |= (u8)(1U << (index & 7U));
        s->conflict = true;
    }

    /* ---- Conflict detection: someone else claims our IP ---- */
    if (my_ip != 0U && sender_ip == my_ip && !mac_eq(sender_mac, my_mac)) {
        arp_stats.conflicts++;
        uart_puts("[arp] CONFLICT: our IP from different MAC!\n");
        return;
    }

    /* ---- Validate sender IP ---- */
    bool address_probe = sender_ip == 0U && opcode == ARP_OP_REQUEST;
    if (!address_probe && !ip_valid_sender(sender_ip)) {
        arp_stats.drop_spoof++;
        return;
    }

    /* ---- Process based on opcode ---- */

    if (opcode == ARP_OP_REQUEST) {
        /* Someone is asking for our MAC */
        if (my_ip == 0U || target_ip != my_ip)
            return; /* not for us */

        /* Rate-limit replies */
        u64 now = now_ms();
        if (now - last_reply_time < ARP_REPLY_INTERVAL_MS) {
            arp_stats.drop_ratelimit++;
            nic_record_rate_limited();
            return;
        }
        last_reply_time = now;

        /* Learn sender (they're talking to us, so they're real) */
        struct arp_entry *e = address_probe ? NULL : find_entry(sender_ip);
        if (address_probe) {
            /* RFC 5227 probes get a reply, but never a zero-IP neighbor. */
        } else if (!e) {
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
            if (e->consistency > 0 && mac_eq(sender_mac, e->pending_mac)) {
                e->consistency++;
            } else {
                simd_memcpy(e->pending_mac, sender_mac, 6);
                e->consistency = 1;
            }

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

void arp_announce_iface(nic_iface_t iface)
{
    arp_set_interface(iface);
    arp_announce();
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

void arp_probe_iface(nic_iface_t iface)
{
    arp_set_interface(iface);
    arp_probe();
}

void arp_probe(void) {
    /* Single broadcast ARP request for our own IP. Used by the stall detector
     * to test whether the TX path can still put a frame on the wire. */
    send_arp(ARP_OP_REQUEST, NULL, my_ip);
    arp_stats.requests_sent++;
}

void arp_tick_iface(nic_iface_t iface)
{
    arp_set_interface(iface);
    arp_tick();
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

void arp_cache_stats(u64 *hits, u64 *misses, u64 *evictions) {
    u64 h = 0, m = 0, ev = 0, ex = 0;
    lru_stats(&arp_index, &h, &m, &ev, &ex);
    if (hits) *hits = h;
    if (misses) *misses = m;
    if (evictions) *evictions = ev;
}

u32 arp_snapshot(struct arp_snapshot_entry *out, u32 max_entries)
{
    u32 n = 0;
    u64 now = now_ms();
    if (!out || max_entries == 0)
        return 0;
    for (u32 i = 0; i < ARP_TABLE_SIZE && n < max_entries; i++) {
        struct arp_entry *e = &table[i];
        if (e->state == ARP_FREE)
            continue;
        out[n].ip = e->ip;
        simd_memcpy(out[n].mac, (e->state == ARP_INCOMPLETE) ? e->pending_mac : e->mac, 6);
        out[n].state = e->state;
        out[n].retries = e->retries;
        out[n].consistency = e->consistency;
        out[n].age_ms = now >= e->timestamp ? now - e->timestamp : 0;
        n++;
    }
    return n;
}
