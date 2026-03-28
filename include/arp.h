/*
 * arp.h - Hardened ARP implementation
 *
 * Dynamic neighbor learning with strict anti-spoofing validation.
 * Static entries (from net_add_neighbor) are never overwritten or aged.
 *
 * Hardening:
 *   - Fixed 32-entry table (prevents memory exhaustion)
 *   - Reject gratuitous ARP (unsolicited replies)
 *   - Validate sender MAC/IP (no zero, broadcast, self, multicast)
 *   - MAC consistency: require N consistent replies before updating
 *   - Rate-limit ARP requests (1/sec per IP) and replies (10/sec)
 *   - Conflict detection (our IP from different MAC → logged)
 *   - Boot announcements (3× with backoff)
 */

#pragma once
#include "types.h"

#define ETH_P_ARP       0x0806

#define ARP_TABLE_SIZE  32

/* ARP entry states */
#define ARP_FREE        0
#define ARP_INCOMPLETE  1   /* request sent, waiting for reply */
#define ARP_REACHABLE   2   /* confirmed by reply to our request */
#define ARP_STALE       3   /* TTL expired, still usable but will re-probe */
#define ARP_STATIC      4   /* manually configured, never expires */

/* ARP stats (added to net_stats_t) */
typedef struct {
    u64 requests_sent;
    u64 replies_sent;
    u64 learned;
    u64 drop_spoof;
    u64 drop_ratelimit;
    u64 conflicts;
} arp_stats_t;

/* Init ARP subsystem (called from net_init) */
void arp_init(u32 our_ip, u32 our_mask, const u8 *our_mac);

/* Add a static entry (never expires, never overwritten) */
void arp_add_static(u32 ip, const u8 *mac);

/* Resolve IP → MAC. Returns pointer to MAC or NULL. If NULL, an ARP
 * request is queued automatically. Caller should retry later. */
const u8 *arp_resolve(u32 ip);

/* Process an incoming ARP frame (called from net_poll on ETH_P_ARP) */
void arp_input(const u8 *frame, u32 len);

/* Send gratuitous ARP announcements (call at boot) */
void arp_announce(void);

/* Age out stale entries (call periodically, e.g. once per second) */
void arp_tick(void);

/* Get ARP stats */
const arp_stats_t *arp_get_stats(void);
