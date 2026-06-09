/*
 * dns.h - Minimal hardened DNS stub resolver
 *
 * UDP-only, A record queries only, 16-entry cache.
 *
 * Hardening:
 *   - TXID randomized (CRC32)
 *   - Source port randomized (49152-65535)
 *   - Strict TXID + source IP matching
 *   - TTL clamped (60s..86400s)
 *   - Reject truncated (TC=1) and multi-question responses
 */

#pragma once
#include "types.h"

#define DNS_ASYNC_IDLE    0U
#define DNS_ASYNC_RUNNING 1U
#define DNS_ASYNC_DONE    2U
#define DNS_ASYNC_FAILED  3U
#define DNS_HOST_MAX      96U

struct dns_async_status {
    u32 state;
    u32 server_ip;
    u32 result_ip;
    u32 attempts;
    u32 last_error;
    u32 rx_total;
    u32 rx_server;
    u32 rx_ignored;
    u32 rx_rejected;
    u32 rx_ok;
    u32 last_rx_src_ip;
    u16 last_rx_src_port;
    u16 last_rx_dst_port;
    u16 last_rx_len;
    u16 _pad;
    char hostname[DNS_HOST_MAX];
} PACKED;

/* Init resolver with DNS server IP (from DHCP or static config) */
void dns_init(u32 server_ip);

/* Resolve hostname to IPv4. Blocking, returns true on success.
 * Checks cache first, sends query if not cached. */
bool dns_resolve(const char *hostname, u32 *ip_out);

bool dns_cache_lookup(const char *hostname, u32 *ip_out);
bool dns_resolve_async_start(const char *hostname);
void dns_poll(void);
void dns_async_status(struct dns_async_status *out);

/* Flush the entire cache */
void dns_cache_flush(void);
void dns_cache_stats(u64 *hits, u64 *misses, u64 *evictions);
