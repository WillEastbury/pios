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

/* Init resolver with DNS server IP (from DHCP or static config) */
void dns_init(u32 server_ip);

/* Resolve hostname to IPv4. Blocking, returns true on success.
 * Checks cache first, sends query if not cached. */
bool dns_resolve(const char *hostname, u32 *ip_out);

/* Flush the entire cache */
void dns_cache_flush(void);
