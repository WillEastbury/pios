/*
 * dhcp.h - Hardened DHCP client
 *
 * Full DHCP state machine with anti-rogue-server hardening.
 * On success, reconfigures the network stack via net_init().
 *
 * Hardening:
 *   - XID randomized via hardware CRC32
 *   - Option bounds validation
 *   - Server pinning after BOUND (rejects other servers)
 *   - ARP conflict detection before accepting lease
 *   - Lease sanity (60s..30d)
 *   - Rate-limited DISCOVER (1 per 5 seconds)
 */

#pragma once
#include "types.h"

/* DHCP state machine */
#define DHCP_INIT       0
#define DHCP_SELECTING  1
#define DHCP_REQUESTING 2
#define DHCP_BOUND      3
#define DHCP_RENEWING   4
#define DHCP_REBINDING  5

/* Obtained configuration */
typedef struct {
    u32 ip;
    u32 mask;
    u32 gateway;
    u32 dns;
    u32 server_id;
    u32 lease_time;     /* seconds */
    u32 t1;             /* renew time (seconds from bind) */
    u32 t2;             /* rebind time (seconds from bind) */
} dhcp_lease_t;

/* Start DHCP on the current interface. Blocks until BOUND or timeout. */
bool dhcp_start(u32 timeout_ms);

/* Poll DHCP state machine (timer / command wait; does not drain a NIC). */
void dhcp_poll(void);

/* Get current lease info (NULL if not bound) */
const dhcp_lease_t *dhcp_get_lease(void);

/* Get current DHCP state */
u32  dhcp_get_state(void);

/* Test-only: option parser exposed for host unit testing (src/dhcp.c).
 * Not for use in production code. */
#ifndef DHCP_NO_TEST_EXPORTS
typedef struct {
    u8  msg_type;
    u32 server_id;
    u32 offered_ip;
    u32 subnet;
    u32 router;
    u32 dns;
    u32 lease_time;
    u32 t1;
    u32 t2;
} dhcp_parsed_test_t;
bool dhcp_parse_options_test(const u8 *opts, u32 len, dhcp_parsed_test_t *out);
#endif
