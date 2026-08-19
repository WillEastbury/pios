#pragma once

#include "types.h"
#include "nic.h"

/*
 * ADR-033 network execution boundary.
 *
 * Hardware and paced transport indications publish only fixed descriptors.
 * Core-0 software handlers consume one FIFO stage and publish the next:
 *
 *   indication -> transport -> MAC -> IPv4 -> TCP/UDP/ICMP -> service
 *                                                       \-> egress -> owned NIC span
 *
 * All queues are core-0-local SPSC queues with explicit release/acquire
 * publication.  The byte storage belongs to a slot until its consuming stage
 * advances the corresponding tail; no raw caller buffer crosses a stage.
 */

#define NET_DISPATCH_HINT_CAPACITY 8U
#define NET_DISPATCH_RX_CAPACITY   4U
#define NET_DISPATCH_TX_CAPACITY   8U

/* A paced SDIO probe and a MAC IRQ are both bounded transport indications. */
#define NET_DISPATCH_CAUSE_IRQ     (1U << 0)
#define NET_DISPATCH_CAUSE_PACED   (1U << 1)
#define NET_DISPATCH_CAUSE_RECHECK (1U << 2)

typedef void (*net_dispatch_service_fn)(void);

void net_dispatch_init(void);
void net_dispatch_enable(void);
bool net_dispatch_enabled(void);

/* Publication only: safe from a timer or a hardware/software top half. */
bool net_dispatch_publish_transport(nic_iface_t iface, u32 cause);
bool net_dispatch_publish_service(void);

/* Bottom halves, each called only by its matching AIRQ software handler. */
void net_dispatch_handle_transport(void);
void net_dispatch_handle_mac(void);
void net_dispatch_handle_ip(void);
void net_dispatch_handle_tcp(void);
void net_dispatch_handle_service(net_dispatch_service_fn service);
void net_dispatch_handle_egress(void);

/*
 * Public NIC submission boundary.  Once enabled, every normal egress caller
 * copies its frame into the bounded owned-span FIFO; only the egress handler
 * calls nic_send_owned_on().
 */
bool net_dispatch_submit_egress(nic_iface_t iface, const u8 *frame, u32 len);

struct net_dispatch_diag {
    u32 transport_published;
    u32 transport_dropped;
    u32 transport_handled;
    u32 rx_published;
    u32 rx_dropped;
    u32 protocol_handled;
    u32 service_published;
    u32 service_dropped;
    u32 service_handled;
    u32 tx_published;
    u32 tx_dropped;
    u32 tx_handled;
} ALIGNED(64);

void net_dispatch_diag_snapshot(struct net_dispatch_diag *out);
