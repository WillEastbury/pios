/*
 * xhci.h - xHCI Host Controller Interface (HCI layer)
 *
 * Low-level xHCI API for the RP1 DWC3 USB controllers.
 * Does NOT do enumeration — that's handled by usb.c.
 *
 * Reference: xHCI Spec 1.2, Linux drivers/usb/host/xhci*.c
 */

#pragma once
#include "types.h"

#define XHCI_USB0_OFFSET    0x200000
#define XHCI_USB1_OFFSET    0x300000

/* Endpoint info for configure_endpoints */
struct xhci_ep_info {
    u8  address;        /* endpoint address (bit 7 = direction) */
    u8  attributes;     /* bits [1:0]: 0=ctrl, 1=isoc, 2=bulk, 3=intr */
    u16 max_packet;
    u8  interval;
};

/* USB speed values (from PORTSC) */
#define USB_SPEED_FULL      1
#define USB_SPEED_LOW       2
#define USB_SPEED_HIGH      3
#define USB_SPEED_SUPER     4

/* Lightweight instrumentation counters */
struct xhci_stats {
    u32 cmd_submitted;
    u32 cmd_completed;
    u32 cmd_timeout;
    u32 xfer_ok;
    u32 xfer_fail;
    u32 evt_polled;
    u32 evt_stale_drained;
    u32 port_resets;
    u32 ep_stalls;
    u32 ep_resets;
    u32 ring_full;
};

/* Controller lifecycle */
bool xhci_init(void);

/* Port operations */
u32  xhci_port_count(void);
bool xhci_port_connected(u32 port);
bool xhci_port_reset(u32 port, u32 *speed);

/* Device slot management */
bool xhci_enable_slot(u32 *slot);
bool xhci_address_device(u32 slot, u32 port, u32 speed, u32 max_packet);
bool xhci_configure_endpoints(u32 slot, u32 port, u32 speed,
                               const struct xhci_ep_info *eps, u32 count);

/* Error recovery */
bool xhci_reset_endpoint(u32 slot, u32 dci);
bool xhci_stop_endpoint(u32 slot, u32 dci);

/* Transfer operations */
bool xhci_control_transfer(u32 slot, u8 bmReq, u8 bReq, u16 wVal,
                            u16 wIdx, u16 wLen, void *data, u32 *actual);
bool xhci_bulk_transfer(u32 slot, u8 ep_addr, void *data, u32 len, u32 *actual);

/* Instrumentation */
const struct xhci_stats *xhci_get_stats(void);
