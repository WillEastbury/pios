/*
 * usb_hci.h - transport-neutral USB host-controller boundary.
 *
 * USB enumeration and class drivers use these operations without knowing the
 * transport-specific slot, endpoint-context, or controller representation.
 */
#pragma once
#include "types.h"

#define USB_HCI_MAX_CONTROLLERS  2U
#define USB_HCI_MAX_ENDPOINTS    8U

#define USB_HCI_SPEED_FULL       1U
#define USB_HCI_SPEED_LOW        2U
#define USB_HCI_SPEED_HIGH       3U
#define USB_HCI_SPEED_SUPER      4U

struct usb_hci_endpoint {
    u8 address;
    u8 attributes;
    u16 max_packet;
    u8 interval;
};

struct usb_hci_ops {
    u32 (*controller_count)(void);
    bool (*controller_init)(u32 controller);
    u32 (*port_count)(u32 controller);
    bool (*port_connected)(u32 controller, u32 port);
    bool (*port_reset)(u32 controller, u32 port, u32 *speed);
    bool (*enable_device)(u32 controller, u32 *device_id);
    bool (*address_device)(u32 controller, u32 device_id, u32 port,
                           u32 speed, u32 max_packet);
    bool (*configure_endpoints)(u32 controller, u32 device_id, u32 port,
                                u32 speed,
                                const struct usb_hci_endpoint *eps,
                                u32 count);
    bool (*control_transfer)(u32 controller, u32 device_id, u8 bm_req,
                             u8 b_req, u16 w_value, u16 w_index,
                             u16 w_length, void *data, u32 *actual);
    bool (*bulk_transfer)(u32 controller, u32 device_id, u8 ep_addr,
                          void *data, u32 length, u32 *actual);
    bool (*interrupt_submit)(u32 controller, u32 device_id, u8 ep_addr,
                             void *data, u32 length);
    bool (*interrupt_poll)(u32 controller, u32 *actual, bool *complete);
};

/*
 * The only enabled transport. This factory returns a complete xHCI adapter;
 * its transport-private definitions never escape this boundary.
 */
const struct usb_hci_ops *usb_hci_xhci_ops(void);
