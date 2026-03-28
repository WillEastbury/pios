/*
 * usb.h - USB core framework
 *
 * Pluggable driver framework: class drivers register via usb_register_driver()
 * and are probed when a matching device is enumerated.
 *
 * Transfer wrappers provide a stable API for class drivers regardless of
 * the underlying host controller (xHCI today, potentially others later).
 */

#pragma once
#include "types.h"

#define USB_MAX_DRIVERS     4
#define USB_MAX_ENDPOINTS   8

#define USB_DIR_IN          0x80
#define USB_DIR_OUT         0x00
#define USB_EP_ATTR_CTRL    0
#define USB_EP_ATTR_ISOC    1
#define USB_EP_ATTR_BULK    2
#define USB_EP_ATTR_INTR    3

/* Device descriptor parsed from enumeration */
struct usb_device {
    u32 slot;
    u32 port;
    u32 speed;
    u16 vendor_id;
    u16 product_id;
    u8  dev_class;
    u8  dev_subclass;
    u8  dev_protocol;
    u16 max_packet_ep0;
    u8  config_value;
    u8  num_interfaces;
    u8  num_eps;
    struct {
        u8  address;
        u8  attributes;
        u16 max_packet;
        u8  interval;
        u8  iface_class;
        u8  iface_subclass;
        u8  iface_protocol;
    } eps[USB_MAX_ENDPOINTS];
};

/* Pluggable class driver */
struct usb_driver {
    const char *name;
    bool (*match)(struct usb_device *dev);
    bool (*probe)(struct usb_device *dev);
    void (*disconnect)(struct usb_device *dev);
};

/* Driver registration (call before usb_init) */
void usb_register_driver(struct usb_driver *drv);

/* Init HCI, enumerate devices, probe matching drivers */
bool usb_init(void);

/* Transfer API for class drivers */
bool usb_control_msg(struct usb_device *dev, u8 bmReq, u8 bReq,
                     u16 wVal, u16 wIdx, u16 wLen, void *data, u32 *actual);
bool usb_bulk_msg(struct usb_device *dev, u8 ep_addr,
                  void *data, u32 len, u32 *actual);

/* Get the currently enumerated device (NULL if none) */
struct usb_device *usb_get_device(void);
