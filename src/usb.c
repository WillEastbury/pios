/*
 * usb.c - USB core: enumeration + pluggable driver framework
 *
 * Initialises xHCI, scans ports for connected devices, enumerates
 * via standard USB descriptors, then matches and probes registered
 * class drivers (e.g. usb_storage, usb_kbd).
 */

#include "usb.h"
#include "xhci.h"
#include "mmio.h"
#include "uart.h"
#include "timer.h"
#include "mmu.h"
#include "fb.h"

/* ---- USB Descriptor Types ---- */

#define DESC_DEVICE         1
#define DESC_CONFIG         2
#define DESC_INTERFACE      4
#define DESC_ENDPOINT       5

/* ---- USB Descriptor Structures ---- */

struct usb_dev_desc {
    u8  bLength, bDescriptorType;
    u16 bcdUSB;
    u8  bDeviceClass, bDeviceSubClass, bDeviceProtocol, bMaxPacketSize0;
    u16 idVendor, idProduct, bcdDevice;
    u8  iManufacturer, iProduct, iSerialNumber, bNumConfigurations;
} PACKED;

struct usb_cfg_desc {
    u8  bLength, bDescriptorType;
    u16 wTotalLength;
    u8  bNumInterfaces, bConfigurationValue, iConfiguration;
    u8  bmAttributes, bMaxPower;
} PACKED;

struct usb_iface_desc {
    u8  bLength, bDescriptorType;
    u8  bInterfaceNumber, bAlternateSetting, bNumEndpoints;
    u8  bInterfaceClass, bInterfaceSubClass, bInterfaceProtocol;
    u8  iInterface;
} PACKED;

struct usb_ep_desc {
    u8  bLength, bDescriptorType;
    u8  bEndpointAddress, bmAttributes;
    u16 wMaxPacketSize;
    u8  bInterval;
} PACKED;

/* ---- Driver Registry ---- */

static struct usb_driver *drivers[USB_MAX_DRIVERS];
static u32 num_drivers;

/* ---- Device State ---- */

static struct usb_device the_device;
static bool device_valid;

static u8 desc_buf[512] ALIGNED(64);

/* ---- Driver Registration ---- */

void usb_register_driver(struct usb_driver *drv) {
    if (num_drivers < USB_MAX_DRIVERS)
        drivers[num_drivers++] = drv;
}

/* ---- Transfer Wrappers ---- */

bool usb_control_msg(struct usb_device *dev, u8 bmReq, u8 bReq,
                     u16 wVal, u16 wIdx, u16 wLen, void *data, u32 *actual) {
    return xhci_control_transfer(dev->slot, bmReq, bReq, wVal, wIdx, wLen, data, actual);
}

bool usb_bulk_msg(struct usb_device *dev, u8 ep_addr,
                  void *data, u32 len, u32 *actual) {
    return xhci_bulk_transfer(dev->slot, ep_addr, data, len, actual);
}

struct usb_device *usb_get_device(void) {
    return device_valid ? &the_device : NULL;
}

/* ---- Enumeration ---- */

static bool enumerate_device(u32 port, u32 speed) {
    struct usb_device *dev = &the_device;
    u32 got = 0;

    dev->port = port;
    dev->speed = speed;
    dev->num_eps = 0;

    /* Default max packet for EP0 based on speed */
    switch (speed) {
    case USB_SPEED_LOW:   dev->max_packet_ep0 = 8; break;
    case USB_SPEED_FULL:  dev->max_packet_ep0 = 8; break;
    case USB_SPEED_HIGH:  dev->max_packet_ep0 = 64; break;
    case USB_SPEED_SUPER: dev->max_packet_ep0 = 512; break;
    default:              dev->max_packet_ep0 = 8; break;
    }

    /* Enable Slot */
    if (!xhci_enable_slot(&dev->slot)) {
        uart_puts("[usb] Enable slot failed\n");
        return false;
    }
    uart_puts("[usb] Slot ");
    uart_hex(dev->slot);
    uart_puts("\n");

    /* Address Device */
    if (!xhci_address_device(dev->slot, port, speed, dev->max_packet_ep0)) {
        uart_puts("[usb] Address device failed\n");
        return false;
    }

    /* GET_DESCRIPTOR: Device */
    struct usb_dev_desc *dd = (struct usb_dev_desc *)desc_buf;
    if (!xhci_control_transfer(dev->slot, USB_DIR_IN, 0x06,
                                (DESC_DEVICE << 8), 0, 18, dd, &got)) {
        uart_puts("[usb] GET_DESCRIPTOR(device) failed\n");
        return false;
    }

    dev->vendor_id = dd->idVendor;
    dev->product_id = dd->idProduct;
    dev->dev_class = dd->bDeviceClass;
    dev->dev_subclass = dd->bDeviceSubClass;
    dev->dev_protocol = dd->bDeviceProtocol;
    dev->max_packet_ep0 = dd->bMaxPacketSize0;

    uart_puts("[usb] Device ");
    uart_hex(dd->idVendor);
    uart_puts(":");
    uart_hex(dd->idProduct);
    uart_puts("\n");

    /* GET_DESCRIPTOR: Configuration (full) */
    if (!xhci_control_transfer(dev->slot, USB_DIR_IN, 0x06,
                                (DESC_CONFIG << 8), 0, sizeof(desc_buf),
                                desc_buf, &got)) {
        uart_puts("[usb] GET_DESCRIPTOR(config) failed\n");
        return false;
    }

    struct usb_cfg_desc *cfg = (struct usb_cfg_desc *)desc_buf;
    dev->config_value = cfg->bConfigurationValue;
    dev->num_interfaces = cfg->bNumInterfaces;

    /* Parse descriptors: collect endpoints with interface context */
    u8 cur_class = 0, cur_sub = 0, cur_proto = 0;
    u32 off = 0;
    while (off + 2 <= got && dev->num_eps < USB_MAX_ENDPOINTS) {
        u8 len = desc_buf[off];
        u8 type = desc_buf[off + 1];
        if (len < 2 || off + len > got) break; /* bounds check */

        if (type == DESC_INTERFACE && len >= 9 && off + 9 <= got) {
            struct usb_iface_desc *id = (struct usb_iface_desc *)(desc_buf + off);
            cur_class = id->bInterfaceClass;
            cur_sub = id->bInterfaceSubClass;
            cur_proto = id->bInterfaceProtocol;
        } else if (type == DESC_ENDPOINT && len >= 7 && off + 7 <= got) {
            struct usb_ep_desc *ep = (struct usb_ep_desc *)(desc_buf + off);
            u32 idx = dev->num_eps;
            dev->eps[idx].address = ep->bEndpointAddress;
            dev->eps[idx].attributes = ep->bmAttributes;
            dev->eps[idx].max_packet = ep->wMaxPacketSize & 0x7FF;
            dev->eps[idx].interval = ep->bInterval;
            dev->eps[idx].iface_class = cur_class;
            dev->eps[idx].iface_subclass = cur_sub;
            dev->eps[idx].iface_protocol = cur_proto;
            dev->num_eps++;
        }
        off += len;
    }

    /* SET_CONFIGURATION */
    if (!xhci_control_transfer(dev->slot, USB_DIR_OUT, 0x09,
                                dev->config_value, 0, 0, NULL, NULL)) {
        uart_puts("[usb] SET_CONFIGURATION failed\n");
        return false;
    }

    /* Configure non-EP0 endpoints on the xHCI */
    if (dev->num_eps > 0) {
        struct xhci_ep_info xeps[USB_MAX_ENDPOINTS];
        for (u32 i = 0; i < dev->num_eps; i++) {
            xeps[i].address = dev->eps[i].address;
            xeps[i].attributes = dev->eps[i].attributes;
            xeps[i].max_packet = dev->eps[i].max_packet;
            xeps[i].interval = dev->eps[i].interval;
        }
        if (!xhci_configure_endpoints(dev->slot, port, speed, xeps, dev->num_eps)) {
            uart_puts("[usb] Configure endpoints failed\n");
            return false;
        }
    }

    device_valid = true;
    uart_puts("[usb] Device enumerated OK\n");
    return true;
}

/* ---- Driver Matching ---- */

static void probe_drivers(void) {
    if (!device_valid) return;
    for (u32 i = 0; i < num_drivers; i++) {
        if (drivers[i]->match && drivers[i]->match(&the_device)) {
            uart_puts("[usb] Matched driver: ");
            uart_puts(drivers[i]->name);
            uart_puts("\n");
            if (drivers[i]->probe)
                drivers[i]->probe(&the_device);
            return;
        }
    }
    uart_puts("[usb] No matching driver\n");
}

/* ---- Public: Init ---- */

bool usb_init(void) {
    device_valid = false;
    num_drivers = num_drivers; /* preserve pre-registered drivers */

    if (!xhci_init())
        return false;

    timer_delay_ms(200); /* settling time */

    /* Scan ports */
    u32 nports = xhci_port_count();
    uart_puts("[usb] Scanning ");
    uart_hex(nports);
    uart_puts(" ports\n");
    for (u32 p = 0; p < nports; p++) {
        bool conn = xhci_port_connected(p);
        uart_puts("[usb] Port ");
        uart_hex(p);
        uart_puts(conn ? " connected\n" : " empty\n");
        if (!conn)
            continue;

        u32 speed = 0;
        if (!xhci_port_reset(p, &speed)) {
            uart_puts("[usb] Port reset failed\n");
            continue;
        }

        uart_puts("[usb] Speed=");
        uart_hex(speed);
        uart_puts("\n");

        timer_delay_ms(200);  /* USB spec: 200ms after port reset */

        if (enumerate_device(p, speed)) {
            probe_drivers();
            return true;
        }
    }

    uart_puts("[usb] No USB device found\n");
    return false;
}
