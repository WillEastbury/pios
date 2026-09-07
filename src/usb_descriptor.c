#include "usb.h"

#define USB_DESC_CONFIG      2U
#define USB_DESC_INTERFACE   4U
#define USB_DESC_ENDPOINT    5U

static u16 read_le16(const u8 *p)
{
    return (u16)p[0] | ((u16)p[1] << 8);
}

bool usb_parse_config_descriptors(const u8 *buf, u32 len,
                                  struct usb_device *dev)
{
    u32 total;
    u32 off;
    bool have_iface = false;
    u8 iface = 0;
    u8 iface_class = 0;
    u8 iface_subclass = 0;
    u8 iface_protocol = 0;

    if (!buf || !dev || len < 9U || len > USB_MAX_CONFIG_DESC)
        return false;
    if (buf[0] < 9U || buf[1] != USB_DESC_CONFIG)
        return false;

    total = read_le16(buf + 2);
    if (total < 9U || total > len)
        return false;
    if (buf[0] > total)
        return false;

    dev->config_value = buf[5];
    dev->num_interfaces = buf[4];
    dev->num_eps = 0;

    off = buf[0];
    while (off < total) {
        u8 dlen;
        u8 dtype;
        if (total - off < 2U)
            return false;
        dlen = buf[off];
        dtype = buf[off + 1U];
        if (dlen < 2U || dlen > total - off)
            return false;

        if (dtype == USB_DESC_INTERFACE) {
            if (dlen < 9U)
                return false;
            iface = buf[off + 2U];
            iface_class = buf[off + 5U];
            iface_subclass = buf[off + 6U];
            iface_protocol = buf[off + 7U];
            have_iface = true;
        } else if (dtype == USB_DESC_ENDPOINT) {
            u32 idx;
            if (dlen < 7U || !have_iface || dev->num_eps >= USB_MAX_ENDPOINTS)
                return false;
            idx = dev->num_eps++;
            dev->eps[idx].address = buf[off + 2U];
            dev->eps[idx].attributes = buf[off + 3U];
            dev->eps[idx].max_packet = read_le16(buf + off + 4U) & 0x7FFU;
            dev->eps[idx].interval = buf[off + 6U];
            dev->eps[idx].iface_number = iface;
            dev->eps[idx].iface_class = iface_class;
            dev->eps[idx].iface_subclass = iface_subclass;
            dev->eps[idx].iface_protocol = iface_protocol;
        }
        off += dlen;
    }
    return off == total;
}
