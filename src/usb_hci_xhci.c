/*
 * usb_hci_xhci.c - xHCI implementation of the generic USB HCI boundary.
 */
#include "types.h"
#include "usb_hci.h"
#include "xhci.h"

static bool xhci_controller_valid(u32 controller)
{
    return controller < USB_HCI_MAX_CONTROLLERS;
}

static u32 xhci_controller_count_adapter(void)
{
    return USB_HCI_MAX_CONTROLLERS;
}

static bool xhci_controller_init_adapter(u32 controller)
{
    if (!xhci_controller_valid(controller))
        return false;
    xhci_select_controller(controller);
    return xhci_init();
}

static u32 xhci_port_count_adapter(u32 controller)
{
    if (!xhci_controller_valid(controller))
        return 0U;
    xhci_select_controller(controller);
    return xhci_port_count();
}

static bool xhci_port_connected_adapter(u32 controller, u32 port)
{
    if (!xhci_controller_valid(controller))
        return false;
    xhci_select_controller(controller);
    return xhci_port_connected(port);
}

static bool xhci_port_reset_adapter(u32 controller, u32 port, u32 *speed)
{
    if (!xhci_controller_valid(controller) || !speed)
        return false;
    xhci_select_controller(controller);
    return xhci_port_reset(port, speed);
}

static bool xhci_enable_device_adapter(u32 controller, u32 *device_id)
{
    if (!xhci_controller_valid(controller) || !device_id)
        return false;
    xhci_select_controller(controller);
    return xhci_enable_slot(device_id);
}

static bool xhci_address_device_adapter(u32 controller, u32 device_id,
                                        u32 port, u32 speed, u32 max_packet)
{
    if (!xhci_controller_valid(controller))
        return false;
    xhci_select_controller(controller);
    return xhci_address_device(device_id, port, speed, max_packet);
}

static bool xhci_configure_endpoints_adapter(
    u32 controller, u32 device_id, u32 port, u32 speed,
    const struct usb_hci_endpoint *eps, u32 count)
{
    struct xhci_ep_info xhci_eps[USB_HCI_MAX_ENDPOINTS];
    u32 i;

    if (!xhci_controller_valid(controller) || (!eps && count != 0U) ||
        count > USB_HCI_MAX_ENDPOINTS)
        return false;
    for (i = 0U; i < count; i++) {
        xhci_eps[i].address = eps[i].address;
        xhci_eps[i].attributes = eps[i].attributes;
        xhci_eps[i].max_packet = eps[i].max_packet;
        xhci_eps[i].interval = eps[i].interval;
    }
    xhci_select_controller(controller);
    return xhci_configure_endpoints(device_id, port, speed, xhci_eps, count);
}

static bool xhci_control_transfer_adapter(
    u32 controller, u32 device_id, u8 bm_req, u8 b_req, u16 w_value,
    u16 w_index, u16 w_length, void *data, u32 *actual)
{
    if (!xhci_controller_valid(controller))
        return false;
    xhci_select_controller(controller);
    return xhci_control_transfer(device_id, bm_req, b_req, w_value, w_index,
                                 w_length, data, actual);
}

static bool xhci_bulk_transfer_adapter(u32 controller, u32 device_id,
                                       u8 ep_addr, void *data, u32 length,
                                       u32 *actual)
{
    if (!xhci_controller_valid(controller))
        return false;
    xhci_select_controller(controller);
    return xhci_bulk_transfer(device_id, ep_addr, data, length, actual);
}

static bool xhci_interrupt_submit_adapter(u32 controller, u32 device_id,
                                          u8 ep_addr, void *data, u32 length)
{
    if (!xhci_controller_valid(controller))
        return false;
    xhci_select_controller(controller);
    return xhci_interrupt_submit(device_id, ep_addr, data, length);
}

static bool xhci_interrupt_poll_adapter(u32 controller, u32 *actual,
                                        bool *complete)
{
    if (!xhci_controller_valid(controller) || !actual || !complete)
        return false;
    xhci_select_controller(controller);
    return xhci_interrupt_poll(actual, complete);
}

static const struct usb_hci_ops xhci_ops = {
    .controller_count = xhci_controller_count_adapter,
    .controller_init = xhci_controller_init_adapter,
    .port_count = xhci_port_count_adapter,
    .port_connected = xhci_port_connected_adapter,
    .port_reset = xhci_port_reset_adapter,
    .enable_device = xhci_enable_device_adapter,
    .address_device = xhci_address_device_adapter,
    .configure_endpoints = xhci_configure_endpoints_adapter,
    .control_transfer = xhci_control_transfer_adapter,
    .bulk_transfer = xhci_bulk_transfer_adapter,
    .interrupt_submit = xhci_interrupt_submit_adapter,
    .interrupt_poll = xhci_interrupt_poll_adapter,
};

const struct usb_hci_ops *usb_hci_xhci_ops(void)
{
    return &xhci_ops;
}
