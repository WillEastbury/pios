/*
 * xhci.h - xHCI USB Host Controller driver
 *
 * Minimal xHCI driver for one USB device (keyboard) via RP1 DWC3.
 * RP1 has two DWC3 controllers: USB0 at BAR+0x200000, USB1 at BAR+0x300000.
 * DWC3 wraps xHCI and requires host-mode init before xHCI registers are usable.
 *
 * Reference: xHCI Spec 1.2, OSDev wiki, Linux drivers/usb/host/xhci*.c
 *            rp1.dtsi: usb@200000 { compatible = "snps,dwc3"; }
 */

#pragma once
#include "types.h"

/* RP1 DWC3 controller offsets from RP1_BAR_BASE */
#define XHCI_USB0_OFFSET    0x200000
#define XHCI_USB1_OFFSET    0x300000

/* Initialise xHCI controller (USB0) and enumerate first connected device */
bool xhci_init(void);

/* Poll for keyboard input. Returns USB HID keycode, or -1 if none. */
i32 xhci_get_keypress(void);

/* Check if a USB device is connected and enumerated */
bool xhci_device_connected(void);
