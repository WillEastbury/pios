/*
 * usb_kbd.h - USB HID Keyboard driver
 *
 * Plugs into the USB framework. Provides scancode-to-ASCII translation
 * with shift/ctrl modifiers and a 256-byte input ring buffer.
 *
 * Uses HID Boot Protocol (8-byte reports) for simplicity.
 */

#pragma once
#include "types.h"

/* Register the keyboard driver with the USB framework */
void usb_kbd_register(void);

/* Check if a keyboard is connected and ready */
bool usb_kbd_available(void);

/* Non-blocking: returns ASCII char or -1 if no key */
i32 usb_kbd_try_getc(void);

/* Non-blocking: returns special key code (USB_KBD_KEY_*) or -1 if none */
i32 usb_kbd_try_getkey(void);

/* Blocking: waits for a keypress */
char usb_kbd_getc(void);

/* Poll for new keyboard input (call periodically from a core loop) */
void usb_kbd_poll(void);

#define USB_KBD_KEY_F1   0x3A
#define USB_KBD_KEY_F2   0x3B
#define USB_KBD_KEY_F3   0x3C
#define USB_KBD_KEY_F4   0x3D
#define USB_KBD_KEY_INSERT 0x49
#define USB_KBD_KEY_HOME   0x4A
#define USB_KBD_KEY_DELETE 0x4C
#define USB_KBD_KEY_END    0x4D
#define USB_KBD_KEY_RIGHT  0x4F
#define USB_KBD_KEY_LEFT   0x50
#define USB_KBD_KEY_DOWN   0x51
#define USB_KBD_KEY_UP     0x52
