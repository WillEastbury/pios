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

/* Blocking: waits for a keypress */
char usb_kbd_getc(void);

/* Poll for new keyboard input (call periodically from a core loop) */
void usb_kbd_poll(void);
