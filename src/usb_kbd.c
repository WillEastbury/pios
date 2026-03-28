/*
 * usb_kbd.c - USB HID Keyboard class driver
 *
 * Uses Boot Protocol (SET_PROTOCOL(0)) for fixed 8-byte reports:
 *   [0] modifier keys (bit0=LCtrl, bit1=LShift, bit5=RShift, etc.)
 *   [1] reserved
 *   [2..7] up to 6 simultaneous HID Usage IDs (keycodes)
 *
 * Key-down is detected when a scancode appears that wasn't in the
 * previous report. The scancode is translated to ASCII via lookup
 * table and pushed into a 256-byte ring buffer.
 *
 * Reference: USB HID Usage Tables (Keyboard/Keypad Page 0x07)
 *            USB HID Spec 1.11 §7.2.6 (Boot Protocol)
 */

#include "usb_kbd.h"
#include "usb.h"
#include "xhci.h"
#include "mmio.h"
#include "uart.h"
#include "timer.h"
#include "mmu.h"

/* ---- HID Class Requests ---- */

#define HID_SET_IDLE        0x0A
#define HID_SET_PROTOCOL    0x0B
#define HID_BOOT_PROTOCOL   0

/* ---- HID Modifier Bits ---- */

#define MOD_LCTRL   (1 << 0)
#define MOD_LSHIFT  (1 << 1)
#define MOD_LALT    (1 << 2)
#define MOD_LGUI    (1 << 3)
#define MOD_RCTRL   (1 << 4)
#define MOD_RSHIFT  (1 << 5)
#define MOD_RALT    (1 << 6)
#define MOD_RGUI    (1 << 7)

#define MOD_SHIFT   (MOD_LSHIFT | MOD_RSHIFT)
#define MOD_CTRL    (MOD_LCTRL | MOD_RCTRL)

/* ---- HID Usage ID → ASCII (unshifted) ---- */

static const char hid_to_ascii[128] = {
    0,0,0,0,                                    /* 0x00-0x03: reserved */
    'a','b','c','d','e','f','g','h','i','j',    /* 0x04-0x0D */
    'k','l','m','n','o','p','q','r','s','t',    /* 0x0E-0x17 */
    'u','v','w','x','y','z',                    /* 0x18-0x1D */
    '1','2','3','4','5','6','7','8','9','0',    /* 0x1E-0x27 */
    '\n',                                        /* 0x28: Enter */
    27,                                          /* 0x29: Escape */
    '\b',                                        /* 0x2A: Backspace */
    '\t',                                        /* 0x2B: Tab */
    ' ',                                         /* 0x2C: Space */
    '-','=','[',']','\\',                        /* 0x2D-0x31 */
    0,                                           /* 0x32: non-US # */
    ';','\'','`',',','.','/',                    /* 0x33-0x38 */
    0,                                           /* 0x39: Caps Lock */
    0,0,0,0,0,0,0,0,0,0,0,0,                    /* 0x3A-0x45: F1-F12 */
    0,0,0,0,0,0,0,0,0,                          /* 0x46-0x4E: misc */
    0,0,0,0,                                     /* 0x4F-0x52: arrows */
    0,                                           /* 0x53: Num Lock */
    '/','*','-','+','\n',                        /* 0x54-0x58: keypad */
    '1','2','3','4','5','6','7','8','9','0',    /* 0x59-0x62: keypad */
    '.', 0, 0, 0,                                /* 0x63-0x66 */
    [0x67 ... 0x7F] = 0,
};

/* ---- HID Usage ID → ASCII (shifted) ---- */

static const char hid_to_ascii_shift[128] = {
    0,0,0,0,
    'A','B','C','D','E','F','G','H','I','J',
    'K','L','M','N','O','P','Q','R','S','T',
    'U','V','W','X','Y','Z',
    '!','@','#','$','%','^','&','*','(',')',
    '\n', 27, '\b', '\t', ' ',
    '_','+','{','}','|',
    0,
    ':','"','~','<','>','?',
    [0x39 ... 0x7F] = 0,
};

/* ---- Input Ring Buffer ---- */

#define KBD_BUF_SIZE 256
static char kbd_buf[KBD_BUF_SIZE];
static volatile u32 kbd_head;
static volatile u32 kbd_tail;

static void kbd_push(char c) {
    u32 next = (kbd_head + 1) & (KBD_BUF_SIZE - 1);
    if (next == kbd_tail) return; /* full, drop */
    kbd_buf[kbd_head] = c;
    kbd_head = next;
}

/* ---- Driver State ---- */

static struct usb_device *kbd_dev;
static u8 int_ep_addr;
static u16 int_ep_maxpkt;
static bool kbd_ready;
static u8 prev_keys[6];

static u8 report_buf[8] ALIGNED(64);
static bool poll_pending;

/* ---- HID Boot Protocol Report Parsing ---- */

static bool was_pressed(u8 code) {
    for (u32 i = 0; i < 6; i++)
        if (prev_keys[i] == code) return true;
    return false;
}

static void process_report(const u8 *report) {
    u8 mods = report[0];
    const u8 *keys = &report[2];

    /* Detect newly pressed keys */
    for (u32 i = 0; i < 6; i++) {
        u8 code = keys[i];
        if (code == 0 || code >= 128) continue;
        if (was_pressed(code)) continue;

        /* Translate to ASCII */
        char c;
        if (mods & MOD_SHIFT)
            c = hid_to_ascii_shift[code];
        else
            c = hid_to_ascii[code];

        if (mods & MOD_CTRL) {
            /* Ctrl+A..Z → 0x01..0x1A */
            if (c >= 'a' && c <= 'z')
                c = (char)(c - 'a' + 1);
            else if (c >= 'A' && c <= 'Z')
                c = (char)(c - 'A' + 1);
        }

        if (c)
            kbd_push(c);
    }

    /* Save for next comparison */
    for (u32 i = 0; i < 6; i++)
        prev_keys[i] = keys[i];
}

/* ---- Queue Interrupt Transfer ---- */

static void queue_poll(void) {
    if (!kbd_ready || poll_pending) return;
    dcache_invalidate_range((u64)(usize)report_buf, sizeof(report_buf));
    u32 actual;
    if (xhci_bulk_transfer(kbd_dev->slot, int_ep_addr, report_buf,
                            int_ep_maxpkt < 8 ? 8 : int_ep_maxpkt, &actual)) {
        dcache_invalidate_range((u64)(usize)report_buf, sizeof(report_buf));
        process_report(report_buf);
    }
}

/* ---- Driver Callbacks ---- */

static bool kbd_match(struct usb_device *dev) {
    for (u32 i = 0; i < dev->num_eps; i++) {
        if (dev->eps[i].iface_class == 3 &&     /* HID */
            dev->eps[i].iface_protocol == 1)     /* Keyboard */
            return true;
    }
    return false;
}

static bool kbd_probe(struct usb_device *dev) {
    kbd_dev = dev;
    kbd_ready = false;
    int_ep_addr = 0;

    /* Find interrupt IN endpoint */
    for (u32 i = 0; i < dev->num_eps; i++) {
        if ((dev->eps[i].attributes & 0x03) == USB_EP_ATTR_INTR &&
            (dev->eps[i].address & USB_DIR_IN)) {
            int_ep_addr = dev->eps[i].address;
            int_ep_maxpkt = dev->eps[i].max_packet;
            break;
        }
    }

    if (!int_ep_addr) {
        uart_puts("[usb_kbd] No interrupt IN endpoint\n");
        return false;
    }

    /* SET_PROTOCOL: Boot Protocol (0) — fixed 8-byte reports */
    usb_control_msg(dev, 0x21, HID_SET_PROTOCOL, HID_BOOT_PROTOCOL,
                    0, 0, NULL, NULL);

    /* SET_IDLE: report only on change */
    usb_control_msg(dev, 0x21, HID_SET_IDLE, 0, 0, 0, NULL, NULL);

    for (u32 i = 0; i < 6; i++) prev_keys[i] = 0;
    kbd_head = 0;
    kbd_tail = 0;
    poll_pending = false;
    kbd_ready = true;

    uart_puts("[usb_kbd] Keyboard ready (EP ");
    uart_hex(int_ep_addr);
    uart_puts(")\n");
    return true;
}

static void kbd_disconnect(struct usb_device *dev) {
    (void)dev;
    kbd_ready = false;
    kbd_dev = NULL;
}

static struct usb_driver kbd_driver = {
    .name = "usb_kbd",
    .match = kbd_match,
    .probe = kbd_probe,
    .disconnect = kbd_disconnect,
};

/* ---- Public API ---- */

void usb_kbd_register(void) {
    usb_register_driver(&kbd_driver);
}

bool usb_kbd_available(void) {
    return kbd_ready;
}

void usb_kbd_poll(void) {
    if (!kbd_ready) return;
    queue_poll();
}

i32 usb_kbd_try_getc(void) {
    usb_kbd_poll();
    if (kbd_head == kbd_tail) return -1;
    char c = kbd_buf[kbd_tail];
    kbd_tail = (kbd_tail + 1) & (KBD_BUF_SIZE - 1);
    return (i32)c;
}

char usb_kbd_getc(void) {
    i32 c;
    do {
        c = usb_kbd_try_getc();
    } while (c < 0);
    return (char)c;
}
