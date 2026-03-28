/*
 * usb_storage.c - USB Mass Storage (Bulk-Only Transport) class driver
 *
 * Implements BBB (Bulk-Only) transport with SCSI transparent command set.
 * Provides raw block I/O for USB flash drives, hard drives, etc.
 *
 * Protocol:
 *   Host sends CBW (31 bytes) → optional data phase → device sends CSW (13 bytes)
 *
 * SCSI commands used:
 *   INQUIRY (0x12), TEST_UNIT_READY (0x00), READ_CAPACITY(10) (0x25),
 *   READ(10) (0x28), WRITE(10) (0x2A)
 *
 * Reference: USB Mass Storage Class Bulk-Only Transport Spec 1.0
 *            SCSI Primary Commands (SPC), SCSI Block Commands (SBC)
 */

#include "usb_storage.h"
#include "usb.h"
#include "uart.h"
#include "timer.h"
#include "mmu.h"
#include "mmio.h"

/* ---- BBB Protocol Structures ---- */

#define CBW_SIGNATURE   0x43425355  /* "USBC" */
#define CSW_SIGNATURE   0x53425355  /* "USBS" */

struct usb_cbw {
    u32 dCBWSignature;
    u32 dCBWTag;
    u32 dCBWDataTransferLength;
    u8  bmCBWFlags;         /* 0x80 = IN, 0x00 = OUT */
    u8  bCBWLUN;
    u8  bCBWCBLength;
    u8  CBWCB[16];
} PACKED;

struct usb_csw {
    u32 dCSWSignature;
    u32 dCSWTag;
    u32 dCSWDataResidue;
    u8  bCSWStatus;         /* 0=OK, 1=failed, 2=phase error */
} PACKED;

/* ---- Interface match criteria ---- */

#define USB_CLASS_MASS_STORAGE  0x08
#define USB_SUBCLASS_SCSI       0x06
#define USB_PROTOCOL_BBB        0x50

/* ---- Driver State ---- */

static struct usb_device *stor_dev;
static u8 bulk_in_addr;
static u8 bulk_out_addr;
static u16 bulk_in_maxpkt;
static u16 bulk_out_maxpkt;
static u32 tag_counter;
static u32 blk_size;
static u64 num_blocks;
static bool stor_ready;

static u8 cbw_buf[31] ALIGNED(64);
static u8 csw_buf[13] ALIGNED(64);
static u8 scsi_buf[512] ALIGNED(64);

/* ---- SCSI Command Helpers ---- */

static bool bbb_transfer(const u8 *scsi_cmd, u8 cmd_len, u8 dir,
                          void *data, u32 data_len) {
    struct usb_cbw *cbw = (struct usb_cbw *)cbw_buf;
    struct usb_csw *csw = (struct usb_csw *)csw_buf;
    u32 actual;

    /* Build CBW */
    cbw->dCBWSignature = CBW_SIGNATURE;
    cbw->dCBWTag = ++tag_counter;
    cbw->dCBWDataTransferLength = data_len;
    cbw->bmCBWFlags = dir;
    cbw->bCBWLUN = 0;
    cbw->bCBWCBLength = cmd_len;
    for (u32 i = 0; i < 16; i++)
        cbw->CBWCB[i] = (i < cmd_len) ? scsi_cmd[i] : 0;

    /* Send CBW via bulk OUT */
    if (!usb_bulk_msg(stor_dev, bulk_out_addr, cbw_buf, 31, &actual))
        return false;

    /* Data phase (if any) */
    if (data_len > 0 && data) {
        u8 ep = (dir & 0x80) ? bulk_in_addr : bulk_out_addr;
        if (!usb_bulk_msg(stor_dev, ep, data, data_len, &actual))
            return false;
    }

    /* Receive CSW via bulk IN */
    if (!usb_bulk_msg(stor_dev, bulk_in_addr, csw_buf, 13, &actual))
        return false;

    if (csw->dCSWSignature != CSW_SIGNATURE) {
        uart_puts("[usb_stor] Bad CSW signature\n");
        return false;
    }
    if (csw->bCSWStatus != 0) {
        return false;
    }
    return true;
}

static bool scsi_inquiry(void) {
    u8 cmd[6] = { 0x12, 0, 0, 0, 36, 0 }; /* INQUIRY, 36 bytes */
    if (!bbb_transfer(cmd, 6, 0x80, scsi_buf, 36))
        return false;

    uart_puts("[usb_stor] INQUIRY: ");
    /* Vendor (bytes 8-15) + Product (bytes 16-31) */
    for (u32 i = 8; i < 32; i++) {
        char c = (char)scsi_buf[i];
        if (c >= 0x20 && c <= 0x7E)
            uart_putc(c);
    }
    uart_puts("\n");
    return true;
}

static bool scsi_test_unit_ready(void) {
    u8 cmd[6] = { 0x00, 0, 0, 0, 0, 0 };
    return bbb_transfer(cmd, 6, 0x00, NULL, 0);
}

static bool scsi_read_capacity(void) {
    u8 cmd[10] = { 0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    if (!bbb_transfer(cmd, 10, 0x80, scsi_buf, 8))
        return false;

    dcache_invalidate_range((u64)(usize)scsi_buf, 8);

    u32 last_lba = ((u32)scsi_buf[0] << 24) | ((u32)scsi_buf[1] << 16) |
                   ((u32)scsi_buf[2] << 8)  | (u32)scsi_buf[3];
    blk_size    = ((u32)scsi_buf[4] << 24) | ((u32)scsi_buf[5] << 16) |
                   ((u32)scsi_buf[6] << 8)  | (u32)scsi_buf[7];
    num_blocks = (u64)last_lba + 1;

    uart_puts("[usb_stor] Capacity: ");
    uart_hex((u32)num_blocks);
    uart_puts(" blocks x ");
    uart_hex(blk_size);
    uart_puts(" bytes\n");
    return true;
}

/* ---- Driver Callbacks ---- */

static bool stor_match(struct usb_device *dev) {
    for (u32 i = 0; i < dev->num_eps; i++) {
        if (dev->eps[i].iface_class == USB_CLASS_MASS_STORAGE &&
            dev->eps[i].iface_subclass == USB_SUBCLASS_SCSI &&
            dev->eps[i].iface_protocol == USB_PROTOCOL_BBB)
            return true;
    }
    return false;
}

static bool stor_probe(struct usb_device *dev) {
    stor_dev = dev;
    bulk_in_addr = 0;
    bulk_out_addr = 0;
    stor_ready = false;

    /* Find bulk IN and bulk OUT endpoints */
    for (u32 i = 0; i < dev->num_eps; i++) {
        if ((dev->eps[i].attributes & 0x03) != USB_EP_ATTR_BULK)
            continue;
        if (dev->eps[i].address & USB_DIR_IN) {
            bulk_in_addr = dev->eps[i].address;
            bulk_in_maxpkt = dev->eps[i].max_packet;
        } else {
            bulk_out_addr = dev->eps[i].address;
            bulk_out_maxpkt = dev->eps[i].max_packet;
        }
    }

    if (!bulk_in_addr || !bulk_out_addr) {
        uart_puts("[usb_stor] Missing bulk endpoints\n");
        return false;
    }

    uart_puts("[usb_stor] Bulk IN=");
    uart_hex(bulk_in_addr);
    uart_puts(" OUT=");
    uart_hex(bulk_out_addr);
    uart_puts("\n");

    /* SCSI init sequence */
    scsi_inquiry();

    /* TEST_UNIT_READY may fail initially, retry */
    for (u32 i = 0; i < 5; i++) {
        if (scsi_test_unit_ready())
            break;
        timer_delay_ms(500);
    }

    if (!scsi_read_capacity())
        return false;

    stor_ready = true;
    uart_puts("[usb_stor] Ready\n");
    return true;
}

static void stor_disconnect(struct usb_device *dev) {
    (void)dev;
    stor_ready = false;
    stor_dev = NULL;
}

static struct usb_driver storage_driver = {
    .name = "usb_storage",
    .match = stor_match,
    .probe = stor_probe,
    .disconnect = stor_disconnect,
};

/* ---- Public API ---- */

void usb_storage_register(void) {
    usb_register_driver(&storage_driver);
}

bool usb_storage_ready(void) { return stor_ready; }
u32  usb_storage_block_size(void) { return blk_size; }
u64  usb_storage_num_blocks(void) { return num_blocks; }

bool usb_storage_read(u32 lba, u32 count, void *buf) {
    if (!stor_ready) return false;

    u8 *dst = (u8 *)buf;
    while (count > 0) {
        u32 n = (count > 128) ? 128 : count; /* max 128 blocks per transfer */
        u32 len = n * blk_size;

        u8 cmd[10] = {
            0x28, 0,                                    /* READ(10) */
            (u8)(lba >> 24), (u8)(lba >> 16),
            (u8)(lba >> 8),  (u8)lba,                   /* LBA (big-endian) */
            0,
            (u8)(n >> 8), (u8)n,                         /* transfer length */
            0
        };

        if (!bbb_transfer(cmd, 10, 0x80, dst, len))
            return false;

        dcache_invalidate_range((u64)(usize)dst, len);
        lba += n;
        count -= n;
        dst += len;
    }
    return true;
}

bool usb_storage_write(u32 lba, u32 count, const void *buf) {
    if (!stor_ready) return false;

    const u8 *src = (const u8 *)buf;
    while (count > 0) {
        u32 n = (count > 128) ? 128 : count;
        u32 len = n * blk_size;

        u8 cmd[10] = {
            0x2A, 0,                                    /* WRITE(10) */
            (u8)(lba >> 24), (u8)(lba >> 16),
            (u8)(lba >> 8),  (u8)lba,
            0,
            (u8)(n >> 8), (u8)n,
            0
        };

        dcache_clean_range((u64)(usize)src, len);
        if (!bbb_transfer(cmd, 10, 0x00, (void *)src, len))
            return false;

        lba += n;
        count -= n;
        src += len;
    }
    return true;
}
