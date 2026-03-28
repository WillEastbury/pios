/*
 * usb_storage.h - USB Mass Storage (Bulk-Only Transport) driver
 *
 * Provides raw block I/O over USB mass storage devices.
 * Uses SCSI transparent command set over BBB (Bulk-Only) transport.
 *
 * Plugs into the USB framework via usb_register_driver().
 */

#pragma once
#include "types.h"

/* Register the mass storage driver with the USB framework */
void usb_storage_register(void);

/* Check if a USB storage device is ready */
bool usb_storage_ready(void);

/* Device geometry */
u32  usb_storage_block_size(void);
u64  usb_storage_num_blocks(void);

/* Block I/O (LBA addressing, count in blocks) */
bool usb_storage_read(u32 lba, u32 count, void *buf);
bool usb_storage_write(u32 lba, u32 count, const void *buf);
