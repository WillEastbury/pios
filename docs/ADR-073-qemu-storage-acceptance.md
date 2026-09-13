# ADR-073: QEMU-only storage acceptance adapter

**Date:** 2026-09-13 · **Decider:** Owner (explicit request) · **Status:** Accepted

## Decision

QEMU storage acceptance may add an optional third MBR primary partition,
`PIOSXFER`, after boot FAT32 p1 and raw PIOS/WALFS p2.  The default QEMU disk
builder layout is unchanged; only `tools/build_qemu_disk_image.py --exchange`
creates p3.

`qemu_xfer` is a narrow test adapter.  It is compiled as a no-op unavailable
stub on every non-QEMU platform.  On QEMU it requires a virtio-blk backend,
validates the MBR signature, nonzero disk identity, p1/p2/p3 in-device
non-overlapping spans, inactive FAT32 p3, and the `PIOSXFER` FAT32 BPB label.
It derives an attachment from that p3 fact and its callback rejects every LBA
outside the exact p3 span.  It never selects, reads through, or writes through
boot p1 or WALFS p2.  Generic `fat32.c` behavior remains unchanged.

The acceptance surface is intentionally small: bounded 8.3 root files and
1 KiB hex command payloads only.  `xfer verify` remounts p3 and checks mirrored
FAT sectors; it is an integrity probe, not a crash-consistency claim.  FAT32
LFNs/directories and concurrent host mounting are outside this test.

## Verification

`tools/qemu_storage_acceptance.py` builds a direct QEMU kernel, makes a
persistent `--exchange` disk, attaches it twice for the QEMU virtio-block
probe, and drives real HTTP terminal commands.  It verifies FAT write/read/
rewrite/append/rename/delete/remount plus WALFS create/update/read/copy/delete
and `walfs verify`.  It then reboots QEMU against the same disk to prove both
stores remount.  During mutation pressure it concurrently probes status,
FIFO/IPC, and one bounded synthetic OTA stream.  The gate requires no command
error, no liveness gap, and every operation to finish within ten seconds; it
reports operation count and average/maximum latency.

This does not validate hardware SD, p1 boot mutation, p2 formatting, power-loss
atomicity, host/guest concurrent access, or performance beyond QEMU's bounded
correctness acceptance.
