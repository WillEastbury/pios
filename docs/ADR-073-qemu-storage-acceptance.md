# ADR-073: QEMU-only storage acceptance adapter

**Date:** 2026-09-13 · **Decider:** Owner (explicit request) · **Status:** Accepted

## Decision

QEMU storage acceptance consumes the shared pre-created three-primary MBR
layout: boot FAT32 p1, raw PIOS/WALFS p2, and FAT32 p3 `PIOSXFER`. The default
QEMU disk builder retains a two-partition legacy fixture; only
`tools/build_qemu_disk_image.py --exchange` creates the three-partition
acceptance fixture. This builder is test-media construction, not a PIOS
runtime partitioning path.

`qemu_xfer` is a narrow compatibility command surface over the generic
`exchange_service` attachment. The generic service mounts valid p3 on every
platform after SD/WALFS setup, with an immutable MBR snapshot and p3-only
callbacks. Hardware mounts are read-only and expose `exchange status`; QEMU
alone enables the bounded mutation commands for acceptance. On QEMU it
requires a virtio-blk backend, validates the shared MBR signature and fixed
role/type/non-overlap predicate, then requires a nonzero disk identity and a
mountable FAT32 p3. The `PIOSXFER` BPB label in the fixture is diagnostic only.
The MBR status bit is not authority; active/inactive is accepted as valid
wire evidence.
It derives an attachment from that p3 fact and its callback rejects every LBA
outside the exact p3 span. The MBR snapshot is acquired before attachment;
afterward it never reads through or writes through boot p1 or WALFS p2.
Generic `fat32.c` behavior remains unchanged.

The acceptance surface is intentionally small: bounded 8.3 root files and
1 KiB hex command payloads only.  `xfer verify` remounts p3 and checks mirrored
FAT sectors; it is an integrity probe, not a crash-consistency claim.  FAT32
LFNs/directories and concurrent host mounting are outside this test.

## Verification

`tools/qemu_storage_acceptance.py` builds a direct QEMU kernel, makes a
persistent preformatted `--exchange` disk, attaches it twice for the QEMU virtio-block
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
