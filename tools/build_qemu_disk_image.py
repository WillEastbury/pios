#!/usr/bin/env python3
"""build_qemu_disk_image.py - build a virtual SD-card-shaped disk image for
QEMU that matches docs/disk_layout.md exactly: MBR (LBA 0) + partition 1
(FAT32 boot, holding PIOSSTG2.PKG) + partition 2 (raw PIOS system area,
WALFS-formatted on first mount). This lets a QEMU-booted stage0 take the
SAME "partition-2" discovery path as real Pi5 hardware (src/walfs.c
discover_partition()), instead of the no-attached-disk RAM fallback.

Attach with (see tools/qemu_stage0_boot.py):
  -drive if=none,format=raw,file=<this image>,id=hd0
  -device virtio-blk-device,drive=hd0
"""
from __future__ import annotations

import argparse
import math
import pathlib
import secrets
import struct


SECTOR = 512
PART1_START = 2048           # matches BOOT_FALLBACK_LBA / real hardware convention
PART1_SECTORS = 64 * 1024 * 1024 // SECTOR    # 64 MiB FAT32 boot partition
PART2_SECTORS = 96 * 1024 * 1024 // SECTOR    # 96 MiB raw PIOS system area (> 10 MiB reserved)
SPC = 1
RESERVED = 32
FATS = 2
ROOT_CLUSTER = 2
EOC = 0x0FFFFFFF

FAT_NAME = b"PIOSSTG2PKG"     # 8.3 short name PIOSSTG2.PKG, matches src/bootstrap.c stage2_fat_name


def le16(v: int) -> bytes:
    return struct.pack("<H", v)


def le32(v: int) -> bytes:
    return struct.pack("<I", v)


def short_entry(name: bytes, ext: bytes, attr: int, cluster: int, size: int = 0) -> bytes:
    if len(name) > 8 or len(ext) > 3:
        raise ValueError("short name too long")
    e = bytearray(32)
    e[0:8] = name.ljust(8, b" ")
    e[8:11] = ext.ljust(3, b" ")
    e[11] = attr
    e[20:22] = le16((cluster >> 16) & 0xFFFF)
    e[26:28] = le16(cluster & 0xFFFF)
    e[28:32] = le32(size)
    return bytes(e)


def build_fat32_partition(pkg: bytes, part_sectors: int) -> bytearray:
    """Builds one FAT32 partition image (relative sector 0 == partition start)."""
    fat_sectors = math.ceil((((part_sectors - RESERVED) // SPC) + 2) * 4 / SECTOR)
    while True:
        data_sectors = part_sectors - RESERVED - (FATS * fat_sectors)
        clusters = data_sectors // SPC
        needed = math.ceil((clusters + 2) * 4 / SECTOR)
        if needed <= fat_sectors:
            break
        fat_sectors = needed

    data_start = RESERVED + (FATS * fat_sectors)

    def cluster_lba(cluster: int) -> int:
        return data_start + ((cluster - 2) * SPC)

    part = bytearray(part_sectors * SECTOR)

    bpb = bytearray(SECTOR)
    bpb[0:3] = b"\xEB\x58\x90"
    bpb[3:11] = b"PIOSFAT "
    bpb[11:13] = le16(SECTOR)
    bpb[13] = SPC
    bpb[14:16] = le16(RESERVED)
    bpb[16] = FATS
    bpb[21] = 0xF8
    bpb[24:26] = le16(63)
    bpb[26:28] = le16(255)
    bpb[28:32] = le32(PART1_START)
    bpb[32:36] = le32(part_sectors)
    bpb[36:40] = le32(fat_sectors)
    bpb[44:48] = le32(ROOT_CLUSTER)
    bpb[48:50] = le16(1)
    bpb[50:52] = le16(6)
    bpb[64] = 0x80
    bpb[66] = 0x29
    bpb[67:71] = le32(0x50494F53)
    bpb[71:82] = b"PIOS BOOT  "
    bpb[82:90] = b"FAT32   "
    bpb[510:512] = b"\x55\xAA"
    part[0:SECTOR] = bpb
    part[6 * SECTOR:7 * SECTOR] = bpb

    fsinfo = bytearray(SECTOR)
    fsinfo[0:4] = le32(0x41615252)
    fsinfo[484:488] = le32(0x61417272)
    fsinfo[488:492] = le32(0xFFFFFFFF)
    fsinfo[492:496] = le32(3)
    fsinfo[508:512] = b"\x00\x00\x55\xAA"
    part[1 * SECTOR:2 * SECTOR] = fsinfo

    file_clusters = max(1, math.ceil(len(pkg) / (SPC * SECTOR)))
    file_start = 3
    fat_entries = [0] * (clusters + 2)
    fat_entries[0] = 0x0FFFFFF8
    fat_entries[1] = 0xFFFFFFFF
    fat_entries[2] = EOC  # root dir
    for i in range(file_clusters):
        c = file_start + i
        fat_entries[c] = EOC if i + 1 == file_clusters else c + 1

    fat = bytearray(fat_sectors * SECTOR)
    for i, v in enumerate(fat_entries):
        fat[i * 4:i * 4 + 4] = le32(v)
    for f in range(FATS):
        start = (RESERVED + (f * fat_sectors)) * SECTOR
        part[start:start + len(fat)] = fat

    root = bytearray(SPC * SECTOR)
    root[0:32] = short_entry(FAT_NAME[:8], FAT_NAME[8:11], 0x20, file_start, len(pkg))
    part[cluster_lba(2) * SECTOR:(cluster_lba(2) * SECTOR) + len(root)] = root

    for i in range(file_clusters):
        chunk = pkg[i * SPC * SECTOR:(i + 1) * SPC * SECTOR]
        off = cluster_lba(file_start + i) * SECTOR
        part[off:off + len(chunk)] = chunk

    return part


def build_image(pkg: bytes, out_path: pathlib.Path, disk_id: int) -> None:
    part1 = build_fat32_partition(pkg, PART1_SECTORS)
    part2_start = PART1_START + PART1_SECTORS
    total_sectors = part2_start + PART2_SECTORS

    img = bytearray(total_sectors * SECTOR)

    mbr = bytearray(SECTOR)
    mbr[440:444] = le32(disk_id)
    # Partition 1: FAT32 LBA, entry at 0x1BE.
    mbr[0x1BE + 0] = 0x00
    mbr[0x1BE + 4] = 0x0C          # FAT32 LBA type
    mbr[0x1BE + 8:0x1BE + 12] = le32(PART1_START)
    mbr[0x1BE + 12:0x1BE + 16] = le32(PART1_SECTORS)
    # Partition 2: raw PIOS system area, entry at 0x1CE (type is not gated by
    # discover_partition(), which only logs it -- 0xDA "non-fs data" signals
    # "not a filesystem partition" to any other tooling that might inspect it).
    mbr[0x1CE + 0] = 0x00
    mbr[0x1CE + 4] = 0xDA
    mbr[0x1CE + 8:0x1CE + 12] = le32(part2_start)
    mbr[0x1CE + 12:0x1CE + 16] = le32(PART2_SECTORS)
    mbr[510:512] = b"\x55\xAA"
    img[0:SECTOR] = mbr

    img[PART1_START * SECTOR:(PART1_START + PART1_SECTORS) * SECTOR] = part1
    # Partition 2 stays all-zero -- WALFS formats it on first mount, exactly
    # like a fresh real SD card (src/walfs.c discover_partition_fallback()/
    # walfs_format() path), so no WALFS-specific bytes need to be pre-built
    # here.

    out_path.write_bytes(img)
    print(f"QEMU disk image: {out_path} bytes={len(img)} "
          f"disk_id=0x{disk_id:08x} "
          f"part1_lba={PART1_START} part1_sectors={PART1_SECTORS} "
          f"part2_lba={part2_start} part2_sectors={PART2_SECTORS}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Build a Pi5-shaped MBR+FAT32+raw disk image for QEMU stage0 boot testing.")
    ap.add_argument("--pkg", type=pathlib.Path, required=True,
                     help="PIOSSTG2.PKG-equivalent stage2 package file (e.g. real_kernel_qemu.img)")
    ap.add_argument("--out", type=pathlib.Path, required=True)
    ap.add_argument("--disk-id", type=lambda value: int(value, 0),
                    help="nonzero 32-bit MBR disk identity (default: generated)")
    args = ap.parse_args()
    pkg = args.pkg.read_bytes()
    if not pkg:
        raise SystemExit("package file is empty")
    disk_id = args.disk_id if args.disk_id is not None else secrets.randbits(32)
    disk_id &= 0xFFFFFFFF
    if disk_id == 0:
        disk_id = 1
    build_image(pkg, args.out, disk_id)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
