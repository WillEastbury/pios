#!/usr/bin/env python3
from __future__ import annotations

import argparse
import binascii
import math
import pathlib
import struct


SECTOR = 512
DISK_BYTES = 64 * 1024 * 1024
PART_START = 2048
SPC = 1
RESERVED = 32
FATS = 2
ROOT_CLUSTER = 2
EOC = 0x0FFFFFFF


def le16(v: int) -> bytes:
    return struct.pack("<H", v)


def le32(v: int) -> bytes:
    return struct.pack("<I", v)


def le64(v: int) -> bytes:
    return struct.pack("<Q", v)


def guid_bytes(text: str) -> bytes:
    a, b, c, d, e = text.split("-")
    return struct.pack("<IHH", int(a, 16), int(b, 16), int(c, 16)) + bytes.fromhex(d + e)


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


def dot_entry(name: bytes, cluster: int, parent: int) -> bytes:
    e = bytearray(64)
    e[0:32] = short_entry(name, b"", 0x10, cluster)
    e[32:64] = short_entry(b"..", b"", 0x10, parent)
    return bytes(e)


def build_image(bootx64: bytes, out_path: pathlib.Path) -> None:
    total_sectors = DISK_BYTES // SECTOR
    part_end = total_sectors - 34
    part_sectors = part_end - PART_START + 1
    fat_sectors = math.ceil((((part_sectors - RESERVED) // SPC) + 2) * 4 / SECTOR)
    while True:
        data_sectors = part_sectors - RESERVED - (FATS * fat_sectors)
        clusters = data_sectors // SPC
        needed_fat_sectors = math.ceil((clusters + 2) * 4 / SECTOR)
        if needed_fat_sectors <= fat_sectors:
            break
        fat_sectors = needed_fat_sectors

    data_start = PART_START + RESERVED + (FATS * fat_sectors)

    def cluster_lba(cluster: int) -> int:
        return data_start + ((cluster - 2) * SPC)

    img = bytearray(DISK_BYTES)

    # GPT disk with one EFI System Partition for Hyper-V Gen2 firmware.
    mbr = bytearray(SECTOR)
    mbr[446 + 4] = 0xEE
    mbr[446 + 8:446 + 12] = le32(1)
    mbr[446 + 12:446 + 16] = le32(min(total_sectors - 1, 0xFFFFFFFF))
    mbr[510:512] = b"\x55\xAA"
    img[0:SECTOR] = mbr

    entries = bytearray(128 * 128)
    entry = bytearray(128)
    entry[0:16] = guid_bytes("C12A7328-F81F-11D2-BA4B-00A0C93EC93B")
    entry[16:32] = guid_bytes("50494F53-4856-414D-4436-344553500001")
    entry[32:40] = le64(PART_START)
    entry[40:48] = le64(part_end)
    name = "PIOS Hyper-V ESP".encode("utf-16le")
    entry[56:56 + len(name)] = name
    entries[0:128] = entry
    entries_crc = binascii.crc32(entries) & 0xFFFFFFFF

    def gpt_header(current_lba: int, backup_lba: int, entries_lba: int) -> bytes:
        hdr = bytearray(SECTOR)
        hdr[0:8] = b"EFI PART"
        hdr[8:12] = le32(0x00010000)
        hdr[12:16] = le32(92)
        hdr[24:32] = le64(current_lba)
        hdr[32:40] = le64(backup_lba)
        hdr[40:48] = le64(34)
        hdr[48:56] = le64(part_end)
        hdr[56:72] = guid_bytes("50494F53-4844-4953-4B30-303030000001")
        hdr[72:80] = le64(entries_lba)
        hdr[80:84] = le32(128)
        hdr[84:88] = le32(128)
        hdr[88:92] = le32(entries_crc)
        crc = binascii.crc32(hdr[:92]) & 0xFFFFFFFF
        hdr[16:20] = le32(crc)
        return bytes(hdr)

    img[2 * SECTOR:2 * SECTOR + len(entries)] = entries
    backup_entries_lba = total_sectors - 33
    img[backup_entries_lba * SECTOR:backup_entries_lba * SECTOR + len(entries)] = entries
    img[SECTOR:2 * SECTOR] = gpt_header(1, total_sectors - 1, 2)
    img[(total_sectors - 1) * SECTOR:total_sectors * SECTOR] = gpt_header(
        total_sectors - 1, 1, backup_entries_lba)

    # FAT32 BPB.
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
    bpb[28:32] = le32(PART_START)
    bpb[32:36] = le32(part_sectors)
    bpb[36:40] = le32(fat_sectors)
    bpb[44:48] = le32(ROOT_CLUSTER)
    bpb[48:50] = le16(1)
    bpb[50:52] = le16(6)
    bpb[64] = 0x80
    bpb[66] = 0x29
    bpb[67:71] = le32(0x50494F53)
    bpb[71:82] = b"PIOS HV ESP"
    bpb[82:90] = b"FAT32   "
    bpb[510:512] = b"\x55\xAA"
    img[PART_START * SECTOR:(PART_START + 1) * SECTOR] = bpb
    img[(PART_START + 6) * SECTOR:(PART_START + 7) * SECTOR] = bpb

    fsinfo = bytearray(SECTOR)
    fsinfo[0:4] = le32(0x41615252)
    fsinfo[484:488] = le32(0x61417272)
    fsinfo[488:492] = le32(0xFFFFFFFF)
    fsinfo[492:496] = le32(6)
    fsinfo[508:512] = b"\x00\x00\x55\xAA"
    img[(PART_START + 1) * SECTOR:(PART_START + 2) * SECTOR] = fsinfo

    file_clusters = math.ceil(len(bootx64) / (SPC * SECTOR))
    file_start = 5
    fat_entries = [0] * (clusters + 2)
    fat_entries[0] = 0x0FFFFFF8
    fat_entries[1] = 0xFFFFFFFF
    fat_entries[2] = EOC  # root
    fat_entries[3] = EOC  # EFI
    fat_entries[4] = EOC  # BOOT
    for i in range(file_clusters):
        c = file_start + i
        fat_entries[c] = EOC if i + 1 == file_clusters else c + 1

    fat = bytearray(fat_sectors * SECTOR)
    for i, v in enumerate(fat_entries):
        fat[i * 4:i * 4 + 4] = le32(v)
    for f in range(FATS):
        start = (PART_START + RESERVED + (f * fat_sectors)) * SECTOR
        img[start:start + len(fat)] = fat

    root = bytearray(SPC * SECTOR)
    root[0:32] = short_entry(b"EFI", b"", 0x10, 3)
    img[cluster_lba(2) * SECTOR:(cluster_lba(2) * SECTOR) + len(root)] = root

    efi_dir = bytearray(SPC * SECTOR)
    efi_dir[0:64] = dot_entry(b".", 3, 2)
    efi_dir[64:96] = short_entry(b"BOOT", b"", 0x10, 4)
    img[cluster_lba(3) * SECTOR:(cluster_lba(3) * SECTOR) + len(efi_dir)] = efi_dir

    boot_dir = bytearray(SPC * SECTOR)
    boot_dir[0:64] = dot_entry(b".", 4, 3)
    boot_dir[64:96] = short_entry(b"BOOTX64", b"EFI", 0x20, file_start, len(bootx64))
    img[cluster_lba(4) * SECTOR:(cluster_lba(4) * SECTOR) + len(boot_dir)] = boot_dir

    for i in range(file_clusters):
        chunk = bootx64[i * SPC * SECTOR:(i + 1) * SPC * SECTOR]
        off = cluster_lba(file_start + i) * SECTOR
        img[off:off + len(chunk)] = chunk

    out_path.write_bytes(img)
    print(f"ESP image: {out_path} bytes={len(img)} fat_sectors={fat_sectors} clusters={clusters}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Build a bootable FAT32 ESP disk image for Hyper-V/QEMU.")
    ap.add_argument("--bootx64", type=pathlib.Path, required=True)
    ap.add_argument("--out", type=pathlib.Path, required=True)
    args = ap.parse_args()
    build_image(args.bootx64.read_bytes(), args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
