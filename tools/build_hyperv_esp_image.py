"""Build the mandatory three-partition PIOS x86 USB image.

GPT index 0 is the UEFI FAT32 ESP, index 1 is raw PIOS/WALFS storage, and
index 2 is a host-mountable FAT32 PIOSXFER volume. The builder never formats
or writes a real removable device.
"""
from __future__ import annotations

import argparse
import binascii
import math
import pathlib
import struct

SECTOR = 512
DISK_BYTES = 260 * 1024 * 1024
ESP_BYTES = 64 * 1024 * 1024
WALFS_BYTES = 128 * 1024 * 1024
XFER_BYTES = 64 * 1024 * 1024
PART_START = 2048
SPC = 1
RESERVED = 32
FATS = 2
ROOT_CLUSTER = 2
EOC = 0x0FFFFFFF
ESP_GUID = "C12A7328-F81F-11D2-BA4B-00A0C93EC93B"
PIOS_WALFS_GUID = "7A8E5F54-3C74-4F7A-8A64-50494F535953"


def le16(value: int) -> bytes:
    return struct.pack("<H", value)


def le32(value: int) -> bytes:
    return struct.pack("<I", value)


def le64(value: int) -> bytes:
    return struct.pack("<Q", value)


def guid_bytes(text: str) -> bytes:
    a, b, c, d, e = text.split("-")
    return struct.pack("<IHH", int(a, 16), int(b, 16), int(c, 16)) + bytes.fromhex(d + e)


def short_entry(name: bytes, ext: bytes, attr: int, cluster: int, size: int = 0) -> bytes:
    if len(name) > 8 or len(ext) > 3:
        raise ValueError("short name too long")
    result = bytearray(32)
    result[0:8] = name.ljust(8, b" ")
    result[8:11] = ext.ljust(3, b" ")
    result[11] = attr
    result[20:22] = le16(cluster >> 16)
    result[26:28] = le16(cluster)
    result[28:32] = le32(size)
    return bytes(result)


def write_at(file, offset: int, data: bytes) -> None:
    if offset < 0 or offset + len(data) > DISK_BYTES:
        raise ValueError("write outside image")
    file.seek(offset)
    file.write(data)


def fat32_partition(part_sectors: int, start_lba: int, label: bytes,
                    files: dict[tuple[bytes, bytes], bytes]) -> bytes:
    if len(label) != 11 or not files or len(files) > 13:
        raise ValueError("invalid FAT32 image shape")
    fat_sectors = math.ceil((((part_sectors - RESERVED) // SPC) + 2) * 4 / SECTOR)
    while True:
        data_sectors = part_sectors - RESERVED - FATS * fat_sectors
        clusters = data_sectors // SPC
        needed = math.ceil((clusters + 2) * 4 / SECTOR)
        if needed <= fat_sectors:
            break
        fat_sectors = needed
    if clusters < 65525:
        raise ValueError("FAT32 partition is too small")
    result = bytearray(part_sectors * SECTOR)

    def cluster_offset(cluster: int) -> int:
        return (RESERVED + FATS * fat_sectors + (cluster - 2) * SPC) * SECTOR

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
    bpb[28:32] = le32(start_lba)
    bpb[32:36] = le32(part_sectors)
    bpb[36:40] = le32(fat_sectors)
    bpb[44:48] = le32(ROOT_CLUSTER)
    bpb[48:50] = le16(1)
    bpb[50:52] = le16(6)
    bpb[64] = 0x80
    bpb[66] = 0x29
    bpb[67:71] = le32(0x50494F53)
    bpb[71:82] = label
    bpb[82:90] = b"FAT32   "
    bpb[510:512] = b"\x55\xAA"
    result[:SECTOR] = bpb
    result[6 * SECTOR:7 * SECTOR] = bpb
    fsinfo = bytearray(SECTOR)
    fsinfo[0:4] = le32(0x41615252)
    fsinfo[484:488] = le32(0x61417272)
    fsinfo[488:492] = le32(0xFFFFFFFF)
    fsinfo[492:496] = le32(3)
    fsinfo[508:512] = b"\x00\x00\x55\xAA"
    result[SECTOR:2 * SECTOR] = fsinfo
    result[7 * SECTOR:8 * SECTOR] = fsinfo

    next_cluster = 5
    allocations: dict[tuple[bytes, bytes], tuple[int, int]] = {}
    for key, payload in files.items():
        count = max(1, math.ceil(len(payload) / SECTOR))
        allocations[key] = (next_cluster, count)
        next_cluster += count
    if next_cluster > clusters + 2:
        raise ValueError("FAT32 files exceed capacity")
    fat = [0] * (clusters + 2)
    fat[0], fat[1], fat[2], fat[3], fat[4] = 0x0FFFFFF8, EOC, EOC, EOC, EOC
    for start, count in allocations.values():
        for index in range(count):
            fat[start + index] = EOC if index + 1 == count else start + index + 1
    fat_bytes = bytearray(fat_sectors * SECTOR)
    for index, value in enumerate(fat):
        fat_bytes[index * 4:index * 4 + 4] = le32(value)
    for copy in range(FATS):
        offset = (RESERVED + copy * fat_sectors) * SECTOR
        result[offset:offset + len(fat_bytes)] = fat_bytes

    root = bytearray(SECTOR)
    root[:32] = short_entry(b"EFI", b"", 0x10, 3)
    result[cluster_offset(2):cluster_offset(2) + SECTOR] = root
    efi = bytearray(SECTOR)
    efi[:32] = short_entry(b".", b"", 0x10, 3)
    efi[32:64] = short_entry(b"..", b"", 0x10, 2)
    efi[64:96] = short_entry(b"BOOT", b"", 0x10, 4)
    result[cluster_offset(3):cluster_offset(3) + SECTOR] = efi
    boot = bytearray(SECTOR)
    boot[:32] = short_entry(b".", b"", 0x10, 4)
    boot[32:64] = short_entry(b"..", b"", 0x10, 3)
    for index, ((name, ext), payload) in enumerate(files.items()):
        cluster, _ = allocations[(name, ext)]
        boot[64 + index * 32:96 + index * 32] = short_entry(name, ext, 0x20, cluster, len(payload))
        result[cluster_offset(cluster):cluster_offset(cluster) + len(payload)] = payload
    result[cluster_offset(4):cluster_offset(4) + SECTOR] = boot
    return bytes(result)


def build_image(bootx64: bytes, out_path: pathlib.Path) -> None:
    total = DISK_BYTES // SECTOR
    sizes = [ESP_BYTES // SECTOR, WALFS_BYTES // SECTOR, XFER_BYTES // SECTOR]
    starts = [PART_START, PART_START + sizes[0], PART_START + sizes[0] + sizes[1]]
    if starts[2] + sizes[2] > total - 34:
        raise ValueError("three partition layout exceeds GPT usable range")
    entries = bytearray(128 * 128)

    def entry(index: int, kind: str, unique: str, name: str) -> None:
        item = bytearray(128)
        item[:16] = guid_bytes(kind)
        item[16:32] = guid_bytes(unique)
        item[32:40] = le64(starts[index])
        item[40:48] = le64(starts[index] + sizes[index] - 1)
        encoded = name.encode("utf-16le")
        item[56:56 + len(encoded)] = encoded
        entries[index * 128:(index + 1) * 128] = item

    entry(0, ESP_GUID, "50494F53-5553-4245-5350-303030000001", "PIOS USB ESP")
    entry(1, PIOS_WALFS_GUID, "50494F53-5553-4257-414C-303030000001", "PIOS WALFS")
    entry(2, ESP_GUID, "50494F53-5553-4258-4645-303030000001", "PIOSXFER")
    entries_crc = binascii.crc32(entries) & 0xFFFFFFFF

    def header(current: int, backup: int, entries_lba: int) -> bytes:
        value = bytearray(SECTOR)
        value[:8] = b"EFI PART"
        value[8:12] = le32(0x00010000)
        value[12:16] = le32(92)
        value[24:32] = le64(current)
        value[32:40] = le64(backup)
        value[40:48] = le64(34)
        value[48:56] = le64(total - 34)
        value[56:72] = guid_bytes("50494F53-5553-4244-4953-4B3030000001")
        value[72:80] = le64(entries_lba)
        value[80:84], value[84:88], value[88:92] = le32(128), le32(128), le32(entries_crc)
        value[16:20] = le32(binascii.crc32(value[:92]) & 0xFFFFFFFF)
        return bytes(value)

    esp = fat32_partition(sizes[0], starts[0], b"PIOSBOOT   ",
                          {(b"BOOTX64", b"EFI"): bootx64})
    xfer = fat32_partition(sizes[2], starts[2], b"PIOSXFER   ",
                           {(b"README", b"TXT"): b"PIOS exchange volume\r\n"})
    with out_path.open("wb") as image:
        image.truncate(DISK_BYTES)
        mbr = bytearray(SECTOR)
        mbr[446 + 4] = 0xEE
        mbr[446 + 8:446 + 12] = le32(1)
        mbr[446 + 12:446 + 16] = le32(total - 1)
        mbr[510:512] = b"\x55\xAA"
        write_at(image, 0, mbr)
        write_at(image, SECTOR, header(1, total - 1, 2))
        backup_entries = total - 33
        write_at(image, 2 * SECTOR, entries)
        write_at(image, backup_entries * SECTOR, entries)
        write_at(image, (total - 1) * SECTOR, header(total - 1, 1, backup_entries))
        write_at(image, starts[0] * SECTOR, esp)
        write_at(image, starts[2] * SECTOR, xfer)
    print(f"USB image: {out_path} bytes={DISK_BYTES} "
          f"ESP={starts[0]}+{sizes[0]} WALFS={starts[1]}+{sizes[1]} "
          f"PIOSXFER={starts[2]}+{sizes[2]}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bootx64", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args()
    build_image(args.bootx64.read_bytes(), args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
