#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import pathlib
import struct


ISO_SECTOR = 2048
FAT_SECTOR = 512
EFISYS_BYTES = 16 * 1024 * 1024
SPC = 1
RESERVED = 1
FATS = 2
ROOT_CLUSTER = 2
EOC = 0x0FFFFFFF


def le16(v: int) -> bytes:
    return struct.pack("<H", v)


def be16(v: int) -> bytes:
    return struct.pack(">H", v)


def le32(v: int) -> bytes:
    return struct.pack("<I", v)


def be32(v: int) -> bytes:
    return struct.pack(">I", v)


def both16(v: int) -> bytes:
    return le16(v) + be16(v)


def both32(v: int) -> bytes:
    return le32(v) + be32(v)


def short_entry(name: bytes, ext: bytes, attr: int, cluster: int, size: int = 0) -> bytes:
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


def build_fat16_efisys(bootx64: bytes) -> bytes:
    total_sectors = EFISYS_BYTES // FAT_SECTOR
    root_entries = 512
    root_dir_sectors = (root_entries * 32 + FAT_SECTOR - 1) // FAT_SECTOR
    fat_sectors = math.ceil((((total_sectors - RESERVED - root_dir_sectors) // SPC) + 2) * 2 / FAT_SECTOR)
    while True:
        data_sectors = total_sectors - RESERVED - (FATS * fat_sectors) - root_dir_sectors
        clusters = data_sectors // SPC
        needed_fat_sectors = math.ceil((clusters + 2) * 2 / FAT_SECTOR)
        if needed_fat_sectors <= fat_sectors:
            break
        fat_sectors = needed_fat_sectors

    data_start = RESERVED + (FATS * fat_sectors)

    def cluster_lba(cluster: int) -> int:
        return data_start + root_dir_sectors + ((cluster - 2) * SPC)

    img = bytearray(EFISYS_BYTES)
    bpb = bytearray(FAT_SECTOR)
    bpb[0:3] = b"\xEB\x3C\x90"
    bpb[3:11] = b"PIOSFAT "
    bpb[11:13] = le16(FAT_SECTOR)
    bpb[13] = SPC
    bpb[14:16] = le16(RESERVED)
    bpb[16] = FATS
    bpb[17:19] = le16(root_entries)
    bpb[19:21] = le16(total_sectors)
    bpb[21] = 0xF8
    bpb[22:24] = le16(fat_sectors)
    bpb[24:26] = le16(63)
    bpb[26:28] = le16(255)
    bpb[36] = 0x80
    bpb[38] = 0x29
    bpb[39:43] = le32(0x50494F53)
    bpb[43:54] = b"PIOS HV ESP"
    bpb[54:62] = b"FAT16   "
    bpb[510:512] = b"\x55\xAA"
    img[0:FAT_SECTOR] = bpb

    file_clusters = math.ceil(len(bootx64) / (SPC * FAT_SECTOR))
    file_start = 5
    fat_entries = [0] * (clusters + 2)
    fat_entries[0] = 0xFFF8
    fat_entries[1] = 0xFFFFFFFF
    fat_entries[2] = 0xFFFF
    fat_entries[3] = 0xFFFF
    fat_entries[4] = 0xFFFF
    for i in range(file_clusters):
        c = file_start + i
        fat_entries[c] = 0xFFFF if i + 1 == file_clusters else c + 1

    fat = bytearray(fat_sectors * FAT_SECTOR)
    for i, v in enumerate(fat_entries):
        fat[i * 2:i * 2 + 2] = le16(v & 0xFFFF)
    for f in range(FATS):
        start = (RESERVED + (f * fat_sectors)) * FAT_SECTOR
        img[start:start + len(fat)] = fat

    root = bytearray(root_dir_sectors * FAT_SECTOR)
    root[0:32] = short_entry(b"EFI", b"", 0x10, 3)
    img[data_start * FAT_SECTOR:(data_start * FAT_SECTOR) + len(root)] = root

    efi_dir = bytearray(SPC * FAT_SECTOR)
    efi_dir[0:64] = dot_entry(b".", 3, 2)
    efi_dir[64:96] = short_entry(b"BOOT", b"", 0x10, 4)
    img[cluster_lba(3) * FAT_SECTOR:(cluster_lba(3) + SPC) * FAT_SECTOR] = efi_dir

    boot_dir = bytearray(SPC * FAT_SECTOR)
    boot_dir[0:64] = dot_entry(b".", 4, 3)
    boot_dir[64:96] = short_entry(b"BOOTX64", b"EFI", 0x20, file_start, len(bootx64))
    img[cluster_lba(4) * FAT_SECTOR:(cluster_lba(4) + SPC) * FAT_SECTOR] = boot_dir

    for i in range(file_clusters):
        chunk = bootx64[i * SPC * FAT_SECTOR:(i + 1) * SPC * FAT_SECTOR]
        off = cluster_lba(file_start + i) * FAT_SECTOR
        img[off:off + len(chunk)] = chunk
    return bytes(img)


def dir_record(lba: int, size: int, flags: int, ident: bytes) -> bytes:
    rec = bytearray()
    rec.append(0)
    rec.append(0)
    rec += both32(lba)
    rec += both32(size)
    rec += bytes([126, 1, 1, 0, 0, 0, 0])
    rec.append(flags)
    rec.append(0)
    rec.append(0)
    rec += both16(1)
    rec.append(len(ident))
    rec += ident
    if len(rec) & 1:
        rec.append(0)
    rec[0] = len(rec)
    return bytes(rec)


def path_table_le(root_lba: int) -> bytes:
    return bytes([1, 0]) + le32(root_lba) + le16(1) + b"\x00\x00"


def path_table_be(root_lba: int) -> bytes:
    return bytes([1, 0]) + be32(root_lba) + be16(1) + b"\x00\x00"


def build_catalog(boot_lba: int, boot_sectors_512: int) -> bytes:
    cat = bytearray(ISO_SECTOR)
    cat[0] = 1
    cat[1] = 0xEF
    ident = b"PIOS HYPERV AMD64"
    cat[4:4 + len(ident)] = ident
    cat[30] = 0x55
    cat[31] = 0xAA
    total = 0
    for i in range(16):
        total = (total + int.from_bytes(cat[i * 2:i * 2 + 2], "little")) & 0xFFFF
    checksum = (-total) & 0xFFFF
    cat[28:30] = le16(checksum)

    off = 32
    cat[off + 0] = 0x88
    cat[off + 1] = 0
    cat[off + 2:off + 4] = le16(0)
    cat[off + 4] = 0
    cat[off + 6:off + 8] = le16(min(boot_sectors_512, 0xFFFF))
    cat[off + 8:off + 12] = le32(boot_lba)
    return bytes(cat)


def build_iso(efisys: bytes, label: str, out_path: pathlib.Path) -> None:
    root_lba = 19
    path_le_lba = 20
    path_be_lba = 21
    catalog_lba = 22
    boot_lba = 23
    boot_iso_sectors = math.ceil(len(efisys) / ISO_SECTOR)
    total_sectors = boot_lba + boot_iso_sectors

    iso = bytearray(total_sectors * ISO_SECTOR)
    root_size = ISO_SECTOR
    root_rec = dir_record(root_lba, root_size, 2, b"\x00")

    pvd = bytearray(ISO_SECTOR)
    pvd[0] = 1
    pvd[1:6] = b"CD001"
    pvd[6] = 1
    pvd[8:40] = b"PIOS".ljust(32, b" ")
    pvd[40:72] = label.encode("ascii")[:32].ljust(32, b" ")
    pvd[80:88] = both32(total_sectors)
    pvd[120:124] = both16(1)
    pvd[124:128] = both16(1)
    pvd[128:132] = both16(ISO_SECTOR)
    ptable_le = path_table_le(root_lba)
    ptable_be = path_table_be(root_lba)
    pvd[132:140] = both32(len(ptable_le))
    pvd[140:144] = le32(path_le_lba)
    pvd[148:152] = be32(path_be_lba)
    pvd[156:156 + len(root_rec)] = root_rec
    pvd[881] = 1
    iso[16 * ISO_SECTOR:17 * ISO_SECTOR] = pvd

    boot = bytearray(ISO_SECTOR)
    boot[0] = 0
    boot[1:6] = b"CD001"
    boot[6] = 1
    boot[7:39] = b"EL TORITO SPECIFICATION".ljust(32, b"\0")
    boot[71:75] = le32(catalog_lba)
    iso[17 * ISO_SECTOR:18 * ISO_SECTOR] = boot

    term = bytearray(ISO_SECTOR)
    term[0] = 255
    term[1:6] = b"CD001"
    term[6] = 1
    iso[18 * ISO_SECTOR:19 * ISO_SECTOR] = term

    root = bytearray(ISO_SECTOR)
    pos = 0
    for rec in (
        dir_record(root_lba, root_size, 2, b"\x00"),
        dir_record(root_lba, root_size, 2, b"\x01"),
        dir_record(catalog_lba, ISO_SECTOR, 0, b"BOOT.CAT;1"),
        dir_record(boot_lba, len(efisys), 0, b"EFISYS.IMG;1"),
    ):
        root[pos:pos + len(rec)] = rec
        pos += len(rec)
    iso[root_lba * ISO_SECTOR:(root_lba + 1) * ISO_SECTOR] = root
    iso[path_le_lba * ISO_SECTOR:path_le_lba * ISO_SECTOR + len(ptable_le)] = ptable_le
    iso[path_be_lba * ISO_SECTOR:path_be_lba * ISO_SECTOR + len(ptable_be)] = ptable_be
    iso[catalog_lba * ISO_SECTOR:(catalog_lba + 1) * ISO_SECTOR] = build_catalog(
        boot_lba, math.ceil(len(efisys) / FAT_SECTOR))
    iso[boot_lba * ISO_SECTOR:boot_lba * ISO_SECTOR + len(efisys)] = efisys
    out_path.write_bytes(iso)
    print(f"UEFI ISO: {out_path} bytes={len(iso)} boot_lba={boot_lba} efisys={len(efisys)}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Build a UEFI El Torito boot ISO with /EFI/BOOT/BOOTX64.EFI.")
    ap.add_argument("--bootx64", type=pathlib.Path, required=True)
    ap.add_argument("--out", type=pathlib.Path, required=True)
    ap.add_argument("--label", default="PIOS_HV_AMD64")
    args = ap.parse_args()
    build_iso(build_fat16_efisys(args.bootx64.read_bytes()), args.label, args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
