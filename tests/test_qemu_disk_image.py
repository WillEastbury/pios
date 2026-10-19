#!/usr/bin/env python3
"""Static test for the optional shared-layout p3 PIOSXFER disk fixture."""

from __future__ import annotations

import importlib.util
import pathlib
import struct


ROOT = pathlib.Path(__file__).resolve().parent.parent
PATH = ROOT / "tools" / "build_qemu_disk_image.py"
spec = importlib.util.spec_from_file_location("qemu_disk_builder", PATH)
assert spec and spec.loader
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


def le16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def le32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def check_fat32(part: bytes, start: int, label: bytes) -> None:
    assert len(part) == builder.PART3_SECTORS * builder.SECTOR
    assert part[3:11] == b"PIOSXFER"
    assert le16(part, 11) == 512
    assert part[13] == 1 and le16(part, 14) == builder.RESERVED
    assert part[16] == 2 and le32(part, 28) == start
    assert le32(part, 32) == builder.PART3_SECTORS
    assert part[71:82] == label and part[82:90] == b"FAT32   "
    assert part[510:512] == b"\x55\xAA"
    assert part[6 * 512:7 * 512] == part[:512]
    assert part[512:1024] == part[7 * 512:8 * 512]
    fats = le32(part, 36)
    fat1 = builder.RESERVED * 512
    fat2 = (builder.RESERVED + fats) * 512
    assert part[fat1:fat1 + fats * 512] == part[fat2:fat2 + fats * 512]


part3_start = builder.PART1_START + builder.PART1_SECTORS + builder.PART2_SECTORS
part3 = builder.build_fat32_partition(
    b"", builder.PART3_SECTORS, part3_start, b"PIOSXFER   ", b"PIOSXFER"
)
check_fat32(part3, part3_start, b"PIOSXFER   ")

# The same MBR helper used by build_image() must leave the historical default
# layout at two entries and add exactly p3 after raw WALFS p2 when requested.
plain_mbr = builder.build_mbr(0x12345678, False)
xfer_mbr = builder.build_mbr(0x12345678, True)
assert plain_mbr[0x1DE:0x1EE] == b"\0" * 16
assert le32(xfer_mbr, 0x1DE + 8) == part3_start
assert le32(xfer_mbr, 0x1DE + 12) == builder.PART3_SECTORS
assert xfer_mbr[0x1DE + 4] == 0x0C and xfer_mbr[510:512] == b"\x55\xAA"
assert builder.PART1_START == 2048
assert part3_start == builder.PART1_START + builder.PART1_SECTORS + builder.PART2_SECTORS
assert builder.PART3_SECTORS >= 65525
print("qemu disk image optional PIOSXFER layout: PASS")

payload = b"stage2" * 100
config = b"# boot fixture\n" * 50
part1 = builder.build_fat32_partition(
    payload, builder.PART1_SECTORS, builder.PART1_START, b"PIOS BOOT  ",
    boot_config=config,
)
data_start = builder.RESERVED + builder.FATS * le32(part1, 36)
root_offset = data_start * builder.SECTOR
assert part1[root_offset:root_offset + 11] == b"PIOSSTG2PKG"
entry = part1[root_offset + 32:root_offset + 64]
assert entry[:11] == b"PIOS    INI" and le32(entry, 28) == len(config)
first = le16(entry, 26) | (le16(entry, 20) << 16)
assert first == 5  # stage2 occupies clusters 3 and 4
offset = (data_start + first - 2) * builder.SECTOR
assert part1[offset:offset + len(config)] == config
fat_start = builder.RESERVED * builder.SECTOR
assert le32(part1, fat_start + first * 4) == first + 1
assert le32(part1, fat_start + (first + 1) * 4) == builder.EOC
assert part1[(data_start + 1) * builder.SECTOR:
             (data_start + 1) * builder.SECTOR + len(payload)] == payload
for invalid_config in (b"", b"x" * 4097):
    try:
        builder.build_fat32_partition(
            payload, builder.PART1_SECTORS, builder.PART1_START, b"PIOS BOOT  ",
            boot_config=invalid_config,
        )
    except ValueError:
        pass
    else:
        raise AssertionError("invalid bootstrap file size accepted")
print("qemu disk image FAT-root pios.ini: PASS")
