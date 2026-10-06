"""Validate mandatory GPT0/1/2 USB storage layout for native x86 PIOS."""
from __future__ import annotations

import importlib.util
import pathlib
import struct
import tempfile
import zlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
PATH = ROOT / "tools" / "build_hyperv_esp_image.py"
spec = importlib.util.spec_from_file_location("hyperv_usb_builder", PATH)
assert spec and spec.loader
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


def le32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def le64(data: bytes, offset: int) -> int:
    return struct.unpack_from("<Q", data, offset)[0]


def gpt_header_valid(data: bytes, offset: int, expected_current: int) -> None:
    header = bytearray(data[offset:offset + builder.SECTOR])
    assert header[:8] == b"EFI PART"
    assert le32(header, 12) == 92
    assert le64(header, 24) == expected_current
    stored = le32(header, 16)
    header[16:20] = bytes(4)
    assert zlib.crc32(header[:92]) & 0xFFFFFFFF == stored


def gpt_entry(entries: bytes, index: int) -> tuple[bytes, int, int, str]:
    entry = entries[index * 128:(index + 1) * 128]
    return entry[:16], le64(entry, 32), le64(entry, 40), entry[56:128].decode("utf-16le").rstrip("\0")


with tempfile.TemporaryDirectory(prefix="pios-x86-usb-") as directory:
    output = pathlib.Path(directory) / "usb.raw"
    boot = b"PIOS X64 UEFI PROBE" * 50
    builder.build_image(boot, output)
    data = output.read_bytes()

assert len(data) == builder.DISK_BYTES
assert data[510:512] == b"\x55\xAA" and data[446 + 4] == 0xEE
gpt_header_valid(data, builder.SECTOR, 1)
total = builder.DISK_BYTES // builder.SECTOR
gpt_header_valid(data, (total - 1) * builder.SECTOR, total - 1)
entries = data[2 * builder.SECTOR:2 * builder.SECTOR + 128 * 128]
assert zlib.crc32(entries) & 0xFFFFFFFF == le32(data, builder.SECTOR + 88)
esp_type, esp_start, esp_end, esp_name = gpt_entry(entries, 0)
walfs_type, walfs_start, walfs_end, walfs_name = gpt_entry(entries, 1)
xfer_type, xfer_start, xfer_end, xfer_name = gpt_entry(entries, 2)
assert esp_type == builder.guid_bytes(builder.ESP_GUID)
assert walfs_type == builder.guid_bytes(builder.PIOS_WALFS_GUID)
assert xfer_type == builder.guid_bytes(builder.ESP_GUID)
assert (esp_name, walfs_name, xfer_name) == ("PIOS USB ESP", "PIOS WALFS", "PIOSXFER")
assert esp_start == builder.PART_START
assert esp_end + 1 == walfs_start and walfs_end + 1 == xfer_start
assert xfer_end < total - 34
assert data[walfs_start * builder.SECTOR:(walfs_start + 1) * builder.SECTOR] == bytes(builder.SECTOR)
for start, expected_label, expected_file in (
    (esp_start, b"PIOSBOOT   ", b"BOOTX64 EFI"),
    (xfer_start, b"PIOSXFER   ", b"README  TXT"),
):
    boot_sector = data[start * builder.SECTOR:(start + 1) * builder.SECTOR]
    assert boot_sector[71:82] == expected_label and boot_sector[82:90] == b"FAT32   "
    fats = le32(boot_sector, 36)
    assert data[(start + builder.RESERVED) * builder.SECTOR:
                (start + builder.RESERVED + fats) * builder.SECTOR] == \
           data[(start + builder.RESERVED + fats) * builder.SECTOR:
                (start + builder.RESERVED + fats * 2) * builder.SECTOR]
    data_start = start + builder.RESERVED + fats * builder.FATS
    boot_dir = data[(data_start + 2) * builder.SECTOR:(data_start + 3) * builder.SECTOR]
    assert expected_file in boot_dir
print("x86 USB image: mandatory ESP/WALFS/PIOSXFER GPT layout PASS")
