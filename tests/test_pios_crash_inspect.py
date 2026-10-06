"""Read-only crash-sector and A/B image-corruption diagnostic regression."""
from io import BytesIO
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from run_host_tests import find_clang

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
import pios_crash_inspect as tool

root = 2048 * tool.SECTOR
disk = bytearray(root + tool.SLOT_OFFSETS[1] + tool.SLOT_BYTES)
disk[510:512] = b"\x55\xaa"
disk[466] = 0xDA
struct.pack_into("<II", disk, 470, 2048, (len(disk) - root) // tool.SECTOR)
payload = bytearray(1024)
struct.pack_into("<I", payload, 128, 0xA9BE7BFD)
for offset in tool.SLOT_OFFSETS:
    struct.pack_into("<6I", disk, root + offset, tool.SLOT_MAGIC, len(payload), 1,
                     10 * 1024 * 1024, tool.SECTOR, tool.SLOT_BYTES - tool.SECTOR)
    disk[root + offset + tool.SECTOR:root + offset + tool.SECTOR + len(payload)] = payload
# Persist the PC, ESR, SP and TTBR fields using the production packed v3 layout.
crash = bytearray(tool.SECTOR)
struct.pack_into("<14I7Q2I8Q48s2I", crash, 0,
                 tool.CRASH_MAGIC, 3, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 7,
                 0, tool.IMAGE_BASE + 128, 0, 0xCE7C90, 0x859000, 0, 500,
                 0, 0, *([0] * 8), b"", 3, 0)
disk[tool.CRASH_LBA * tool.SECTOR:(tool.CRASH_LBA + 1) * tool.SECTOR] = crash
control = bytearray(tool.SECTOR)
struct.pack_into("<8I", control, 0, 0x50424330, 2, 0, 0xFFFFFFFF, 0, 0, 3, 9)
checksum = 0xB007C0DE
for byte in control[:48]:
    checksum = ((checksum << 5) ^ (checksum >> 27) ^ byte) & 0xFFFFFFFF
struct.pack_into("<I", control, 48, checksum)
disk[root + tool.CONTROL_OFFSET:root + tool.CONTROL_OFFSET + tool.SECTOR] = control
original = bytes(disk)
report, saved = tool.inspect(BytesIO(disk), payload)
assert saved == crash
assert report["crash"]["pc"] == tool.IMAGE_BASE + 128
assert report["crash"]["sp"] == 0xCE7C90 and report["crash"]["consecutive"] == 3
assert report["boot_control"]["valid"] and report["boot_control"]["last_boot"] == 0
assert all(s["matches_expected"] for s in report["slots"])
assert report["expected_pc_word"] == "0xa9be7bfd"
assert bytes(disk) == original

struct.pack_into("<I", disk, root + tool.SECTOR + 128, 1)
report, _ = tool.inspect(BytesIO(disk), payload)
assert report["slots"][0]["pc_word"] == "0x00000001"
assert not report["slots"][0]["matches_expected"]
assert report["slots"][1]["matches_expected"]
assert report["slots"][0]["first_differences"][0]["offset"] == 128

def rejects(function, *args):
    try:
        function(*args)
    except ValueError:
        return
    raise AssertionError("malformed diagnostic input accepted")

rejects(tool.inspect, BytesIO(disk), b"")
rejects(tool.inspect, BytesIO(disk), payload, tool.IMAGE_BASE - 4)
rejects(tool.inspect, BytesIO(disk), payload, tool.IMAGE_BASE + 1)
rejects(tool.inspect, BytesIO(disk[:root + 50]), payload)
bad = bytearray(crash)
struct.pack_into("<I", bad, 4, 99)
rejects(tool.crash_record, bad)
struct.pack_into("<I", bad, 4, 3)
struct.pack_into("<I", bad, 116, 9)
rejects(tool.crash_record, bad)
rejects(tool.crash_record, crash[:32])
control[48] ^= 1
assert not tool.boot_control(control)["valid"]
struct.pack_into("<II", disk, 470, 0xFFFFFFFF, 0xFFFF)
rejects(tool.inspect, BytesIO(disk), payload)
assert struct.calcsize("<14I7Q2I8Q48s2I") == tool.CRASH_SIZE
with tempfile.TemporaryDirectory(prefix="pios-crash-layout-") as temporary:
    source = Path(temporary) / "layout.c"
    source.write_text("""
#include "exception.h"
#include "walfs.h"
#define OFFSET(field, value) _Static_assert(__builtin_offsetof(struct exception_crash_record, field) == value, #field)
_Static_assert(EXCEPTION_CRASH_RECORD_VERSION == 3, "version");
_Static_assert(sizeof(struct exception_crash_record) == 240, "record size");
OFFSET(esr,56); OFFSET(elr,64); OFFSET(sp,80); OFFSET(ttbr0,88);
OFFSET(reason,112); OFFSET(value_count,116); OFFSET(values,120);
OFFSET(label,184); OFFSET(consecutive,232); OFFSET(archived,236);
_Static_assert(PIOS_BOOT_SLOT_BYTES == 0x380000, "slot size");
_Static_assert(PIOS_BOOT_SLOT_A_OFFSET == 0, "slot A");
_Static_assert(PIOS_BOOT_SLOT_B_OFFSET == 0x400000, "slot B");
_Static_assert(PIOS_BOOTCTRL_OFFSET == 0x380000, "boot-control");
_Static_assert(PIOS_STAGE2_OFFSET == 512, "payload start");
""", encoding="ascii")
    subprocess.run([find_clang(), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    "-fsyntax-only", "-include", str(ROOT / "tests" / "stubinc" / "types.h"),
                    "-I", str(ROOT / "include"), str(source)], check=True)
print("PIOS crash inspector: persisted frame, slot comparison, corruption and read-only bounds passed")
