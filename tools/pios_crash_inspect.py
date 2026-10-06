#!/usr/bin/env python3
"""Read a PIOS SD/image without writing it; decode LBA24 and compare raw slots."""
import argparse
import hashlib
from itertools import islice
import json
from pathlib import Path
import struct

SECTOR = 512
CRASH_LBA = 24
CRASH_MAGIC = 0x43524153
CRASH_SIZE = 240
SLOT_MAGIC = 0x50494F53
SLOT_BYTES = 0x380000
SLOT_OFFSETS = (0, 0x400000)
CONTROL_OFFSET = 0x380000
IMAGE_BASE = 0x80000
MAX_LBA = 1 << 32


def read_at(source, offset, length):
    if offset < 0 or length <= 0 or length > SLOT_BYTES:
        raise ValueError("invalid bounded read")
    source.seek(offset)
    data = source.read(length)
    if len(data) != length:
        raise ValueError(f"short read at byte {offset:#x}: {len(data)}/{length}")
    return data


def crash_record(data):
    if len(data) != SECTOR:
        raise ValueError("crash sector must be exactly 512 bytes")
    magic, version = struct.unpack_from("<II", data)
    if magic != CRASH_MAGIC:
        return {"present": False, "magic": f"0x{magic:08x}"}
    if version != 3:
        raise ValueError(f"unsupported persisted crash version {version}")
    names = ("magic", "version", "kind", "core", "el", "ec", "pid", "capsule",
             "process_generation", "owner_principal", "descriptor_id",
             "descriptor_generation", "descriptor_owner", "last_fifo_seq")
    result = dict(zip(names, struct.unpack_from("<14I", data)))
    for i, name in enumerate(("esr", "pc", "far", "sp", "ttbr0", "syndrome", "ticks")):
        result[name] = struct.unpack_from("<Q", data, 56 + i * 8)[0]
    reason, count = struct.unpack_from("<II", data, 112)
    if count > 8:
        raise ValueError("crash payload count exceeds record")
    result.update(present=True, reason=reason,
                  values=list(struct.unpack_from("<8Q", data, 120)[:count]))
    result["label"] = data[184:232].split(b"\0", 1)[0].decode("ascii", "backslashreplace")
    result["consecutive"], result["archived"] = struct.unpack_from("<II", data, 232)
    return result


def boot_control(data):
    magic, version = struct.unpack_from("<II", data)
    if magic != 0x50424330:
        return {"valid": False, "reason": "boot-control magic absent"}
    if version not in (1, 2):
        return {"valid": False, "reason": f"unknown boot-control version {version}"}
    end = 32 if version == 1 else 48
    check = 0xB007C0DE
    for byte in data[:end]:
        check = ((check << 5) ^ (check >> 27) ^ byte) & 0xFFFFFFFF
    if check != struct.unpack_from("<I", data, end)[0]:
        return {"valid": False, "reason": "boot-control checksum mismatch"}
    fields = ("active_slot", "pending_slot", "tries_left", "last_boot", "good_mask", "generation")
    return {"valid": True, "version": version,
            **dict(zip(fields, struct.unpack_from("<6I", data, 8)))}


def inspect(source, expected, pc=None):
    if not expected or len(expected) > SLOT_BYTES - SECTOR:
        raise ValueError("expected payload exceeds raw slot or is empty")
    mbr = read_at(source, 0, SECTOR)
    if mbr[510:512] != b"\x55\xaa":
        raise ValueError("not an MBR disk image")
    partition = mbr[462:478]  # Production PIOS raw system partition is p2.
    kind = partition[4]
    start, sectors = struct.unpack_from("<II", partition, 8)
    if kind in (0, 0xEE) or start <= CRASH_LBA or not sectors or start + sectors > MAX_LBA:
        raise ValueError("missing or invalid PIOS p2 geometry")
    required = SLOT_OFFSETS[1] + SLOT_BYTES
    if sectors * SECTOR < required:
        raise ValueError("p2 is too small for the current PIOS A/B layout")
    root = start * SECTOR
    headers = [read_at(source, root + offset, SECTOR) for offset in SLOT_OFFSETS]
    if not any(struct.unpack_from("<I", h)[0] == SLOT_MAGIC for h in headers):
        raise ValueError("p2 has no PIOS slot header; refusing to inspect another disk")
    crash_data = read_at(source, CRASH_LBA * SECTOR, SECTOR)
    crash = crash_record(crash_data)
    if pc is None:
        pc = crash.get("pc")
    if pc is not None and (pc & 3 or not IMAGE_BASE <= pc <= IMAGE_BASE + len(expected) - 4):
        raise ValueError("PC is outside expected Pi5 payload")
    slots = []
    for index, header in enumerate(headers):
        magic, length, version = struct.unpack_from("<III", header)
        entry = {"slot": "AB"[index], "header_valid": False}
        offset, capacity = struct.unpack_from("<II", header, 16)
        if magic != SLOT_MAGIC or version not in (0, 1) or not 0 < length <= SLOT_BYTES - SECTOR or \
                offset not in (0, SECTOR) or capacity not in (0, SLOT_BYTES - SECTOR):
            entry["reason"] = "invalid or empty slot header"
            slots.append(entry)
            continue
        rounded = (length + SECTOR - 1) & ~(SECTOR - 1)
        payload = read_at(source, root + SLOT_OFFSETS[index] + SECTOR, rounded)[:length]
        entry.update(header_valid=True, bytes=length, sha256=hashlib.sha256(payload).hexdigest(),
                     matches_expected=payload == expected)
        mismatches = islice((i for i in range(min(length, len(expected)))
                            if payload[i] != expected[i]), 16)
        entry["first_differences"] = [
            {"offset": i, "address": IMAGE_BASE + i, "stored": payload[i], "expected": expected[i]}
            for i in mismatches]
        if pc is not None and pc - IMAGE_BASE + 4 <= length:
            entry["pc_word"] = f"0x{struct.unpack_from('<I', payload, pc - IMAGE_BASE)[0]:08x}"
        slots.append(entry)
    report = {"read_only": True, "p2_lba": start, "crash": crash,
              "boot_control": boot_control(read_at(source, root + CONTROL_OFFSET, SECTOR)),
              "expected_sha256": hashlib.sha256(expected).hexdigest(), "slots": slots}
    if pc is not None:
        report.update(pc=f"0x{pc:x}",
                      expected_pc_word=f"0x{struct.unpack_from('<I', expected, pc - IMAGE_BASE)[0]:08x}")
    return report, crash_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="disk image or explicitly identified read-only physical disk")
    parser.add_argument("--expected-payload", type=Path, required=True)
    parser.add_argument("--pc", type=lambda value: int(value, 0))
    parser.add_argument("--evidence", type=Path, required=True, help="new local directory; never overwrite evidence")
    args = parser.parse_args()
    with args.expected_payload.open("rb") as payload_file:
        expected = payload_file.read(SLOT_BYTES)
    args.evidence.mkdir(parents=False, exist_ok=False)
    with args.source.open("rb", buffering=0) as source:
        report, crash = inspect(source, expected, args.pc)
    (args.evidence / "crash-sector.bin").write_bytes(crash)
    (args.evidence / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="ascii")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
