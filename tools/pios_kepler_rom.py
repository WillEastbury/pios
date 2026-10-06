#!/usr/bin/env python3
"""Capture and validate a K2000 ROM locally; never execute its code/scripts."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import struct
import urllib.parse
import urllib.request

MAX_BYTES = 256 * 1024
MAX_IMAGES = 8
CHUNK = 128


def u16(data, offset):
    if offset < 0 or offset + 2 > len(data):
        raise ValueError("short ROM field")
    return struct.unpack_from("<H", data, offset)[0]


def image_header(data, start):
    if start + 0x1A > len(data) or data[start:start + 2] != b"\x55\xaa":
        raise ValueError("missing option-ROM signature")
    pcir = start + u16(data, start + 0x18)
    if pcir < start + 0x1A or pcir + 0x18 > len(data):
        raise ValueError("PCIR outside captured header")
    if data[pcir:pcir + 4] != b"PCIR":
        raise ValueError("missing PCIR signature")
    vendor, device = u16(data, pcir + 4), u16(data, pcir + 6)
    size = u16(data, pcir + 0x10) * 512
    if vendor != 0x10DE or device != 0x0FFE:
        raise ValueError("ROM identity does not match K2000")
    if not size or start + size > MAX_BYTES:
        raise ValueError("ROM image exceeds capture bound")
    pcir_size = u16(data, pcir + 0x0A)
    if pcir_size < 0x18 or pcir + pcir_size > start + size:
        raise ValueError("PCIR structure outside image")
    return {
        "offset": start, "bytes": size, "pcir": pcir,
        "vendor": vendor, "device": device,
        "code_type": data[pcir + 0x14],
        "last": bool(data[pcir + 0x15] & 0x80),
    }


def validate_rom(data):
    if not data or len(data) > MAX_BYTES:
        raise ValueError("invalid ROM length")
    images = []
    start = 0
    for _ in range(MAX_IMAGES):
        entry = image_header(data, start)
        end = start + entry["bytes"]
        if end > len(data):
            raise ValueError("short ROM image")
        # PCI legacy executable images require the whole-image checksum.
        if entry["code_type"] == 0 and sum(data[start:end]) & 255:
            raise ValueError("legacy ROM checksum mismatch")
        if entry["code_type"] not in (0, 3):
            raise ValueError("unsupported option-ROM code type")
        images.append(entry)
        start = end
        if entry["last"]:
            if start != len(data):
                raise ValueError("unexpected trailing ROM data")
            break
    else:
        raise ValueError("image-count budget exhausted")

    # GK107 BIT directory: signature, 12-byte header, bounded entry records.
    # Reference: Nouveau subdev/bios/bit.c and bios/init.c.
    first = data[:images[0]["bytes"]]
    bit = first.find(b"\xff\xb8BIT\x00")
    entries = []
    scripts = []
    if bit >= 0:
        if bit + 12 > len(first) or first[bit + 8] != 12:
            raise ValueError("invalid BIT header")
        stride, count = first[bit + 9], first[bit + 10]
        if stride < 6 or count > 64 or bit + 12 + stride * count > len(first):
            raise ValueError("invalid BIT directory bounds")
        if sum(first[bit:bit + 12]) & 255:
            raise ValueError("BIT header checksum mismatch")
        for i in range(count):
            pos = bit + 12 + stride * i
            length, offset = u16(first, pos + 2), u16(first, pos + 4)
            if offset + length > len(first):
                raise ValueError("BIT payload outside image")
            ident = chr(first[pos])
            entries.append({"id": ident, "version": first[pos + 1],
                            "offset": offset, "bytes": length})
            if ident == "I":
                if length < 2:
                    raise ValueError("short init-table directory")
                table = u16(first, offset)
                if table:
                    for index in range(64):
                        script = u16(first, table + index * 2)
                        if script == 0:
                            break
                        if script >= len(first):
                            raise ValueError("init script outside ROM")
                        scripts.append(script)
                    else:
                        raise ValueError("init-script count budget exhausted")
    return {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest(),
            "images": images, "bit_offset": bit, "bit_entries": entries,
            "init_scripts": scripts, "executed": False}


def terminal(host, command):
    url = f"http://{host}/api/terminal?cmd=" + urllib.parse.quote(command, safe="")
    with urllib.request.urlopen(url, timeout=10) as response:
        return response.read(8192).decode("ascii", "strict").strip()


def inspect_scripts(data):
    """Static inventory only: explore both paths, never evaluate or execute."""
    report = validate_rom(data)
    first = data[:report["images"][0]["bytes"]]
    directory = {e["id"]: e for e in report["bit_entries"]}
    if "I" not in directory or not report["init_scripts"]:
        raise ValueError("no bounded BIT init directory")
    ram_count = 0
    if "M" in directory:
        memory = directory["M"]
        if memory["version"] == 2 and memory["bytes"] >= 3:
            ram_count = first[memory["offset"]]
        elif memory["version"] == 1 and memory["bytes"] >= 5:
            ram_count = first[memory["offset"] + 2]
    # Instruction layouts from Nouveau nvkm/subdev/bios/init.c.
    fixed = {
        0x33: ("REPEAT", 2), 0x36: ("END_REPEAT", 1),
        0x37: ("COPY", 11), 0x38: ("NOT", 1), 0x39: ("IO_FLAG_CONDITION", 2),
        0x3B: ("IO_MASK_OR", 2), 0x3C: ("IO_OR", 2),
        0x47: ("ANDN_REG", 9), 0x48: ("OR_REG", 9), 0x4B: ("PLL2", 9),
        0x52: ("CR", 4), 0x53: ("ZM_CR", 3), 0x56: ("CONDITION_TIME", 3),
        0x57: ("LTIME", 3), 0x59: ("PLL_INDIRECT", 7),
        0x5A: ("ZM_REG_INDIRECT", 7), 0x5B: ("SUB_DIRECT", 3),
        0x5C: ("JUMP", 3), 0x5E: ("I2C_IF", 6), 0x5F: ("COPY_NV_REG", 22),
        0x62: ("ZM_INDEX_IO", 5), 0x63: ("COMPUTE_MEM", 1),
        0x65: ("RESET", 13), 0x66: ("CONFIGURE_MEM", 1),
        0x67: ("CONFIGURE_CLK", 1), 0x68: ("CONFIGURE_PREINIT", 1),
        0x69: ("IO", 5), 0x6B: ("SUB", 2), 0x6D: ("RAM_CONDITION", 3),
        0x6E: ("NV_REG", 13), 0x6F: ("MACRO", 2), 0x71: ("DONE", 1),
        0x72: ("RESUME", 1), 0x73: ("STRAP_CONDITION", 9),
        0x74: ("TIME", 3), 0x75: ("CONDITION", 2), 0x76: ("IO_CONDITION", 2),
        0x77: ("ZM_REG16", 7), 0x78: ("INDEX_IO", 6), 0x79: ("PLL", 7),
        0x7A: ("ZM_REG", 9), 0x8C: ("RESET_BEGUN", 1), 0x8D: ("RESET_END", 1),
        0x8E: ("GPIO", 1), 0x90: ("COPY_ZM_REG", 9), 0x92: ("RESERVED", 1),
        0x96: ("XLAT", 17), 0x97: ("ZM_MASK_ADD", 13),
        0x9A: ("I2C_LONG_IF", 7), 0xAA: ("RESERVED_AA", 4),
    }
    variable = {
        # opcode -> name, fixed length, count-byte offset, record stride
        0x32: ("IO_RESTRICT_PROG", 11, 6, 4),
        0x34: ("IO_RESTRICT_PLL", 12, 7, 2),
        0x49: ("INDEX_ADDRESS_LATCHED", 18, 17, 2),
        0x4A: ("IO_RESTRICT_PLL2", 11, 6, 4),
        0x4C: ("I2C_BYTE", 4, 3, 3),
        0x4D: ("ZM_I2C_BYTE", 4, 3, 2),
        0x4E: ("ZM_I2C", 4, 3, 1),
        0x54: ("ZM_CR_GROUP", 2, 1, 2),
        0x58: ("ZM_REG_SEQUENCE", 6, 5, 4),
        0x91: ("ZM_REG_GROUP", 6, 5, 4),
        0x98: ("AUXCH", 6, 5, 2),
        0x99: ("ZM_AUXCH", 6, 5, 1),
        0xA9: ("GPIO_NE", 2, 1, 1),
    }
    def span(offset, length):
        if offset < 0 or length < 0 or offset + length > len(first):
            raise ValueError(f"init instruction outside ROM at 0x{offset:x}")
        return first[offset:offset + length]

    init = directory["I"]
    roots = list(report["init_scripts"])
    if init["bytes"] >= 16:
        extra = u16(first, init["offset"] + 14)
        if extra:
            roots.append(extra)
    pending = list(roots)
    visited, occupied = set(), set()
    counts = Counter()
    records = []
    registers = set()
    conditions = set()
    delays_us = 0
    while pending:
        pc = pending.pop()
        if pc in visited:
            continue
        if len(visited) >= 4096:
            raise ValueError("init instruction-count budget exhausted")
        opcode = span(pc, 1)[0]
        if opcode in fixed:
            name, length = fixed[opcode]
        elif opcode in variable:
            name, header, pos, stride = variable[opcode]
            length = header + span(pc, header)[pos] * stride
        elif opcode == 0x8F:
            name = "RAM_RESTRICT_ZM_REG_GROUP"
            if not 0 < ram_count <= 32:
                raise ValueError("invalid RAM configuration count")
            length = 7 + span(pc, 7)[6] * ram_count * 4
        elif opcode == 0x87:
            name = "RAM_RESTRICT_PLL"
            if not 0 < ram_count <= 32:
                raise ValueError("invalid RAM configuration count")
            length = 2 + ram_count * 4
        elif opcode == 0x3A:
            name = "GENERIC_CONDITION"
            head = span(pc, 3)
            length = 3 + (head[2] if head[1] not in (0, 1, 2, 5, 7) else 0)
        else:
            raise ValueError(f"unsupported init opcode 0x{opcode:02x} at 0x{pc:04x}")
        instruction = span(pc, length)
        region = set(range(pc, pc + length))
        if region & occupied:
            raise ValueError(f"overlapping instruction at 0x{pc:x}")
        occupied.update(region)
        visited.add(pc)
        counts[name] += 1
        records.append({"offset": pc, "opcode": opcode, "bytes": length, "name": name})
        if opcode in (0x47, 0x48, 0x4B, 0x58, 0x59, 0x5A,
                      0x65, 0x6E, 0x77, 0x79, 0x7A, 0x8F, 0x91, 0x97):
            registers.add(struct.unpack_from("<I", instruction, 1)[0])
        if opcode in (0x56, 0x75):
            conditions.add(instruction[1])
        if opcode in (0x57, 0x74):
            delays_us += u16(instruction, 1) * (1000 if opcode == 0x57 else 1)
        if opcode in (0x5B, 0x5C):
            target = u16(instruction, 1)
            if target:
                pending.append(target)
        if opcode == 0x6B:
            index = instruction[1]
            if index >= len(report["init_scripts"]):
                raise ValueError("init SUB index outside script table")
            pending.append(report["init_scripts"][index])
        if opcode != 0x71:
            pending.append(pc + length)
    return {"instruction_count": len(visited), "opcodes": dict(sorted(counts.items())),
            "instructions": sorted(records, key=lambda entry: entry["offset"]),
            "entrypoints": [f"0x{x:04x}" for x in roots], "ram_configurations": ram_count,
            "direct_register_operands": [f"0x{x:08x}" for x in sorted(registers)],
            "condition_indices": sorted(conditions),
            "unique_delay_operands_us": delays_us,
            "note": "Static inventory, not an execution trace, timing bound or safe-write list",
            "executed": False}


def capture(host):
    probe = terminal(host, "kepler probe")
    if "kepler probe ok" not in probe or "rom_available=1" not in probe:
        raise RuntimeError(f"ROM access unavailable: {probe}")
    data = bytearray()

    def ensure(end):
        if end > MAX_BYTES:
            raise ValueError("ROM capture byte budget exceeded")
        while len(data) < end:
            offset = len(data)
            reply = terminal(host, f"kepler rom {offset}")
            match = re.fullmatch(r"rom ([0-9A-Fa-f]{8}) ([0-9A-Fa-f]{256})", reply)
            if not match or int(match[1], 16) != offset:
                raise RuntimeError(f"ROM chunk {offset} failed: {reply[:160]}")
            data.extend(bytes.fromhex(match[2]))

    start = 0
    for _ in range(MAX_IMAGES):
        ensure(start + 0x1A)
        pcir = start + u16(data, start + 0x18)
        ensure(pcir + 0x18)
        header = image_header(data, start)
        end = start + header["bytes"]
        ensure(end)
        if header["last"]:
            return bytes(data[:end])
        start = end
    raise ValueError("ROM image budget exceeded")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.0.201")
    parser.add_argument("--out", type=Path, help="save validated capture; never overwrite")
    parser.add_argument("--inspect", type=Path, help="validate an existing local capture")
    parser.add_argument("--scripts", action="store_true", help="inventory init opcodes offline")
    args = parser.parse_args()
    if args.inspect:
        with args.inspect.open("rb") as stream:
            data = stream.read(MAX_BYTES + 1)
    else:
        if not args.out:
            parser.error("--out is required for a hardware capture")
        if args.out.exists():
            parser.error("output already exists")
        data = capture(args.host)
    report = validate_rom(data)
    if args.scripts:
        report["scripts"] = inspect_scripts(data)
    if not args.inspect:
        with args.out.open("xb") as stream:
            stream.write(data)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
