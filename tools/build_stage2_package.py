#!/usr/bin/env python3
import argparse
import pathlib
import struct


MAGIC = 0x32534750
VERSION = 1
FLAG_PACKAGED = 1

PLATFORM_PI5 = 1
PLATFORM_QEMU = 2

FEAT_AARCH64 = 1 << 0
FEAT_GENERIC_TIMER = 1 << 1
FEAT_GICV2 = 1 << 2
FEAT_PL011 = 1 << 3
FEAT_PI_FIRMWARE = 1 << 4
FEAT_RP1 = 1 << 5
FEAT_PCIE = 1 << 6
FEAT_SD = 1 << 7
FEAT_GENET = 1 << 8
FEAT_MAILBOX_FB = 1 << 9
FEAT_RAM_WALFS = 1 << 10
FEAT_UEFI = 1 << 11

PI_LOAD = 0x00080000
QEMU_LOAD = 0x40080000
QEMU_MEMORY = 0x02000000
ALIGN = 512
MAX_PACKAGE = 0x1FFE00
PAYLOAD_FLAG_COMPRESSED = 1
PAYLOAD_CODEC_NONE = 0
PAYLOAD_CODEC_PICOCOMPRESS = 1


def align_up(value: int, align: int = ALIGN) -> int:
    return (value + align - 1) & ~(align - 1)


def write_at(buf: bytearray, off: int, data: bytes) -> None:
    end = off + len(data)
    if len(buf) < end:
        buf.extend(b"\0" * (end - len(buf)))
    buf[off:end] = data


def simple_picocompress(data: bytes) -> bytes:
    block_size = 508
    match_min = 3
    match_max = 10
    offset_max = 4095
    literal_max = 128
    out = bytearray()
    for start in range(0, len(data), block_size):
        block = data[start:start + block_size]
        ip = 0
        comp = bytearray()
        literals = bytearray()
        while ip < len(block):
            best_len = 0
            best_off = 0
            scan_start = max(0, ip - offset_max)
            if ip + match_min <= len(block):
                for prev in range(scan_start, ip):
                    if block[prev:prev + match_min] != block[ip:ip + match_min]:
                        continue
                    m = match_min
                    while m < match_max and ip + m < len(block) and block[prev + m] == block[ip + m]:
                        m += 1
                    if m > best_len:
                        best_len = m
                        best_off = ip - prev
                        if m == match_max:
                            break
            if best_len >= match_min:
                while literals:
                    chunk = literals[:literal_max]
                    del literals[:len(chunk)]
                    comp.append(len(chunk) - 1)
                    comp.extend(chunk)
                comp.append(0x80 | ((best_len - match_min) << 4) | ((best_off >> 8) & 0x0F))
                comp.append(best_off & 0xFF)
                ip += best_len
            else:
                literals.append(block[ip])
                ip += 1
                if len(literals) == literal_max:
                    comp.append(len(literals) - 1)
                    comp.extend(literals)
                    literals.clear()
        while literals:
            chunk = literals[:literal_max]
            del literals[:len(chunk)]
            comp.append(len(chunk) - 1)
            comp.extend(chunk)
        if len(comp) < len(block):
            out.extend(len(block).to_bytes(2, "little"))
            out.extend(len(comp).to_bytes(2, "little"))
            out.extend(comp)
        else:
            out.extend(len(block).to_bytes(2, "little"))
            out.extend((0).to_bytes(2, "little"))
            out.extend(block)
    return bytes(out)


def entry(platform: int, name: str, payload_off: int, payload_size: int,
          load_addr: int, memory_size: int, required: int, optional: int,
          payload_flags: int = 0, payload_codec: int = 0,
          uncompressed_size: int | None = None) -> bytes:
    raw_name = name.encode("ascii")
    if len(raw_name) >= 32:
        raise SystemExit(f"entry name too long: {name}")
    raw_name = raw_name + b"\0" * (32 - len(raw_name))
    base = struct.pack("<IIQQQ32s", platform, 0, 0, required, optional, raw_name)
    if uncompressed_size is None:
        uncompressed_size = payload_size
    return base + struct.pack("<QQQQIIQ", payload_off, payload_size, load_addr,
                              memory_size, payload_flags, payload_codec,
                              uncompressed_size)


def main() -> int:
    ap = argparse.ArgumentParser(description="Build one PGS2 package containing Pi5 and QEMU stage2 payloads.")
    ap.add_argument("--pi", required=True, type=pathlib.Path)
    ap.add_argument("--qemu", required=True, type=pathlib.Path)
    ap.add_argument("--out", required=True, type=pathlib.Path)
    ap.add_argument("--compress-qemu", action="store_true")
    args = ap.parse_args()

    pi = args.pi.read_bytes()
    qemu = args.qemu.read_bytes()
    if not pi:
        raise SystemExit("Pi payload is empty")
    if not qemu:
        raise SystemExit("QEMU payload is empty")

    qemu_payload = qemu
    qemu_flags = 0
    qemu_codec = PAYLOAD_CODEC_NONE
    if args.compress_qemu:
        qemu_payload = simple_picocompress(qemu)
        qemu_flags = PAYLOAD_FLAG_COMPRESSED
        qemu_codec = PAYLOAD_CODEC_PICOCOMPRESS
        if len(qemu_payload) >= len(qemu):
            raise SystemExit("compressed QEMU payload did not shrink")

    header_bytes = 32
    entry_bytes = 112
    entry_count = 2
    pi_off = align_up(header_bytes + entry_count * entry_bytes)
    qemu_off = align_up(pi_off + len(pi))
    total = align_up(qemu_off + len(qemu_payload))
    if total > MAX_PACKAGE:
        raise SystemExit(f"stage2 package too large: {total} > {MAX_PACKAGE}")

    out = bytearray(total)
    header = struct.pack("<IHHHHIQQ", MAGIC, VERSION, header_bytes, entry_count,
                         entry_bytes, FLAG_PACKAGED, 0, total)
    write_at(out, 0, header)
    write_at(out, header_bytes, entry(
        PLATFORM_PI5,
        "pi5-bcm2712",
        pi_off,
        len(pi),
        PI_LOAD,
        len(pi),
        FEAT_AARCH64 | FEAT_PI_FIRMWARE,
        FEAT_GENERIC_TIMER | FEAT_GICV2 | FEAT_PL011 | FEAT_RP1 |
        FEAT_PCIE | FEAT_SD | FEAT_GENET | FEAT_MAILBOX_FB,
    ))
    write_at(out, header_bytes + entry_bytes, entry(
        PLATFORM_QEMU,
        "qemu-virt",
        qemu_off,
        len(qemu_payload),
        QEMU_LOAD,
        QEMU_MEMORY,
        FEAT_AARCH64 | FEAT_GENERIC_TIMER | FEAT_GICV2 | FEAT_PL011 |
        FEAT_RAM_WALFS | FEAT_UEFI,
        0,
        qemu_flags,
        qemu_codec,
        len(qemu),
    ))
    write_at(out, pi_off, pi)
    write_at(out, qemu_off, qemu_payload)
    args.out.write_bytes(out)
    qemu_note = f"{len(qemu_payload)}:{len(qemu)}" if args.compress_qemu else str(len(qemu))
    print(f"stage2 package: {args.out} total={len(out)} pi={len(pi)}@{pi_off} qemu={qemu_note}@{qemu_off}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
