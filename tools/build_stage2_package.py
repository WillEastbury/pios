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
QEMU_MEMORY = 0x01400000
ALIGN = 512
MAX_PACKAGE = 0x1FFE00


def align_up(value: int, align: int = ALIGN) -> int:
    return (value + align - 1) & ~(align - 1)


def write_at(buf: bytearray, off: int, data: bytes) -> None:
    end = off + len(data)
    if len(buf) < end:
        buf.extend(b"\0" * (end - len(buf)))
    buf[off:end] = data


def entry(platform: int, name: str, payload_off: int, payload_size: int,
          load_addr: int, memory_size: int, required: int, optional: int) -> bytes:
    raw_name = name.encode("ascii")
    if len(raw_name) >= 32:
        raise SystemExit(f"entry name too long: {name}")
    raw_name = raw_name + b"\0" * (32 - len(raw_name))
    base = struct.pack("<IIQQQ32s", platform, 0, 0, required, optional, raw_name)
    return base + struct.pack("<QQQQ", payload_off, payload_size, load_addr, memory_size)


def main() -> int:
    ap = argparse.ArgumentParser(description="Build one PGS2 package containing Pi5 and QEMU stage2 payloads.")
    ap.add_argument("--pi", required=True, type=pathlib.Path)
    ap.add_argument("--qemu", required=True, type=pathlib.Path)
    ap.add_argument("--out", required=True, type=pathlib.Path)
    args = ap.parse_args()

    pi = args.pi.read_bytes()
    qemu = args.qemu.read_bytes()
    if not pi:
        raise SystemExit("Pi payload is empty")
    if not qemu:
        raise SystemExit("QEMU payload is empty")

    header_bytes = 32
    entry_bytes = 96
    entry_count = 2
    pi_off = align_up(header_bytes + entry_count * entry_bytes)
    qemu_off = align_up(pi_off + len(pi))
    total = align_up(qemu_off + len(qemu))
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
        len(qemu),
        QEMU_LOAD,
        QEMU_MEMORY,
        FEAT_AARCH64 | FEAT_GENERIC_TIMER | FEAT_GICV2 | FEAT_PL011 |
        FEAT_RAM_WALFS | FEAT_UEFI,
        0,
    ))
    write_at(out, pi_off, pi)
    write_at(out, qemu_off, qemu)
    args.out.write_bytes(out)
    print(f"stage2 package: {args.out} total={len(out)} pi={len(pi)}@{pi_off} qemu={len(qemu)}@{qemu_off}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
