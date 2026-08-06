#!/usr/bin/env python3
"""PicoScript source compiler / assembler.

PicoScript bytecode is deliberately small: it stores validated console
commands as records. The kernel `source` command can execute .pbc files by
feeding each command to the existing console executor, so compiled scripts
inherit current command semantics without a second VM.
"""

from __future__ import annotations

import argparse
import pathlib
import struct
import sys

MAGIC = 0x31434250  # PBC1
VERSION = 1
MAX_IMAGE = 4096
MAX_LINE = 255


def clean_source(text: str) -> list[str]:
    commands: list[str] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if len(line.encode("ascii", "strict")) > MAX_LINE:
            raise ValueError(f"line too long: {line[:40]!r}")
        commands.append(line)
    return commands


def assemble(text: str) -> list[str]:
    commands: list[str] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or line.startswith(";"):
            continue
        if line.startswith(".cmd "):
            line = line[5:].strip()
        elif line.startswith("cmd "):
            line = line[4:].strip()
        elif line.startswith("."):
            raise ValueError(f"unknown assembler directive: {line}")
        line.encode("ascii", "strict")
        if len(line) > MAX_LINE:
            raise ValueError(f"command too long: {line[:40]!r}")
        commands.append(line)
    return commands


def encode(commands: list[str]) -> bytes:
    body = bytearray()
    for cmd in commands:
        data = cmd.encode("ascii")
        if len(data) > MAX_LINE:
            raise ValueError(f"command too long: {cmd[:40]!r}")
        body += struct.pack("<HBB", len(data), 0, 0)
        body += data
    total = 16 + len(body)
    if total > MAX_IMAGE:
        raise ValueError(f"bytecode too large: {total} > {MAX_IMAGE}")
    return struct.pack("<IHHII", MAGIC, VERSION, len(commands), total, 0) + body


def decode(data: bytes) -> list[str]:
    if len(data) < 16:
        raise ValueError("bytecode too small")
    magic, version, count, image_bytes, _ = struct.unpack_from("<IHHII", data, 0)
    if magic != MAGIC or version != VERSION or image_bytes > len(data):
        raise ValueError("invalid bytecode header")
    off = 16
    out: list[str] = []
    for _i in range(count):
        if off + 4 > image_bytes:
            raise ValueError("truncated record")
        length, flags, _rsvd = struct.unpack_from("<HBB", data, off)
        off += 4
        if flags & 1:
            off += length
            continue
        if length > MAX_LINE or off + length > image_bytes:
            raise ValueError("invalid record length")
        out.append(data[off : off + length].decode("ascii", "replace"))
        off += length
    return out


def cmd_compile(args: argparse.Namespace) -> int:
    commands = clean_source(args.input.read_text(encoding="utf-8"))
    args.output.write_bytes(encode(commands))
    print(f"compiled {len(commands)} commands -> {args.output}")
    return 0


def cmd_asm(args: argparse.Namespace) -> int:
    commands = assemble(args.input.read_text(encoding="utf-8"))
    args.output.write_bytes(encode(commands))
    print(f"assembled {len(commands)} commands -> {args.output}")
    return 0


def cmd_disasm(args: argparse.Namespace) -> int:
    for cmd in decode(args.input.read_bytes()):
        print(f"cmd {cmd}")
    return 0


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="PicoScript compiler / assembler")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("compile", help="compile .pis source to .pbc")
    c.add_argument("input", type=pathlib.Path)
    c.add_argument("output", type=pathlib.Path)
    c.set_defaults(func=cmd_compile)
    a = sub.add_parser("asm", help="assemble .pasm command source to .pbc")
    a.add_argument("input", type=pathlib.Path)
    a.add_argument("output", type=pathlib.Path)
    a.set_defaults(func=cmd_asm)
    d = sub.add_parser("disasm", help="disassemble .pbc")
    d.add_argument("input", type=pathlib.Path)
    d.set_defaults(func=cmd_disasm)
    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
