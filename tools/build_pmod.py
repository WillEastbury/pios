#!/usr/bin/env python3
"""Package a raw position-independent AArch64 driver module as PMOD v1."""
from __future__ import annotations

import argparse
import hashlib
import pathlib
import struct

MAGIC = 0x504D4F44
VERSION = 1
ABI = 1
HEADER = struct.Struct("<IHH7I32s")
SLOT_BYTES = 0x40000


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("code", type=pathlib.Path)
    ap.add_argument("--out", type=pathlib.Path, required=True)
    ap.add_argument("--module-id", type=int, required=True)
    ap.add_argument("--generation", type=int, required=True)
    ap.add_argument("--arena-schema", type=int, required=True)
    ap.add_argument("--capabilities", type=lambda x: int(x, 0), default=0)
    ap.add_argument("--entry-offset", type=lambda x: int(x, 0), default=0)
    args = ap.parse_args()
    code = args.code.read_bytes()
    if not code or len(code) > SLOT_BYTES:
        raise SystemExit("code must be 1..262144 bytes")
    if not 0 <= args.module_id < 8:
        raise SystemExit("module-id must be 0..7")
    if args.generation <= 0 or args.arena_schema <= 0:
        raise SystemExit("generation and arena-schema must be nonzero")
    if not 0 <= args.entry_offset < len(code):
        raise SystemExit("entry-offset outside code")
    header = HEADER.pack(
        MAGIC, VERSION, HEADER.size, args.module_id, ABI,
        args.arena_schema, args.capabilities, args.generation,
        len(code), args.entry_offset, hashlib.sha256(code).digest()
    )
    args.out.write_bytes(header + code)
    print(f"PMOD module={args.module_id} generation={args.generation} "
          f"schema={args.arena_schema} code={len(code)} bytes "
          f"sha256={hashlib.sha256(code).hexdigest()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
