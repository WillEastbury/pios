#!/usr/bin/env python3
"""Build deterministic PIAS editor packs from src/ide_assets.c string literals.

The Pi5 raw stage2 image embeds the PBRP (Brotli + checksummed PIAS) pack and
installs it into WALFS after mount. ``--shared`` retains the legacy stage0 FAT
pack for the non-Pi5 build flows that still use it.
"""
from __future__ import annotations

import pathlib
import re
import struct
import sys
import pios_brotli

REPO = pathlib.Path(__file__).resolve().parent.parent
SRC = REPO / "src" / "ide_assets.c"
OUT_SHARED = REPO / "assets" / "pios_shared_assets.bin"
OUT_BROTLI = REPO / "assets" / "pios_ide_assets.brp"

MAGIC = 0x53414950  # 'PIAS'
VERSION = 2
BROTLI_MAGIC = 0x50425250  # 'PBRP'
BROTLI_VERSION = 1
MAX_RAW_BYTES = 2_100_000
IDS = (
    ("IDE_HTML_EMBED", 1),
    ("IDE_PICOWAL_HTML_EMBED", 2),
    ("IDE_PICO_HOOKS_JS_EMBED", 3),
    ("IDE_BAREMETAL_BINARY_JS_EMBED", 4),
)
HEX = set("0123456789abcdefABCDEF")


def parse_c_strings(src: str, name: str) -> bytes:
    m = re.search(r"const u8 %s\[\]\s*=" % re.escape(name), src)
    if not m:
        raise SystemExit("pack_ide_assets: missing %s in %s" % (name, SRC))
    i = m.end()
    out = bytearray()
    n = len(src)
    while i < n and src[i] != ";":
        if src[i] != '"':
            i += 1
            continue
        i += 1
        while i < n and src[i] != '"':
            if src[i] != "\\":
                out.append(ord(src[i]))
                i += 1
                continue
            i += 1
            if i >= n:
                break
            c = src[i]
            i += 1
            if c == "n":
                out.append(0x0A)
            elif c == "r":
                out.append(0x0D)
            elif c == "t":
                out.append(0x09)
            elif c in "\\\"?":
                out.append(ord(c if c != "?" else "?"))
            elif c == "x" and i + 1 < n:
                h = ""
                while i < n and src[i] in HEX and len(h) < 2:
                    h += src[i]
                    i += 1
                out.append(int(h, 16) if h else 0)
            else:
                out.append(ord(c))
        if i < n and src[i] == '"':
            i += 1
    return bytes(out)


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def build_raw_pack() -> bytes:
    src = SRC.read_text(encoding="utf-8")
    blobs = []
    for name, aid in IDS:
        data = parse_c_strings(src, name)
        blobs.append((aid, data))
        print("  id=%u %s %u bytes" % (aid, name, len(data)))
    recs = []
    data = bytearray()
    hdr = 16 + 16 * len(blobs)
    off = hdr
    for aid, blob in blobs:
        recs.append((aid, off, len(blob)))
        data.extend(blob)
        off += len(blob)
    total = hdr + len(data)
    buf = bytearray(total)
    struct.pack_into("<IHHII", buf, 0, MAGIC, VERSION, len(blobs), total, 0)
    for i, (aid, offset, length) in enumerate(recs):
        struct.pack_into("<IIII", buf, 16 + i * 16, aid, offset, length, 0)
    buf[hdr:] = data
    struct.pack_into("<I", buf, 12, crc32c(buf[16:]))
    return bytes(buf)


def build_brotli_pack(raw: bytes) -> bytes:
    if len(raw) > MAX_RAW_BYTES:
        raise SystemExit(
            "pack_ide_assets: raw pack exceeds kernel extraction bound: "
            f"{len(raw)} > {MAX_RAW_BYTES}"
        )
    compressed = pios_brotli.compress(raw)
    if pios_brotli.decompress(compressed) != raw:
        raise SystemExit("pack_ide_assets: compressor round-trip failed")
    header = bytearray(struct.pack("<IHHIIIII", BROTLI_MAGIC, BROTLI_VERSION, 28,
                                   len(compressed), len(raw), crc32c(compressed),
                                   crc32c(raw), 0))
    struct.pack_into("<I", header, 24, crc32c(header[:24]))
    return bytes(header) + compressed


def main() -> int:
    want_brotli = "--brotli" in sys.argv[1:] or len(sys.argv) == 1
    want_shared = "--shared" in sys.argv[1:]
    unknown = set(sys.argv[1:]) - {"--brotli", "--shared"}
    if unknown:
        raise SystemExit("usage: pack_ide_assets.py [--brotli] [--shared]")
    raw = build_raw_pack()
    OUT_BROTLI.parent.mkdir(exist_ok=True)
    if want_brotli:
        brotli = build_brotli_pack(raw)
        OUT_BROTLI.write_bytes(brotli)
        print("pack_ide_assets: wrote %s (%u raw, %u Brotli bytes)" %
              (OUT_BROTLI, len(raw), len(brotli)))
    if want_shared:
        # Version 1 remains ABI-compatible with stage0's PIOS_SHARED reader.
        legacy = bytearray(raw)
        struct.pack_into("<H", legacy, 4, 1)
        struct.pack_into("<I", legacy, 12, 0)
        OUT_SHARED.write_bytes(legacy)
        print("pack_ide_assets: wrote %s (%u legacy bytes)" % (OUT_SHARED, len(legacy)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
