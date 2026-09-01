#!/usr/bin/env python3
"""Build the shared PIAS asset pack from src/ide_assets.c string literals.

One copy of the PicoScript IDE HTML/JS goes in PIOSSTG2.PKG (platform SHARED)
instead of being linked into every board kernel.
"""
from __future__ import annotations

import pathlib
import re
import struct
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
SRC = REPO / "src" / "ide_assets.c"
OUT = REPO / "assets" / "pios_shared_assets.bin"

MAGIC = 0x53414950  # 'PIAS'
VERSION = 1
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


def main() -> int:
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
    struct.pack_into("<IHH I", buf, 0, MAGIC, VERSION, len(blobs), total)
    for i, (aid, offset, length) in enumerate(recs):
        struct.pack_into("<IIII", buf, 16 + i * 16, aid, offset, length, 0)
    buf[hdr:] = data
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_bytes(buf)
    print("pack_ide_assets: wrote %s (%u bytes)" % (OUT, total))
    return 0


if __name__ == "__main__":
    sys.exit(main())
