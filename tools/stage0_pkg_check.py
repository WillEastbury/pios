#!/usr/bin/env python3
"""
Host-side validator for the PIOS stage0 FAT updater package (PIOSSTG2.PKG).

This mirrors, in pure host logic, exactly what the on-hardware stage0 loader
(src/bootstrap.c) does with the package it reads from FAT:/PIOSSTG2.PKG before
it is committed to raw slot A:

  * package_identity()      -> PGS2 header sanity + declared size == file size +
                               whole-package FNV-1a id (bytes 16..23 excluded)
  * select_stage2_image()   -> locate the platform entry, validate payload
                               offset/size/entry bounds and load_addr
  * write_slot_package()    -> only the SELECTED platform's payload subset is
                               extracted and written into the raw slot (see
                               stage0_apply_fat_update()), so the raw slot
                               stays capped at PIOS_STAGE2_ZONE_BYTES (one
                               platform's payload) while the FAT-resident
                               package file itself is capped at the larger
                               PIOS_FAT_PACKAGE_MAX_BYTES -- it can carry
                               multiple platforms' payloads (e.g. Pi5 AND
                               BCM2837-family/Pi3/Pi Zero 2W) simultaneously,
                               since stage0 now genuinely detects which board
                               it's running on via MIDR_EL1 (board_detect.c)
                               before extracting only that one payload.

This harness is Pi5-focused by default but can validate any platform id.

Exit code is 0 only when every check passes, so it can gate a hardware deploy
next to tools/qemu_smoke.py.

Usage:
  python tools/stage0_pkg_check.py [PACKAGE] [--platform pi5|qemu|uefi|bcm2837|<id>]
"""

from __future__ import annotations

import argparse
import pathlib
import struct
import sys

# --- constants mirrored from include/stage2_manifest.h + include/walfs.h ---
MANIFEST_MAGIC = 0x32534750          # 'PGS2'
MANIFEST_VERSION = 1
FLAG_PACKAGED = 1 << 0
HEADER_BYTES = 32
ENTRY_BASE_BYTES = 64                # sizeof(struct pios_stage2_manifest_entry)
PACKAGED_ENTRY_BYTES = 96            # PIOS_STAGE2_PACKAGED_ENTRY_BYTES (V1 min)

PLATFORM = {"pi5": 1, "qemu": 2, "uefi": 3, "hyperv-arm": 4, "hyperv-amd64": 5, "bcm2837": 6, "pi3": 6, "pizero2w": 6}
PI_LOAD = 0x00080000                 # BOOT_DST_ADDR / PI_LOAD
STAGE2_ZONE_BYTES = 0x37FE00         # PIOS_STAGE2_ZONE_BYTES (3,669,504) -- per-platform raw-slot payload cap
FAT_PACKAGE_MAX_BYTES = 16 * 1024 * 1024  # PIOS_FAT_PACKAGE_MAX_BYTES -- whole multi-platform FAT file cap

FNV_BASIS = 0xCBF29CE484222325
FNV_PRIME = 0x100000001B3
U64_MASK = 0xFFFFFFFFFFFFFFFF

GREEN, RED, DIM, RST = "\033[32m", "\033[31m", "\033[2m", "\033[0m"


class Report:
    def __init__(self) -> None:
        self.passed = 0
        self.failed = 0

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        tag = f"{GREEN}[PASS]{RST}" if ok else f"{RED}[FAIL]{RST}"
        if ok:
            self.passed += 1
        else:
            self.failed += 1
        line = f"  {tag} {name}"
        if detail:
            line += f"  {DIM}{detail}{RST}"
        print(line)
        return ok


def package_id(buf: bytes) -> int:
    """FNV-1a over the whole image, with the id field (bytes 16..23) excluded.

    Identical to package_identity() in src/bootstrap.c and package_id_for() in
    tools/build_stage2_package.py.
    """
    value = FNV_BASIS
    for index, byte in enumerate(buf):
        value ^= 0 if 16 <= index < 24 else byte
        value = (value * FNV_PRIME) & U64_MASK
    return value or 1


def validate(path: pathlib.Path, want_platform: int, rep: Report) -> None:
    data = path.read_bytes()
    image_len = len(data)
    rep.check(f"package readable ({path.name})", image_len > 0,
              f"{image_len} bytes")
    if image_len < HEADER_BYTES:
        rep.check("package >= header", False, f"{image_len} < {HEADER_BYTES}")
        return

    magic, version, header_bytes, entry_count, entry_bytes, flags = struct.unpack_from(
        "<IHHHHI", data, 0)
    stored_id, declared = struct.unpack_from("<QQ", data, 16)

    # ---- package_identity() gate ------------------------------------------
    rep.check("PGS2 magic", magic == MANIFEST_MAGIC,
              f"0x{magic:08X} (want 0x{MANIFEST_MAGIC:08X})")
    rep.check("manifest version", version == MANIFEST_VERSION, f"v{version}")
    rep.check("PACKAGED flag set", bool(flags & FLAG_PACKAGED), f"flags=0x{flags:X}")
    rep.check("declared size == file size", declared == image_len,
              f"declared={declared} file={image_len}")
    rep.check("package id non-zero", stored_id != 0, f"id=0x{stored_id:016X}")
    computed = package_id(data)
    rep.check("FNV-1a package id matches", computed == stored_id,
              f"computed=0x{computed:016X} stored=0x{stored_id:016X}")

    # ---- header field sanity (select_stage2_image) ------------------------
    rep.check("header_bytes >= 32", header_bytes >= HEADER_BYTES, f"{header_bytes}")
    rep.check("entry_bytes >= packaged entry", entry_bytes >= PACKAGED_ENTRY_BYTES,
              f"{entry_bytes} >= {PACKAGED_ENTRY_BYTES}")
    rep.check("entry_count in 1..16", 1 <= entry_count <= 16, f"{entry_count}")
    table_bytes = header_bytes + entry_count * entry_bytes
    rep.check("entry table within image", table_bytes <= image_len,
              f"table={table_bytes} image={image_len}")

    # ---- whole FAT-resident package fits the staging/FAT-read cap ---------
    # (Not the raw-slot zone: only the per-platform payload extracted below
    # needs to fit that -- see the "payload fits stage2 zone" check.)
    rep.check("whole package fits FAT package cap", image_len <= FAT_PACKAGE_MAX_BYTES,
              f"{image_len} <= {FAT_PACKAGE_MAX_BYTES} (headroom {FAT_PACKAGE_MAX_BYTES - image_len})")

    # ---- select the requested platform payload (select_stage2_image) ------
    found = False
    for i in range(entry_count):
        eoff = header_bytes + i * entry_bytes
        if eoff + PACKAGED_ENTRY_BYTES > image_len:
            break
        platform_id, = struct.unpack_from("<I", data, eoff)
        entry_offset, = struct.unpack_from("<Q", data, eoff + 8)
        payload_off, payload_size, load_addr = struct.unpack_from(
            "<QQQ", data, eoff + 64)
        name = data[eoff + 32:eoff + 64].split(b"\0", 1)[0].decode("ascii", "replace")
        print(f"  {DIM}entry[{i}] platform={platform_id} name={name!r} "
              f"payload={payload_size}@{payload_off} entry_off={entry_offset} "
              f"load=0x{load_addr:X}{RST}")
        if platform_id != want_platform:
            continue
        found = True
        rep.check("payload size non-zero", payload_size > 0, f"{payload_size}")
        rep.check("payload offset within image", payload_off <= image_len,
                  f"{payload_off} <= {image_len}")
        rep.check("payload fits within image",
                  payload_off <= image_len and payload_size <= image_len - payload_off,
                  f"{payload_off}+{payload_size} <= {image_len}")
        rep.check("entry offset within payload", entry_offset < payload_size,
                  f"{entry_offset} < {payload_size}")
        rep.check("payload fits stage2 zone", payload_size <= STAGE2_ZONE_BYTES,
                  f"{payload_size} <= {STAGE2_ZONE_BYTES}")
        want_load = PI_LOAD if want_platform in (PLATFORM["pi5"], PLATFORM["bcm2837"]) else load_addr
        rep.check("load_addr == 0x00080000 (Pi5 BOOT_DST_ADDR)",
                  load_addr == want_load,
                  f"0x{load_addr:X}")
        break

    rep.check(f"platform {want_platform} entry present", found)


def main() -> int:
    ap = argparse.ArgumentParser(description="Validate a PIOS stage0 FAT package.")
    ap.add_argument("package", nargs="?", default="PIOSSTG2.PKG", type=pathlib.Path)
    ap.add_argument("--platform", default="pi5",
                    help="platform to select: pi5|qemu|uefi|<numeric id> (default pi5)")
    args = ap.parse_args()

    if args.platform in PLATFORM:
        want = PLATFORM[args.platform]
    else:
        try:
            want = int(args.platform, 0)
        except ValueError:
            print(f"unknown platform: {args.platform}", file=sys.stderr)
            return 2

    path = args.package
    if not path.exists():
        print(f"{RED}package not found: {path}{RST}", file=sys.stderr)
        return 2

    print(f"[stage0-pkg] validating {path} for platform {want}")
    rep = Report()
    validate(path, want, rep)
    total = rep.passed + rep.failed
    color = GREEN if rep.failed == 0 else RED
    print(f"\n{color}STAGE0-PKG: {rep.passed}/{total} checks passed{RST}")
    return 0 if rep.failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
