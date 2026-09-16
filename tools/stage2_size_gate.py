#!/usr/bin/env python3
"""Fail the build if one raw stage2 payload cannot fit an A/B slot."""
from __future__ import annotations

import argparse
import pathlib

RAW_SLOT_CAP = 0x37FE00


def payload_fits(size: int) -> bool:
    return 0 < size <= RAW_SLOT_CAP


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("payload", type=pathlib.Path)
    args = ap.parse_args()
    size = args.payload.stat().st_size
    if not payload_fits(size):
        raise SystemExit(
            f"ERROR: {args.payload} is {size} bytes; raw slot cap is {RAW_SLOT_CAP}"
        )
    print(f"stage2 size gate: {size}/{RAW_SLOT_CAP} bytes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
