#!/usr/bin/env python3
"""Fetch the pinned official CYW43455 Raspberry Pi 5 firmware set."""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import urllib.request


REVISION = "c9d3ae6584ab79d19a4f94ccf701e888f9f87a53"
BASE = (
    "https://raw.githubusercontent.com/RPi-Distro/firmware-nonfree/"
    f"{REVISION}/debian/config/brcm80211"
)
FILES = {
    "firmware.bin": (
        "cypress/cyfmac43455-sdio-minimal.bin",
        "3075cb0bdc4b28ed4f08e01b1a216d0ebc70f4022d9d3272a4a43b3c90456e60",
    ),
    "nvram.txt": (
        "brcm/brcmfmac43455-sdio.txt",
        "ca709be81a78bdb6932936374f39943acbd7af07fae6151011127599a3ce9e3d",
    ),
    "clm.bin": (
        "cypress/cyfmac43455-sdio.clm_blob",
        "9823842cae9fb9a5dd1e5fb31f595516ec7deee341354bef30bb3026eee29cc1",
    ),
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out", type=pathlib.Path, default=pathlib.Path("wifi_fw_staging")
    )
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    for name, (remote, expected) in FILES.items():
        print(f"[wifi-fw] fetching {name}")
        data = urllib.request.urlopen(f"{BASE}/{remote}", timeout=60).read()
        actual = hashlib.sha256(data).hexdigest()
        if actual != expected:
            raise RuntimeError(
                f"{name}: SHA-256 mismatch: expected {expected}, got {actual}"
            )
        (args.out / name).write_bytes(data)
        print(f"[wifi-fw] {name}: {len(data)} bytes sha256={actual}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
