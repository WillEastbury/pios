#!/usr/bin/env python3
"""Fetch pinned, manifest-matched CYW firmware sets for PIOS FAT media."""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import urllib.request


PROFILES = {
    "pi5": {
        "revision": "c9d3ae6584ab79d19a4f94ccf701e888f9f87a53",
        "root": "debian/config/brcm80211",
        "files": {
            "firmware.bin": (
                "cypress/cyfmac43455-sdio-standard.bin",
                "d608f866582519c0a28d86db43040f4f1b98dd1d153e72e9752586546b4a36c3",
            ),
            "nvram.txt": (
                "brcm/brcmfmac43455-sdio.txt",
                "ca709be81a78bdb6932936374f39943acbd7af07fae6151011127599a3ce9e3d",
            ),
            "clm.bin": (
                "cypress/cyfmac43455-sdio.clm_blob",
                "9823842cae9fb9a5dd1e5fb31f595516ec7deee341354bef30bb3026eee29cc1",
            ),
        },
    },
    "zero2w": {
        "revision": "3bab0f823f5b53150b76aab77093adef6655b920",
        "root": "debian/added-firmware/brcm",
        "files": {
            "zero2w/43436/firmware.bin": (
                "brcmfmac43436-sdio.bin",
                "510a7dd1e056199b309425548ee0bd846993a1837ac7fa1e4d3e641f05a1327a",
            ),
            "zero2w/43436/nvram.txt": (
                "brcmfmac43436-sdio.txt",
                "4cda90facd8844cff60d80b34b24ecbae76adb9a62508a109461b8bf42b478d1",
            ),
            "zero2w/43436/clm.bin": (
                "brcmfmac43436-sdio.clm_blob",
                "fce7cbb62ffa6a5a65ca97b13f6fbf28d06c02d986c2072d65bf72164755fc34",
            ),
            "zero2w/43436s/firmware.bin": (
                "brcmfmac43436s-sdio.bin",
                "68b9bcc9855d91733cd44c21de4cb507c91b0d32d838c0696def5eb96c99e2de",
            ),
            "zero2w/43436s/nvram.txt": (
                "brcmfmac43436s-sdio.txt",
                "37a8b85a5a9742761101b764a07bc4d0c8b09f2e180eaea3b503a834277ad595",
            ),
        },
    },
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out", type=pathlib.Path, default=pathlib.Path("wifi_fw_staging")
    )
    parser.add_argument("--profile", choices=PROFILES, default="pi5")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    profile = PROFILES[args.profile]
    base = (
        "https://raw.githubusercontent.com/RPi-Distro/firmware-nonfree/"
        f"{profile['revision']}/{profile['root']}"
    )

    for name, (remote, expected) in profile["files"].items():
        print(f"[wifi-fw] fetching {name}")
        data = urllib.request.urlopen(f"{base}/{remote}", timeout=60).read()
        actual = hashlib.sha256(data).hexdigest()
        if actual != expected:
            raise RuntimeError(
                f"{name}: SHA-256 mismatch: expected {expected}, got {actual}"
            )
        destination = args.out / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
        print(f"[wifi-fw] {name}: {len(data)} bytes sha256={actual}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
