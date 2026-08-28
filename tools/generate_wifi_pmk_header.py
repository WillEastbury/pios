#!/usr/bin/env python3
"""Generate the ignored WiFi build header from the local JSON configuration."""

from __future__ import annotations

import hashlib
import json
import pathlib


def main() -> int:
    config_path = pathlib.Path("tools/wifi_config.local.json")
    output_path = pathlib.Path("wifi_config.h")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    ssid = str(config.get("ssid", ""))
    password = str(config.get("password", ""))
    if not ssid or not password:
        raise SystemExit(f"Set non-empty ssid/password in {config_path}")
    if not ssid.isascii() or any(ch in ssid for ch in '\\"') or len(ssid) > 32:
        raise SystemExit("SSID must be ASCII, <=32 bytes, and contain no quote/backslash")

    pmk = hashlib.pbkdf2_hmac(
        "sha1", password.encode("utf-8"), ssid.encode("utf-8"), 4096, 32
    )
    bytes_text = ", ".join(f"0x{value:02X}U" for value in pmk)
    output_path.write_text(
        "/* Generated locally; ignored and must never be committed. */\n"
        f"#define PIOS_WIFI_CONFIG_SSID \"{ssid}\"\n"
        f"#define PIOS_WIFI_CONFIG_SSID_LEN {len(ssid)}U\n"
        f"#define PIOS_WIFI_CONFIG_PMK {{ {bytes_text} }}\n",
        encoding="ascii",
        newline="\n",
    )
    print(f"generated {output_path} (ssid_len={len(ssid)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
