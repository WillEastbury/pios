#!/usr/bin/env python3
"""Join a WiFi network using credentials from an ignored local config."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import time
import urllib.parse
import urllib.request


def terminal_command(host: str, command: str, timeout: float) -> str:
    encoded = urllib.parse.quote(command, safe="")
    url = f"http://{host}/api/terminal?cmd={encoded}"
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read().decode("utf-8", "replace").strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=pathlib.Path,
        default=pathlib.Path("tools/wifi_config.local.json"),
    )
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--activate", action="store_true")
    parser.add_argument("--no-scan", action="store_true")
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    host = str(config.get("host", "192.168.0.201"))
    ssid = str(config.get("ssid", ""))
    password = str(config.get("password", ""))
    mode = str(config.get("mode", "wpa2")).lower()
    activate = args.activate or bool(config.get("activate", False))
    if not ssid or not password:
        raise SystemExit(
            f"Set non-empty ssid/password in {args.config}; "
            "copy tools/wifi_config.example.json first."
        )
    if any(ch.isspace() for ch in ssid) or any(ch.isspace() for ch in password):
        raise SystemExit("PIOS wifi join currently requires values without spaces.")

    if not args.no_scan:
        print(terminal_command(host, "wifi scan", args.timeout))
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            time.sleep(2.0)
            scan = terminal_command(host, "wifi results", args.timeout)
            if "WiFi scan count=" in scan and any(
                line.startswith(f"{ssid} ") for line in scan.splitlines()
            ):
                print(scan)
                break
        else:
            raise SystemExit(f"SSID {ssid!r} not found before scan timeout")
        while time.monotonic() < deadline:
            status = terminal_command(host, "wifi status", args.timeout)
            if "event type=69 status=0" in status:
                break
            time.sleep(2.0)
        else:
            raise SystemExit("escan did not report completion before timeout")

    try:
        if mode == "wpa3":
            result = terminal_command(
                host, f"wifi join3 {ssid} {password}", args.timeout
            )
        elif mode == "wpa2":
            pmk = hashlib.pbkdf2_hmac(
                "sha1",
                password.encode("utf-8"),
                ssid.encode("utf-8"),
                4096,
                32,
            ).hex()
            result = terminal_command(
                host, f"wifi joinpmk {ssid} {pmk}", args.timeout
            )
        else:
            raise ValueError("mode must be 'wpa2' or 'wpa3'")
    except (OSError, TimeoutError) as exc:
        print(f"WiFi join request failed: {exc}")
        return 1
    print(result)
    if "WiFi join OK" not in result:
        return 1

    if activate:
        result = terminal_command(host, "wifi activate", args.timeout)
        print(result)
        if "WiFi activate OK" not in result:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
