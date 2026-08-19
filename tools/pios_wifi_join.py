#!/usr/bin/env python3
"""Join a WiFi network using credentials from an ignored local config."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import re
import time
import urllib.parse
import urllib.request


def terminal_command(host: str, command: str, timeout: float) -> str:
    encoded = urllib.parse.quote(command, safe="")
    url = f"http://{host}/api/terminal?cmd={encoded}"
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read().decode("utf-8", "replace").strip()


def select_scan_target(scan: str, ssid: str) -> tuple[str, int] | None:
    best: tuple[int, str, int] | None = None
    for line in scan.splitlines():
        fields = line.split()
        if not fields or fields[0] != ssid:
            continue
        values: dict[str, str] = {}
        for field in fields[1:]:
            key, separator, value = field.partition("=")
            if separator:
                values[key] = value
        bssid = values.get("bssid", "")
        chanspec = values.get("cs", "")
        rssi = values.get("rssi", "")
        if not re.fullmatch(r"[0-9a-fA-F]{2}(?::[0-9a-fA-F]{2}){5}", bssid):
            continue
        try:
            candidate = (int(rssi), bssid, int(chanspec, 16))
        except ValueError:
            continue
        if best is None or candidate[0] > best[0]:
            best = candidate
    if best is None:
        return None
    return best[1], best[2]


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
    bssid = str(config.get("bssid", "")).replace(":", "").lower()
    chanspec_raw = config.get("chanspec", "")
    chanspec = (
        int(chanspec_raw, 0)
        if isinstance(chanspec_raw, str) and chanspec_raw
        else int(chanspec_raw or 0)
    )
    activate = args.activate or bool(config.get("activate", False))
    if not ssid or not password:
        raise SystemExit(
            f"Set non-empty ssid/password in {args.config}; "
            "copy tools/wifi_config.example.json first."
        )
    if any(ch.isspace() for ch in ssid) or any(ch.isspace() for ch in password):
        raise SystemExit("PIOS wifi join currently requires values without spaces.")

    fixed_target = len(bssid) == 12 and chanspec != 0
    join_bssid = bssid
    join_chanspec = chanspec
    print(terminal_command(host, "wifi init", args.timeout))
    if not args.no_scan:
        print(terminal_command(host, "wifi scan", args.timeout))
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            time.sleep(2.0)
            scan = terminal_command(host, "wifi results", args.timeout)
            target = select_scan_target(scan, ssid)
            if "WiFi scan count=" in scan and target:
                print(scan)
                join_bssid, join_chanspec = target
                fixed_target = True
                print(
                    "Using current scan target "
                    f"{join_bssid} chanspec=0x{join_chanspec:04x}"
                )
                break
        else:
            raise SystemExit(f"SSID {ssid!r} not found before scan timeout")
        # Reinitialize after scanning: this refreshes the SDPCM control window
        # before the targeted join while preserving the completed scan cache.
        print(terminal_command(host, "wifi init", args.timeout))

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
                host,
                f"wifi joinpmk {ssid} {pmk}"
                + (
                    f" {join_bssid} {join_chanspec:04x}"
                    if fixed_target
                    else ""
                ),
                args.timeout,
            )
        else:
            raise ValueError("mode must be 'wpa2' or 'wpa3'")
    except (OSError, TimeoutError) as exc:
        print(f"WiFi join request failed: {exc}")
        return 1
    print(result)
    if "WiFi join started" not in result and "WiFi join OK" not in result:
        return 1

    deadline = time.monotonic() + args.timeout
    stable_since: float | None = None
    while time.monotonic() < deadline:
        time.sleep(1.0)
        status = terminal_command(host, "wifi status", args.timeout)
        if " link=2 " in status:
            if stable_since is None:
                stable_since = time.monotonic()
            if time.monotonic() - stable_since >= 10.0:
                print("WiFi join OK (stable 10s)")
                break
            continue
        stable_since = None
        if " link=3 " in status:
            print(status)
            print(terminal_command(host, "wifi fwlog", args.timeout))
            return 1
    else:
        print(terminal_command(host, "wifi status", args.timeout))
        print(terminal_command(host, "wifi fwlog", args.timeout))
        return 1

    if activate:
        result = terminal_command(host, "wifi activate", args.timeout)
        print(result)
        if "WiFi activate OK" not in result:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
