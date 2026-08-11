"""Fire a WPA2 join and watch board liveness/logs while it runs.

Diagnostic helper for the issue #76 investigation: the join handler blocks
inside the HTTP request, so a normal client call cannot observe whether the
board resets mid-handshake. This issues the join on a worker thread and polls
/api/status uptime plus the log stream from the main thread.
"""
from __future__ import annotations

import hashlib
import json
import pathlib
import threading
import time
import urllib.parse
import urllib.request

CONFIG = pathlib.Path("tools/wifi_config.local.json")


def cmd(host: str, command: str, timeout: float) -> str:
    url = f"http://{host}/api/terminal?cmd=" + urllib.parse.quote(command, safe="")
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read().decode("utf-8", "replace").strip()


def get(host: str, path: str, timeout: float) -> str:
    with urllib.request.urlopen(f"http://{host}{path}", timeout=timeout) as r:
        return r.read().decode("utf-8", "replace").strip()


def main() -> int:
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    host = str(config.get("host", "192.168.0.201"))
    ssid = str(config["ssid"])
    bssid = str(config.get("bssid", "")).replace(":", "").lower()
    chanspec = int(str(config.get("chanspec", "0")), 0)
    pmk = hashlib.pbkdf2_hmac(
        "sha1", str(config["password"]).encode(), ssid.encode(), 4096, 32
    ).hex()

    join = f"wifi joinpmk {ssid} {pmk}"
    if len(bssid) == 12 and chanspec:
        join += f" {bssid} {chanspec:04x}"

    result: dict[str, str] = {}

    def worker() -> None:
        try:
            result["out"] = cmd(host, join, 120.0)
        except Exception as exc:  # noqa: BLE001 - diagnostic surface
            result["out"] = f"<join call failed: {exc}>"

    t = threading.Thread(target=worker, daemon=True)
    t.start()

    prev_uptime = None
    for _ in range(60):
        time.sleep(2.0)
        try:
            up = json.loads(get(host, "/api/status", 5.0)).get("uptime")
        except Exception as exc:  # noqa: BLE001 - board may be mid-reset
            print(f"  [poll] unreachable: {exc}")
            continue
        marker = ""
        if prev_uptime is not None and up < prev_uptime:
            marker = "  <-- BOARD RESET"
        print(f"  [poll] uptime={up}{marker}")
        prev_uptime = up
        if not t.is_alive():
            break

    t.join(timeout=5.0)
    print("join result:", result.get("out", "<still running>"))
    for name in ("wifi status", "wifi wpadiag", "wifi fwlog"):
        try:
            print(f"--- {name} ---")
            print(cmd(host, name, 30.0))
        except Exception as exc:  # noqa: BLE001 - diagnostic surface
            print(f"{name} failed: {exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
