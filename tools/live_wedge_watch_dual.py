#!/usr/bin/env python3
"""Dual-channel live wedge watcher: polls HTTP (rxdiag/irq status)
and, whenever HTTP is unreachable, falls back to a UART probe on the same
cadence so we get continuous coverage straight through a wedge boundary
(HTTP dies -> UART stays alive -> HTTP recovers). Everything is timestamped
and appended to a log file for later correlation against rx_wedge /
rx_live_recover / per_core irq counters.

Usage: live_wedge_watch_dual.py [COM5] [115200] [seconds]
"""
import sys
import time
import urllib.request
import serial

HOST = "192.168.0.201"
PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
DURATION = float(sys.argv[3]) if len(sys.argv) > 3 else 300.0
LOG = "tools/wedge_watch.log"

CMDS = ["rxdiag", "irq status"]


def uart_fetch(sp, cmd):
    try:
        sp.reset_input_buffer()
        sp.write((cmd + "\r\n").encode())
        sp.flush()
        deadline = time.time() + 1.5
        got = bytearray()
        while time.time() < deadline:
            chunk = sp.read(4096)
            if chunk:
                got += chunk
                deadline = time.time() + 0.4
        text = got.decode("utf-8", "replace")
        lines = []
        for raw in text.splitlines():
            line = raw.strip()
            if not line or line == "ready>" or line == f"ready> {cmd}":
                continue
            lines.append(line)
        return "\n".join(lines) or None
    except Exception:
        return None


def main():
    try:
        sp = serial.Serial(PORT, BAUD, timeout=0.3)
        sp.dtr = True
        sp.rts = True
        time.sleep(0.2)
    except Exception as exc:
        print(f"UART open failed: {exc} (HTTP-only mode)")
        sp = None

    t0 = time.time()
    last_mode = None
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(f"\n=== watch start {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
        f.flush()
        while time.time() - t0 < DURATION:
            ts = time.strftime("%H:%M:%S")
            http_ok = False
            for cmd in CMDS:
                url = f"http://{HOST}/api/terminal?cmd={cmd.replace(' ', '%20')}"
                try:
                    with urllib.request.urlopen(url, timeout=1.5) as r:
                        body = r.read().decode("ascii", "replace").strip()
                    http_ok = True
                    line = f"[{ts}] HTTP {cmd}: {body}"
                    print(line)
                    f.write(line + "\n")
                except Exception as exc:
                    line = f"[{ts}] HTTP {cmd}: UNREACHABLE ({exc})"
                    print(line)
                    f.write(line + "\n")
                    if sp is not None:
                        val = uart_fetch(sp, cmd)
                        uline = f"[{ts}] UART {cmd}: {val if val else '(no response)'}"
                        print(uline)
                        f.write(uline + "\n")
            mode = "HTTP-UP" if http_ok else "HTTP-DOWN"
            if mode != last_mode:
                marker = f"[{ts}] *** transition -> {mode} ***"
                print(marker)
                f.write(marker + "\n")
                last_mode = mode
            f.flush()
            time.sleep(2.0)
    print(f"[watch] done, log at {LOG}")


if __name__ == "__main__":
    main()
