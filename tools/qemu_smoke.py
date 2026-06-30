#!/usr/bin/env python3
"""
PIOS QEMU smoke-test harness.

Boots the full-parity PIOS kernel under qemu-system-aarch64 (headless, with
user-mode networking + virtio-net), waits for the network stack to come up, then
runs a battery of HTTP assertions against the running OS and reports PASS/FAIL.
Exits 0 only if every assertion passes, so it can gate a hardware deploy.

Assertions:
  - /api/status returns 200 with a version string
  - `selftest` battery reports ALL PASS
  - `stackdiag` shows a healthy TCP/IP data plane
  - `dtrace` enable + dump works
  - `schedquanta` reports the expected quantum
  - `cdump` snapshot + diff works
  - an OTA stream upload completes with {"ok":true}
  - a 20-request HTTP burst is 20/20

Usage:
  python tools/qemu_smoke.py [--build] [--kernel PATH] [--keep-log]
"""

from __future__ import annotations

import argparse
import pathlib
import socket
import subprocess
import sys
import time
import urllib.parse
import urllib.request

REPO = pathlib.Path(__file__).resolve().parent.parent
QEMU = r"C:\Program Files\qemu\qemu-system-aarch64.exe"
KERNEL = REPO / "build_qemu_full" / "PIOS_QEMU_FULL.BIN"
HTTP = "http://127.0.0.1:8088"
OTA_HOST, OTA_PORT = "127.0.0.1", 8082
SERIAL_LOG = REPO / "qemu_smoke_serial.log"

GREEN, RED, DIM, RST = "\033[32m", "\033[31m", "\033[2m", "\033[0m"


def term(cmd: str, timeout: float = 8.0) -> str:
    url = f"{HTTP}/api/terminal?cmd={urllib.parse.quote(cmd)}"
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read().decode("utf-8", "replace")


def get(path: str, timeout: float = 8.0):
    with urllib.request.urlopen(f"{HTTP}{path}", timeout=timeout) as r:
        return r.status, r.read().decode("utf-8", "replace")


def ota_stream(image: bytes, timeout: float = 60.0) -> str:
    total = len(image)
    q = f"/api/admin/kernel-stream?confirm=1&total={total}"
    header = (
        f"POST {q} HTTP/1.0\r\nHost: {OTA_HOST}\r\nConnection: close\r\n"
        f"Content-Type: application/octet-stream\r\nContent-Length: {total}\r\n\r\n"
    ).encode("ascii")
    with socket.create_connection((OTA_HOST, OTA_PORT), timeout=timeout) as s:
        s.settimeout(timeout)
        s.sendall(header)
        sent = 0
        while sent < total:
            end = min(sent + 2048, total)
            s.sendall(image[sent:end])
            sent = end
        resp = b""
        while b"\r\n\r\n" not in resp or len(resp) < 64:
            c = s.recv(4096)
            if not c:
                break
            resp += c
        return resp.decode("utf-8", "replace")


class Smoke:
    def __init__(self) -> None:
        self.passed = 0
        self.failed = 0

    def check(self, name: str, ok: bool, detail: str = "") -> None:
        tag = f"{GREEN}PASS{RST}" if ok else f"{RED}FAIL{RST}"
        line = f"  [{tag}] {name}"
        if detail:
            line += f"  {DIM}{detail}{RST}"
        print(line)
        if ok:
            self.passed += 1
        else:
            self.failed += 1

    def run(self) -> int:
        # status
        try:
            code, body = get("/api/status")
            self.check("status 200 + version",
                       code == 200 and '"version"' in body,
                       body[:60].replace("\n", " "))
        except Exception as e:
            self.check("status 200 + version", False, str(e))

        # selftest battery
        try:
            out = term("selftest", timeout=20)
            self.check("selftest battery ALL PASS", "ALL PASS" in out,
                       out.strip().splitlines()[-1] if out.strip() else "")
            if "ALL PASS" not in out:
                for ln in out.splitlines():
                    if ln.startswith("FAIL"):
                        print(f"      {RED}{ln}{RST}")
        except Exception as e:
            self.check("selftest battery ALL PASS", False, str(e))

        # stackdiag health
        try:
            out = term("stackdiag")
            self.check("stackdiag TCP/IP healthy",
                       "TCP" in out and "IP" in out and "FIFO" in out)
        except Exception as e:
            self.check("stackdiag TCP/IP healthy", False, str(e))

        # dtrace
        try:
            term("dtrace on")
            term("dtrace clear")
            d = term("dtrace dump 5")
            self.check("dtrace enable+dump", "dtrace ON" in d)
        except Exception as e:
            self.check("dtrace enable+dump", False, str(e))

        # schedquanta
        try:
            out = term("schedquanta")
            self.check("schedquanta quantum=5ms", "quantum_ms=5" in out)
        except Exception as e:
            self.check("schedquanta quantum=5ms", False, str(e))

        # cdump snapshot + diff
        try:
            term("cdump snap a")
            time.sleep(0.2)
            term("cdump snap b")
            out = term("cdump diff")
            self.check("cdump snapshot+diff", "diff A->B" in out)
        except Exception as e:
            self.check("cdump snapshot+diff", False, str(e))

        # OTA stream (small synthetic image well under the 2MB staging zone)
        try:
            img = bytes((i * 7 + 3) & 0xFF for i in range(64 * 1024))
            resp = ota_stream(img)
            self.check("OTA stream upload ok", '"ok":true' in resp,
                       resp.strip().splitlines()[-1] if resp.strip() else "")
        except Exception as e:
            self.check("OTA stream upload ok", False, str(e))

        # HTTP burst
        ok = 0
        for _ in range(20):
            try:
                if get("/api/status", timeout=4)[0] == 200:
                    ok += 1
            except Exception:
                pass
        self.check("HTTP burst 20/20", ok == 20, f"{ok}/20")

        total = self.passed + self.failed
        color = GREEN if self.failed == 0 else RED
        print(f"\n{color}SMOKE: {self.passed}/{total} passed{RST}")
        return 0 if self.failed == 0 else 1


def wait_boot(timeout: float = 60.0) -> bool:
    start = time.time()
    while time.time() - start < timeout:
        try:
            if get("/api/status", timeout=3)[0] == 200:
                return True
        except Exception:
            time.sleep(2)
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", action="store_true", help="build the QEMU kernel first")
    ap.add_argument("--kernel", default=str(KERNEL))
    ap.add_argument("--keep-log", action="store_true")
    args = ap.parse_args()

    if args.build:
        print("[smoke] building QEMU kernel...")
        r = subprocess.run(["cmd", "/c", str(REPO / "build_qemu_full.bat")],
                           cwd=REPO, capture_output=True, text=True)
        if r.returncode != 0 or "BUILD COMPLETE" not in r.stdout:
            print(r.stdout[-2000:])
            print("[smoke] build FAILED")
            return 2

    if not pathlib.Path(args.kernel).exists():
        print(f"[smoke] kernel not found: {args.kernel} (use --build)")
        return 2

    net = ("user,id=n0,net=192.168.0.0/24,host=192.168.0.1,"
           "hostfwd=tcp:127.0.0.1:8088-192.168.0.201:80,"
           "hostfwd=tcp:127.0.0.1:8082-192.168.0.201:8082")
    qargs = [QEMU, "-M", "virt", "-cpu", "cortex-a53", "-smp", "4", "-m", "1G",
             "-display", "none", "-serial", f"file:{SERIAL_LOG}",
             "-kernel", args.kernel, "-netdev", net,
             "-device", "virtio-net-device,netdev=n0"]
    print("[smoke] booting QEMU (headless)...")
    proc = subprocess.Popen(qargs)
    rc = 3
    try:
        if not wait_boot():
            print(f"{RED}[smoke] board never reached /api/status (boot hang){RST}")
            return 3
        print("[smoke] board up — running assertions:\n")
        rc = Smoke().run()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        if rc == 0 and not args.keep_log:
            SERIAL_LOG.unlink(missing_ok=True)
        elif rc != 0:
            print(f"[smoke] serial log: {SERIAL_LOG}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
