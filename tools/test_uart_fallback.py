#!/usr/bin/env python3
"""Regression test for the UART/F3 console's shared-command fallback:
confirms ui_console_exec_shared_fallback() correctly routes commands that
only exist in the HTTP terminal's big dispatcher (http_exec_terminal_command,
src/kernel.c) through to the UART console, and that genuinely-unknown
commands still produce "ERR: unknown command" via the same shared handler.

Deliberately avoids any 'rp1 irq *' / hardware-MMIO-poking variants here:
those touch real RP1/MACB registers that don't exist under QEMU's virt
platform and are not expected to be QEMU-safe (a pre-existing property of
those specific commands, unrelated to the shared-dispatch mechanism itself).
"""
from __future__ import annotations

import re
import socket
import subprocess
import sys
import time

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]")
QEMU = r"C:\Program Files\qemu\qemu-system-aarch64.exe"
KERNEL = r"C:\source\pios\build_qemu_full\PIOS_QEMU_FULL.BIN"
PORT = 4556


def strip_ansi(s): return ANSI_RE.sub("", s)


def main():
    qargs = [
        QEMU, "-M", "virt", "-cpu", "cortex-a53", "-smp", "4", "-m", "1G",
        "-display", "none",
        "-serial", f"tcp:127.0.0.1:{PORT},server,nowait",
        "-kernel", KERNEL,
        "-netdev", "user,id=n0", "-device", "virtio-net-device,netdev=n0",
    ]
    proc = subprocess.Popen(qargs)
    try:
        deadline = time.time() + 30
        sock = None
        while time.time() < deadline:
            try:
                sock = socket.create_connection(("127.0.0.1", PORT), timeout=2.0)
                break
            except OSError:
                time.sleep(0.5)
        assert sock, "could not connect"
        buf = b""
        sock.settimeout(0.3)
        deadline = time.time() + 40
        while time.time() < deadline:
            text = strip_ansi(buf.decode("ascii", "replace"))
            if text.rstrip().endswith("ready>"):
                break
            try:
                chunk = sock.recv(4096)
                if chunk:
                    buf += chunk
            except (socket.timeout, TimeoutError):
                continue
        print("[boot] prompt reached")

        checks = {
            "boguscmdxyz": "ERR: unknown command",
            "macbdiag": "macbdiag unavailable",
            "irq status": "irq total=",
            "stackdiag": "IP   rx=",
        }
        failures = []
        for cmd, expect in checks.items():
            sock.settimeout(0.3)
            try:
                while True:
                    junk = sock.recv(4096)
                    if not junk:
                        break
            except (socket.timeout, TimeoutError):
                pass
            sock.sendall((cmd + "\r\n").encode())
            time.sleep(1.5)
            resp = b""
            rdeadline = time.time() + 6
            while time.time() < rdeadline:
                try:
                    chunk = sock.recv(4096)
                    if chunk:
                        resp += chunk
                except (socket.timeout, TimeoutError):
                    break
            text = strip_ansi(resp.decode("ascii", "replace"))
            ok = expect in text
            print(f"[{'PASS' if ok else 'FAIL'}] {cmd!r} -> expected {expect!r} in reply")
            if not ok:
                failures.append(cmd)
                print(f"       raw: {text!r}")

        if failures:
            print(f"\nFAILED: {failures}")
            return 1
        print("\nALL CHECKS PASSED")
        return 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    sys.exit(main())

