#!/usr/bin/env python3
"""End-to-end TCP-2323 debugger/console regression test under QEMU."""

from __future__ import annotations

import pathlib
import socket
import subprocess
import sys
import time


REPO = pathlib.Path(__file__).resolve().parent.parent
QEMU = r"C:\Program Files\qemu\qemu-system-aarch64.exe"
KERNEL = REPO / "build_qemu_full" / "PIOS_QEMU_FULL.BIN"
DEBUG_PORT = 4557


class DebugConn:
    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self.buf = b""

    def read_until(self, marker: bytes, timeout: float = 12.0) -> str:
        deadline = time.time() + timeout
        self.sock.settimeout(0.25)
        while marker not in self.buf and time.time() < deadline:
            try:
                chunk = self.sock.recv(4096)
            except (socket.timeout, TimeoutError):
                continue
            if not chunk:
                break
            self.buf += chunk
        if marker not in self.buf:
            raise RuntimeError(f"timeout waiting for {marker!r}: {self.buf[-300:]!r}")
        end = self.buf.index(marker) + len(marker)
        out, self.buf = self.buf[:end], self.buf[end:]
        return out.decode("ascii", "replace")

    def command(self, data: bytes, timeout: float = 15.0) -> str:
        self.sock.sendall(data)
        return self.read_until(b"pios> ", timeout)


def connect_with_retry(port: int, timeout: float = 45.0) -> socket.socket:
    deadline = time.time() + timeout
    last_error: OSError | None = None
    while time.time() < deadline:
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=2.0)
        except OSError as exc:
            last_error = exc
            time.sleep(0.5)
    raise RuntimeError(f"could not connect to TCP debug console: {last_error}")


def main() -> int:
    if not KERNEL.exists():
        print(f"[test] kernel not found: {KERNEL}")
        return 2

    net = (
        "user,id=n0,net=192.168.0.0/24,host=192.168.0.1,"
        f"hostfwd=tcp:127.0.0.1:{DEBUG_PORT}-192.168.0.201:2323"
    )
    qargs = [
        QEMU, "-M", "virt", "-cpu", "cortex-a53", "-smp", "4", "-m", "1G",
        "-display", "none", "-serial", "null", "-kernel", str(KERNEL),
        "-netdev", net, "-device", "virtio-net-device,netdev=n0",
    ]
    proc = subprocess.Popen(qargs)
    failures: list[str] = []
    try:
        conn = DebugConn(connect_with_retry(DEBUG_PORT))
        banner = conn.read_until(b"debug> ")
        print("[PASS] debug banner" if "unlock pios" in banner else "[FAIL] debug banner")
        if "unlock pios" not in banner:
            failures.append("banner")

        # Bare CR must execute, not wait forever for LF.
        conn.sock.sendall(b"status\r")
        locked = conn.read_until(b"debug> ")
        locked_ok = "unlock pios" in locked
        print(f"[{'PASS' if locked_ok else 'FAIL'}] locked command rejection via bare CR")
        if not locked_ok:
            failures.append("locked-cr")

        conn.sock.sendall(b"unlock pios\r")
        unlocked = conn.read_until(b"pios> ")
        unlock_ok = "debug console unlocked" in unlocked
        print(f"[{'PASS' if unlock_ok else 'FAIL'}] unlock via bare CR")
        if not unlock_ok:
            failures.append("unlock-cr")

        checks = [
            (b"status\r", "PIOS", "status"),
            (b"help\r\n", "PIOS help", "CRLF pairing"),
            (b"core status\r", "CORE PSCI_RET STAGE", "multicore status"),
            (b"irq status\r", "irq total=", "IRQ status"),
            (b"ipc bench 64\r", "fifo_irq_delta=", "FIFO IRQ benchmark"),
            (b"time status\r", "elapsed_ticks=", "nested command timing"),
            (b"statuX\x08s\r", "PIOS", "backspace editing"),
        ]
        for payload, expected, name in checks:
            out = conn.command(payload, timeout=30.0)
            ok = expected in out
            print(f"[{'PASS' if ok else 'FAIL'}] {name}")
            if not ok:
                failures.append(name)
                print(f"       reply: {out[-500:]!r}")

        long_reply = conn.command((b"x" * 300) + b"\r")
        long_ok = "line too long" in long_reply and long_reply.endswith("pios> ")
        print(f"[{'PASS' if long_ok else 'FAIL'}] overlong-line recovery")
        if not long_ok:
            failures.append("line-too-long")

        out = conn.command(b"break\r")
        break_ok = "requested freeze on 3 core(s)" in out
        freeze_ok = False
        for _ in range(15):
            state = conn.command(b"freeze status\r")
            freeze_ok = break_ok and all(
                f"core{core}: requested=1 frozen=1" in state for core in (1, 2, 3)
            )
            if freeze_ok:
                break
            time.sleep(0.2)
        print(f"[{'PASS' if freeze_ok else 'FAIL'}] all-core freeze over TCP")
        if not freeze_ok:
            failures.append("freeze")

        regs = conn.command(b"regs 3\r")
        regs_ok = "core3 frozen at tick=" in regs and "x30=" in regs
        print(f"[{'PASS' if regs_ok else 'FAIL'}] register inspection over TCP")
        if not regs_ok:
            failures.append("regs")

        conn.command(b"resume\r")
        resume_ok = False
        for _ in range(15):
            state = conn.command(b"freeze status\r")
            resume_ok = all(
                f"core{core}: requested=0 frozen=0" in state for core in (1, 2, 3)
            )
            if resume_ok:
                break
            time.sleep(0.2)
        print(f"[{'PASS' if resume_ok else 'FAIL'}] all-core resume over TCP")
        if not resume_ok:
            failures.append("resume")

        if failures:
            print(f"\nFAILED: {failures}")
            return 1
        print("\nALL TCP DEBUG CONSOLE TESTS PASSED")
        return 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    sys.exit(main())
