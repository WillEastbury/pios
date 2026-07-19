#!/usr/bin/env python3
"""End-to-end test of the 'uartflash' protocol against QEMU's virt-machine
PL011 UART, exposed over a TCP socket (QEMU '-serial tcp:...') instead of
real hardware. PIOS's console UART already falls back to the standard virt
PL011 (PIOS_UART0_BASE=0x09000000) under QEMU, so this needs zero PIOS code
changes -- just a bidirectional serial backend instead of the smoke test's
write-only log file.

Tests both transfer modes:
  - FULL:  uartflash begin <n>  + raw chunks + ACK/TIMEOUT handling
  - PATCH: uartflash patchbegin + patch <off> <len> + finalize (the "zap"
           delta mode)
"""
from __future__ import annotations

import pathlib
import re
import socket
import subprocess
import sys
import time

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]")


def strip_ansi(s: str) -> str:
    return ANSI_RE.sub("", s)


REPO = pathlib.Path(__file__).resolve().parent.parent
QEMU = r"C:\Program Files\qemu\qemu-system-aarch64.exe"
KERNEL = REPO / "build_qemu_full" / "PIOS_QEMU_FULL.BIN"
UART_PORT = 4555
CHUNK_SIZE = 1024


class UartConn:
    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.buf = b""

    def send(self, data: bytes) -> None:
        self.sock.sendall(data)

    def read_line(self, timeout: float = 8.0) -> str:
        self.sock.settimeout(timeout)
        while b"\n" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise RuntimeError("connection closed")
            self.buf += chunk
        line, _, self.buf = self.buf.partition(b"\n")
        return strip_ansi(line.decode("ascii", "replace")).strip()


def wait_for_connect(port: int, timeout: float = 30.0) -> socket.socket:
    deadline = time.time() + timeout
    last_exc = None
    while time.time() < deadline:
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=2.0)
        except OSError as exc:
            last_exc = exc
            time.sleep(0.5)
    raise RuntimeError(f"could not connect to UART TCP port {port}: {last_exc}")


def drain_boot_banner(conn: UartConn) -> None:
    """Read and discard boot messages until the 'ready>' prompt appears."""
    deadline = time.time() + 40.0
    while time.time() < deadline:
        try:
            line = conn.read_line(timeout=5.0)
        except (socket.timeout, TimeoutError, RuntimeError):
            continue
        print(f"[boot] {line}")
        if "ready>" in line.lower() or line.strip().endswith(">"):
            return
    raise RuntimeError("never saw a console prompt")


def extract_reply(line: str, prefixes: tuple[str, ...]) -> str | None:
    """The console can emit a stale 'ready> ' prompt concatenated directly
    onto the front of a real reply with no separating newline. Rather than
    requiring an exact prefix match at position 0, find the reply prefix
    anywhere in the line and return from there."""
    for p in prefixes:
        idx = line.find(p)
        if idx >= 0:
            return line[idx:]
    return None


PROMPT = "ready>"


def send_cmd(conn: UartConn, cmd: str, read_timeout: float = 6.0) -> str:
    """Send a text command and return its reply across arbitrary TCP chunking.
    The console's no-newline, color-wrapped prompt may share a recv() chunk
    with the next command echo, and a delayed prior prompt may precede the
    current prompt. Rather than parsing line-by-line,
    accumulate raw bytes in conn.buf until the decoded text ends with the
    prompt marker, then strip every prompt occurrence and the echoed
    command text (found by exact substring, not a blind replace) in one
    shot over the whole blob."""
    conn.send((cmd + "\r\n").encode("ascii"))
    deadline = time.time() + read_timeout
    conn.sock.settimeout(0.3)
    while time.time() < deadline:
        text = strip_ansi(conn.buf.decode("ascii", "replace"))
        if text.rstrip().endswith(PROMPT):
            # Give any further immediately-pending bytes (e.g. a second,
            # late prompt from an earlier command) a brief moment to
            # arrive too, so they don't leak into the NEXT command's read.
            time.sleep(0.05)
            try:
                extra = conn.sock.recv(4096)
            except (socket.timeout, TimeoutError, OSError):
                extra = b""
            if extra:
                conn.buf += extra
                continue
            break
        try:
            chunk = conn.sock.recv(4096)
        except (socket.timeout, TimeoutError, OSError):
            continue
        if not chunk:
            break
        conn.buf += chunk
    text = strip_ansi(conn.buf.decode("ascii", "replace"))
    conn.buf = b""  # fully consumed for this command
    text = text.replace(PROMPT, "")
    idx = text.find(cmd)
    if idx >= 0:
        text = text[idx + len(cmd):]
    return text.strip()


def send_binary_with_ack(conn: UartConn, data: bytes, ack_prefix: str, max_retries: int = 5) -> str:
    for attempt in range(max_retries):
        conn.send(data)
        try:
            reply = conn.read_line(timeout=5.0)
        except (socket.timeout, TimeoutError):
            print(f"  [retry {attempt}] no reply, resending")
            continue
        extracted = extract_reply(reply, (ack_prefix, "TIMEOUT"))
        if extracted and extracted.startswith(ack_prefix):
            return extracted
        if extracted and extracted.startswith("TIMEOUT"):
            print(f"  [retry {attempt}] device reported {extracted}, resending")
            continue
        print(f"  [retry {attempt}] unexpected reply: {reply!r}, resending")
    raise RuntimeError(f"no {ack_prefix} after {max_retries} retries")


def test_full_mode(conn: UartConn, image: bytes) -> None:
    print("\n=== FULL mode test ===")
    total = len(image)
    reply = send_cmd(conn, f"uartflash begin {total}")
    print(f"[begin] {reply}")
    assert reply.startswith("READY"), f"begin failed: {reply}"

    sent = 0
    while sent < total:
        chunk = image[sent : sent + CHUNK_SIZE]
        reply = send_binary_with_ack(conn, chunk, "ACK")
        sent += len(chunk)
        print(f"[chunk] sent={sent}/{total} -> {reply}")

    reply = send_cmd(conn, "uartflash status")
    print(f"[status] {reply}")
    assert f"received={total}" in reply, f"incomplete transfer: {reply}"

    reply = send_cmd(conn, "uartflash commit")
    print(f"[commit] {reply}")
    assert reply.startswith("OK"), f"commit failed: {reply}"
    print("FULL mode: PASS")


def test_patch_mode(conn: UartConn, base_len: int, patches: list[tuple[int, bytes]], final_total: int) -> None:
    print("\n=== PATCH mode test ===")
    reply = send_cmd(conn, "uartflash abort")
    print(f"[abort-prior] {reply}")

    reply = send_cmd(conn, f"uartflash patchbegin {base_len}")
    print(f"[patchbegin] {reply}")
    assert reply.startswith("OK"), f"patchbegin failed: {reply}"

    for offset, data in patches:
        reply = send_cmd(conn, f"uartflash patch {offset} {len(data)}")
        print(f"[patch-cmd] offset={offset} len={len(data)} -> {reply}")
        assert reply.startswith("OK"), f"patch command failed: {reply}"
        reply = send_binary_with_ack(conn, data, "PACK")
        print(f"[patch-data] -> {reply}")
        expected_sum = 0
        for b in data:
            expected_sum = (expected_sum * 31 + b) & 0xFFFFFFFF
        assert reply == f"PACK {offset} {expected_sum:08x}", (
            f"checksum mismatch for offset {offset}: {reply} (expected PACK {offset} {expected_sum:08x})")

    reply = send_cmd(conn, f"uartflash finalize {final_total}")
    print(f"[finalize] {reply}")
    assert reply.startswith("OK"), f"finalize failed: {reply}"

    reply = send_cmd(conn, "uartflash commit")
    print(f"[commit] {reply}")
    assert reply.startswith("OK"), f"commit failed: {reply}"
    print("PATCH mode: PASS")


def main() -> int:
    if not KERNEL.exists():
        print(f"[test] kernel not found: {KERNEL} (run build_qemu_full.bat first)")
        return 2

    net = ("user,id=n0,net=192.168.0.0/24,host=192.168.0.1,"
           "hostfwd=tcp:127.0.0.1:8088-192.168.0.201:80")
    qargs = [
        QEMU, "-M", "virt", "-cpu", "cortex-a53", "-smp", "4", "-m", "1G",
        "-display", "none",
        "-serial", f"tcp:127.0.0.1:{UART_PORT},server,nowait",
        "-kernel", str(KERNEL), "-netdev", net,
        "-device", "virtio-net-device,netdev=n0",
    ]
    print("[test] booting QEMU with TCP-backed UART...")
    proc = subprocess.Popen(qargs)
    try:
        sock = wait_for_connect(UART_PORT)
        conn = UartConn(sock)
        drain_boot_banner(conn)

        image = bytes((i * 37 + 11) & 0xFF for i in range(37000))  # synthetic ~36KB test image
        test_full_mode(conn, image)

        # Patch mode: base = the image we just committed, with 2 small
        # regions deliberately altered, to confirm sparse-region patching
        # correctly merges onto a real read-back-from-SD base.
        patched = bytearray(image)
        patches = []
        for offset in (100, 20000):
            new_bytes = bytes((b ^ 0xFF) for b in patched[offset : offset + 256])
            patched[offset : offset + 256] = new_bytes
            patches.append((offset, new_bytes))
        test_patch_mode(conn, len(image), patches, len(patched))

        print("\nALL TESTS PASSED")
        return 0
    except Exception as exc:
        print(f"\nTEST FAILED: {exc}")
        return 1
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
