#!/usr/bin/env python3
"""Real-hardware 'uartflash' client: deploys a PIOS kernel image over a
physical serial port (the Raspberry Pi Debug Probe's UART bridge), reusing
the same FULL-mode protocol validated end-to-end against QEMU's TCP-backed
UART in tools/uartflash_qemu_test.py.

Bypasses the network entirely -- useful when a bulk network OTA transfer is
unreliable (the still-unresolved NIC RX wedge under load), since the console
UART path is a completely independent code path from the network stack.
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys
import time

import serial  # pyserial

ANSI_RE = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]")


def strip_ansi(s: str) -> str:
    return ANSI_RE.sub("", s)


PROMPT = "ready>"
CHUNK_SIZE = 1024


class SerialConn:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.3)
        self.ser.dtr = True
        self.ser.rts = True
        self.buf = b""

    def send(self, data: bytes) -> None:
        self.ser.write(data)
        self.ser.flush()

    def close(self):
        self.ser.close()


def send_cmd(conn: SerialConn, cmd: str, read_timeout: float = 8.0) -> str:
    conn.send((cmd + "\r\n").encode("ascii"))
    deadline = time.time() + read_timeout
    while time.time() < deadline:
        text = strip_ansi(conn.buf.decode("ascii", "replace"))
        if text.rstrip().endswith(PROMPT):
            time.sleep(0.05)
            extra = conn.ser.read(4096)
            if extra:
                conn.buf += extra
                continue
            break
        chunk = conn.ser.read(4096)
        if chunk:
            conn.buf += chunk
    text = strip_ansi(conn.buf.decode("ascii", "replace"))
    conn.buf = b""
    text = text.replace(PROMPT, "")
    idx = text.find(cmd)
    if idx >= 0:
        text = text[idx + len(cmd):]
    return text.strip()


def send_binary_with_ack(conn: SerialConn, data: bytes, ack_prefix: str, max_retries: int = 8) -> str:
    for attempt in range(max_retries):
        conn.send(data)
        deadline = time.time() + 8.0
        raw = b""
        while time.time() < deadline:
            chunk = conn.ser.read(4096)
            if chunk:
                raw += chunk
                text = strip_ansi(raw.decode("ascii", "replace"))
                if "\n" in text or ack_prefix in text or "TIMEOUT" in text:
                    break
        text = strip_ansi(raw.decode("ascii", "replace")).strip()
        if ack_prefix in text:
            idx = text.find(ack_prefix)
            return text[idx:].splitlines()[0].strip()
        if "TIMEOUT" in text:
            print(f"  [retry {attempt}] device reported TIMEOUT, resending", file=sys.stderr)
            continue
        print(f"  [retry {attempt}] no/unexpected reply: {text!r}, resending", file=sys.stderr)
    raise RuntimeError(f"no {ack_prefix} after {max_retries} retries")


def drain_prompt(conn: SerialConn) -> None:
    conn.send(b"\r\n")
    time.sleep(0.5)
    conn.buf += conn.ser.read(4096)
    conn.buf = b""


def flash_full(conn: SerialConn, image: bytes, reboot: bool) -> None:
    total = len(image)
    print(f"[uartflash] deploying {total} bytes over serial (FULL mode)...")
    reply = send_cmd(conn, f"uartflash begin {total}")
    print(f"[begin] {reply}")
    assert reply.startswith("READY"), f"begin failed: {reply}"

    sent = 0
    last_print = time.time()
    while sent < total:
        chunk = image[sent: sent + CHUNK_SIZE]
        reply = send_binary_with_ack(conn, chunk, "ACK")
        sent += len(chunk)
        if time.time() - last_print > 3.0 or sent >= total:
            print(f"[chunk] sent={sent}/{total} -> {reply}")
            last_print = time.time()

    reply = send_cmd(conn, "uartflash status")
    print(f"[status] {reply}")
    assert f"received={total}" in reply, f"incomplete transfer: {reply}"

    reply = send_cmd(conn, "uartflash commit" + (" reboot" if reboot else ""))
    print(f"[commit] {reply}")
    assert reply.startswith("OK"), f"commit failed: {reply}"
    print("[uartflash] DONE")


def main() -> int:
    ap = argparse.ArgumentParser(description="Deploy a PIOS kernel image over the physical UART console.")
    ap.add_argument("image", type=pathlib.Path, nargs="?", default=pathlib.Path("real_kernel.img"))
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--reboot", action="store_true", help="reboot into the new image after commit")
    args = ap.parse_args()

    image = args.image.read_bytes()
    if not image:
        raise SystemExit("image is empty")

    conn = SerialConn(args.port, args.baud)
    try:
        drain_prompt(conn)
        flash_full(conn, image, args.reboot)
        return 0
    except Exception as exc:
        print(f"\n[uartflash] FAILED: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.close()


if __name__ == "__main__":
    raise SystemExit(main())
