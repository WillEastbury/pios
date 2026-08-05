#!/usr/bin/env python3
"""Stream CYW firmware resources into a running PIOS image."""

from __future__ import annotations

import argparse
import pathlib
import socket
import time


def upload(host: str, port: int, kind: str, path: pathlib.Path,
           timeout: float) -> None:
    data = path.read_bytes()
    target = (
        f"/api/admin/wifi-stream?confirm=1&kind={kind}&total={len(data)}"
    )
    header = (
        f"POST {target} HTTP/1.0\r\n"
        f"Host: {host}\r\n"
        "Connection: close\r\n"
        "Content-Type: application/octet-stream\r\n"
        f"Content-Length: {len(data)}\r\n"
        "\r\n"
    ).encode("ascii")
    print(f"[wifi] streaming {kind} {len(data)} bytes...")
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.sendall(header)
        sent = 0
        last = time.time()
        while sent < len(data):
            end = min(sent + 2048, len(data))
            sock.sendall(data[sent:end])
            sent = end
            if time.time() - last > 2:
                print(f"[wifi] {kind} {sent}/{len(data)}")
                last = time.time()
        response = bytearray()
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            response.extend(chunk)
    body = bytes(response).split(b"\r\n\r\n", 1)[-1]
    print(f"[wifi] {kind}: {body.decode('utf-8', 'replace').strip()}")
    if b'"ok":true' not in body:
        raise RuntimeError(f"{kind} upload failed")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.0.201")
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument("--dir", type=pathlib.Path,
                        default=pathlib.Path("wifi_fw_staging"))
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()
    for kind, name in (
        ("firmware", "firmware.bin"),
        ("nvram", "nvram.txt"),
        ("clm", "clm.bin"),
    ):
        upload(args.host, args.port, kind, args.dir / name, args.timeout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
