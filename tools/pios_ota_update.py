#!/usr/bin/env python3
"""Chunked OTA updater for PIOS raw stage1 slot."""

from __future__ import annotations

import argparse
import json
import pathlib
import socket
import sys
import time
import urllib.parse


def request(host: str, port: int, method: str, path: str, body: bytes = b"", timeout: float = 5.0) -> tuple[int, str]:
    req = (
        f"{method} {path} HTTP/1.0\r\n"
        f"Host: {host}\r\n"
        "Connection: close\r\n"
        f"Content-Length: {len(body)}\r\n"
        "\r\n"
    ).encode("ascii") + body
    data = bytearray()
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(req)
        header_done = False
        body_start = 0
        saw_body = False
        while True:
            try:
                chunk = sock.recv(4096)
            except TimeoutError:
                break
            if not chunk:
                break
            data.extend(chunk)
            if not header_done:
                marker = data.find(b"\r\n\r\n")
                if marker < 0:
                    continue
                header_done = True
                body_start = marker + 4
            response_body = bytes(data[body_start:])
            if response_body and not saw_body:
                saw_body = True
                sock.settimeout(min(timeout, 0.25))
            if response_body.startswith(b"{") and response_body.endswith(b"\n"):
                break
            if response_body.startswith(b"PIOS log-stream") and len(response_body) > 512:
                break

    raw = bytes(data)
    marker = raw.find(b"\r\n\r\n")
    if marker < 0:
        raise RuntimeError(f"incomplete HTTP response from {port}{path}: {raw[:120]!r}")
    status_line = raw.split(b"\r\n", 1)[0].decode("ascii", "replace")
    parts = status_line.split()
    status = int(parts[1]) if len(parts) >= 2 and parts[1].isdigit() else 0
    return status, raw[marker + 4 :].decode("utf-8", "replace")


def request_json(host: str, port: int, method: str, path: str, body: bytes = b"", timeout: float = 5.0) -> dict:
    status, text = request(host, port, method, path, body, timeout)
    if status != 200:
        raise RuntimeError(f"HTTP {status} from {port}{path}: {text[:200]}")
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"non-JSON response from {port}{path}: {text[:300]}") from exc
    if not payload.get("ok"):
        raise RuntimeError(f"OTA request failed: {payload}")
    return payload


def update_path(**params: object) -> str:
    params["confirm"] = "1"
    return "/?" + urllib.parse.urlencode(params)


def fetch_logs(host: str, port: int, since: int, timeout: float) -> int:
    status, text = request(host, port, "GET", f"/logs?since={since}", timeout=timeout)
    if status != 200:
        print(f"[logs] HTTP {status}: {text[:120]}", file=sys.stderr)
        return since
    next_seq = since
    for line in text.splitlines():
        if line.startswith("PIOS log-stream next="):
            try:
                next_seq = int(line.rsplit("=", 1)[1])
            except ValueError:
                pass
        elif line:
            print("[pios]", line)
    return next_seq


def main() -> int:
    ap = argparse.ArgumentParser(description="Upload a PIOS real_kernel.img to the OTA raw slot.")
    ap.add_argument("image", type=pathlib.Path, nargs="?", default=pathlib.Path("real_kernel.img"))
    ap.add_argument("--host", default="192.168.0.101")
    ap.add_argument("--update-port", type=int, default=8082)
    ap.add_argument("--status-port", type=int, default=8080)
    ap.add_argument("--reboot-port", type=int, default=8081)
    ap.add_argument("--chunk-size", type=int, default=4096)
    ap.add_argument("--log-every", type=int, default=16)
    ap.add_argument("--timeout", type=float, default=5.0)
    ap.add_argument("--reboot", action="store_true", help="Call the reboot port after a successful commit.")
    args = ap.parse_args()

    image = args.image.read_bytes()
    total = len(image)
    if total == 0:
        raise SystemExit("image is empty")
    if args.chunk_size <= 0 or args.chunk_size % 512 != 0:
        raise SystemExit("--chunk-size must be a positive multiple of 512")

    log_seq = fetch_logs(args.host, args.status_port, 0, args.timeout)
    begin = request_json(
        args.host,
        args.update_port,
        "POST",
        update_path(action="begin", total=total),
        timeout=args.timeout,
    )
    print(f"[ota] begin total={total} capacity={begin.get('capacity')} slotLba={begin.get('slotLba')}")

    offset = int(begin.get("nextOffset", 0))
    while offset < total:
        chunk = image[offset : offset + args.chunk_size]
        for attempt in range(1, 4):
            try:
                rsp = request_json(
                    args.host,
                    args.update_port,
                    "POST",
                    update_path(action="chunk", offset=offset, total=total),
                    chunk,
                    timeout=args.timeout,
                )
                next_offset = int(rsp.get("nextOffset", offset + len(chunk)))
                if next_offset < offset:
                    raise RuntimeError(f"server moved backwards: {rsp}")
                offset = next_offset
                break
            except Exception as exc:
                if attempt == 3:
                    raise
                print(f"[ota] retry offset={offset} attempt={attempt}: {exc}", file=sys.stderr)
                time.sleep(0.5)
                status = request_json(
                    args.host,
                    args.update_port,
                    "GET",
                    update_path(action="status"),
                    timeout=args.timeout,
                )
                offset = int(status.get("nextOffset", offset))
        chunk_index = (offset + args.chunk_size - 1) // args.chunk_size
        if offset == total or (args.log_every > 0 and (chunk_index % args.log_every) == 0):
            print(f"[ota] {offset}/{total}")
            log_seq = fetch_logs(args.host, args.status_port, log_seq, args.timeout)

    commit = request_json(
        args.host,
        args.update_port,
        "POST",
        update_path(action="commit", total=total),
        timeout=args.timeout,
    )
    print(f"[ota] committed received={commit.get('received')} commits={commit.get('commits')}")
    log_seq = fetch_logs(args.host, args.status_port, log_seq, args.timeout)

    if args.reboot:
        status, text = request(args.host, args.reboot_port, "GET", "/?confirm=1", timeout=args.timeout)
        print(f"[ota] reboot HTTP {status}: {text.strip()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
