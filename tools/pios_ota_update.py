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

RAW_SLOT_MAX_BYTES = 0x37FE00
PGS2_MAGIC = b"PGS2"


def validate_raw_slot_image(image: bytes, path: pathlib.Path) -> None:
    """Reject files the raw-slot OTA endpoint cannot safely install.

    `PIOSSTG2.PKG` is a FAT package. Stage0 extracts one payload from it before
    writing a raw slot, but the HTTP updater receives no such package parser and
    must never write the container as a bootable payload.
    """
    size = len(image)
    if size == 0:
        raise ValueError("image is empty")
    if image.startswith(PGS2_MAGIC):
        raise ValueError(
            f"{path} is a PGS2 FAT package, not a raw platform payload; "
            "pass build_pi5_stage2\\PIOS_PI5_STAGE2.BIN to this updater"
        )
    if size > RAW_SLOT_MAX_BYTES:
        raise ValueError(
            f"{path} is {size} bytes, exceeding the raw OTA slot capacity "
            f"of {RAW_SLOT_MAX_BYTES}; pass a single platform payload, not "
            "PIOSSTG2.PKG"
        )


def status_has_expected_version(body: str, expected_version: str) -> bool:
    """Return true only for a successful status response from the candidate."""
    try:
        payload = json.loads(body)
    except json.JSONDecodeError:
        return False
    return payload.get("ok") is True and payload.get("version") == expected_version


def editor_is_available(host: str, editor_port: int) -> bool:
    """Raw Pi5 OTA candidates are accepted only after WALFS editor extraction."""
    try:
        status, body = request(host, editor_port, "GET", "/picoscript", timeout=8)
    except Exception:
        return False
    return status == 200 and "PicoScript" in body


def wait_for_expected_version(host: str, status_port: int, expected_version: str,
                              attempts: int, delay_seconds: float,
                              editor_port: int = 80) -> bool:
    """Wait for the requested candidate, not merely for any older image."""
    last_status = ""
    for _ in range(attempts):
        try:
            status, body = request(host, status_port, "GET", "/api/status", timeout=4)
            if status == 200 and status_has_expected_version(body, expected_version):
                if editor_is_available(host, editor_port):
                    print(f"[ota] candidate online: version={expected_version}; editor ready")
                    return True
                last_status = f"{expected_version} (editor unavailable)"
                time.sleep(delay_seconds)
                continue
            if status == 200:
                try:
                    last_status = str(json.loads(body).get("version", "missing version"))
                except json.JSONDecodeError:
                    last_status = "invalid JSON"
        except Exception:
            pass
        time.sleep(delay_seconds)
    if last_status:
        print(f"[ota] board came online with {last_status}, expected {expected_version}",
              file=sys.stderr)
    else:
        print(f"[ota] candidate {expected_version} did not answer within the timeout",
              file=sys.stderr)
    return False


def stream_upload(host: str, update_port: int, image: bytes, reboot: bool,
                  send_chunk: int = 4096, timeout: float = 30.0) -> None:
    """Upload the whole image over ONE connection. TCP's receive window (the
    board advertises a small window) self-paces the transfer to whatever core0
    can drain, so there is no connection churn and no RX-ring overrun — unlike
    the per-chunk protocol. The board streams the body straight into its RAM
    staging buffer, then (reboot=1) flushes to flash and reboots."""
    total = len(image)
    q = f"/api/admin/kernel-stream?confirm=1&total={total}"
    if reboot:
        q += "&reboot=1"
    header = (
        f"POST {q} HTTP/1.0\r\n"
        f"Host: {host}\r\n"
        "Connection: close\r\n"
        "Content-Type: application/octet-stream\r\n"
        f"Content-Length: {total}\r\n"
        "\r\n"
    ).encode("ascii")
    print(f"[ota] streaming {total} bytes over a single self-paced connection...")
    with socket.create_connection((host, update_port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # disable Nagle: test theory
        sock.sendall(header)
        sent = 0
        last_print = time.time()
        piece = 2048  # small pieces keep the board's 4KB RX window fed continuously
        # The kernel's tcp_read now sends a proper window-update ACK so the bulk
        # of the transfer is fully TCP-self-paced at line rate. Only the final
        # stretch is gently paced: that keeps the board's 4KB rx window from ever
        # hitting zero at the very end, which on a board that lacks the
        # window-update *self-heal* would otherwise deadlock on a single lost
        # window-update ACK ~250 bytes short. Harmless once the board has both.
        tail_pace_from = total  # no pacing: the board self-heals lost window-updates
        while sent < total:
            end = min(sent + piece, total)
            try:
                sock.sendall(image[sent:end])
            except Exception as exc:
                # If the board committed + rebooted before draining the last
                # bytes, the connection resets — that is success for reboot=1.
                print(f"[ota] send ended at {sent}/{total} (board rebooting?): {exc}")
                return
            sent = end
            if sent >= tail_pace_from and sent < total:
                time.sleep(0.1)  # ~20KB/s: keeps the board 4KB rx window from ever filling
            if time.time() - last_print > 2.0:
                print(f"[ota] streamed {sent}/{total}")
                last_print = time.time()
        print(f"[ota] streamed {sent}/{total}; awaiting board response...")
        try:
            resp = b""
            while b"\r\n\r\n" not in resp:
                c = sock.recv(4096)
                if not c:
                    break
                resp += c
            print(f"[ota] stream response: {resp.split(b'\\r\\n\\r\\n')[-1][:200]!r}")
        except Exception as exc:
            print(f"[ota] no response (board rebooting): {exc}")


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
    ap = argparse.ArgumentParser(
        description="Upload a single-platform PIOS stage2 payload to the raw OTA slot."
    )
    ap.add_argument(
        "image", type=pathlib.Path, nargs="?",
        default=pathlib.Path("build_pi5_stage2\\PIOS_PI5_STAGE2.BIN"),
        help="raw single-platform stage2 payload (default: %(default)s); "
             "PIOSSTG2.PKG is a FAT package and is rejected",
    )
    ap.add_argument("--host", default="192.168.0.201")
    ap.add_argument("--update-port", type=int, default=8082)
    ap.add_argument("--status-port", type=int, default=8080)
    ap.add_argument("--editor-port", type=int, default=80,
                    help="HTTP port serving /picoscript for post-boot acceptance")
    ap.add_argument("--reboot-port", type=int, default=8081)
    ap.add_argument("--chunk-size", type=int, default=4096)
    ap.add_argument("--log-every", type=int, default=16)
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--commit-timeout", type=float, default=180.0,
                    help="timeout for the commit request; the board flushes the RAM-staged "
                         "image to the SD slot here (a single multi-MB write), so allow plenty.")
    ap.add_argument("--delay", type=float, default=0.0,
                    help="seconds to sleep after each chunk (rarely needed now that chunks are "
                         "RAM-staged on the board and no longer block core0 on SD writes)")
    ap.add_argument("--reboot", action="store_true", help="Flash + reboot into the new image after upload.")
    ap.add_argument("--expected-version",
                    help="required with --reboot; exact /api/status version expected "
                         "after boot (for example v20260909.115725)")
    ap.add_argument("--chunked", action="store_true",
                    help="use the legacy per-chunk protocol instead of the single-connection stream")
    ap.add_argument("--max-retries", type=int, default=40,
                    help="per-chunk retry budget (resumable upload grinds through NIC wedges)")
    ap.add_argument("--recover-wait", type=float, default=20.0,
                    help="seconds to wait for the board's NIC to self-recover before resuming")
    args = ap.parse_args()

    if args.reboot and not args.expected_version:
        ap.error("--reboot requires --expected-version so rollback/wrong-image boots fail")
    image = args.image.read_bytes()
    try:
        validate_raw_slot_image(image, args.image)
    except ValueError as exc:
        raise SystemExit(f"[ota] refusing image: {exc}") from exc
    total = len(image)
    if args.chunk_size <= 0 or args.chunk_size % 512 != 0:
        raise SystemExit("--chunk-size must be a positive multiple of 512")

    # Default path: single self-paced streaming connection (no churn -> no wedge).
    if not args.chunked:
        stream_upload(args.host, args.update_port, image, args.reboot,
                      send_chunk=args.chunk_size, timeout=args.commit_timeout)
        if not args.reboot:
            print("[ota] streamed (no reboot requested); call writeandreboot or pass --reboot")
            return 0
        print(f"[ota] waiting for candidate {args.expected_version}...")
        time.sleep(8)
        return 0 if wait_for_expected_version(args.host, args.status_port,
                                              args.expected_version, 40, 4,
                                              args.editor_port) else 1

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
        for attempt in range(1, args.max_retries + 1):
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
                if attempt == args.max_retries:
                    raise
                # On an old (pre-RAM-staging) board the per-chunk SD write can
                # briefly wedge the NIC; it self-recovers via the RX-overrun
                # backstop. Wait out the recovery, then resume from the server's
                # acknowledged nextOffset (the protocol is fully resumable).
                wait = args.recover_wait
                print(f"[ota] wedge? offset={offset} attempt={attempt}: {exc}; "
                      f"waiting {wait}s for recovery then resuming", file=sys.stderr)
                time.sleep(wait)
                for _ in range(6):
                    try:
                        status = request_json(
                            args.host,
                            args.update_port,
                            "GET",
                            update_path(action="status"),
                            timeout=args.timeout,
                        )
                        offset = int(status.get("nextOffset", offset))
                        break
                    except Exception:
                        time.sleep(wait)
        chunk_index = (offset + args.chunk_size - 1) // args.chunk_size
        if offset == total or (args.log_every > 0 and (chunk_index % args.log_every) == 0):
            print(f"[ota] {offset}/{total}")
            log_seq = fetch_logs(args.host, args.status_port, log_seq, args.timeout)
        if args.delay > 0.0 and offset < total:
            time.sleep(args.delay)

    # Verify the board received the whole image before writing it to flash.
    status = request_json(
        args.host, args.update_port, "GET",
        update_path(action="status"), timeout=args.timeout,
    )
    print(f"[ota] uploaded received={status.get('received')}/{total} "
          f"staged={status.get('staged')} chunks={status.get('chunks')}")
    if int(status.get("received", 0)) != total:
        raise SystemExit(f"[ota] incomplete upload: {status.get('received')}/{total}")

    if args.reboot:
        # writeandreboot: board flushes the RAM-staged image to the SD slot and
        # reboots into it. The connection drops as the board reboots — that is
        # expected and counts as success; we then poll for it to come back.
        print("[ota] writeandreboot: flushing staged image to flash + rebooting...")
        try:
            wr = request_json(
                args.host, args.update_port, "POST",
                update_path(action="writeandreboot", total=total),
                timeout=args.commit_timeout,
            )
            print(f"[ota] writeandreboot acked commits={wr.get('commits')}")
        except Exception as exc:
            print(f"[ota] writeandreboot connection dropped (board rebooting): {exc}")
        print(f"[ota] waiting for candidate {args.expected_version}...")
        time.sleep(8)
        return 0 if wait_for_expected_version(args.host, args.status_port,
                                              args.expected_version, 30, 4,
                                              args.editor_port) else 1

    commit = request_json(
        args.host,
        args.update_port,
        "POST",
        update_path(action="commit", total=total),
        timeout=args.commit_timeout,
    )
    print(f"[ota] committed received={commit.get('received')} commits={commit.get('commits')}")
    log_seq = fetch_logs(args.host, args.status_port, log_seq, args.timeout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
