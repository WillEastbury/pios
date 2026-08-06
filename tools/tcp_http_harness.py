#!/usr/bin/env python3
"""
Host-side harness for PIOS TCP/HTTP logic.

This intentionally does not import PIOS C objects. It mirrors the pure logic
that is hard to inspect on hardware: SYN-cookie inputs, TCP checksum byte
ordering, HTTP header buffering, Basic auth parsing, and response queueing.
"""

from __future__ import annotations

import base64
import struct


def ip4(a: int, b: int, c: int, d: int) -> int:
    return (a << 24) | (b << 16) | (c << 8) | d


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x82F63B78
            else:
                crc >>= 1
            crc &= 0xFFFFFFFF
    return crc ^ 0xFFFFFFFF


def make_syn_cookie(local_port: int, remote_ip: int, remote_port: int, their_seq: int, secret: int) -> int:
    blob = struct.pack(
        "<IHHII",
        remote_ip & 0xFFFFFFFF,
        remote_port & 0xFFFF,
        local_port & 0xFFFF,
        their_seq & 0xFFFFFFFF,
        secret & 0xFFFFFFFF,
    )
    return crc32c(blob)


def validate_syn_cookie(local_port: int, remote_ip: int, remote_port: int, their_seq: int, our_ack_minus1: int, secret: int) -> bool:
    return make_syn_cookie(local_port, remote_ip, remote_port, their_seq, secret) == (our_ack_minus1 & 0xFFFFFFFF)


def csum_add_bytes(total: int, data: bytes) -> int:
    for i in range(0, len(data) - 1, 2):
        total += (data[i] << 8) | data[i + 1]
    if len(data) & 1:
        total += data[-1] << 8
    return total


def csum_fold(total: int) -> int:
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def tcp_checksum_network(src_ip: int, dst_ip: int, tcp_segment: bytes) -> int:
    total = 0
    total += (src_ip >> 16) & 0xFFFF
    total += src_ip & 0xFFFF
    total += (dst_ip >> 16) & 0xFFFF
    total += dst_ip & 0xFFFF
    total += 6
    total += len(tcp_segment)
    total = csum_add_bytes(total, tcp_segment)
    return csum_fold(total)


def build_tcp_header(src_port: int, dst_port: int, seq: int, ack: int, flags: int, window: int = 4096) -> bytearray:
    return bytearray(
        struct.pack(
            "!HHIIBBHHH",
            src_port,
            dst_port,
            seq & 0xFFFFFFFF,
            ack & 0xFFFFFFFF,
            5 << 4,
            flags,
            window,
            0,
            0,
        )
    )


def add_tcp_checksum(src_ip: int, dst_ip: int, header: bytearray, payload: bytes = b"") -> bytes:
    segment = bytes(header) + payload
    checksum = tcp_checksum_network(src_ip, dst_ip, segment)
    header[16] = (checksum >> 8) & 0xFF
    header[17] = checksum & 0xFF
    return bytes(header) + payload


def verify_tcp_checksum(src_ip: int, dst_ip: int, segment: bytes) -> bool:
    return tcp_checksum_network(src_ip, dst_ip, segment) == 0


def headers_complete(buf: bytes) -> bool:
    return b"\r\n\r\n" in buf


def parse_basic_auth_line(line: bytes) -> tuple[str, str] | None:
    name, sep, value = line.partition(b":")
    if sep != b":" or name.lower() != b"authorization":
        return None
    value = value.strip()
    if not value.lower().startswith(b"basic "):
        return None
    try:
        decoded = base64.b64decode(value[6:], validate=True).decode("ascii")
    except Exception:
        return None
    user, sep, password = decoded.partition(":")
    if sep != ":":
        return None
    return user, password


def early_auth(buf: bytes, expected_user: str = "root", expected_password: str = "pios") -> tuple[bool, bool]:
    """Return (auth_line_complete, authorized)."""
    for raw_line in buf.splitlines(keepends=True):
        if not raw_line.endswith((b"\n", b"\r\n")):
            return False, False
        parsed = parse_basic_auth_line(raw_line.strip(b"\r\n"))
        if parsed is None:
            continue
        return True, parsed == (expected_user, expected_password)
    return False, False


def simulate_http_buffering(chunks: list[bytes]) -> tuple[bool, bool, bool]:
    buf = b""
    early_complete = False
    early_ok = False
    for chunk in chunks:
        buf += chunk
        if not early_complete:
            early_complete, early_ok = early_auth(buf)
            if early_complete and not early_ok:
                return False, early_complete, early_ok
    return headers_complete(buf), early_complete, early_ok


def simulate_response_queue(response_len: int, writable_each_poll: int = 4095, chunk_limit: int = 512) -> tuple[int, int]:
    off = 0
    writes = 0
    while off < response_len and writes < 100:
        writable = writable_each_poll
        remain = response_len - off
        if writable > 0:
            chunk = min(remain, writable, chunk_limit)
            off += chunk
            writes += 1
    return off, writes


def check(name: str, condition: bool) -> None:
    if not condition:
        raise AssertionError(name)
    print(f"PASS {name}")


def main() -> None:
    local_ip = ip4(192, 168, 218, 101)
    remote_ip = ip4(192, 168, 218, 9)
    local_port = 80
    remote_port = 49152
    their_isn = 0x10203040
    secret = 0xA5C39E17

    cookie = make_syn_cookie(local_port, remote_ip, remote_port, their_isn, secret)
    check("syn cookie validates", validate_syn_cookie(local_port, remote_ip, remote_port, their_isn, cookie, secret))
    check("syn cookie rejects wrong peer port", not validate_syn_cookie(local_port, remote_ip, remote_port + 1, their_isn, cookie, secret))

    synack = build_tcp_header(local_port, remote_port, cookie, their_isn + 1, flags=0x12)
    synack_segment = add_tcp_checksum(local_ip, remote_ip, synack)
    check("synack checksum verifies", verify_tcp_checksum(local_ip, remote_ip, synack_segment))

    payload = b"PIOS OK\n"
    data_hdr = build_tcp_header(local_port, remote_port, cookie + 1, their_isn + 1, flags=0x18)
    data_segment = add_tcp_checksum(local_ip, remote_ip, data_hdr, payload)
    check("payload checksum verifies", verify_tcp_checksum(local_ip, remote_ip, data_segment))

    auth = base64.b64encode(b"root:pios")
    req = b"GET / HTTP/1.0\r\nHost: pios\r\nAuthorization: Basic " + auth + b"\r\n\r\n"
    complete, auth_complete, auth_ok = simulate_http_buffering([req[:10], req[10:42], req[42:]])
    check("buffered auth request completes", complete)
    check("early auth eventually completes", auth_complete)
    check("basic auth accepts root:pios", auth_ok)

    bad_req = b"GET / HTTP/1.0\r\nAuthorization: Basic " + base64.b64encode(b"root:bad") + b"\r\nX-Slow: "
    complete, auth_complete, auth_ok = simulate_http_buffering([bad_req[:20], bad_req[20:]])
    check("bad auth can fail before full headers", not complete and auth_complete and not auth_ok)

    plain_req = b"GET / HTTP/1.0\r\nHost: pios\r\n\r\n"
    complete, auth_complete, auth_ok = simulate_http_buffering([plain_req[:5], plain_req[5:]])
    check("plain request completes headers", complete)
    check("plain request has no early auth", not auth_complete and not auth_ok)

    off, writes = simulate_response_queue(2840)
    check("response queue drains all bytes", off == 2840)
    check("response queue uses bounded chunks", writes == 6)


if __name__ == "__main__":
    main()
