#!/usr/bin/env python3
"""Probe the PIOS kernel TLS-style port 443 test endpoint.

This validates the current PIOS record-wrapper boundary, not browser TLS.
It sends a CHLO, expects an SHLO, then reads one encrypted response record.
The response is not decrypted because the production endpoint derives its PSK
from the board keystore.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys


CHLO_MAGIC = 0x43484C4F
SHLO_MAGIC = 0x53484C4F
HANDSHAKE_VERSION = 1
HANDSHAKE_LEN = 37
TAG_LEN = 16


def read_exact(sock: socket.socket, n: int) -> bytes:
    out = bytearray()
    while len(out) < n:
        chunk = sock.recv(n - len(out))
        if not chunk:
            raise RuntimeError(f"connection closed after {len(out)}/{n} bytes")
        out.extend(chunk)
    return bytes(out)


def main() -> int:
    ap = argparse.ArgumentParser(description="Probe PIOS kernel TLS port 443")
    ap.add_argument("--host", default="192.168.0.101")
    ap.add_argument("--port", type=int, default=443)
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    client_random = bytes(range(32))
    chlo = struct.pack("<IB32s", CHLO_MAGIC, HANDSHAKE_VERSION, client_random)

    with socket.create_connection((args.host, args.port), timeout=args.timeout) as sock:
        sock.settimeout(args.timeout)
        sock.sendall(chlo)
        shlo = read_exact(sock, HANDSHAKE_LEN)
        magic, version, server_random = struct.unpack("<IB32s", shlo)
        if magic != SHLO_MAGIC or version != HANDSHAKE_VERSION:
            raise RuntimeError(f"bad SHLO magic=0x{magic:08x} version={version}")
        header = read_exact(sock, 2)
        record_len = int.from_bytes(header, "big")
        payload_and_tag = read_exact(sock, record_len + TAG_LEN)

    print(f"OK SHLO version={version} server_random={server_random.hex()}")
    print(f"OK encrypted_record len={record_len} tag_len={TAG_LEN} total={len(payload_and_tag)}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERR {exc}", file=sys.stderr)
        raise SystemExit(1)
