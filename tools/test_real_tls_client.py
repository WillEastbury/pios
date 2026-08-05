#!/usr/bin/env python3
"""Independent TLS 1.3 interoperability check for the PIOS server."""

from __future__ import annotations

import argparse
import socket
import ssl


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--server-name", default="pios-test")
    args = parser.parse_args()

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    ctx.minimum_version = ssl.TLSVersion.TLSv1_3
    ctx.maximum_version = ssl.TLSVersion.TLSv1_3
    # PIOS intentionally implements only secp256r1/P-256 ECDHE today.
    ctx.set_ecdh_curve("prime256v1")

    raw = socket.create_connection((args.host, args.port), timeout=10)
    with ctx.wrap_socket(raw, server_hostname=args.server_name) as tls:
        protocol = tls.version()
        cipher = tls.cipher()
        cert = tls.getpeercert(binary_form=True)
        tls.settimeout(10)
        tls.sendall(
            b"GET / HTTP/1.0\r\n"
            b"Host: pios-test\r\n"
            b"Connection: close\r\n\r\n"
        )
        chunks: list[bytes] = []
        while True:
            try:
                chunk = tls.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            chunks.append(chunk)
        response = b"".join(chunks)

        if protocol != "TLSv1.3":
            raise RuntimeError(f"unexpected protocol: {protocol}")
        if not cipher or cipher[0] != "TLS_AES_128_GCM_SHA256":
            raise RuntimeError(f"unexpected cipher: {cipher}")
        if not cert:
            raise RuntimeError("server did not provide a certificate")
        if b"HTTP/1.0 200 OK" not in response or b"PIOS kernel TLS 443" not in response:
            raise RuntimeError(f"unexpected HTTP response: {response[:500]!r}")

        print(
            f"PASS TLSv1.3 {cipher[0]} cert={len(cert)} "
            f"http_bytes={len(response)}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
