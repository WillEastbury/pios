#!/usr/bin/env python3
"""Upload and atomically roll one PMOD driver without rebooting PIOS."""
from __future__ import annotations

import argparse
import json
import pathlib
import struct
import urllib.parse
import urllib.request

MAGIC = 0x504D4F44
HEADER = struct.Struct("<IHH7I32s")


def request(host: str, port: int, action: str, *,
            body: bytes = b"", **query: int) -> dict:
    query.update(action=action, confirm=1)
    url = f"http://{host}:{port}/api/admin/module-update?" + \
        urllib.parse.urlencode(query)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    req = urllib.request.Request(url, data=body if body else None,
                                 method="POST" if body else "GET")
    with opener.open(req, timeout=30) as response:
        payload = response.read().split(b"\r\n\r\n")[-1]
    result = json.loads(payload)
    if not result.get("ok"):
        raise SystemExit(f"module update failed: {result}")
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("image", nargs="?", type=pathlib.Path)
    ap.add_argument("--host", default="192.168.0.201")
    ap.add_argument("--port", type=int, default=8082)
    ap.add_argument("--chunk", type=int, default=4096)
    ap.add_argument("--rollback", type=int)
    args = ap.parse_args()
    if args.rollback is not None:
        print(request(args.host, args.port, "rollback",
                      module=args.rollback))
        return 0
    if not args.image:
        raise SystemExit("image is required unless --rollback is used")
    image = args.image.read_bytes()
    if len(image) < HEADER.size:
        raise SystemExit("short PMOD image")
    fields = HEADER.unpack_from(image)
    if fields[0] != MAGIC or fields[2] != HEADER.size:
        raise SystemExit("invalid PMOD header")
    total = len(image)
    print(request(args.host, args.port, "begin", total=total))
    offset = 0
    while offset < total:
        chunk = image[offset:offset + args.chunk]
        request(args.host, args.port, "chunk", body=chunk,
                total=total, offset=offset)
        offset += len(chunk)
        print(f"[module] {offset}/{total}")
    result = request(args.host, args.port, "commit", total=total)
    print(f"[module] active module={result['module']} "
          f"generation={result['artifactGeneration']} "
          f"epoch={result['dispatchEpoch']} no-reboot")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
