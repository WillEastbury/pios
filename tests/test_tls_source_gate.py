#!/usr/bin/env python3
"""ADR-044 source gate: normal TLS code contains no hidden progress pumps."""

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parent.parent
FILES = [
    "src/tls.c",
    "include/tls.h",
    "src/picotlsserver.c",
    "include/picotlsserver.h",
    "src/tls13_client.c",
    "include/tls13_client.h",
    "src/tls13_handshake.c",
    "include/tls13_handshake.h",
    "src/tls13_verify.c",
    "include/tls13_verify.h",
    "src/tls13_record.c",
    "include/tls13_record.h",
    "src/tls13_keysched.c",
    "include/tls13_keysched.h",
]

FORBIDDEN_CALLS = re.compile(
    r"\b(?:net_poll|timer_delay_ms|timer_delay_us|delay|wfe|wfi|"
    r"malloc|calloc|realloc|free)\s*\("
)
PRIVATE_PROTOCOL = re.compile(r"\b(?:CHLO|SHLO|TLS_MAGIC_CHLO|TLS_MAGIC_SHLO)\b")


def main() -> int:
    failures: list[str] = []
    for relative in FILES:
        text = (ROOT / relative).read_text(encoding="utf-8")
        if FORBIDDEN_CALLS.search(text):
            failures.append(f"{relative}: forbidden blocking/progress call")
        if PRIVATE_PROTOCOL.search(text):
            failures.append(f"{relative}: private CHLO/SHLO protocol retained")
    if failures:
        for failure in failures:
            print(f"[FAIL] {failure}")
        return 1
    hello = (ROOT / "src" / "tls13_handshake.c").read_text(encoding="utf-8")
    if "0x001cU" not in hello or "TLS13_MAX_INNER_PLAINTEXT" not in hello:
        print("[FAIL] ClientHello does not negotiate the supported record-size limit")
        return 1
    kernel = (ROOT / "src" / "kernel.c").read_text(encoding="utf-8")
    assert "tls_server_start(" in kernel
    assert "tls_handshake_step(" in kernel
    assert "tls_close_step(" in kernel
    print("[PASS] TLS source gate: no polling, waits, heap, or CHLO/SHLO")
    return 0


if __name__ == "__main__":
    sys.exit(main())
