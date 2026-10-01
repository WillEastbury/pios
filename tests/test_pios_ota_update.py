from __future__ import annotations

import importlib.util
import pathlib
from unittest.mock import patch


REPO = pathlib.Path(__file__).resolve().parent.parent
SPEC = importlib.util.spec_from_file_location(
    "pios_ota_update", REPO / "tools" / "pios_ota_update.py"
)
assert SPEC and SPEC.loader
ota = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ota)


def expect_rejected(image: bytes, label: str) -> None:
    try:
        ota.validate_raw_slot_image(image, pathlib.Path(label))
    except ValueError:
        return
    raise AssertionError(f"{label} was accepted")


ota.validate_raw_slot_image(b"\x00" * ota.RAW_SLOT_MAX_BYTES,
                            pathlib.Path("PIOS_PI5_STAGE2.BIN"))
expect_rejected(b"", "empty.bin")
expect_rejected(b"PGS2" + b"\x00" * 64, "PIOSSTG2.PKG")
expect_rejected(b"\x00" * (ota.RAW_SLOT_MAX_BYTES + 1), "too-large.bin")

assert ota.status_has_expected_version(
    '{"ok":true,"version":"v20260909.115725"}', "v20260909.115725"
)
assert not ota.status_has_expected_version(
    '{"ok":true,"version":"v20260909.115724"}', "v20260909.115725"
)
assert not ota.status_has_expected_version(
    '{"ok":false,"version":"v20260909.115725"}', "v20260909.115725"
)
assert not ota.status_has_expected_version("not JSON", "v20260909.115725")

responses = iter([
    (200, '{"ok":true,"version":"v20260909.115724"}'),
    (200, '{"ok":true,"version":"v20260909.115725"}'),
    (200, "PicoScript"),
])
original_request = ota.request
original_sleep = ota.time.sleep
try:
    ota.request = lambda *args, **kwargs: next(responses)
    ota.time.sleep = lambda _: None
    assert ota.wait_for_expected_version(
        "unused", 8080, "v20260909.115725", attempts=2, delay_seconds=0
    )
    ota.request = lambda *args, **kwargs: (200, '{"ok":true,"version":"v20260909.115724"}')
    assert not ota.wait_for_expected_version(
        "unused", 8080, "v20260909.115725", attempts=1, delay_seconds=0
    )
    ota.request = lambda *args, **kwargs: (503, "IDE assets missing")
    assert not ota.editor_is_available("unused", 8080)
finally:
    ota.request = original_request
    ota.time.sleep = original_sleep

print("pios OTA updater: raw-image, candidate-version, and editor gates passed")


class FragmentedSocket:
    def __init__(self, chunks):
        self.chunks = iter(chunks)
        self.timeout = 0

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendall(self, _):
        pass

    def recv(self, _):
        if self.timeout < 1:
            raise TimeoutError("response fragment takes more than 250 ms")
        return next(self.chunks, b"")


body = b'{"ok":true,"version":"candidate"}\n'
header = b"HTTP/1.0 200 OK\r\nContent-Length: " + str(len(body)).encode() + b"\r\n\r\n"
with patch.object(ota.socket, "create_connection", return_value=FragmentedSocket(
        [header + body[:10], body[10:]])):
    assert ota.request("unused", 8080, "GET", "/api/status") == (200, body.decode())
with patch.object(ota.socket, "create_connection", return_value=FragmentedSocket(
        [b"HTTP/1.0 200 OK\r\nConnection: close\r\n\r\n" + body[:10], body[10:]])):
    assert ota.request("unused", 8080, "GET", "/api/status") == (200, body.decode())
with patch.object(ota.socket, "create_connection", return_value=FragmentedSocket(
        [header + body[:10]])):
    try:
        ota.request("unused", 8080, "GET", "/api/status")
    except RuntimeError as exc:
        assert "short HTTP body" in str(exc)
    else:
        raise AssertionError("truncated status body accepted")
print("pios OTA updater: delayed fragments and explicit short-body rejection passed")
