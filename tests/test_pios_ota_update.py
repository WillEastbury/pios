from __future__ import annotations

import importlib.util
import pathlib


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
])
original_request = ota.request
original_sleep = ota.time.sleep
try:
    ota.request = lambda *args, **kwargs: next(responses)
    ota.time.sleep = lambda _: None
    assert ota.wait_for_expected_version(
        "unused", 8080, "v20260909.115725", attempts=2, delay_seconds=0
    )
    ota.request = lambda *args, **kwargs: (
        200, '{"ok":true,"version":"v20260909.115724"}'
    )
    assert not ota.wait_for_expected_version(
        "unused", 8080, "v20260909.115725", attempts=1, delay_seconds=0
    )
finally:
    ota.request = original_request
    ota.time.sleep = original_sleep

print("pios OTA updater: raw-image and candidate-version gates passed")
