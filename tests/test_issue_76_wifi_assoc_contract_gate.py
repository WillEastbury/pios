#!/usr/bin/env python3
"""#76 gate: association control remains offline, pure, and disabled."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "wifi_assoc_contract.c").read_text(encoding="utf-8")
header = (ROOT / "include" / "wifi_assoc_contract.h").read_text(encoding="utf-8")

for required in (
    "WIFI_ASSOC_OWNER_CORE", "WIFI_ASSOC_MAX_STEPS",
    "WIFI_ASSOC_SET_SSID_PUBLISHED", "WIFI_ASSOC_PSK_SUP_WAIT_M1",
    "WIFI_ASSOC_PSK_SUP_PREP_G2", "wifi_assoc_eapol_decode",
    "wifi_assoc_control_publication_copy", "wifi_assoc_hardware_enable_allowed",
):
    assert required in header or required in source, f"missing contract item: {required}"

for forbidden in (
    '#include "cyw43', '#include "sd', '#include "mmio', '#include "kernel',
    '#include "wifi_nic', "cyw43_", "sdio_", "sd_", "mmio_", "gpio_",
    "WLC_", "CMD53", "eapol_tx", "eapol_send", "wpa_", "malloc", "free(",
    "memcpy(", "memset(", "while (", "for (;;)", "wfi(", "wfe(",
):
    assert forbidden not in source, f"forbidden integration or blocking token: {forbidden}"

assert "core_id() == WIFI_ASSOC_OWNER_CORE" in source
assert "msr daifset, #2" in source
assert "msr daif, %0" in source
assert "return false;" in source[source.index("bool wifi_assoc_hardware_enable_allowed"):]
assert "const u8 *packet" in source and "packet_len" in source
assert "WIFI_ASSOC_EAPOL_M1" in source and "WIFI_ASSOC_EAPOL_M3" in source
assert "WIFI_ASSOC_EAPOL_G1" in source

base = subprocess.run(["git", "merge-base", "HEAD", "origin/main"],
                      cwd=ROOT, capture_output=True, text=True)
if base.returncode != 0 or not base.stdout.strip():
    print("FAIL unable to resolve origin/main merge base")
    sys.exit(1)
changed = subprocess.check_output(
    ["git", "diff", "--name-only", base.stdout.strip(), "HEAD"],
    cwd=ROOT, text=True).splitlines()
changed += subprocess.check_output(["git", "diff", "--name-only", "HEAD"],
                                   cwd=ROOT, text=True).splitlines()
for path in changed:
    lower = path.lower()
    assert not (lower == "src/cyw43.c" or lower == "src/kernel.c"), (
        f"protected live join path changed: {path}")

print("issue #76: Wi-Fi association contract remains offline and hardware-disabled")
