#!/usr/bin/env python3
"""Reversible 64-byte GK107 VRAM proof through BAR0 PRAMIN; no DMA."""
import argparse
import json
from pathlib import Path
import re
import time
import urllib.request
from pios_kepler_rom import terminal

BAR0 = 0x1B80000000
SCRATCH = 0x00100000  # Above Nouveau's 256 KiB VGA reservation, not the VBIOS tail.
WORDS = 32           # 64-byte payload plus 32-byte guards on each side.


def prove(host, log):
    deadline = time.monotonic() + 60

    def command(text):
        if time.monotonic() >= deadline:
            raise RuntimeError("VRAM proof deadline expired")
        return terminal(host, text)

    def read(offset):
        text = command(f"peek 0x{BAR0 + offset:x} 4")
        match = re.fullmatch(r"0x([0-9a-fA-F]+) = 0x([0-9a-fA-F]+)", text)
        if not match or int(match[1], 16) != BAR0 + offset:
            raise RuntimeError("invalid VRAM/register read response")
        return int(match[2], 16)

    def write(offset, value):
        log.write(json.dumps({"offset": offset, "value": value}) + "\n")
        log.flush()
        text = command(f"poke 0x{BAR0 + offset:x} 0x{value:08x} 4")
        if not text.startswith("OK:"):
            raise RuntimeError("VRAM/register write refused")

    status = command("kepler probe")
    if "kepler probe ok" not in status or " posted=1 " not in status or \
            "command=00000002" not in status:
        raise RuntimeError("posted GK107 with bus mastering off is required")
    # Pin only the uniform 2 GiB K2000 layout for this initial proof.
    if read(0x22438) != 2 or read(0x2243C) != 2 or read(0x22554) != 0 or \
            read(0x11020C) != 1024 or read(0x11120C) != 1024:
        raise RuntimeError("VRAM geometry differs from the reviewed uniform 2 GiB layout")
    bios = read(0x619F04)
    if bios & 8:
        raise RuntimeError("VBIOS shadow reservation needs explicit exclusion")
    saved_selector = read(0x1700)
    original = None
    modified = False
    success = False
    try:
        write(0x1700, SCRATCH >> 16)
        if read(0x1700) != SCRATCH >> 16:
            raise RuntimeError("PRAMIN selector readback mismatch")
        original = [read(0x700000 + 4 * i) for i in range(WORDS)]
        for seed in (0xA5A50000, 0x5A5A0000):
            modified = True
            pattern = [(seed ^ (i * 0x01010101)) & 0xFFFFFFFF for i in range(16)]
            for i, value in enumerate(pattern, 8):
                write(0x700000 + 4 * i, value)
            observed = [read(0x700000 + 4 * i) for i in range(WORDS)]
            if observed[:8] != original[:8] or observed[24:] != original[24:]:
                raise RuntimeError("VRAM guard changed")
            if observed[8:24] != pattern:
                raise RuntimeError("VRAM known-pattern mismatch")
        success = True
    finally:
        # A bounded diagnostic failure does not authorize silent repair claims.
        # Restoration is checked explicitly, and any failure propagates.
        try:
            if modified and original is not None:
                for i in range(8, 24):
                    write(0x700000 + 4 * i, original[i])
                if [read(0x700000 + 4 * i) for i in range(WORDS)] != original:
                    raise RuntimeError("original VRAM bytes could not be restored")
        finally:
            write(0x1700, saved_selector)
            if read(0x1700) != saved_selector:
                raise RuntimeError("PRAMIN selector could not be restored")
    if not success:
        raise RuntimeError("VRAM proof failed")
    aer = command("pcie1 aer")
    if "uncorr=00000000" not in aer or "corr=00000000" not in aer:
        raise RuntimeError(f"PCIe error after VRAM proof: {aer}")
    with urllib.request.urlopen(f"http://{host}/api/status", timeout=5) as r:
        status = json.load(r)
    if status["diag"]["error"] != 0 or status["perf"]["nic_rx_wedge"] != 0:
        raise RuntimeError("management health degraded")
    result = {"vram_reported_bytes": 2 * 1024**3, "scratch_offset": SCRATCH,
              "tested_bytes": 64, "guard_bytes": 64, "patterns": 2,
              "original_restored": True, "selector_restored": True,
              "dma_enabled": False, "compute_ready": False}
    log.write(json.dumps({"result": result}) + "\n")
    log.flush()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.0.201")
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    with args.log.open("x", encoding="utf-8") as log:
        print(json.dumps(prove(args.host, log), indent=2))


if __name__ == "__main__":
    main()
