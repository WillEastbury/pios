"""Deterministic editor-pack tool and raw-slot-size gate checks."""
from __future__ import annotations

import importlib.util
import pathlib
import struct
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
PACK = REPO / "tools" / "pack_ide_assets.py"
OUT = REPO / "assets" / "pios_ide_assets.brp"
sys.path.insert(0, str(REPO / "tools"))


def load(name: str, path: pathlib.Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


assert subprocess.run([sys.executable, str(PACK), "--brotli"],
                      cwd=REPO, check=False).returncode == 0
first = OUT.read_bytes()
assert subprocess.run([sys.executable, str(PACK), "--brotli"],
                      cwd=REPO, check=False).returncode == 0
assert OUT.read_bytes() == first, "Brotli pack must be reproducible"

magic, version, header_bytes, compressed_bytes, raw_bytes, compressed_crc, raw_crc, header_crc = \
    struct.unpack_from("<IHHIIIII", first)
assert (magic, version, header_bytes) == (0x50425250, 1, 28)
assert header_bytes + compressed_bytes == len(first)

pack_tool = load("pack_ide_assets", PACK)
assert raw_bytes <= pack_tool.MAX_RAW_BYTES
compressed = first[header_bytes:]
raw = pack_tool.pios_brotli.decompress(compressed)
assert len(raw) == raw_bytes
assert pack_tool.crc32c(compressed) == compressed_crc
assert pack_tool.crc32c(raw) == raw_crc
assert pack_tool.crc32c(first[:24]) == header_crc
assert struct.unpack_from("<IHHI", raw)[:3] == (0x53414950, 2, 4)
assert pack_tool.crc32c(raw[16:]) == struct.unpack_from("<I", raw, 12)[0]

gate = load("stage2_size_gate", REPO / "tools" / "stage2_size_gate.py")
assert gate.payload_fits(gate.RAW_SLOT_CAP)
assert not gate.payload_fits(gate.RAW_SLOT_CAP + 1)

print("IDE asset pack: deterministic Brotli, integrity, and raw-slot gate passed")
