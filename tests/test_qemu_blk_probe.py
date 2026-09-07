#!/usr/bin/env python3
"""Regression gate for issue #98: one virtio-blk device is usable."""

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sd = (ROOT / "src" / "sd.c").read_text(encoding="utf-8")
gotchas = (ROOT / "docs" / "gotchas.md").read_text(encoding="utf-8")
platforms = (ROOT / "docs" / "platforms.md").read_text(encoding="utf-8")

probe_start = sd.index("static bool qemu_blk_probe(void)")
probe_end = sd.index("\n}\n", probe_start) + 3
probe = sd[probe_start:probe_end]

assert "if (blocks == 0U || selected == 0U)" in probe
assert "blocks < 2" not in probe
assert "qemu-blk: no virtio-blk devices" in probe
assert "qemu-blk: discovered " in probe
assert "qemu_blk_diag = blocks;" in probe
assert "discovered BLK device" in platforms
assert "first discovered virtio-mmio BLK device" in gotchas

print("issue #98: zero devices falls back; one or more devices use virtio-blk")
