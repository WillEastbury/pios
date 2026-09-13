#!/usr/bin/env python3
"""ADR-071 static isolation gate for the offline FAT32 exchange core."""

from __future__ import annotations

import pathlib
import subprocess


ROOT = pathlib.Path(__file__).resolve().parent.parent


def git(*args: str) -> str:
    return subprocess.check_output(
        ["git", *args], cwd=ROOT, text=True, encoding="utf-8"
    ).strip()


# This intentionally includes committed and worktree changes.  Comparing HEAD
# alone would miss an uncommitted edit to a boot/live-storage path.
base = git("merge-base", "origin/main", "HEAD")
changed = set(filter(None, git("diff", "--name-only", base).splitlines()))
protected = {
    "src/fat32.c", "include/fat32.h",
    "src/sd.c", "include/sd.h", "src/sdhost.c", "include/sdhost.h",
    "src/walfs.c", "include/walfs.h",
    "src/bootstrap.c", "src/bootstrap_start.S", "src/bootstrap_trampoline.S",
}
assert not (changed & protected), (
    "ADR-071 must not modify live storage or boot paths: "
    + ", ".join(sorted(changed & protected))
)

header = (ROOT / "include" / "fat32_exchange_core.h").read_text(encoding="utf-8")
source = (ROOT / "src" / "fat32_exchange_core.c").read_text(encoding="utf-8")
adr = (ROOT / "docs" / "ADR-071-fat32-exchange-core.md").read_text(encoding="utf-8")

for needle in (
    "fat32_exchange_mount", "fat32_exchange_create",
    "fat32_exchange_rewrite", "fat32_exchange_append",
    "fat32_exchange_delete", "fat32_exchange_rename",
    "fat32_exchange_flush", "fat32_exchange_read",
):
    assert needle in header
for forbidden in ('#include "sd.h"', '#include "fat32.h"', '#include "walfs.h"',
                  '#include "kernel.h"', "sd_read_block", "sd_write_block"):
    assert forbidden not in source, f"offline core must not use live path: {forbidden}"
assert "identity" in header and "epoch" in header
assert "no LFN" in adr and "no live mount" in adr
assert "not crash safe" in adr

adapter = (ROOT / "src" / "qemu_xfer.c").read_text(encoding="utf-8")
selector = (ROOT / "src" / "qemu_xfer_partition.c").read_text(encoding="utf-8")
for needle in ("PIOS_PLATFORM_QEMU_VIRT", "authorized_lba",
               "qemu_xfer_partition_select", "sd_qemu_virtio_blk_ready"):
    assert needle in adapter, f"QEMU adapter missing isolation guard: {needle}"
for forbidden in ('"fat32.h"', '"walfs.h"', '"bootstrap.h"'):
    assert forbidden not in adapter, f"QEMU adapter must not use live FS path: {forbidden}"
for needle in ("p3", "spans_overlap", "PIOSXFER"):
    assert needle in selector or needle in (ROOT / "include" / "qemu_xfer_partition.h").read_text(encoding="utf-8")

print(f"ADR-071 exchange core gate: merge-base {base[:12]}, live paths isolated")
