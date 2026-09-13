#!/usr/bin/env python3
"""Static gate: fixed layout validation is read-only and never repartitions."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "storage_layout.c").read_text(encoding="utf-8")
header = (ROOT / "include" / "storage_layout.h").read_text(encoding="utf-8")
walfs = (ROOT / "src" / "walfs.c").read_text(encoding="utf-8")
setup = (ROOT / "src" / "setup.c").read_text(encoding="utf-8")

for token in ("STORAGE_LAYOUT_THREE_PARTITION", "STORAGE_LAYOUT_LEGACY_TWO_PARTITION",
              "STORAGE_LAYOUT_P3_ABSENT", "storage_layout_validate_exchange",
              "p3_present", "p3_result", "is_fat_type", "entry_empty",
              "overlap"):
    assert token in source or token in header, f"missing layout guard: {token}"
for forbidden in ("sd_", "walfs", "fat32_", "write(", "format_", "partition_create",
                  "repartition", "malloc", "free("):
    assert forbidden not in source, f"layout validator must remain read-only: {forbidden}"
for path, text in (("src/walfs.c", walfs), ("src/setup.c", setup)):
    assert "sd_write_block(0" not in text, f"{path} must never write the MBR"
    assert "partition_create" not in text and "repartition" not in text, (
        f"{path} must never create or repartition media"
    )
init = walfs[walfs.index("bool walfs_init(void)"):walfs.index("void walfs_status")]
assert "format_disk" not in init, "WALFS normal initialization must not format"
assert "out->facts[2U].mbr_type != STORAGE_LAYOUT_MBR_TYPE_PIOS_RAW" not in source
print("storage layout source gate: read-only fixed MBR roles")
