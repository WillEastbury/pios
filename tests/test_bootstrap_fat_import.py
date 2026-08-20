#!/usr/bin/env python3
"""Static regression for the stage0 FAT-first recovery/update policy."""

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
bootstrap = (ROOT / "src" / "bootstrap.c").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:index]
    raise AssertionError(f"unterminated {signature}")


main = function_body(bootstrap, "NORETURN void bootstrap_main(void)")
assert "if (!partition_valid)\n        root_lba = BOOT_FALLBACK_LBA;\n    else" in main
assert "(void)stage0_apply_fat_update(root_lba);" in main
assert "valid raw slot; FAT import skipped" not in main

update = function_body(bootstrap, "static bool stage0_apply_fat_update(u32 root_lba)")
assert "slot_package_matches(target_lba, payload, payload_len, true)" in update
assert "FAT package already cached in slot A" in update

print("Stage0 imports a valid FAT package before selecting a raw slot")
