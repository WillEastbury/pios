"""Pin static callback registration while B50 implementation extraction proceeds."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
source = (root / "src" / "b50_native.c").read_text(encoding="utf-8")
steps = (
    "firmware_stage_step", "forcewake_step", "ggtt_step",
    "boot_prepare_step", "guc_boot_step", "ct_step",
    "context_step", "submit_step",
)
for step in steps:
    assert f"B50_STEP_TRAMPOLINE({step}," in source
    assert re.search(
        rf"adrv_submit\(\s*\"[^\"]+\",\s*{step}_trampoline,", source
    ), step
assert "b50_bottom_trampoline, NULL)" in source
assert "irq_register(B50_ROOT_MSI_INTID, native_irq)" in source
assert "#define B50_MODULE_OP_COMMAND MODULE_OP_CALL" in source
assert "#define B50_MODULE_OP_SERVICE (MODULE_OP_CALL + 1U)" in source
assert "#define B50_MODULE_OP_BLOCKS  (MODULE_OP_CALL + 2U)" in source
print("B50 callback ABI: static IRQ, AIRQ/ADRV dispatch trampolines PASS")
