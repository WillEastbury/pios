from pathlib import Path


root = Path(__file__).resolve().parent.parent
platform = (root / "include" / "platform.h").read_text(encoding="utf-8")
pcie1 = (root / "src" / "pcie1.c").read_text(encoding="utf-8")
mmu = (root / "src" / "mmu.c").read_text(encoding="utf-8")
kernel = (root / "src" / "kernel.c").read_text(encoding="utf-8")


def body_after(source: str, signature: str) -> str:
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


assert "#define PIOS_PCIE1_RC_BASE          0x1000110000UL" in platform
assert "#define PIOS_PCIE_RC_BASE           0x1000120000UL" in platform
assert "#define PIOS_PCIE1_RESET_ID         43U" in platform
assert "#define PIOS_PCIE1_CPU_WIN_BASE     0x1B00000000UL" in platform
assert "#define PIOS_RP1_BAR_BASE           0x1F00000000UL" in platform
assert "#define PIOS_HAS_PCIE1              1" in platform
assert "#define PIOS_HAS_PCIE1              0" in platform

assert "l1[108] = ((u64)108U * L1_BLOCK_SIZE) | dev_attr;" in mmu
assert "0x1800000000" in mmu
assert "12 GiB prefetch" in mmu

assert "PCIE_RC_BASE" not in pcie1
assert "PCIE1_RC_BASE               PIOS_PCIE1_RC_BASE" in pcie1
assert "PIOS_PCIE1_RESET_ID" in pcie1
assert "PIOS_PCIE1_CPU_WIN_BASE" in pcie1
assert "PIOS_PCIE1_IRQ" not in pcie1
assert "PIOS_PCIE1_MSI_IRQ" not in pcie1
assert "gic_enable_irq" not in pcie1

phase = kernel.index("if (pcie_init())")
phase_end = kernel.index("bp_done(3", phase)
pcie_phase = kernel[phase:phase_end]
assert pcie_phase.index("pcie1_init()") < pcie_phase.index("lzero_probe();")

tick = body_after(kernel, "static void core0_io_tick_hook(u32 core, u64 tick)\n{")
assert "pcie1" not in tick
assert "lzero" not in tick
assert "pcie_cfg" not in tick

print("issue #137: PCIe1 host separation and Milestone-A fail-closed wiring are gated")
