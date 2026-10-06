from pathlib import Path


root = Path(__file__).resolve().parent.parent
platform = (root / "include" / "platform.h").read_text(encoding="utf-8")
pcie1 = (root / "src" / "pcie1.c").read_text(encoding="utf-8")
mmu = (root / "src" / "mmu.c").read_text(encoding="utf-8")
kernel = (root / "src" / "kernel.c").read_text(encoding="utf-8")
config = (root / "config.txt").read_text(encoding="utf-8")


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
assert "#define PIOS_PCIE1_CPU_WIN_BASE     0x1B80000000UL" in platform
assert "#define PIOS_PCIE1_PCI_WIN_BASE     0x80000000UL" in platform
assert "#define PIOS_RP1_BAR_BASE           0x1F00000000UL" in platform
assert "#define PIOS_HAS_PCIE1              1" in platform
assert "#define PIOS_HAS_PCIE1              0" in platform
assert "[pi5]\ndtparam=pciex1=on\n" in config.replace("\r\n", "\n")
assert "pciex1_gen=" not in config

assert "map_kernel_pcie1_window(l1);" in mmu
assert "map_kernel_pcie1_window(l1_table_cached);" in mmu
assert "root[base / L1_BLOCK_SIZE]" in mmu

assert "PCIE_RC_BASE" not in pcie1
assert "PCIE1_RC_BASE               PIOS_PCIE1_RC_BASE" in pcie1
assert "PIOS_PCIE1_RESET_ID" in pcie1
assert "PIOS_PCIE1_CPU_WIN_BASE" in pcie1
assert "PIOS_PCIE1_PCI_WIN_BASE" in pcie1
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

dashboard = body_after(kernel, "static void hdmi_dashboard_render(void)\n{")
assert "pcie1_status(&p1);" in dashboard
assert "dash_pcie1_summary(p1caps" in dashboard
assert "pcie1_rescan(" not in dashboard
assert "pcie1_cfg_read(" not in dashboard and "pcie1_cfg_write(" not in dashboard

print("issue #137: PCIe1 host separation and Milestone-A fail-closed wiring are gated")
