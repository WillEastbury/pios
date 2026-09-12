from pathlib import Path


root = Path(__file__).resolve().parent.parent
source = (root / "src" / "pcie1_containment.c").read_text(encoding="utf-8")
header = (root / "include" / "pcie1_containment.h").read_text(encoding="utf-8")
pcie1 = (root / "src" / "pcie1.c").read_text(encoding="utf-8")

for forbidden in (
    "mmio_read", "mmio_write", "pcie1_cfg_write", "gic_enable_irq",
    "irq_register", "airq_post_from(", "PCI_CMD_MASTER",
):
    assert forbidden not in source

assert "return false;" in source[source.index(
    "bool pcie1_containment_hardware_enable_allowed"):]
assert "PIOS_DMA_PCIE1_BASE" in source
assert "PCIE1_CONTAINMENT_IOVA_BASE" in source
assert "PCIE1_CONTAINMENT_CACHE_NORMAL_NC" in source
assert "dmb();" in source
assert "AIRQ_SRC_PCIE1_MSI" in header
assert "PCIE1_CONTAINMENT_MSI_EVENT_PENDING" in header
assert "PCIE1_CONTAINMENT_FAULT_AER" in header
assert 'msr daifset, #2' in source
assert 'msr daif, %0' in source
assert "pcie1_containment_msi_dispatch_begin" in source
assert "pcie1_containment_completion_submit" in source

# Existing hardware remains exactly as gated by #147. The new contract is not
# wired into PCIe1 initialization and cannot unmask MSI or bus mastering.
assert "pcie1_containment" not in pcie1
assert "gic_enable_irq" not in pcie1
assert "irq_register" not in pcie1
assert "airq_post" not in pcie1

print("issue #189: PCIe1 DMA/MSI containment is hardware-disabled")
