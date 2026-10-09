"""Pin ADR-083's Pi5-only, 1GiB-safe, split GuC inbound layout."""
from pathlib import Path
import re

root = Path(__file__).resolve().parent.parent
platform = (root / "include" / "platform.h").read_text()
pcie1 = (root / "src" / "pcie1.c").read_text()
b50 = (root / "src" / "b50_native.c").read_text()

def values(name):
    matches = re.findall(rf"#define {name}\s+(0x[0-9A-Fa-f]+)UL", platform)
    assert matches, name
    return {int(match, 16) for match in matches}

base = 0x13000000
size = 0x01000000
canary = next(iter(values("PIOS_DMA_PCIE1_CANARY_SIZE")))
fw_size = next(iter(values("PIOS_DMA_PCIE1_GUC_FW_SIZE")))
ads_size = next(iter(values("PIOS_DMA_PCIE1_GUC_ADS_SIZE")))
queue_size = next(iter(values("PIOS_DMA_PCIE1_GUC_Q_SIZE")))

assert base in values("PIOS_DMA_PCIE1_BASE")
assert size in values("PIOS_DMA_PCIE1_SIZE")
assert size == 0x01000000 and base % size == 0
assert base + size <= 0x40000000
assert canary == 0x00200000
assert fw_size == 0x00100000
assert ads_size == 0x00B00000
assert queue_size == 0x00200000
assert canary + fw_size + ads_size + queue_size == size
assert "PIOS_DMA_PCIE1_CANARY_SIZE" in b50
assert "pcie1_link_gen3_active(2U, 8U, 0U)" in b50
assert "PCIE1_BAR2_SIZE_ENC     9U" in (root / "include" / "pcie1.h").read_text()
assert "must fit a 1 GiB board" in pcie1
print("B50 GuC layout: aligned 16MiB, 1GiB-safe split and Gen3 gate passed")
