from pathlib import Path


root = Path(__file__).resolve().parent.parent
pcie1 = (root / "src" / "pcie1.c").read_text(encoding="utf-8")
lzero = (root / "src" / "lzero.c").read_text(encoding="utf-8")


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


record = body_after(pcie1, "static void record_function(struct pcie1_status *s")
assert "pcie1_cfg_read(bus, dev, func, PCI_REG_CMD)" in record
assert "cmd &= ~(PCI_CMD_MEM | PCI_CMD_MASTER);" in record
assert "pcie1_cfg_write(bus, dev, func, PCI_REG_CMD, cmd);" in record

scan = body_after(pcie1, "static void scan_endpoints(struct pcie1_status *s)")
assert "record_function(s, bus, dev, func);" in scan

probe = body_after(lzero, "bool lzero_probe_bars(void)")
assert "cmd & ~(PCI_CMD_MEM | PCI_CMD_MASTER)" in probe
assert "pcie1_cfg_write(bus, dev, fn, PCI_REG_CMD," in probe

mapped = body_after(lzero, "bool lzero_map_bar0(void)")
assert "(cmd | PCI_CMD_MEM) & ~PCI_CMD_MASTER" in mapped
assert "PCI_CMD_MASTER" in mapped

assert "void pcie1_rescan(void)" in pcie1
assert "publish_snap(g_link_up ? \"ok\" : (g_fail ? g_fail : \"no link\"));" in pcie1

print("issue #143: PCIe1 enumeration and BAR probing keep endpoint DMA disabled")
