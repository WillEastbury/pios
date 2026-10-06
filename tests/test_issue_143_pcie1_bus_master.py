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


record = body_after(pcie1, "static bool record_function(struct pcie1_status *s")
assert "pcie1_cfg_write(" not in record
assert "PCI_REG_CMD" not in record
assert "record_bridge_range(reachable, bus, e)" in record

scan = body_after(pcie1, "static void scan_endpoints(struct pcie1_status *s)")
assert "reachable[PCIE1_SCAN_BUS_LO] = true;" in scan
assert "if (!reachable[bus])" in scan
assert "record_function(s, reachable, bus, dev, func);" in scan

probe = body_after(lzero, "bool lzero_probe_bars(void)")
assert "cmd & ~(PCI_CMD_MEM | PCI_CMD_MASTER)" in probe
assert "pcie1_cfg_write(bus, dev, fn, PCI_REG_CMD," in probe

mapped = body_after(lzero, "bool lzero_map_bar0(void)")
assert "(cmd | PCI_CMD_MEM) & ~PCI_CMD_MASTER" in mapped
assert "PCI_CMD_MASTER" in mapped

assert "void pcie1_rescan(void)" in pcie1
assert "publish_snap(g_link_up ? \"ok\" : \"link lost\");" in pcie1
assert "if (g_lzero.gpu_found && p.link_up)" not in body_after(
    lzero, "void lzero_probe(void)",
)

print("issue #143: passive enum is endpoint-write-free; explicit BAR probing keeps DMA disabled")
