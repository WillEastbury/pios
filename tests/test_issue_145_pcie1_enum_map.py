from pathlib import Path


root = Path(__file__).resolve().parent.parent
pcie1 = (root / "src" / "pcie1.c").read_text(encoding="utf-8")
lzero = (root / "src" / "lzero.c").read_text(encoding="utf-8")
header = (root / "include" / "lzero.h").read_text(encoding="utf-8")


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


bridge = body_after(pcie1, "static void program_bridge(u32 bus")
assert "PCI_REG_BUS_NUM" in bridge
assert "PCI_REG_MEM_BASE_LIMIT" in bridge
assert "PCIE1_SCAN_BUS_HI" in bridge
assert "0xFFF00000U" in bridge

scan = body_after(pcie1, "static void scan_endpoints(struct pcie1_status *s)")
assert "pcie1_next_bridge_bus = PCIE1_SCAN_BUS_LO + 1U;" in scan
assert "record_function(s, bus, dev, func);" in scan

init = body_after(pcie1, "bool pcie1_init(void)")
assert init.index("perst_set(true);") < init.index("perst_set(false);")
assert "dsb();\n    perst_set(false);" in init
assert "pw(RC_CFG_PRIV1_ID_VAL3, tmp);\n    dmb();" in init

window = body_after(pcie1, "bool pcie1_set_outbound_window(u64 size)")
assert "size < 0x00100000ULL" in window
assert "size > PIOS_PCIE1_CPU_WIN_SIZE" in window
assert "(size & (size - 1ULL)) != 0ULL" in window
assert "set_outbound_win(PIOS_PCIE1_CPU_WIN_BASE, 0, size);" in window

probe = body_after(lzero, "static u64 probe_bar_size(")
first_all_ones = probe.index("pcie1_cfg_write(bus, dev, fn, off, 0xFFFFFFFFU);")
second_all_ones = probe.index(
    "pcie1_cfg_write(bus, dev, fn, off + 4U, 0xFFFFFFFFU);",
)
mask_reads = probe.index("mask_lo = pcie1_cfg_read", second_all_ones)
assert first_all_ones < second_all_ones < mask_reads

mapped = body_after(lzero, "bool lzero_map_bar0(void)")
assert "pcie1_set_outbound_window(g_lzero.bar0_size)" in mapped
assert "pcie1_enable_memory_path(g_lzero.gpu_bus)" in mapped
assert "g_lzero.atu_size = g_lzero.bar0_size;" in mapped

assert "static inline u32 lzero_compute_rank" in header
assert "pcie1_is_b50(e->vendor, e->device)" in header
assert "e->base_class == 0x03U && e->subclass == 0x02U" in header
assert "lzero_pick_compute" in header

print("issue #145: PCIe1 bridges, BAR64 probing, ATU sizing, and GPU selection are gated")
