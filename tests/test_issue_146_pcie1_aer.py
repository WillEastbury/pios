from pathlib import Path


root = Path(__file__).resolve().parent.parent
header = (root / "include" / "pcie1.h").read_text(encoding="utf-8")
pcie1 = (root / "src" / "pcie1.c").read_text(encoding="utf-8")
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


assert "struct pcie1_aer_snapshot" in header
assert "void pcie1_aer_init(void);" in header
assert "void pcie1_aer_dump(const char *tag);" in header
assert "void pcie1_aer_snapshot(struct pcie1_aer_snapshot *out, bool clear);" in header

assert "static u32 pcie1_aer_offset_rc;" in pcie1
assert "static u32 pcie1_find_aer_cap" in pcie1
assert "pcie1_cfg_read(bus, dev, fn, off)" in pcie1
assert "0x0001U" in pcie1

aer_init = body_after(pcie1, "void pcie1_aer_init(void)")
assert "pcie1_find_aer_cap(0, 0, 0)" in aer_init
assert "PCIE1_AER_UNCORR_ERR" in aer_init
assert "PCIE1_AER_CORR_ERR" in aer_init
assert "PCIE1_AER_UNCORR_MASK" in aer_init
assert "PCIE1_AER_CORR_MASK" in aer_init

aer_snapshot = body_after(
    pcie1, "void pcie1_aer_snapshot(struct pcie1_aer_snapshot *out, bool clear)",
)
assert "pcie1_aer_offset_rc" in aer_snapshot
assert "if (clear)" in aer_snapshot
assert "PCIE1_AER_HDR_LOG3" in aer_snapshot

assert "pcie1_aer_init();" in body_after(
    pcie1, "bool pcie1_init(void)",
)
assert "void pcie1_aer_dump(const char *tag)" in pcie1
assert "pcie1 aer clear" in kernel
assert "pcie1_aer_snapshot(&a, clear);" in kernel
assert "pcie_aer_snapshot" not in pcie1
assert "pcie_cfg_read" not in pcie1
assert "static u32 aer_offset_rc" not in pcie1

print("issue #146: PCIe1 AER uses separate discovery, state, and console diagnostics")
