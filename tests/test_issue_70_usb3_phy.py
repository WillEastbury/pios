from pathlib import Path


xhci = (Path(__file__).resolve().parent.parent / "src" / "xhci.c").read_text(
    encoding="utf-8"
)

start = xhci.index("static bool dwc3_init(u64 base)")
end = xhci.index("void xhci_select_controller(u32 controller)", start)
init = xhci[start:end]

assert "DWC3_GSNPSID" in init
assert "DWC3_GUSB2PHYCFG" in init
assert "DWC3_GUSB3PIPECTL" in init
assert "GUSB3_PHYSOFTRST" in init
assert "GUSB3_SUSPHY" in init
reset = init.index(
    "mmio_write(base + DWC3_GUSB3PIPECTL, pipectl | GUSB3_PHYSOFTRST)"
)
deassert = init.index("mmio_write(base + DWC3_GUSB3PIPECTL,", reset + 1)
assert reset < deassert
assert "stats.gusb3_before" in init
assert "stats.gusb3_after" in init

print("issue #70: DWC3 USB3 PHY reset/deassert sequence is pinned")
