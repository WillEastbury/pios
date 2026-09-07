from pathlib import Path


root = Path(__file__).resolve().parent.parent
platform = (root / "include" / "platform.h").read_text(encoding="utf-8")
pcie1 = (root / "src" / "pcie1.c").read_text(encoding="utf-8")


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


assert "#define PIOS_PCIE1_IRQ              255U" in platform
assert "#define PIOS_PCIE1_MSI_IRQ          256U" in platform
assert "still masked" in platform
assert "MSI INTID 255/256 masked (no handler yet)" in pcie1

init = body_after(pcie1, "bool pcie1_init(void)")
assert "gic_enable_irq" not in init
assert "irq_register" not in init
assert "airq_post" not in init
assert "PIOS_PCIE1_IRQ" not in init
assert "PIOS_PCIE1_MSI_IRQ" not in init

assert "gic_enable_irq" not in pcie1
assert "irq_register" not in pcie1
assert "airq_post" not in pcie1

print("issue #147: PCIe1 MSI INTIDs remain masked until an AIRQ-only top half exists")
