from pathlib import Path


root = Path(__file__).resolve().parent.parent
gic = (root / "include" / "gic.h").read_text(encoding="utf-8")
legacy = (root / "src" / "irqc_legacy.c").read_text(encoding="utf-8")
sdio = (root / "src" / "sdio.c").read_text(encoding="utf-8")


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


assert "#define LEGACY_GPU_IRQ_SDHCI PIOS_BCM2837_GPU_IRQ_SDIO1" in gic
assert "#define ARMCTRL_BASE                 (PIOS_PERIPH_BASE + 0xB200UL)" in legacy
assert "#define ARMCTRL_IRQ_PENDING2         0x08U" in legacy
assert "#define ARMCTRL_ENABLE_IRQS2         0x14U" in legacy
assert "#define ARMCTRL_DISABLE_IRQS2        0x20U" in legacy
assert "#define ARMCTRL_GPU_IRQ_SDIO1_BIT    (1U << 30)" in legacy

route = body_after(legacy, "bool gic_legacy_sdhci_route_core0(void)\n")
assert "route &= ~QA7_GPU_ROUTE_CORE_MASK;" in route
assert "mmio_write(QA7_BASE + QA7_LOCAL_GPU_ROUTING, route);" in route
assert "QA7_GPU_ROUTE_CORE_MASK) == 0U" in route

ack = body_after(legacy, "u32 gic_acknowledge(void)\n")
assert ack.index("QA7_LOCAL_IRQ_PENDING0") < ack.index("ARMCTRL_IRQ_PENDING2")
assert "QA7_BIT_GPU_FAST" in ack
assert "return LEGACY_GPU_IRQ_SDHCI;" in ack

enable = body_after(legacy, "void gic_enable_irq(u32 intid)\n")
disable = body_after(legacy, "void gic_disable_irq(u32 intid)\n")
assert "ARMCTRL_ENABLE_IRQS2" in enable and "ARMCTRL_DISABLE_IRQS2" not in enable
assert "ARMCTRL_DISABLE_IRQS2" in disable and "ARMCTRL_ENABLE_IRQS2" not in disable
assert "mmio_read(ARMCTRL" not in enable + disable

top = body_after(sdio, "static void sdio_gic_irq_handler(void)\n")
assert top.index("sdio_card_irq_mask();") < top.index("sr16(REG_INTERRUPT)")
assert top.index("sr16(REG_INTERRUPT)") < top.index("sdio_card_irq_ack();")
assert "if (!airq_post_from(CORE_NET, AIRQ_SRC_WIFI" in top
assert "sdio_irq_diag.airq_post_failed++;" in top
assert "Both SDHCI and ARMCTRL remain masked" in top

arm = body_after(sdio, "void sdio_card_irq_arm(void)\n")
assert arm.index("sdio_card_irq_mask();") < arm.index("irq_register(PIOS_WIFI_SDIO_IRQ")
assert arm.index("irq_register(PIOS_WIFI_SDIO_IRQ") < arm.index("gic_legacy_sdhci_route_core0()")
assert arm.index("gic_legacy_sdhci_route_core0()") < arm.index("gic_enable_irq(PIOS_WIFI_SDIO_IRQ)")
assert arm.index("gic_enable_irq(PIOS_WIFI_SDIO_IRQ)") < arm.index("sdio_card_irq_host_unmask();")

unmask = body_after(sdio, "void sdio_card_irq_unmask(void)\n")
assert unmask.index("gic_enable_irq(PIOS_WIFI_SDIO_IRQ)") < unmask.index("sdio_card_irq_host_unmask();")

print("issue #134: BCM2837 ARMCTRL -> QA7 -> AIRQ SDIO1 routing is pinned")
