from pathlib import Path


root = Path(__file__).resolve().parent.parent
platform = (root / "include" / "platform.h").read_text(encoding="utf-8")
gic = (root / "include" / "gic.h").read_text(encoding="utf-8")
kernel = (root / "src" / "kernel.c").read_text(encoding="utf-8")
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


assert "#define GIC_RP1_ETH_MSI     166" in gic
assert "#define PIOS_GENET_IRQ              189U   /* GIC_SPI 157 */" in platform
assert "#define PIOS_WIFI_SDIO_IRQ          306U   /* GIC_SPI 274 */" in platform
assert "#define PIOS_WIFI_SDIO_IRQ          158U   /* GIC_SPI 126, Arasan" in platform
assert "#define PIOS_GENET_IRQ              0U" in platform
assert "#define PIOS_WIFI_SDIO_IRQ          0U" in platform

genet_irq = body_after(kernel, "static void core0_genet_irq_handler(void)\n{")
assert "genet_irq_mask_rx();" in genet_irq
assert "genet_irq_ack();" in genet_irq
assert "airq_post_from(CORE_NET, AIRQ_SRC_ETH_RX" in genet_irq
assert "net_poll(" not in genet_irq
assert "if (!airq_post_from(" in genet_irq
assert "net_dispatch_publish_transport(" in genet_irq
assert "net_dispatch_handle_" not in genet_irq
assert "net_ingress_" not in genet_irq

genet_arm = body_after(kernel, "static void core0_genet_irq_arm(void)\n{")
assert "irq_register(PIOS_GENET_IRQ, core0_genet_irq_handler);" in genet_arm
assert "gic_enable_irq(PIOS_GENET_IRQ);" in genet_arm

wifi_irq = body_after(sdio, "static void sdio_gic_irq_handler(void)\n{")
assert "sdio_card_irq_mask();" in wifi_irq
assert "sdio_card_irq_ack();" in wifi_irq
assert "airq_post_from(CORE_NET, AIRQ_SRC_WIFI" in wifi_irq
assert "cyw43_" not in wifi_irq
assert "net_dispatch_" not in wifi_irq

wifi_arm = body_after(sdio, "void sdio_card_irq_arm(void)\n{")
assert "irq_register(PIOS_WIFI_SDIO_IRQ, sdio_gic_irq_handler);" in wifi_arm
assert "gic_enable_irq(PIOS_WIFI_SDIO_IRQ);" in wifi_arm

transport = body_after(
    kernel,
    "static void airq_net_transport_handler(const struct airq_record *rec, void *ctx)\n{",
)
assert "genet_irq_unmask_rx();" in transport
assert "sdio_card_irq_unmask();" in transport

tick = body_after(kernel, "static void core0_io_tick_hook(u32 core, u64 tick)\n{")
assert "AIRQ_SRC_ETH_RX" not in tick
assert "AIRQ_SRC_WIFI" not in tick

print("issue #136: board-specific IRQ INTIDs and IRQ-to-FIFO ownership are pinned")
