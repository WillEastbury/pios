from pathlib import Path


kernel = (Path(__file__).resolve().parent.parent / "src" / "kernel.c").read_text(
    encoding="utf-8"
)
dispatch = (Path(__file__).resolve().parent.parent / "src" / "net_dispatch.c").read_text(
    encoding="utf-8"
)
nic = (Path(__file__).resolve().parent.parent / "src" / "nic.c").read_text(
    encoding="utf-8"
)
airq = (Path(__file__).resolve().parent.parent / "include" / "airq.h").read_text(
    encoding="utf-8"
)


def body_after(signature: str) -> str:
    start = kernel.index(signature)
    opening = kernel.index("{", start)
    depth = 0
    for index in range(opening, len(kernel)):
        if kernel[index] == "{":
            depth += 1
        elif kernel[index] == "}":
            depth -= 1
            if depth == 0:
                return kernel[opening + 1:index]
    raise AssertionError(f"unterminated {signature}")


eth_irq = body_after("static void core0_eth_irq_handler(void)\n{")
assert "airq_post_from(CORE_NET, AIRQ_SRC_ETH_RX" in eth_irq
assert "CORE0_IO_NET" not in eth_irq
assert "net_poll(" not in eth_irq
assert "net_dispatch_" not in eth_irq

tick = body_after("static void core0_io_tick_hook(u32 core, u64 tick)\n{")
assert "net_poll(" not in tick
assert "cyw43_poll(" not in tick
assert "CORE0_IO_NET" not in tick
assert "CORE0_IO_WIFI" not in tick
assert "airq_post_from(CORE_NET, AIRQ_SRC_WIFI" in tick

quench = body_after("static bool core0_eth_irq_drain_and_quench(bool host_route)\n{")
assert "net_poll(" not in quench
assert "net_dispatch_publish_transport" not in quench
assert "core0_eth_irq_stall_streak++" in quench
assert "for (" not in quench
assert "while (" not in quench

transport_handler = body_after(
    "static void airq_net_transport_handler(const struct airq_record *rec, void *ctx)\n{"
)
assert "net_dispatch_handle_transport()" in transport_handler
assert "net_poll(" not in transport_handler

for stage, handler in (
    ("mac", "net_dispatch_handle_mac()"),
    ("ip", "net_dispatch_handle_ip()"),
    ("tcp", "net_dispatch_handle_tcp()"),
):
    stage_handler = body_after(
        f"static void airq_net_{stage}_handler(const struct airq_record *rec, void *ctx)\n{{"
    )
    assert handler in stage_handler

service_handler = body_after(
    "static void airq_net_service_handler(const struct airq_record *rec, void *ctx)\n{"
)
assert "net_dispatch_handle_service(core0_network_service_step)" in service_handler

egress_handler = body_after(
    "static void airq_net_egress_handler(const struct airq_record *rec, void *ctx)\n{"
)
assert "net_dispatch_handle_egress()" in egress_handler

core0_main = body_after("NORETURN void core0_main(void) {")
assert "net_poll(" not in core0_main
assert "CORE0_IO_NET" not in core0_main
assert "CORE0_IO_WIFI" not in core0_main

for source in (
    "AIRQ_SRC_NET_TRANSPORT",
    "AIRQ_SRC_NET_MAC",
    "AIRQ_SRC_NET_IP",
    "AIRQ_SRC_NET_TCP",
    "AIRQ_SRC_NET_SERVICE",
    "AIRQ_SRC_NET_EGRESS",
):
    assert source in airq
    assert source in kernel

assert "NET_DISPATCH_HINT_CAPACITY" in dispatch
assert "NET_DISPATCH_RX_CAPACITY" in dispatch
assert "NET_DISPATCH_TX_CAPACITY" in dispatch
assert "net_poll(" not in dispatch
assert "net_ingress_mac_process" in dispatch
assert "net_ingress_ip_process" in dispatch
assert "net_ingress_l4_process" in dispatch
assert "net_ingress_process(" not in dispatch
assert "rx_release_tail" in dispatch
assert "ip_descs" in dispatch
assert "tcp_descs" in dispatch
assert "rx_complete" in dispatch
assert "nic_send_owned_on" in dispatch
assert "nic_send_on(" not in dispatch
assert "net_dispatch_submit_egress" in nic
assert "nic_send_owned_on" in nic
assert "core0_eth_irq_poll_fallback" not in kernel
assert "CORE0_ETH_IRQ_STALL_THRESHOLD" not in kernel
assert "CORE0_ETH_IRQ_FALLBACK_COOLDOWN_MS" not in kernel

print("network dispatch: ADR-033 MAC/IP/TCP FIFO ingress and owned-span egress")
