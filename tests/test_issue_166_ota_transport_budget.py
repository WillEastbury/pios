from pathlib import Path


net_dispatch = (
    Path(__file__).resolve().parent.parent / "src" / "net_dispatch.c"
).read_text(encoding="utf-8")


def body_after(signature: str) -> str:
    start = net_dispatch.index(signature)
    opening = net_dispatch.index("{", start)
    depth = 0
    for index in range(opening, len(net_dispatch)):
        if net_dispatch[index] == "{":
            depth += 1
        elif net_dispatch[index] == "}":
            depth -= 1
            if depth == 0:
                return net_dispatch[opening + 1:index]
    raise AssertionError(f"unterminated {signature}")


publish = body_after("bool net_dispatch_publish_transport(nic_iface_t iface, u32 cause)")
assert "hint_pending_for(iface)" in publish
assert "dispatch_irq_save()" in publish
assert publish.index("hint_push(iface, cause)") < publish.index("wake_queues()")

transport = body_after("void net_dispatch_handle_transport(void)")
assert transport.count("hint_pop(&hint)") == 1
assert "while (attempts < NET_DISPATCH_TRANSPORT_BURST)" in transport
assert "attempts++" in transport
assert transport.index("if (!rx_has_credit())") < transport.index("hint_pop(&hint)")
assert "NET_DISPATCH_CAUSE_RECHECK" in transport
assert "dispatch_diag.transport_handled++" in transport
assert "u32 handled" not in transport

kernel = (
    Path(__file__).resolve().parent.parent / "src" / "kernel.c"
).read_text(encoding="utf-8")
quench = kernel.split(
    "static bool core0_eth_irq_drain_and_quench(bool host_route)\n{", 1
)[1].split("static u32 core0_io_take_flags", 1)[0]
assert quench.count("rp1_eth_irq_rearm();") == 1
assert quench.index("rp1_eth_irq_rearm();") < quench.index("return ")
assert "return clear;" in quench
assert "net_dispatch_publish_transport" not in quench
assert "macb_irq_ack_rx()" not in quench
handler = kernel.split(
    "static void airq_net_transport_handler(const struct airq_record *rec, void *ctx)\n{", 1
)[1].split("static void airq_net_mac_handler", 1)[0]
assert handler.index("macb_irq_ack_rx()") < handler.index("net_dispatch_handle_transport()")
assert handler.index("net_dispatch_handle_transport()") < handler.index("core0_eth_irq_drain_and_quench(false)")

write = kernel.split("static bool http_write_kernel_slot_range(", 1)[1].split(
    "static bool http_write_kernel_slot_header", 1
)[0]
assert "if (++since_pet >= 8U)" in write
assert write.index("written += n;") < write.index("net_dispatch_yield();")
assert write.index("net_dispatch_yield();") < write.index("watchdog_hw_pet();")
static_body = kernel.split(
    "else if (http_resp_len > 0 && http_static_body && http_static_off < http_static_len)", 1
)[1].split("else if (http_resp_len > 0 && http_file_id", 1)[0]
assert "if (chunk > HTTP_TX_CHUNK_MAX)" in static_body

stream_setup = kernel.split(
    "if (is_kernel_stream) {\n"
    "                    ota_update.target_slot = pios_bootctrl_target_slot();",
    1,
)[1].split("svc->stream_mode = true;", 1)[0]
invalidate = "if (!http_write_kernel_slot_header("
assert "ota_update.active = false;" in stream_setup
assert invalidate in stream_setup
assert stream_setup.index("ota_update.active = false;") < stream_setup.index(invalidate)
failure = stream_setup.split(invalidate, 1)[1]
assert '"failed to invalidate slot header"' in failure
assert "svc->stream_mode = false;" in failure
assert "500 Internal Server Error" in failure
assert failure.index("return;") < failure.index("ota_update.active = true;")

print("issue #166: bounded RX/SD progress, pre-drain GEM ack and scheduled RP1 IACK")
