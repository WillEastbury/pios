from pathlib import Path

root = Path(__file__).resolve().parents[1]
kernel = (root / "src" / "kernel.c").read_text(encoding="utf-8")
capsvc = (root / "src" / "capsvc.c").read_text(encoding="utf-8")

assert "module_dispatch_call(MODULE_ID_MAC" in kernel
assert "module_dispatch_call(MODULE_ID_IP" in kernel
assert kernel.count("module_dispatch_call(MODULE_ID_TCP") >= 2
assert "module_dispatch_call(" in capsvc and "MODULE_ID_CAPSULE" in capsvc

transport = kernel[kernel.index("static void airq_net_transport_handler"):
                   kernel.index("static u64 module_mac_fallback")]
assert "module_dispatch_call" not in transport
egress = kernel[kernel.index("static void airq_net_egress_handler"):
                kernel.index("/* Arm RP1 Ethernet")]
assert "module_dispatch_call" not in egress

assert "net_dispatch_handle_mac();" in kernel
assert "net_dispatch_handle_ip();" in kernel
assert "net_dispatch_handle_tcp();" in kernel
assert "net_dispatch_handle_service(core0_network_service_step);" in kernel
assert "capsvc_preload_program_impl" in capsvc
assert "capsvc_poll_impl" in capsvc

print("reloadable stages: capsule + MAC/IP/TCP dispatch; IRQ/FIFO ownership static")
