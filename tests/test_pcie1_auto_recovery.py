from pathlib import Path

root = Path(__file__).resolve().parents[1]
pcie = (root / "src" / "pcie1.c").read_text(encoding="utf-8")
kernel = (root / "src" / "kernel.c").read_text(encoding="utf-8")
header = (root / "include" / "pcie1.h").read_text(encoding="utf-8")

assert "#define PCIE1_AUTO_FAST_WINDOW_MS 60000ULL" in pcie
assert "#define PCIE1_AUTO_STEADY_RETRY_MS 30000ULL" in pcie
assert "#define PCIE1_AUTO_BUDGET_MS       20ULL" in pcie
assert "#define PCIE1_AUTO_MAX_DEPTH       8U" in pcie
assert "g_auto.count >= PCIE1_SCAN_MAX" in pcie
assert "g_auto.next_bus > PCIE1_SCAN_BUS_HI" in pcie

service = pcie[pcie.index("bool pcie1_auto_service(void)"):]
service = service[:service.index("\n}", service.index("{")) + 2]
assert "if (!g_snap.malformed_topology)" in service
assert "pcie1_cfg_read(ep->bus, ep->dev, ep->func, 4U) & 7U" in service
assert "auto_configure()" in service
assert "g_path_activated" in service
assert "PCIE1_ROOT_RECOVERY_RESET_ASSERTED" in service
assert "root_recovery_step(now)" in service

root_step = pcie[pcie.index("static bool root_recovery_step(u64 now)"):]
root_step = root_step[:root_step.index("\n}", root_step.index("{")) + 2]
assert "bridge_reset_brcm(false)" in root_step
assert "program_root_link_registers(false)" in root_step
assert "g_root_recovery.due_ms = now + 20ULL" in root_step
assert "g_root_recovery.due_ms = now + 100ULL" in root_step
assert "g_root_recovery.deadline_ms = now + 200ULL" in root_step
assert "PCI_REG_BUS_NUM" in root_step
assert "pcie1_aer_init();" in root_step

writer = pcie[pcie.index("static bool auto_write_buses("):]
writer = writer[:writer.index("\n}", writer.index("{")) + 2]
assert "pcie1_cfg_write(record->bus, record->dev, record->func, 0x18U, value)" in writer
assert "pcie1_cfg_read(record->bus, record->dev, record->func," in writer

configure = pcie[pcie.index("static bool auto_configure(void)"):]
configure = configure[:configure.index("\n}", configure.index("{")) + 2]
assert "PCIE1_PLX_8748_ID" in configure
assert "record->identity" in configure
assert "record->old_buses & 0xFF000000U" in configure
assert "false);" in configure

maint = kernel[kernel.index("if (flags & CORE0_IO_MAINT)"):]
maint = maint[:maint.index("\n        }") + len("\n        }")]
assert "if (pcie1_auto_service())" in maint
assert "lzero_probe();" in maint
assert "bool pcie1_auto_service(void);" in header

print("pcie1 auto recovery: bounded bus-only BME-off policy pinned")
