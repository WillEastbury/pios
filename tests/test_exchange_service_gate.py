#!/usr/bin/env python3
"""Static guard for bounded p3-only exchange boot attachment and formatting."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
service = (ROOT / "src" / "exchange_service.c").read_text(encoding="utf-8")
adapter = (ROOT / "src" / "exchange.c").read_text(encoding="utf-8")
kernel = (ROOT / "src" / "kernel.c").read_text(encoding="utf-8")
header = (ROOT / "include" / "exchange_service.h").read_text(encoding="utf-8")

for token in ("storage_layout_validate_exchange", "exchange_volume_policy_attach",
              "fat32_exchange_mount", "fat32_exchange_format", "authorized_lba",
              "service_format_write", "boot_sector_is_all_zero"):
    assert token in service, f"missing attachment guard: {token}"
for forbidden in ("partition_create", "repartition", "malloc", "free("):
    assert forbidden not in service, f"service must not format or allocate: {forbidden}"
assert "sd_read_block" not in service and "sd_write_block" not in service, (
    "generic service must receive only its injected, fenced backend"
)
write = service[service.index("static bool service_write"):
                service.index("static void policy_fact_make")]
assert write.count("authorized_lba(service, lba)") == 2 and "backend.write" in write, (
    "every exchange write must be constrained to validated p3"
)
init = service[service.index("enum exchange_service_result exchange_service_init"):]
init = init[:init.index("void exchange_service_status")]
for forbidden in ("fat32_exchange_create", "fat32_exchange_rewrite",
                  "fat32_exchange_append", "fat32_exchange_delete",
                  "fat32_exchange_rename"):
    assert forbidden not in init, f"boot attachment must not mutate: {forbidden}"
assert "if (!boot_sector_is_all_zero(boot))" in init, (
    "autoformat must require a positively all-zero p3 boot sector"
)
assert "sd_read_block(0U" in adapter, "only the platform adapter may acquire MBR"
assert "PIOS_PLATFORM_QEMU_VIRT" in adapter and ".writable = false" in adapter
assert ".format_writable = true" in adapter
assert "exchange_init();" in kernel and "exchange status" in kernel
assert "read_only" in header and "exchange_service_backend" in header
print("exchange service gate: p3-only bounded autoformat and read-only attachment")
