from pathlib import Path


root = Path(__file__).resolve().parent.parent
header = (root / "include" / "bluetooth_hcd_bootstrap.h").read_text(
    encoding="utf-8")
source = (root / "src" / "bluetooth_hcd_bootstrap.c").read_text(
    encoding="utf-8")
kernel = (root / "src" / "kernel.c").read_text(encoding="utf-8")
runner = (root / "tests" / "run_host_tests.py").read_text(encoding="utf-8")

# The only dependencies are types and immutable Bluetooth profile identity.
assert '#include "types.h"' in header
assert '#include "bluetooth_platform_contract.h"' in header
assert '#include "types.h"' in source
assert '#include "bluetooth_hcd_bootstrap.h"' in source

# #182 is pure offline evidence logic. It must not gain a physical adapter,
# interrupt hook, artifact reader, blocking primitive, or lifecycle wiring.
for forbidden in (
    '#include "uart.h"', '#include "gpio', '#include "mmio',
    '#include "airq', '#include "watchdog', '#include "timer',
    '#include "kernel', "mmio_read(", "mmio_write(", "uart_",
    "gpio_", "airq_", "irq_register(", "watchdog_hw_", "wfe(",
    "wfi(", "delay_", "bluetooth_hcd_bootstrap_rearm",
):
    assert forbidden not in source
    assert forbidden not in header

assert "BLUETOOTH_HCD_BOOTSTRAP_WAIT_ARTIFACT" in header
assert "BLUETOOTH_HCD_BOOTSTRAP_WAIT_SAFE_STATE" in header
assert "BLUETOOTH_HCD_BOOTSTRAP_WAIT_HCD_ACK" in header
assert "BLUETOOTH_HCD_BOOTSTRAP_REQUEST_BAUD" in header
assert "BLUETOOTH_HCD_BOOTSTRAP_WAIT_BAUD_ACK" in header
assert "BLUETOOTH_HCD_BOOTSTRAP_QUARANTINED" in header
assert "source_sha256" in header and "command_sha256" in header
assert "activation_authority_attested" in header
assert "bluetooth_hcd_bootstrap_progress_evidence_since" in header
assert "instance_epoch" in header
assert "attempt_generation" in header
assert "controller_id" in header and "profile_id" in header
assert "bootstrap->control.instance_epoch" in source
assert "bootstrap->control.attempt_generation" in source
assert "bootstrap_state_has_deadline(bootstrap->control.state)" in source

hardware_gate = source[source.index(
    "bool bluetooth_hcd_bootstrap_hardware_enable_allowed"):]
assert "(void)bootstrap;" in hardware_gate
assert "(void)handle;" in hardware_gate
assert "return false;" in hardware_gate
assert "bluetooth_hcd_bootstrap" not in kernel
assert '"test_bluetooth_hcd_bootstrap.c"' in runner

print("issue #182: Bluetooth HCD bootstrap remains offline and hardware-disabled")
