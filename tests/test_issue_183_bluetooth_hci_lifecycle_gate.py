#!/usr/bin/env python3
"""#183 gate: HCI lifecycle is pure, offline-safe, and permanently disabled."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "bt_hci_lifecycle.c").read_text(encoding="utf-8")
header = (ROOT / "include" / "bt_hci_lifecycle.h").read_text(encoding="utf-8")

for token in (
    "mmio_", "uart_", "gpio_", "mailbox_", "kernel_", "airq_",
    "hcd", "reset_gpio", "malloc", "calloc", "realloc", "free(",
    "while (", "for (;;)", "wfi(", "wfe(",
):
    assert token not in source, f"forbidden integration or blocking token: {token}"

for token in ("BT_HCI_OWNER_CORE", "BT_HCI_COMMAND_SLOT_COUNT",
              "BT_HCI_EVENT_COMMAND_COMPLETE", "BT_HCI_EVENT_COMMAND_STATUS",
              "bt_hci_event_from_irq", "bt_hci_controller_reset",
              "BT_HCI_CAPABILITY_LE_PASSIVE_SCAN"):
    assert token in header, f"required lifecycle contract missing: {token}"

# Core-0 lifecycle mutations and snapshots must be indivisible against the
# same core's HCI event IRQ. The host type shim deliberately makes these no-op.
state_apis = {
    "bt_hci_lifecycle_init": "hci_lifecycle_init_locked",
    "bt_hci_submit": "hci_submit_locked",
    "bt_hci_tx_next": "hci_tx_next_locked",
    "bt_hci_event_from_irq": "hci_event_from_irq_locked",
    "bt_hci_result_copy": "hci_result_copy_locked",
    "bt_hci_result_release": "hci_result_release_locked",
    "bt_hci_cancel": "hci_cancel_locked",
    "bt_hci_poll_timeouts": "hci_poll_timeouts_locked",
    "bt_hci_quarantine": "hci_quarantine_locked",
    "bt_hci_controller_reset": "hci_controller_reset_locked",
    "bt_hci_status_copy": "hci_status_copy_locked",
}

assert "#ifdef PIOS_HOST_TYPES_SHIM" in source
assert source.count("hci_irq_save()") == len(state_apis)
assert source.count("hci_irq_restore(irq_state)") == len(state_apis)
for public, locked in state_apis.items():
    assert source.count(f"{public}(") == 1, f"recursive public API: {public}"
    helper_at = source.index(locked)
    helper_line = source[source.rfind("\n", 0, helper_at) + 1:helper_at]
    assert "static " in helper_line, f"missing locked helper: {locked}"
    wrapper = source[source.index(f"{public}("):]
    wrapper_end = wrapper.find("\n}\n")
    if wrapper_end < 0:
        wrapper_end = wrapper.find("\n}")
    assert wrapper_end >= 0, f"unterminated API: {public}"
    wrapper = wrapper[:wrapper_end + 2]
    assert "hci_irq_save()" in wrapper, f"unlocked API: {public}"
    assert f"{locked}(" in wrapper, f"API bypasses locked helper: {public}"
    assert "hci_irq_restore(irq_state)" in wrapper, f"unrestored IRQ state: {public}"

hardware_body = source[source.index("bool bt_hci_hardware_enable_allowed"):]
assert "return false;" in hardware_body
authorization_body = source[source.index("bool bt_hci_capability_authorized"):]
assert "return false;" in authorization_body
assert "LE_SET_SCAN" not in source
assert "LE_SET_SCAN" not in header

for unrelated in ("src/kernel.c", "src/uart.c", "src/airq.c",
                  "src/bt_h4.c", "include/bt_h4.h"):
    assert "bt_hci_lifecycle" not in (ROOT / unrelated).read_text(
        encoding="utf-8"), f"unexpected runtime wiring in {unrelated}"

print("issue #183: Bluetooth HCI lifecycle remains offline and activation-disabled")
