from pathlib import Path


root = Path(__file__).resolve().parent.parent
hooks = (root / "include" / "pico_hooks.h").read_text(encoding="utf-8")
vm = (root / "src" / "picovm.c").read_text(encoding="utf-8")


for name, code in (
    ("PV_HOOK_NET_DATAGRAMBIND", "0x38B"),
    ("PV_HOOK_NET_DATAGRAMRECV", "0x38C"),
    ("PV_HOOK_NET_DATAGRAMPEER", "0x38D"),
    ("PV_HOOK_NET_DATAGRAMSETPEER", "0x38E"),
    ("PV_HOOK_NET_DATAGRAMSEND", "0x38F"),
    ("PV_HOOK_NET_DATAGRAMCLOSE", "0x390"),
):
    assert f"#define {name}" in hooks
    assert code in hooks

assert "hook >= PV_HOOK_NET_DATAGRAMBIND" in vm
assert "hook <= PV_HOOK_NET_DATAGRAMCLOSE" in vm
assert "hook == PV_HOOK_NET_DATAGRAMRECV" in vm
assert "hook == PV_HOOK_NET_DATAGRAMPEER" in vm

print("picoscript datagram hooks: contract and fail-closed VM fallback are pinned")
