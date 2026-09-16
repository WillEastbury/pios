from pathlib import Path


nic = (Path(__file__).resolve().parent.parent / "src" / "nic.c").read_text(
    encoding="utf-8"
)


def body_after(signature: str) -> str:
    start = nic.index(signature)
    opening = nic.index("{", start)
    depth = 0
    for index in range(opening, len(nic)):
        if nic[index] == "{":
            depth += 1
        elif nic[index] == "}":
            depth -= 1
            if depth == 0:
                return nic[opening + 1:index]
    raise AssertionError(f"unterminated {signature}")


wifi_init = body_after("bool nic_init_wifi(void)")
assert 'nic_load("wifi-cyw43455", NIC_IFACE_WIFI)' in wifi_init
assert "if (!nic_iface_active(NIC_IFACE_WIRED))" in wifi_init
assert "iface_initialized[NIC_IFACE_WIFI] = true;" in wifi_init
assert "g_nic = iface_backends[NIC_IFACE_WIFI];" in wifi_init
assert "PIOS_PLATFORM_PI3" not in wifi_init
assert "PIOS_PLATFORM_PIZERO2W" not in wifi_init

print("issue #106: WiFi is default whenever no wired backend is active")
