"""Pin the live command protocol without reading a real credential file."""
import importlib.util
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("wifi_join", ROOT / "tools" / "pios_wifi_join.py")
join = importlib.util.module_from_spec(spec)
spec.loader.exec_module(join)

scan = ("WiFi scan count=1\nTest bssid=02:01:02:03:04:05 "
        "ch=8 cs=00001008 rssi=-42 sec=00000001")
assert join.select_scan_target(scan, "Test") == ("02:01:02:03:04:05", 0x1008)
assert join.select_scan_target(scan, "Absent") is None

config = '{"ssid":"Test","password":"test-only-password","mode":"wpa2"}'
calls = []


def terminal(host, command, timeout):
    calls.append(command.split()[1])
    if command == "wifi init":
        return "WiFi init OK"
    if command == "wifi scan":
        return "WiFi scan started; use wifi results"
    if command == "wifi results":
        return scan
    if command.startswith("wifi joinpmk "):
        assert command.endswith("02:01:02:03:04:05 1008")
        return "WiFi join started"
    if command == "wifi status":
        return "wifi active=0 link=2 initialized=1"
    if command == "wifi activate":
        return "WiFi active at 192.168.0.202/16; wired remains at 192.168.0.201/16"
    raise AssertionError(command.split()[1])


with patch.object(Path, "read_text", return_value=config), \
     patch.object(sys, "argv", ["join", "--activate"]), \
     patch.object(join, "terminal_command", side_effect=terminal), \
     patch.object(join.time, "sleep"), \
     patch.object(join.time, "monotonic", side_effect=range(0, 1000, 6)):
    assert join.main() == 0
assert calls.count("init") == 1
assert calls[-1] == "activate"

with patch.object(Path, "read_text", return_value=config), \
     patch.object(sys, "argv", ["join"]), \
     patch.object(join, "terminal_command", return_value="WiFi init FAILED") as cmd:
    assert join.main() == 1
    assert cmd.call_count == 1
print("WiFi join tool: initialization, fresh target and activation protocol passed")
