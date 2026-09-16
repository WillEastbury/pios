from pathlib import Path


cyw = (Path(__file__).resolve().parent.parent / "src" / "cyw43.c").read_text(
    encoding="utf-8"
)

start = cyw.index("bool cyw43_load_firmware(void)")
end = cyw.index("/* Request ALP clock", start)
load = cyw[start:end]
first_probe = load.index("if (!probe_f1_multiblock())")
high_speed = load.index("if (!sdio_enable_high_speed() || !probe_f1_multiblock())")
assert first_probe < high_speed
assert load.count("probe_f1_multiblock()") >= 2

print("issue #120: Function-1 probe gates SDIO high-speed transition")
