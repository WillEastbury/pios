from pathlib import Path


source = (Path(__file__).resolve().parent.parent / "src" / "cyw43.c").read_text(
    encoding="utf-8"
)
kernel = (Path(__file__).resolve().parent.parent / "src" / "kernel.c").read_text(
    encoding="utf-8"
)

helper_start = source.index("static bool cyw43_preload_file(")
start = source.index("bool cyw43_preload_blobs(void)")
end = source.index("bool cyw43_load_firmware(void)", start)
helper = source[helper_start:start]
preload = source[start:end]

firmware_read = helper.index("u32 got = fat32_read(file, buffer + off, chunk);")
firmware_progress = helper.index("if (cyw_progress_hook)", firmware_read)
assert firmware_read < firmware_progress
assert "got == 0U || got > chunk" in helper

assert "cyw43_preload_file(&fw, fw_buf, CYW_FW_MAX_SIZE," in preload
assert "cyw43_preload_file(&nv, nvram_buf, CYW_NVRAM_MAX," in preload
assert "cyw43_preload_file(&clm, clm_buf, CYW_CLM_MAX," in preload

auto_wifi = kernel.index("bp_log(\"[wifi] auto init (no wired NIC)...\")")
auto_hook = kernel.index("cyw43_set_progress_hook(wifi_upload_progress);", auto_wifi)
auto_init = kernel.index("wifi_boot_ready = nic_init_wifi();", auto_wifi)
assert auto_hook < auto_init

print("WiFi preload renews watchdog only after bounded FAT-read progress")
