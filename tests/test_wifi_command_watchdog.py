from pathlib import Path


kernel = (Path(__file__).resolve().parent.parent / "src" / "kernel.c").read_text(
    encoding="utf-8"
)


def body_after(signature: str) -> str:
    start = kernel.index(signature)
    opening = kernel.index("{", start)
    depth = 0
    for index in range(opening, len(kernel)):
        if kernel[index] == "{":
            depth += 1
        elif kernel[index] == "}":
            depth -= 1
            if depth == 0:
                return kernel[opening + 1:index]
    raise AssertionError(f"unterminated {signature}")


begin = body_after("static void wifi_command_begin(void)")
finish = body_after("static void wifi_command_finish(void)")
assert "cyw43_set_progress_hook(wifi_upload_progress)" in begin
assert "watchdog_hw_pet()" in begin
assert "watchdog_hw_pet()" in finish
assert "watchdog_hw_disable()" not in begin + finish

for command in ("wifi probe", "wifi prepare", "wifi load", "wifi chip", "wifi init"):
    branch = kernel.split(f'}} else if (http_streq(cmd, "{command}")) {{', 1)[1].split(
        '} else if (http_streq(cmd,', 1
    )[0]
    assert "wifi_command_begin();" in branch, command
    assert "wifi_command_finish();" in branch, command
    assert "watchdog_hw_disable()" not in branch, command

chip = kernel.split('} else if (http_streq(cmd, "wifi chip")) {', 1)[1].split(
    '} else if (http_streq(cmd, "wifi init")) {', 1
)[0]
assert chip.index("cyw43_preload_blobs()") < chip.index("cyw43_init()")

print("WiFi command watchdog and preload contracts passed")
