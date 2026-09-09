from pathlib import Path


sdio = (Path(__file__).resolve().parent.parent / "src" / "sdio.c").read_text(
    encoding="utf-8"
)


def body_after(signature: str) -> str:
    start = sdio.index(signature)
    opening = sdio.index("{", start)
    depth = 0
    for index in range(opening, len(sdio)):
        if sdio[index] == "{":
            depth += 1
        elif sdio[index] == "}":
            depth -= 1
            if depth == 0:
                return sdio[opening + 1:index]
    raise AssertionError(f"unterminated {signature}")


base = body_after("static bool sdio_controller_base_khz(u32 *base_khz)")
assert "PIOS_HAS_WIFI_SDIO1" in base
assert "fb_get_clock_rate_id(1U)" in base
assert "base_mhz == 0U" in base
assert "fallback" not in base

clock = body_after("static bool sdio_set_clock(u32 freq_khz)")
assert "sdio_controller_base_khz(&base_khz)" in clock
assert "if (timeout == 0U)" in clock
assert "return true;" in clock

init = body_after("bool sdio_init(void)")
assert init.count("if (!sdio_set_clock(") >= 2
assert init.index("sdio_cmd52_write(SDIO_FUNC_CIA, CCCR_HIGH_SPEED") < \
       init.index("!sdio_set_clock(50000U)")

print("SDIO uses controller-specific clocks and fails closed on setup failure")
