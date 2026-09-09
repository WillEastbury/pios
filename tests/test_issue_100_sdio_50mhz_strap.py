from pathlib import Path


sdio = (Path(__file__).resolve().parent.parent / "src" / "sdio.c").read_text(
    encoding="utf-8"
)
header = (Path(__file__).resolve().parent.parent / "include" / "sdio.h").read_text(
    encoding="utf-8"
)

start = sdio.index("static bool sdio_enable_bcm2712_50mhz(void)")
end = sdio.index("/* ── GPIO and power setup ── */", start)
strap = sdio[start:end]
assert "SDIO_CFG_MAX_50MHZ_STRAP_OVERRIDE" in strap
assert "SDIO_CFG_MAX_50MHZ_ENABLE" in strap
assert "mmio_write(cfg + SDIO_CFG_MAX_50MHZ_MODE, mode)" in strap
assert "u32 readback = mmio_read" in strap

high_speed = sdio.index("u8 high_speed = 0U;")
strap_call = sdio.index("sdio_enable_bcm2712_50mhz()", high_speed)
card_enable = sdio.index("sdio_cmd52_write(SDIO_FUNC_CIA, CCCR_HIGH_SPEED", high_speed)
clock_50 = sdio.index("sdio_set_clock(50000U)", high_speed)
assert strap_call < card_enable < clock_50
assert "_Static_assert(sizeof(struct sdio_diag) == 64U" in header

print("issue #100: BCM2712 50MHz strap is set before SDIO high speed")
