#!/usr/bin/env python3
"""Regression gate for issue #132: IO-only SDIO starts with CMD5."""

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sdio = (ROOT / "src" / "sdio.c").read_text(encoding="utf-8")

init_start = sdio.index("bool sdio_init(void)")
cmd5_start = sdio.index("/* CMD5: IO_SEND_OP_COND", init_start)
prefix = sdio[init_start:cmd5_start]

assert "skip CMD0 for IO-only card" in prefix
assert "sdio_send_cmd(SDIO_CMD0" not in prefix
assert "SDIO_CMD0" not in prefix
assert "sdio_send_cmd(SDIO_CMD5" in sdio[cmd5_start:]

print("issue #132: IO-only SDIO skips CMD0 and begins enumeration with CMD5")
