from pathlib import Path


kernel = (Path(__file__).resolve().parent.parent / "src" / "kernel.c").read_text(
    encoding="utf-8"
)

start = kernel.index("/* Core 0: Kernel services + network */")
end = kernel.index("timer_set_tick_hook(pios_tick_hook);", start)
boot_reactor = kernel[start:end]

assert "bool dma_ok = dma_selftest();" in boot_reactor
assert "hardware copy/zero enabled" in boot_reactor
assert "hardware proof failed; see dma status" in boot_reactor
dma = (Path(__file__).resolve().parent.parent / "src" / "dma.c").read_text(encoding="utf-8")
assert "DMA_ENABLE_OFFSET" not in dma
assert "dma_start_direct" not in dma
assert "dma_selftest_mode" not in dma
assert "timer_monotonic_ms() - start >= DMA_WAIT_MS" in dma
assert "dma_zero_hw(DMA_CHAN_MEMCPY" in dma
assert "watchdog_hw_pet" not in dma

print("DMA boot: fixed shifted-CB copy/zero proofs, bounded waits, no mode guessing")
