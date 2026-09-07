from pathlib import Path


kernel = (Path(__file__).resolve().parent.parent / "src" / "kernel.c").read_text(
    encoding="utf-8"
)

start = kernel.index("/* Core 0: Kernel services + network */")
end = kernel.index("timer_set_tick_hook(pios_tick_hook);", start)
boot_reactor = kernel[start:end]

assert "hardware selftest deferred; NEON fallback" in boot_reactor
assert "dma_selftest()" not in boot_reactor
assert "`dma selftest`" in boot_reactor

print("issue #162: boot defers hardware DMA selftest and preserves NEON fallback")
