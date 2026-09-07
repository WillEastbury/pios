from pathlib import Path


root = Path(__file__).resolve().parent.parent
header = (root / "include" / "dma.h").read_text(encoding="utf-8")
source = (root / "src" / "dma.c").read_text(encoding="utf-8")

assert "#define DMA_CS_PROT          (3 << 8)" in header
assert "#define DMA_CS_ERROR         (1 << 10)" in header
assert "DMA_CS_PROT | DMA_CS_END | DMA_CS_INT | DMA_CS_ERROR" in source
assert "DMA_CS_PROT | DMA_CS_ACTIVE" in source
assert "DMA_CS_PROT | DMA_CS_ABORT" in source
assert "DMA_CS_PROT | DMA_CS_RESET" in source
assert "DMA_CS_PROT | DMA_CS_ERROR" in source

print("issue #164: BCM2712 DMA40 PROT and hardware-error bits are pinned")
