from pathlib import Path


root = Path(__file__).resolve().parent.parent
platform = (root / "include" / "platform.h").read_text(encoding="utf-8")
pcie1_header = (root / "include" / "pcie1.h").read_text(encoding="utf-8")
pcie1 = (root / "src" / "pcie1.c").read_text(encoding="utf-8")
mmu = (root / "src" / "mmu.c").read_text(encoding="utf-8")
start = (root / "src" / "start.S").read_text(encoding="utf-8")


assert "#define PIOS_DMA_PCIE1_SIZE         0x00200000UL" in platform
assert "#define PIOS_DMA_PCIE1_BASE         0x04E00000UL" in platform
assert "#define PIOS_DMA_PCIE1_BASE         0x46800000UL" in platform

assert "PCIE1_DMA_PCIE_BASE" in pcie1
assert "PCIE1_BAR2_SIZE_ENC" in pcie1
assert "PIOS_DMA_PCIE1_BASE" in pcie1
assert "MISC_RC_BAR3_CONFIG_LO, 0" in pcie1
assert "MISC_UBUS_BAR3_CONFIG_REMAP, 0" in pcie1
assert "PIOS_DMA_PCIE1_BASE + PIOS_DMA_PCIE1_SIZE == PIOS_FB_BACK_BASE" in pcie1
assert "PIOS_IPC_SHM_BASE + PIOS_IPC_SHM_SIZE == PIOS_DMA_PCIE1_BASE" in pcie1

assert "bool pcie1_dma_prepare_to_device" in pcie1
assert "dcache_clean_range((u64)(usize)ptr, len);" in pcie1
assert "bool pcie1_dma_prepare_from_device" in pcie1
assert "bool pcie1_dma_complete_from_device" in pcie1
assert "dcache_invalidate_range((u64)(usize)ptr, len);" in pcie1
assert "dsb();" in pcie1

assert "if (arena > ~0ULL - arena_sz" in pcie1_header
assert "if (n > arena_sz - off || off > ~0ULL - pcie_base)" in pcie1_header
assert "return pcie1_dma_addr_in" in pcie1_header

# The boot table starts NC, makes only private RAM and the framebuffer WB, and
# leaves blocks 36-39 (which contain the PCIe1 arena) NC at first enable.
assert "mov     x2, #0x705" in start
assert "mov     x5, #4" in start
assert "cmp     x5, #36" in start
assert "mov     x5, #40" in start
assert "cmp     x5, #48" in start
assert "blocks 36-39" in start
assert "addr >= CORE0_RAM_BASE && addr < SHARED_FIFO_BASE" in mmu
assert "addr >= FB_BACK_BASE && addr < FB_BACK_BASE + FB_BACK_SIZE" in mmu
assert "addr >= PROC_ARENA_BASE" in mmu
assert "ram_block_2m_nc(addr)" in mmu

print("issue #142: PCIe1 inbound DMA is bounded, first-enable NC, and cache-gated")
