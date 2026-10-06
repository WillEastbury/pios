"""Exercise the boot-time PCIe mapping and reject absent active translations."""
from pathlib import Path
import subprocess
import tempfile
from run_host_tests import find_clang

ROOT = Path(__file__).resolve().parent.parent
mmu = (ROOT / "src" / "mmu.c").read_text(encoding="utf-8")
lzero = (ROOT / "src" / "lzero.c").read_text(encoding="utf-8")
kernel = (ROOT / "src" / "kernel.c").read_text(encoding="utf-8")


def function(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for end in range(opening, len(source)):
        if source[end] == "{":
            depth += 1
        elif source[end] == "}":
            depth -= 1
            if not depth:
                return source[start:end + 1]
    raise AssertionError(signature)


remap = function(mmu, "void mmu_enable_caching(void)")
assert remap.index("map_kernel_pcie1_window(l1_table_cached)") < remap.index(
    'msr    ttbr0_el1')
assert "map_kernel_pcie1_window(l1);" in function(mmu, "void mmu_init(void)")
assert "mmu_enable_caching();" in kernel
assert "mmu_init();" not in kernel
check = function(mmu, "bool mmu_device_read32_valid(")
assert "at s1e1r" in check and "(par >> 56) == 0U" in check
assert "(par & 1U) == 0U" in check
assert "mmio_read(" not in check
mapped = function(lzero, "bool lzero_map_bar0(void)")
assert mapped.index("mmu_device_read32_valid") < mapped.index(
    "pcie1_set_outbound_window")

harness = r"""
#include <assert.h>
#include <string.h>
#include "types.h"
#include "platform.h"
#include "mmu.h"
#include "lzero.h"
/* Windows has 32-bit unsigned long, unlike the target's AArch64 LP64 ABI. */
#undef PTE_UXN
#undef PTE_PXN
#define PTE_UXN (1ULL << 54)
#define PTE_PXN (1ULL << 53)
#undef L1_BLOCK_SIZE
#undef L2_BLOCK_SIZE
#define L1_BLOCK_SIZE (1ULL << 30)
#define L2_BLOCK_SIZE (1ULL << 21)
static u64 l2_table_pcie1[512] ALIGNED(4096);
static u32 cleans, reads;
void dcache_clean_range(u64 start, u64 size) {
    assert(start == (u64)(usize)l2_table_pcie1 && size == 4096);
    cleans++;
}
static struct lzero_status g_lzero;
static bool translation_valid;
bool mmu_device_read32_valid(u64 addr) {
    assert(addr >= PIOS_PCIE1_CPU_WIN_BASE);
    return translation_valid;
}
static void uart_puts(const char *msg) { (void)msg; }
static u32 mmio_read(u64 addr) {
    assert(addr == PIOS_PCIE1_CPU_WIN_BASE);
    reads++;
    return 0x0e7000a1;
}
""" + function(mmu, "static inline u64 dev_block_2m(") + "\n" + \
    function(mmu, "static void map_kernel_pcie1_window(") + "\n" + \
    function(lzero, "bool lzero_bar0_read32(") + r"""
int main(void) {
    u64 root[512] = {0};
    root[124] = 0x0060001f00000401ULL;
    map_kernel_pcie1_window(root);
    assert(cleans == 1);
    assert(PIOS_PCIE1_CPU_WIN_BASE / L1_BLOCK_SIZE == 110);
    assert(root[110] == ((u64)(usize)l2_table_pcie1 | PTE_VALID | PTE_TABLE));
    assert(root[108] == 0 && root[109] == 0 && root[111] == 0);
    assert(root[124] == 0x0060001f00000401ULL);
    for (u32 i = 0; i < 512; i++) {
        if (i < 16)
            assert(l2_table_pcie1[i] == dev_block_2m(
                PIOS_PCIE1_CPU_WIN_BASE + (u64)i * L2_BLOCK_SIZE));
        else
            assert(l2_table_pcie1[i] == 0);
    }
    u32 value = 0;
    g_lzero.bar0_size = 16U << 20;
    assert(!lzero_bar0_read32(0, &value) && reads == 0);
    g_lzero.bar0_mapped = true;
    assert(!lzero_bar0_read32(0, &value) && reads == 0);
    translation_valid = true;
    assert(!lzero_bar0_read32(1, &value) && reads == 0);
    assert(!lzero_bar0_read32(16U << 20, &value) && reads == 0);
    g_lzero.bar0_size = 0;
    assert(!lzero_bar0_read32(0, &value) && reads == 0);
    g_lzero.bar0_size = 16U << 20;
    assert(lzero_bar0_read32(0, &value) && reads == 1 && value == 0x0e7000a1);
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="pios-pcie-map-") as tmp:
    path = Path(tmp) / "mapping.c"
    exe = Path(tmp) / "mapping.exe"
    path.write_text(harness, encoding="utf-8")
    subprocess.run([find_clang(), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    "-DPIOS_PLATFORM=1", "-include", str(ROOT / "tests/stubinc/types.h"),
                    "-I", str(ROOT / "include"), str(path), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("PCIe live mapping: exact Device window, active-translation gate and bounds passed")
