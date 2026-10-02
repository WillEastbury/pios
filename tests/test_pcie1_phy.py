"""Run the PCIe1 PHY/setup code against a bounded MMIO model."""
from pathlib import Path
import re
import subprocess
import tempfile
from run_host_tests import find_clang

ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "pcie1.c").read_text(encoding="utf-8")


def function(signature):
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


init = function("bool pcie1_init(void)")
assert init.index("perst_set(true)") < init.index("setup_refclk_54mhz()")
assert init.index("setup_refclk_54mhz()") < init.index("cap_set_gen2()")
assert init.index("cap_set_gen2()") < init.index("perst_set(false)")
assert init.index("timer_delay_ms(100U)") < init.index("wait_link_ms(100U)")
assert "rescal_deassert(" not in source
assert "gic_enable_irq(" not in source

constants = "\n".join(re.findall(r"^#define [A-Z0-9_]+ +[^\n]+", source, re.M))
harness = r"""
#include <assert.h>
#include <stdio.h>
#include "types.h"
#include "pcie1.h"
""" + constants + r"""
static u32 registers[0x10000/4], phy[32], address, delay_us, accesses;
static u32 writes, mismatch_reg, timeout_reg, dead_reg;
static bool read_timeout, write_timeout, drop_lcap, drop_lc2;
static const u8 expected_regs[] = {0x1f,0x16,0x17,0x18,0x19,0x1b,0x1c,0x1e};
static const u16 expected_values[] = {
    0x1600,0x50b9,0xbda1,0x0094,0x97b4,0x5030,0x5030,0x0007
};
static u32 pr(u32 off) {
    assert(off < 0x10000 && !(off & 3));
    assert(++accesses < 1000);
    if (off == RC_DL_MDIO_RD_DATA) {
        u32 reg = address & 31;
        if (reg == dead_reg) return 0xffffffff;
        if (read_timeout && reg == timeout_reg) return 0;
        return MDIO_DONE | (phy[reg] ^ (reg == mismatch_reg ? 1 : 0));
    }
    if (off == RC_DL_MDIO_WR_DATA)
        return write_timeout && (address & 31) == timeout_reg ? MDIO_DONE : 0;
    return registers[off/4];
}
static void pw(u32 off, u32 value) {
    assert(off < 0x10000 && !(off & 3));
    if (off == RC_DL_MDIO_ADDR) address = value;
    if (off == RC_DL_MDIO_WR_DATA) {
        assert(writes < sizeof(expected_regs));
        assert(address == expected_regs[writes]);
        assert(value == (MDIO_DONE | expected_values[writes]));
        phy[address & 31] = value & 0xffff;
        writes++;
    }
    if (!drop_lcap || off != 0xb8) registers[off/4] = value;
}
static u16 mmio_read16(u64 addr) {
    assert(addr == PCIE1_RC_BASE + 0xdc);
    return (u16)registers[0xdc/4];
}
static void mmio_write16(u64 addr, u16 value) {
    assert(addr == PCIE1_RC_BASE + 0xdc);
    if (!drop_lc2) registers[0xdc/4] = (registers[0xdc/4] & 0xffff0000) | value;
}
static void timer_delay_us(u32 n) { delay_us += n; }
static void uart_puts(const char *s) { (void)s; }
static void uart_hex(u32 n) { (void)n; }
""" + "\n".join(function(signature) for signature in (
    "static bool mdio_wait(", "static bool mdio_read(",
    "static bool mdio_write(", "static bool setup_refclk_54mhz(",
    "static bool cap_set_gen2(",
)) + r"""
static void reset(void) {
    memset(registers,0,sizeof(registers)); memset(phy,0,sizeof(phy));
    address=delay_us=accesses=writes=0;
    mismatch_reg=timeout_reg=dead_reg=99;
    read_timeout=write_timeout=drop_lcap=drop_lc2=false;
    registers[RC_PL_PHY_CTL_15/4]=0x4dbc0014;
    registers[PCI_REG_CAP_PTR/4]=0x48;
    registers[0x48/4]=0x4813ac01;
    registers[0xac/4]=0x00420010;
    registers[0xb8/4]=0x00655813;
    registers[0xdc/4]=0x12340003;
}
int main(void) {
    reset();
    assert(setup_refclk_54mhz());
    assert(writes==8 && delay_us==100);
    assert(registers[RC_PL_PHY_CTL_15/4]==0x4dbc0012);
    assert(cap_set_gen2());
    assert(registers[0xb8/4]==0x00655812);
    assert(registers[0xdc/4]==0x12340002); /* Upper status half untouched. */
    for (u32 n=0;n<sizeof(expected_regs);n++) {
        reset(); write_timeout=true; timeout_reg=expected_regs[n];
        assert(!setup_refclk_54mhz());
        assert(writes==n+1 && delay_us==100);
        assert(registers[RC_PL_PHY_CTL_15/4]==0x4dbc0014);
    }
    for (u32 n=1;n<sizeof(expected_regs);n++) {
        reset(); read_timeout=true; timeout_reg=expected_regs[n];
        assert(!setup_refclk_54mhz() && delay_us==100);
        reset(); mismatch_reg=expected_regs[n];
        assert(!setup_refclk_54mhz());
        assert(writes==n+1);
        reset(); dead_reg=expected_regs[n];
        assert(!setup_refclk_54mhz() && delay_us==100);
    }
    reset(); registers[RC_PL_PHY_CTL_15/4]=0xffffffff;
    assert(!setup_refclk_54mhz());
    reset(); drop_lcap=true; assert(!cap_set_gen2());
    reset(); drop_lc2=true; assert(!cap_set_gen2());
    reset(); registers[0x48/4]=0x4801; assert(!cap_set_gen2()); /* cycle */
    reset(); registers[PCI_REG_CAP_PTR/4]=0x49; assert(!cap_set_gen2());
    reset(); registers[PCI_REG_CAP_PTR/4]=0xfc;registers[0xfc/4]=0x10;
    assert(!cap_set_gen2());
    puts("PCIe1: 54MHz PLL, bounds, readback failures, Gen2 and ordering passed");
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="pios-pcie-phy-") as tmp:
    c = Path(tmp) / "phy.c"
    exe = Path(tmp) / "phy.exe"
    c.write_text(harness, encoding="utf-8")
    subprocess.run([find_clang(), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "tests" / "stubinc"),
                    "-I", str(ROOT / "include"), str(c), "-o", str(exe)],
                   check=True)
    subprocess.run([str(exe)], check=True)
