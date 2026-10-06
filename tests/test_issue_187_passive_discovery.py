"""#187 must not disguise endpoint configuration as passive discovery."""
from pathlib import Path
import subprocess
import tempfile
from run_host_tests import find_clang

ROOT = Path(__file__).resolve().parent.parent
header = (ROOT / "include" / "pcie1.h").read_text(encoding="utf-8")
pcie1 = (ROOT / "src" / "pcie1.c").read_text(encoding="utf-8")
lzero = (ROOT / "src" / "lzero.c").read_text(encoding="utf-8")


def body_after(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for position in range(opening, len(source)):
        if source[position] == "{":
            depth += 1
        elif source[position] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:position]
    raise AssertionError(f"unterminated {signature}")


for signature in (
    "static bool record_bridge_range(",
    "static bool record_function(",
    "static void scan_endpoints(",
):
    assert "pcie1_cfg_write(" not in body_after(pcie1, signature)

assert "program_bridge(" not in pcie1
assert "pcie1_next_bridge_bus" not in pcie1
assert "malformed_topology" in header
assert "pcie1_cfg_addr_valid" in header
assert "pcie1_bridge_range_valid" in header
assert "st != 0xFFFFFFFFU" in body_after(pcie1, "bool pcie1_link_up(void)")

rescan = body_after(pcie1, "void pcie1_rescan(void)")
assert '"link lost"' in rescan
snap = body_after(pcie1, "static void publish_snap(const char *fail)")
assert "g_snap.ep_count = 0U;" in snap
assert "g_snap.b50_found = false;" in snap

probe = body_after(lzero, "void lzero_probe(void)")
assert "lzero_probe_bars" not in probe
assert "lzero bars" in (ROOT / "src" / "kernel.c").read_text(encoding="utf-8")

harness = r"""
#include <assert.h>
#include "pcie1.h"
int main(void) {
    assert(pcie1_cfg_addr_valid(1, 31, 7, 0xffc));
    assert(pcie1_cfg_addr_valid(0, 0, 0, 0));
    assert(!pcie1_cfg_addr_valid(1, 32, 0, 0));
    assert(!pcie1_cfg_addr_valid(1, 0, 8, 0));
    assert(!pcie1_cfg_addr_valid(64, 0, 0, 0));
    assert(!pcie1_cfg_addr_valid(1, 0, 0, 0x1000));
    assert(!pcie1_cfg_addr_valid(1, 0, 0, 2));
    assert(pcie1_bridge_range_valid(1, 2, 8));
    assert(!pcie1_bridge_range_valid(1, 1, 8));
    assert(!pcie1_bridge_range_valid(1, 3, 2));
    assert(!pcie1_bridge_range_valid(63, 64, 64));
    return 0;
}
"""
with tempfile.TemporaryDirectory(prefix="pios-issue187-") as tmp:
    source = Path(tmp) / "test.c"
    executable = Path(tmp) / "test.exe"
    source.write_text(harness, encoding="ascii")
    subprocess.run([find_clang(), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                    "-include", str(ROOT / "tests" / "stubinc" / "types.h"),
                    "-I", str(ROOT / "include"), str(source), "-o", str(executable)],
                   check=True)
    subprocess.run([str(executable)], check=True)
print("#187: passive config scan, strict BDF/range guards and link-loss invalidation passed")
