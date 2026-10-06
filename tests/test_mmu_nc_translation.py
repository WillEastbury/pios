"""Execute the production PAR_EL1 decoder used by the native DMA preflight."""
from pathlib import Path
import subprocess
import tempfile
from run_host_tests import find_clang

root = Path(__file__).resolve().parent.parent
header = (root / "include" / "mmu.h").read_text()
start = header.index("static inline bool mmu_par_identity_nc_valid(")
decoder = header[start:header.index("\n}", start) + 2]
source = '#include <assert.h>\n#include "types.h"\n' + decoder + r"""
int main(void) {
    u64 pa = 0x04e00000ULL;
    u64 valid = 0x4400000000000100ULL | pa;
    assert(mmu_par_identity_nc_valid(pa, valid));
    assert(!mmu_par_identity_nc_valid(pa, valid | 1));
    assert(!mmu_par_identity_nc_valid(pa, valid ^ 0x1000));
    assert(!mmu_par_identity_nc_valid(pa + 1, valid));
    assert(!mmu_par_identity_nc_valid(pa, valid ^ (1ULL << 7)));
    assert(!mmu_par_identity_nc_valid(pa, valid & ~(3ULL << 7)));
    assert(!mmu_par_identity_nc_valid(pa, valid ^ (0xbbULL << 56)));
    assert(!mmu_par_identity_nc_valid(pa, valid & ~(0xffULL << 56)));
    return 0;
}
"""
with tempfile.TemporaryDirectory() as directory:
    cfile = Path(directory) / "nc.c"
    exe = Path(directory) / "nc.exe"
    cfile.write_text(source)
    subprocess.run([find_clang(), "-Wall", "-Wextra", "-Werror",
                    "-I", str(root / "tests" / "stubinc"),
                    str(cfile), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print("Active NC PAR decoder: identity, faults, cacheability and shareability passed")
