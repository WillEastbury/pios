from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import pios_b50_mmio as b


class IO:
    def __init__(self, fault=False):
        self.regs = {}
        self.fault = fault
        self.reads = 0
        for bdf, identity in b.PATH:
            self.regs[bdf, 0] = identity
            self.regs[bdf, 4] = 0
            self.regs[bdf, 24] = 1 | (2 << 8) | (15 << 16)
            self.regs[bdf, 32] = 0
        self.regs[b.GPU, 0] = 0xE2128086
        self.regs[b.GPU, 4] = 0

    def read(self, bdf, reg):
        return self.regs[bdf, reg]

    def write(self, bdf, reg, value):
        assert reg in (4, 32) and (reg != 4 or not value & 4)
        self.regs[bdf, reg] = value

    def command(self, text):
        if text == "lzero status":
            return "5:0.0 id=E2128086 bar0=16777216 mapped=0"
        assert text == "lzero map"
        assert all(self.regs[x, 32] == b.WINDOW for x, _ in b.PATH)
        for x, _ in b.PATH:
            self.regs[x, 4] = 2
        self.regs[b.GPU, 4] = 2
        self.regs[b.GPU, 16] = 0x80000004
        return "mapped=1"

    def peek(self, address):
        assert address == b.CPU_BASE + 0xD8C
        self.reads += 1
        return 0xFFFFFFFF if self.fault else (20 << 22) | (1 << 14) | 4

    def record(self, **entry):
        pass

    def health(self):
        pass


io = IO()
assert b.prove(io)["architecture"] == 20 and io.reads == 3
io = IO(True)
try:
    b.prove(io)
except RuntimeError:
    pass
else:
    raise AssertionError("bad MMIO accepted")
assert all(io.regs[x, 4] == io.regs[x, 32] == 0 for x, _ in b.PATH)
assert io.regs[b.GPU, 4] == 0
print("B50 MMIO: exact bridge windows, BME-off gates, GMD decode and failure revocation pass")
