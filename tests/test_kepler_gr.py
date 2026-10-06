from io import StringIO
from pathlib import Path
import sys
import struct

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import pios_kepler_gr as g
import pios_kepler_compute as c


def reference():
    ref = g.Reference.__new__(g.Reference)
    ref.tables, ref.packs, ref.firmware, ref.clock_tables = {}, {}, {}, {}
    ref.parse("""
const struct gf100_gr_init example[] = {
    { 0x418000, 33, 4, 0 },
    { 0x418100, 1, 4, 1 },
    {}
};
const struct gf100_gr_pack test[] = {
    { example },
    {}
};
static uint32_t firmware_data[] = {
    0x1234, 0x5678,
};
""")
    return ref


ref = reference()
assert ref.csdata("test", 0x418000) == [31 << 26, 128, 256]
assert ref.firmware["firmware_data"] == [0x1234, 0x5678]
for invalid in ("a+1", "-1", "0x100000000", "1;write()"):
    try:
        g.number(invalid)
    except ValueError:
        pass
    else:
        raise AssertionError("nonliteral source accepted")
ref.tables["example"] = [(0x418000, 4097, 4, 0)]
try:
    ref.entries("test")
except ValueError:
    pass
else:
    raise AssertionError("unbounded pack accepted")


class Firmware(g.GR):
    def __init__(self):
        self.writes = []
        self.reads = iter((0x300, 0x304))

    def write32(self, reg, value):
        self.writes.append((reg, value))

    def read32(self, reg):
        assert reg == 0x4091C4
        return next(self.reads)


firmware = Firmware()
firmware.firmware_load(0x409000, [11, 22], list(range(65)))
assert [(r, v) for r, v in firmware.writes if r == 0x409188] == [(0x409188, 0), (0x409188, 1)]
code = [v for r, v in firmware.writes if r == 0x409184]
assert code == list(range(65)) + [0] * 63
firmware.writes.clear()
firmware.csdata_load(0x409000, 0, [9, 10])
assert firmware.writes == [(0x4091C0, 0x02000000), (0x4091C0, 0x01000304),
                           (0x4091C4, 9), (0x4091C4, 10),
                           (0x4091C0, 0x01000004), (0x4091C4, 0x30C)]
firmware.writes.clear()
firmware.reads = iter((0x300, 0x300))
try:
    firmware.csdata_load(0x409000, 0, [1])
except RuntimeError:
    pass
else:
    raise AssertionError("hub list overwritten inside its fixed firmware data")


class Ref:
    packs = {"gk104_clkgate_pack": []}
    firmware = {name + kind: [0] for name in ("gk104_grhub", "gk104_grgpc")
                for kind in ("_code", "_data")}

    def csdata(self, name, base):
        return [base]


class Board(g.GR):
    def __init__(self, fail=False):
        self.regs = {0x140: 3, 0x200: 0xE011212D, 0x260: 1,
                     0x409604: 0x20001, 0x502608: 0x40002, 0x500C30: 3,
                     0x100C80: 1, 0x100800: 2, 0x120074: 2, 0x409804: 0x20000}
        self.original = self.regs.copy()
        self.generation = 6
        self.fail = fail
        self.events = []

    def begin_posted(self, for_copy):
        assert not for_copy

    def zero(self, address, length):
        assert address in (g.DEBUG_READ, g.DEBUG_WRITE) and length == g.DEBUG_BYTES

    def write32(self, reg, value):
        self.regs[reg] = value

    def read32(self, reg):
        return self.regs.get(reg, 0)

    def mmio_pack(self, ref, name):
        assert name == "gk104_gr_pack_mmio"

    def idle(self):
        pass

    def firmware_load(self, base, data, code):
        assert self.regs[0x140] == 0 and self.regs[0x200] & 0x1000
        self.events.append(("firmware", base))

    def csdata_load(self, base, slot, values):
        assert len(self.events) >= 2
        self.events.append(("csdata", base, slot))

    def poll(self, reg, mask, value, seconds):
        assert len(self.events) == 7
        if self.fail:
            raise RuntimeError("injected firmware boot deadline")

    def check_health(self):
        self.events.append("healthy")

    def record(self, **event):
        pass

    def gr_diagnose(self):
        self.events.append("diagnostic")


board = Board()
result = g.initialize(board, Ref())
assert result["fecs_ready"] and not result["shader_executed"]
assert board.regs[0x140] == 0 and board.regs[0x200] & 0x1000
assert board.regs[0x4188B4] == g.DEBUG_WRITE >> 8
assert board.regs[0x503038] == 0xC0000000
board = Board(fail=True)
try:
    g.initialize(board, Ref())
except RuntimeError as exc:
    assert "deadline" in str(exc)
else:
    raise AssertionError("failed firmware boot accepted")
assert all(board.regs[reg] == board.original[reg] for reg in (0x140, 0x200, 0x260))
assert board.events[-2:] == ["diagnostic", "healthy"]
print("Kepler GR: literal tables, bounded context lists, firmware padding and failure reset passed")

pages = c.mapping_pages()
assert all(0x100000 <= address < 0x500000 for address in pages)
assert all(((pte >> 33) & 3) == 0 for pte in pages.values())
assert pages[c.CONTEXT] & 2 and pages[c.PAGEPOOL] & 2
assert not pages[c.kernels.CODE_BASE] & 2
assert g.DEBUG_READ not in pages and g.DEBUG_WRITE not in pages
descriptor = c.qmd({"threads": 1, "registers": 3}, 0)
assert len(descriptor) == 256
value = int.from_bytes(descriptor, "little")
field = lambda lo, width: (value >> lo) & ((1 << width) - 1)
assert field(202, 1) == 1 and field(736, 32) == c.DONE
assert field(800, 32) == c.DONE_VALUE
assert field(384, 32) == field(416, 16) == field(432, 16) == 1
assert field(592, 16) == field(608, 16) == field(624, 16) == 1
assert field(928, 32) == c.kernels.PARAMS and field(975, 17) == 256
assert field(1496, 8) == 3 and field(1528, 8) == 0x30
assert struct.unpack_from("<I", c.pushbuffer(), 4)[0] == 0xA0C0
assert len(c.pushbuffer()) <= 4096
patches = dict(c.patch_registers(1, 2))
assert patches[0x5030C0] == 0x14300000
assert patches[0x5030E4] == 0x0C900648
assert len(bytes.fromhex(c.proof_case("vector")["expected_le_hex"])) == 256
assert struct.unpack("<i", bytes.fromhex(c.proof_case("dot")["expected_le_hex"])) == (16777216,)
assert (c.proof_case("bitnet")["argmax"], c.proof_case("bitnet")["checksum"]) == (57, 170896)
print("Kepler compute: private VRAM maps, QMD00_06 fields and initial context patch shapes passed")
