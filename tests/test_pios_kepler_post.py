"""Test board-table gates and GPIO translations used by the operator harness."""
from io import StringIO
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
from pios_kepler_post import Board
from kepler_post import PostError


def board_rom():
    rom = bytearray(2048)
    struct.pack_into("<H", rom, 0x36, 0x100)
    rom[0x100:0x104] = bytes([0x40, 12, 1, 8])
    struct.pack_into("<H", rom, 0x104, 0x200)
    struct.pack_into("<H", rom, 0x10a, 0x300)
    rom[0x200:0x205] = bytes([0x40, 5, 3, 4, 2])
    rom[0x20d:0x211] = bytes([0x32, 0, 0, 5])
    rom[0x300:0x304] = bytes([0x41, 6, 1, 5])
    struct.pack_into("<I", rom, 0x306, 0x01080581)
    rom[0x30a] = 0x40
    return rom


rom = board_rom()
board = Board("unused", rom, StringIO(), False)
board.preflight({"GPIO", "I2C_BYTE"})
assert board.i2c_primary == 2
assert board.gpio == [(1, 1, 8, 1)]
regs = {0xd614: 0x12340000, 0xd604: 0, 0xd740: 0xff}
board.read32 = lambda addr: regs[addr]
board.write32 = lambda addr, value: regs.__setitem__(addr, value)
board.gpio_reset(rom)
assert regs[0xd614] == 0x12343008
assert regs[0xd604] == 1
assert regs[0xd740] == 1
try:
    board.command("kepler probe")
except PostError as exc:
    assert "no hardware access" in str(exc)
else:
    raise AssertionError("dry run reached hardware")
for offset, value in ((0x100, 0x30), (0x300, 0x42), (0x30a, 0),
                      (0x20d, 0x34), (0x20e, 1), (0x306, 63)):
    wrong = board_rom()
    wrong[offset] = value
    candidate = Board("unused", wrong, StringIO(), False)
    if offset == 0x30a:
        candidate.preflight({"GPIO"})
        assert candidate.gpio[0][1] == 0
        continue
    try:
        candidate.preflight({"GPIO", "I2C_BYTE"})
    except PostError:
        pass
    else:
        raise AssertionError(f"invalid board table at {offset:#x} accepted")

board.command = lambda command: "kepler i2c ok value=64"
assert board.i2c_read(0x80, 0x4c, 9) == 64
for bus, addr, reg in ((0x81, 0x4c, 9), (0x80, 0x50, 9), (0x80, 0x4c, 0)):
    try:
        board.i2c_read(bus, addr, reg)
    except PostError:
        pass
    else:
        raise AssertionError("unreviewed I2C request accepted")
print("Kepler operator POST: board GPIO/I2C topology gates and no-write mode passed")
