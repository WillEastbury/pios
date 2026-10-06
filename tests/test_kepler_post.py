"""Execute POST control flow against an explicit fake device, never hardware."""
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
from kepler_post import Post, PostError


def wr(addr, value):
    return b"\x7a" + struct.pack("<II", addr, value)


def image(code, sub=b"", condition=(0x100, 1, 1)):
    data = bytearray(4096)
    data[:3] = b"\x55\xaa\x08"
    struct.pack_into("<H", data, 0x18, 0x40)
    data[0x40:0x44] = b"PCIR"
    struct.pack_into("<HH", data, 0x44, 0x10de, 0x0ffe)
    struct.pack_into("<H", data, 0x4a, 0x18)
    struct.pack_into("<H", data, 0x50, 8)
    data[0x55] = 0x80
    data[0x80:0x86] = b"\xff\xb8BIT\x00"
    data[0x88:0x8b] = bytes([12, 6, 1])
    data[0x8b] = (-sum(data[0x80:0x8c])) & 255
    data[0x8c:0x92] = b"I\x01\x12\x00\xa0\x00"
    struct.pack_into("<HHHHHHHHH", data, 0xa0,
                     0xb8, 0, 0x100, 0xc0, 0xd0, 0, 0, 0, 0x110)
    struct.pack_into("<HH", data, 0xb8, 0x200, 0)
    struct.pack_into("<III", data, 0xc0, *condition)
    data[0x200:0x200 + len(code)] = code
    data[0x400:0x400 + len(sub)] = sub
    data[-1] = (-sum(data)) & 255
    return data


class Device:
    def __init__(self, registers=None):
        self.registers = registers or {}
        self.reads = []
        self.writes = []
        self.delays = []
        self.time = 0
        self.allow_dependencies = True

    def preflight(self, names):
        if not self.allow_dependencies and names & {"GPIO", "I2C_BYTE"}:
            raise PostError("hardware dependency unavailable")

    def read32(self, addr):
        self.reads.append(addr)
        if addr not in self.registers:
            raise PostError(f"unmodeled read {addr:#x}")
        return self.registers[addr]

    def write32(self, addr, value):
        self.writes.append((addr, value))
        self.registers[addr] = value

    def delay_us(self, delay):
        self.delays.append(delay)
        self.time += delay / 1_000_000


bus = Device()
post = Post(image(wr(0x200, 0x20) + b"\x71"), bus, lambda: bus.time)
assert post.run()["complete"]
assert bus.writes == [(0x200, 0x21)]  # PMC bit zero must be set, like Nouveau.
try:
    post.run()
except PostError:
    pass
else:
    raise AssertionError("reused POST attempt")

# Repeat counts and direct subroutine return, with bounded depth.
bus = Device()
post = Post(image(b"\x33\x03" + wr(0x204, 5) + b"\x36\x5b\x00\x04\x71",
                  wr(0x208, 7) + b"\x71"), bus, lambda: bus.time)
assert post.run()["writes"] == 4
assert bus.writes == [(0x204, 5)] * 3 + [(0x208, 7)]

# Predicates persist through a subroutine. NOT and RESUME match BIOS flow.
bus = Device({0x100: 0})
post = Post(image(b"\x75\x00" + wr(0x204, 5) + b"\x38\x5b\x00\x04\x72\x71",
                  wr(0x208, 7) + b"\x71"), bus, lambda: bus.time)
assert post.run()["writes"] == 1
assert bus.writes == [(0x208, 7)]

# The disabled condition read produces zero, not a hardware transaction.
bus = Device({0x100: 0})
post = Post(image(b"\x75\x00\x75\x00\x71"), bus, lambda: bus.time)
post.run()
assert bus.reads == [0x100]

bus = Device({0x100: 0})
post = Post(image(b"\x56\x00\x01" + wr(0x204, 5) + b"\x72\x71"),
            bus, lambda: bus.time)
post.run()
assert len(bus.delays) == 50 and not bus.writes

# MMIO failures/deadlines terminate immediately, no following write.
bus = Device({0x100: 0})
post = Post(image(b"\x56\x00\xff" + wr(0x204, 5) + b"\x71"), bus, lambda: bus.time)
post.MAX_SECONDS = 0.01
try:
    post.run()
except PostError:
    assert post.error and not post.complete and not bus.writes
else:
    raise AssertionError("deadline ignored")

bus = Device()
post = Post(image(b"\x5b\x00\x02\x71"), bus, lambda: bus.time)
try:
    post.run()
except PostError as exc:
    assert "depth" in str(exc)
else:
    raise AssertionError("recursive script unbounded")

for code in (wr(0x88004, 6) + b"\x71", b"\x33\x00\x36\x71",
             b"\x7a\x00", b"\xff", b"\x5b\xff\xff\x71"):
    bus = Device()
    try:
        Post(image(code), bus, lambda: bus.time)
    except ValueError:
        assert not bus.writes and not bus.reads
    else:
        raise AssertionError("unsafe preflight accepted")

bus = Device()
bus.allow_dependencies = False
try:
    Post(image(b"\x8e\x71"), bus, lambda: bus.time)
except PostError:
    assert not bus.writes
else:
    raise AssertionError("missing GPIO adapter silently skipped")

# Mask and copy semantics preserve bits and bound shifts.
bus = Device({0x300: 0x12345678, 0x304: 0xFFFF0000})
code = (b"\x6e" + struct.pack("<III", 0x300, 0xFFFF0000, 0xABCD) +
        b"\x90" + struct.pack("<II", 0x300, 0x304) + b"\x71")
post = Post(image(code), bus, lambda: bus.time)
post.run()
assert bus.registers[0x300] == bus.registers[0x304] == 0x1234ABCD
print("Kepler POST: real interpreter flow, effects, time/depth bounds and preflight gates passed")
