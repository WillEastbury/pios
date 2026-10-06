"""Pin the offline channel descriptors and bounded copy experiment lifecycle."""
from io import StringIO
from pathlib import Path
import struct
import sys
import re
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import pios_kepler_channel as k

plan = dict(k.descriptors())
for pa in (0, k.INST - 4096, 0x500000, k.INST + 1, 1 << 40):
    try:
        k.page_entry(pa)
    except ValueError:
        pass
    else:
        raise AssertionError("PTE escaped reserved VRAM")
for pa in (k.GPFIFO, k.PUSH, k.SOURCE, k.DEST, k.FENCE):
    pte = struct.unpack("<Q", plan[k.SPT + (pa // 4096) * 8])[0]
    assert (pte >> 33) & 3 == 0
    assert pte == (pa >> 8) | 1 | (1 << 32)
assert plan[k.PGD] == struct.pack("<II", 0, 0x1401)
assert plan[k.BAR_PGD] == struct.pack("<II", 0, 0x1801)
assert plan[k.BAR_SPT] == struct.pack("<Q", k.page_entry(k.USERD))
assert struct.unpack_from("<QQ", plan[k.INST], 0x200) == (k.PGD, k.VA_LIMIT - 1)
assert struct.unpack_from("<I", plan[k.INST], 0x4C)[0] == 9 << 16
assert struct.unpack_from("<QQ", plan[k.BAR_INST], 0x200) == (k.BAR_PGD, 4095)
assert struct.unpack_from("<I", plan[k.PUSH])[0] == 0x20018000
assert struct.unpack_from("<I", plan[k.PUSH], 4)[0] == 0xA0B5
assert struct.unpack_from("<I", plan[k.PUSH], len(plan[k.PUSH]) - 4)[0] == 0x18E
assert plan[k.GPFIFO] == struct.pack("<II", k.PUSH, len(plan[k.PUSH]) // 4 << 10)
assert set(plan) == {k.INST, k.BAR_INST, k.RUNLIST, k.PUSH, k.GPFIFO,
                    k.PGD, k.BAR_PGD, k.BAR_SPT,
                    *(k.SPT + (pa // 4096) * 8 for pa in
                      (k.GPFIFO, k.PUSH, k.SOURCE, k.DEST, k.FENCE))}

transport = k.Channel("unused", StringIO())
transport.generation = 0xFFFFFFFF
transport.deadline = float("inf")
storage = bytearray(256)


def terminal(host, text):
    kernel = (Path(__file__).resolve().parent.parent / "src" / "kernel.c").read_text()
    response = kernel[kernel.index("static u32 http_build_terminal_response("):]
    limit = int(re.search(r"char cmd\[(\d+)\];", response)[1])
    assert host == "unused" and len(text) < limit
    verb, generation, address, *payload = text.split()[2:]
    assert int(generation) == 0xFFFFFFFF
    offset = int(address) - k.INST
    if verb == "read":
        return "kepler vram ok " + storage[offset:offset + 128].hex()
    assert verb == "write" and len(payload[0]) == 64
    storage[offset:offset + 32] = bytes.fromhex(payload[0])
    return "kepler vram ok"


with patch("pios_kepler_post.terminal", side_effect=terminal):
    transport.memwrite(k.INST + 60, bytes(range(132)))
assert storage == b"\0" * 60 + bytes(range(132)) + b"\0" * 64


class FakeBoard:
    def __init__(self, fault=None):
        self.mem = bytearray(0x100000)
        self.reg = {0x204: 0, 0x2630: 0, 0x2140: 0, 0x2A04: 0,
                    0x4413C: 0x10000100, 0x4410C: 0, 0x4414C: 0}
        self.initial = self.reg.copy()
        self.events = []
        self.fault = fault
        self.reads = 0
        self.deadline = float("inf")

    def begin_posted(self):
        self.events.append("posted")

    def zero(self, pa, length):
        assert k.INST <= pa < 0x200000 and pa + length <= 0x200000
        self.mem[pa - k.INST:pa - k.INST + length] = b"\0" * length

    def memwrite(self, pa, data):
        assert k.INST <= pa and pa + len(data) <= 0x200000
        self.mem[pa - k.INST:pa - k.INST + len(data)] = data

    def memread(self, pa):
        self.reads += 1
        assert self.reads < 100
        return self.mem[pa - k.INST:pa - k.INST + 128]

    def read32(self, reg):
        return self.reg.get(reg, 0)

    def write32(self, reg, value):
        assert reg != 0x2000, "no fabricated doorbell"
        self.events.append((reg, value))
        self.reg[reg] = value
        if reg == 0x2274 and value == (4 << 20) | 1:
            assert self.reg[0x200] & 0x40
            assert self.reg[0x140] == self.reg[0x104904] == 0
            assert self.memread(k.USERD + 0x80)[12:16] == k.words([1])
            assert "flush_bar" in self.events
            if self.fault != "missing":
                self.memwrite(k.DEST + 32, self.memread(k.SOURCE + 32)[:64])
                self.memwrite(k.FENCE, k.words([k.TOKEN]))
                if self.fault == "corrupt":
                    self.memwrite(k.DEST, k.words([0]))

    def flush_bar(self):
        self.events.append("flush_bar")

    def flush_vm(self, pgd, hub=False):
        assert (pgd, hub) in ((k.PGD, False), (k.BAR_PGD, True))

    def poll(self, reg, mask, expected):
        assert self.reg.get(reg, 0) & mask == expected

    def check_health(self):
        self.events.append("healthy")

    def record(self, **event):
        self.events.append(event)

    def diagnose(self):
        self.events.append("diagnose")


for fault in (None, "missing", "corrupt"):
    board = FakeBoard(fault)
    with patch.object(k.time, "sleep"), patch("sys.stdout", new=StringIO()):
        if fault:
            try:
                k.run(board)
            except RuntimeError as exc:
                assert ("completion" if fault == "missing" else "red zones") in str(exc)
            else:
                raise AssertionError("failed GPU operation accepted")
            assert "diagnose" in board.events
        else:
            k.run(board)
    for reg, value in board.initial.items():
        assert board.reg[reg] == value
    assert board.reg[0x800000] == board.reg[0x1704] == board.reg[0x2254] == 0
    assert board.reg[0x2274] == 4 << 20
    if fault:
        assert board.events[-1] == "healthy"
    else:
        assert board.events[-1]["result"]["cleanup_complete"]

print("Kepler channel: VRAM-only descriptors, known-answer and failure cleanup passed")
