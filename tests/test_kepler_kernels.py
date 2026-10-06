"""Decode the fixed binary subset, check numerical contracts and safe loading.

This deterministic instruction model is not GPU execution/timing validation.
Field reference: envytools gf100.c at the revision pinned by the builder.
"""
from io import StringIO
import hashlib
import json
from pathlib import Path
import re
import struct
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from kepler_kernels import build as k
import pios_kepler_load as loader

header = (ROOT / "include/kepler_kernels.h").read_text()
codes = {}
for name in k.PROFILES:
    array = re.search(r"kepler_" + name + r"_code\[\].*?=\s*\{([^}]+)\}", header, re.S)[1]
    codes[name] = b"".join(struct.pack("<Q", int(word, 16))
                           for word in re.findall(r"0x([0-9A-F]+)ULL", array))
    k.validate_code(name, codes[name])


def signed(value, bits):
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def execute(case):
    code = codes[case["kernel"]]
    params = bytes.fromhex(case["params_hex"])
    expected = bytes.fromhex(case["expected_le_hex"])
    output = bytearray(b"\xCD" * len(expected))
    inputs = [(k.INPUT_A, bytes.fromhex(case["input_a_hex"])),
              (k.INPUT_B, bytes.fromhex(case["input_b_hex"]))]
    for lane in range(k.PROFILES[case["kernel"]]["threads"]):
        regs, predicates, pc = [0] * 64, [False] * 7 + [True], 0
        for _ in range(20000):
            assert 0 <= pc <= len(code) - 8 and pc % 8 == 0
            word = struct.unpack_from("<Q", code, pc)[0]
            pc += 8
            if word == k.SCHEDULE_WORD:
                continue
            assert not word & 8
            if not predicates[(word >> 10) & 7]:
                continue
            opcode = word & 0xF800000000000007
            dst, src, other = (word >> 14) & 63, (word >> 20) & 63, (word >> 26) & 63
            immediate = (word >> 26) & 0xFFFFFFFF
            if opcode == 0x2800000000000004:
                if word & (1 << 58):
                    assert (word >> 26) & 255 == 0x21
                    regs[dst] = lane
                else:
                    offset = immediate & 0xFFFF
                    assert offset + 4 <= len(params)
                    regs[dst] = struct.unpack_from("<I", params, offset)[0]
            elif opcode == 0x1800000000000002:
                regs[dst] = immediate
            elif opcode == 0x4800000000000003:
                regs[dst] = (regs[src] + regs[other]) & 0xFFFFFFFF
            elif opcode == 0x0800000000000002:
                regs[dst] = (regs[src] + immediate) & 0xFFFFFFFF
            elif opcode == 0x5000000000000003:
                regs[dst] = (regs[src] * regs[other]) & 0xFFFFFFFF
            elif opcode == 0x6000000000000003:
                assert (word >> 46) & 3 == 3  # immediate operand
                regs[dst] = (regs[src] << (immediate & 0xFFFFF)) & 0xFFFFFFFF
            elif opcode == 0x1800000000000003:
                condition = (word >> 55) & 15
                assert condition in (1, 6)
                value = regs[src] < regs[other] if condition == 1 else regs[src] >= regs[other]
                predicates[(word >> 17) & 7] = value
                predicates[(word >> 14) & 7] = not value
            elif opcode in (0x8000000000000005, 0x9000000000000005):
                assert word & (1 << 58)  # 64-bit GPU VA
                address = regs[src] | (regs[src + 1] << 32)
                address += signed(immediate, 32)
                kind = (word >> 5) & 7
                assert kind in (1, 4)
                size = 1 if kind == 1 else 4
                if opcode == 0x8000000000000005:
                    for base, data in inputs:
                        if base <= address and address + size <= base + len(data):
                            regs[dst] = int.from_bytes(data[address - base:address - base + size],
                                                       "little", signed=kind == 1) & 0xFFFFFFFF
                            break
                    else:
                        raise AssertionError("kernel read outside supplied spans")
                else:
                    assert kind == 4 and k.OUTPUT <= address and address + 4 <= k.OUTPUT + len(output)
                    output[address - k.OUTPUT:address - k.OUTPUT + 4] = struct.pack("<I", regs[dst])
            elif opcode == 0x4000000000000007:
                pc += signed(immediate & 0xFFFFFF, 24)
            elif opcode == 0x8000000000000007:
                break
            elif opcode == 0x4000000000000004:
                pass
            else:
                raise AssertionError(f"unmodelled opcode {word:016x}")
        else:
            raise AssertionError("kernel instruction budget exceeded")
    assert output == expected
    return output


execute(k.make_case("store_const"))
for count in (1, 31, 32, 63, 64):
    a = struct.pack("<" + "I" * count, *([0x7FFFFFFF, 0xFFFFFFFF, 0, 7] * 16)[:count])
    b = struct.pack("<" + "I" * count, *([1, 1, 0x80000000, 0xFFFFFFF9] * 16)[:count])
    execute(k.make_case("vector_add_i32", a, b, cols=count))
for rows, cols in ((1, 1), (1, 31), (3, 64), (64, 1024)):
    a = bytes((i * 73 + 128) & 255 for i in range(rows * cols))
    b = bytes((i * 37 + 128) & 255 for i in range(cols))
    execute(k.make_case("matvec_i8", a, b, rows, cols))
assert struct.unpack("<i", execute(k.make_case("dot_i8", b"\x80" * 1024,
                                               b"\x80" * 1024, cols=1024)))[0] == 16777216
bitnet = k.bitnet_case()
execute(bitnet)
assert (bitnet["argmax"], bitnet["checksum"]) == (57, 170896)
for rows, cols in ((0, 1), (1, 0), (65, 1), (1, 1025), (1, 1 << 64)):
    try:
        k.make_case("matvec_i8", b"", b"", rows, cols)
    except ValueError:
        pass
    else:
        raise AssertionError("invalid launch dimensions accepted")
try:
    k.make_case("dot_i8", b"\x01", b"\x02", cols=2)
except ValueError:
    pass
else:
    raise AssertionError("short input span accepted")


class Board:
    def __init__(self, corrupt=False):
        self.memory = bytearray(0x40000)
        self.generation = 7
        self.events = []
        self.corrupt = corrupt

    def begin_posted(self, for_copy):
        assert not for_copy

    def zero(self, address, length):
        assert k.CODE_BASE <= address and address + length <= 0x240000
        self.memory[address - k.CODE_BASE:address - k.CODE_BASE + length] = b"\0" * length

    def memwrite(self, address, data):
        assert k.CODE_BASE <= address and address + len(data) <= 0x240000
        self.memory[address - k.CODE_BASE:address - k.CODE_BASE + len(data)] = data

    def memread(self, address):
        result = bytearray(self.memory[address - k.CODE_BASE:address - k.CODE_BASE + 128])
        if self.corrupt:
            result[0] ^= 1
        return result

    def record(self, **event):
        self.events.append(event)

    def flush_bar(self):
        pass

    def check_health(self):
        pass

    def read32(self, reg):
        assert reg in (0x200, 0x204)
        return 0


with tempfile.TemporaryDirectory(prefix="pios-kepler-bundle-") as tmp:
    tmp = Path(tmp)
    entries = []
    for i, (name, profile) in enumerate(k.PROFILES.items()):
        code = codes[name]
        (tmp / (name + ".bin")).write_bytes(code)
        entries.append({"name": name, "file": name + ".bin", "bytes": len(code),
                        "sha256": hashlib.sha256(code).hexdigest(),
                        "vram_offset": k.CODE_BASE + i * k.CODE_SLOT, "entry_offset": 0,
                        "shared_bytes": 0, "local_bytes": 0, **profile})
    manifest = {"chipset": 0xE7, "isa": "gf100:gk104", "sm": 30,
                "envytools_revision": k.ENVY_REVISION, "hardware_verified": False,
                "kernels": entries}
    (tmp / "manifest.json").write_text(json.dumps(manifest))
    kernels = loader.bundle(tmp)
    board = Board()
    result = loader.load(board, kernels)
    assert result["readback_verified"] and not result["dispatched"]
    assert board.memory[k.OUTPUT - k.CODE_BASE:k.OUTPUT - k.CODE_BASE + 4] == b"\xCD" * 4
    try:
        loader.load(Board(corrupt=True), kernels)
    except RuntimeError as exc:
        assert "readback" in str(exc)
    else:
        raise AssertionError("corrupted upload accepted")
    entries[0]["vram_offset"] = 0
    (tmp / "manifest.json").write_text(json.dumps(manifest))
    try:
        loader.bundle(tmp)
    except ValueError:
        pass
    else:
        raise AssertionError("relocated kernel accepted")
print("Kepler SM30: fixed binary model, span bounds, BitNet Wq reference and upload gates passed")
