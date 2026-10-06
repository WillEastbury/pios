import importlib.util
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("kepler_rom", ROOT / "tools/pios_kepler_rom.py")
rom = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rom)


def image():
    data = bytearray(512)
    data[:3] = b"\x55\xaa\x01"
    struct.pack_into("<H", data, 0x18, 0x40)
    data[0x40:0x44] = b"PCIR"
    struct.pack_into("<HH", data, 0x44, 0x10de, 0x0ffe)
    struct.pack_into("<H", data, 0x4a, 0x18)
    struct.pack_into("<H", data, 0x50, 1)
    data[0x55] = 0x80
    return data


def checksum(data):
    data[-1] = 0
    data[-1] = (-sum(data)) & 255
    return data


def rejected(data):
    try:
        rom.validate_rom(data)
    except ValueError:
        return
    raise AssertionError("bad ROM accepted")


good = checksum(image())
assert rom.validate_rom(good)["images"][0]["bytes"] == 512
assert rom.validate_rom(good)["executed"] is False
for n in (0, 1, 25, 88, 511):
    rejected(good[:n])
bad = image(); bad[0x45] = 0x80; rejected(checksum(bad))
bad = image(); bad[0x51] = 0xff; rejected(checksum(bad))
bad = image(); bad[0x55] = 0; rejected(checksum(bad))
bad = bytearray(good); bad[0x80] ^= 1; rejected(bad)
data = image()
data[0x80:0x86] = b"\xff\xb8BIT\x00"
data[0x88:0x8b] = bytes([12,6,1])
data[0x8b] = (-sum(data[0x80:0x8c])) & 255
data[0x8c:0x92] = b"I\x01\x02\x00\xa0\x00"
struct.pack_into("<H",data,0xa0,0xb0)
struct.pack_into("<HH",data,0xb0,0xc0,0)
data[0xc0] = 0x71
report = rom.validate_rom(checksum(data))
assert report["init_scripts"] == [0xc0]
assert report["bit_entries"][0]["id"] == "I"
bad = bytearray(data);struct.pack_into("<H",bad,0x90,511);rejected(checksum(bad))
bad = bytearray(data);struct.pack_into("<H",bad,0xb0,513);rejected(checksum(bad))

inventory = rom.inspect_scripts(checksum(data))
assert inventory["instruction_count"] == 1 and inventory["opcodes"] == {"DONE": 1}
assert inventory["executed"] is False
# Static inventory explores both conditional jump paths, but never executes.
branch = bytearray(data)
branch[0xc0:0xc4] = b"\x5c\xd0\x00\x71"
branch[0xd0:0xd4] = b"\x74\x64\x00\x71"
inventory = rom.inspect_scripts(checksum(branch))
assert inventory["instruction_count"] == 4
assert inventory["unique_delay_operands_us"] == 100

def rejected_script(data):
    try:
        rom.inspect_scripts(checksum(data))
    except ValueError:
        return
    raise AssertionError("malformed init script accepted")

bad = bytearray(data); bad[0xc0] = 0xff; rejected_script(bad)
bad = bytearray(data); bad[0xc0:0xc3] = b"\x5b\xff\xff"; rejected_script(bad)
bad = bytearray(data); bad[0xc0:0xc4] = b"\x5b\xc1\x00\x71"; rejected_script(bad)
bad = bytearray(data); bad[0xc0:0xc2] = b"\x6b\x08"; rejected_script(bad)
bad = bytearray(data); bad[0xc0:0xc2] = b"\x87\x01"; rejected_script(bad)
bad = bytearray(data); bad[0xc0:0xc6] = b"\x58\x00\x00\x00\x00\xff"; rejected_script(bad)
print("Kepler ROM: bounded PCIR, checksum, identity and BIT directory validation passed")
print("Kepler scripts: static branch inventory, bounds and unknown-op rejection passed")
