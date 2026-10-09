"""Pin the redistributed BMG GuC binary and execute the documented CSS policy."""
from hashlib import sha256
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parent.parent
BLOB = ROOT / "firmware" / "xe" / "bmg_guc_70.bin"
EXPECTED_SHA256 = "de81c75f46a127c33cd59f604d800e9ffc7ed3495967ba0d8767cd6985ab398b"
EXPECTED_SOURCE_COMMIT = "79104f902411a949afe1da25dc25e05e3160fab0"

data = BLOB.read_bytes()
assert sha256(data).hexdigest() == EXPECTED_SHA256
u32 = lambda offset: struct.unpack_from("<I", data, offset)[0]
version = lambda value: ((value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF)

header_dw, total_dw = u32(4), u32(24)
key_dw, modulus_dw, exponent_dw = u32(28), u32(32), u32(36)
css_bytes = (header_dw - key_dw - modulus_dw - exponent_dw) * 4
ucode_bytes = (total_dw - header_dw) * 4
rsa_bytes = key_dw * 4

assert len(data) == 385856
assert u32(0) == 6
assert css_bytes == 128
assert ucode_bytes == 385344
assert rsa_bytes == 384
assert css_bytes + ucode_bytes + rsa_bytes == len(data)
assert version(u32(64)) == (70, 72, 1)
assert version(u32(68)) == (1, 38, 1)
assert u32(120) == 9441280
assert (u32(124) >> 2) & 3 == 0
print(f"BMG GuC artifact: v70.72.1, source {EXPECTED_SOURCE_COMMIT}, SHA-256 pinned")
