import hashlib
import pathlib
import struct
import subprocess
import sys
import tempfile
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

root = pathlib.Path(__file__).resolve().parents[1]
module = (root / "src" / "module.c").read_text(encoding="utf-8")
mmu = (root / "src" / "mmu.c").read_text(encoding="utf-8")
kernel = (root / "src" / "kernel.c").read_text(encoding="utf-8")
platform = (root / "include" / "platform.h").read_text(encoding="utf-8")

assert "PIOS_MODULE_CODE_BASE       0x14000000UL" in platform
assert "PIOS_MODULE_STATE_BASE      0x14400000UL" in platform
assert "tlbi vmalle1is" in mmu
assert "PTE_AP_RO_EL1" in mmu and "PTE_AP_RW_EL1 | PTE_PXN" in mmu
assert "module_stage_image" in module and "sha256(" in module
assert "MODULE_DRAIN_TARGET_MS" in module
assert "slot_has_callers" in module and "module_rollback" in module
assert "image_bytes - signed_bytes != MODULE_SIGNATURE_BYTES" in module
assert "ed25519_verify(image + signed_bytes, image, signed_bytes" in module
assert "header->arena_schema == 0U" in module
assert "header->module_id == MODULE_ID_B50" in module
assert "header->module_id == MODULE_ID_TENSOR" in module
assert "old && old(MODULE_OP_QUIESCE" in module
assert "control->active_slot = ~0U" in module
assert "control->entry = 0U" in module
assert "/api/admin/module-update" in kernel
assert "module_update.active" in kernel
assert "module_stage_image(ota_stage_buf" in kernel
assert "module_activate(header->module_id)" in kernel

with tempfile.TemporaryDirectory() as td:
    td = pathlib.Path(td)
    code = td / "code.bin"
    out = td / "proof.pmod"
    code.write_bytes(bytes(range(64)))
    key = Ed25519PrivateKey.generate()
    key_path = td / "key.pem"
    key_path.write_bytes(key.private_bytes(
        serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
        serialization.NoEncryption()))
    subprocess.run([
        sys.executable, str(root / "tools" / "build_pmod.py"), str(code),
        "--out", str(out), "--module-id", "7", "--generation", "3",
        "--arena-schema", "1", "--signing-key", str(key_path)
    ], check=True, capture_output=True, text=True)
    blob = out.read_bytes()
    header = struct.Struct("<IHH7I32s")
    values = header.unpack_from(blob)
    assert values[0] == 0x504D4F44 and values[1] == 2
    assert values[2] == header.size and values[3] == 7
    assert values[7] == 3 and values[8] == 64
    assert values[10] == hashlib.sha256(blob[header.size:-64]).digest()
    key.public_key().verify(blob[-64:], blob[:-64])
    altered = bytearray(blob[:-64])
    altered[12] ^= 1
    from cryptography.exceptions import InvalidSignature
    try:
        key.public_key().verify(blob[-64:], bytes(altered))
        raise AssertionError("modified manifest accepted")
    except InvalidSignature:
        pass

print("module loader: PMOD, W^X, A/B drain/rollback, module OTA pinned")
