"""Fail closed unless offline disassembly and metadata prove signed DPAS."""
import hashlib
import json
import pathlib
import re
import sys


def check(root):
    root = pathlib.Path(root)
    asm = (root / "disasm" / ".text.b50_w2a8_xmx.asm").read_text()
    metadata = (root / "disasm" / ".ze_info").read_text()
    instructions = [
        line.strip() for line in asm.splitlines()
        if re.search(r"\bdpas\.8x8\s+\(16\|M0\).*:d\s+null:d\s+\w+:b\s+\w+\.0:b", line)
    ]
    if not instructions:
        raise ValueError("Missing SIMD16 signed-byte DPAS.8x8 / int32 result")
    for pattern in (
        r"name:\s+b50_w2a8_xmx\b", r"has_dpas:\s+true\b",
        r"simd_size:\s+16\b", r"intel_reqd_sub_group_size:\s+16\b",
        r"required_work_group_size:\s+\[\s*16,\s*1,\s*1\s*\]",
    ):
        if not re.search(pattern, metadata):
            raise ValueError(f"Missing execution metadata: {pattern}")
    files = [
        root / "b50_w2a8_xmx.bin",
        root / "disasm" / ".text.b50_w2a8_xmx.asm",
        root / "disasm" / ".ze_info",
        pathlib.Path(__file__).resolve().parents[1] / "modules" / "b50_w2a8_xmx.cl",
        pathlib.Path(__file__).resolve().parents[1] / "modules" / "b50_w2a8_xmx_layout.h",
    ]
    report = {
        "device": "0xe212 / bmg-g21", "gpu_executed": False,
        "instructions": instructions,
        "sha256": {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
    }
    (root / "proof.json").write_text(json.dumps(report, indent=2) + "\n")
    print("Signed SIMD16 DPAS + metadata: PASS (offline compilation only)")


if __name__ == "__main__":
    check(sys.argv[1])
