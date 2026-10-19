"""Capture reviewed dense Q8.8 offline contract, never authorize dispatch."""
import hashlib
import json
from pathlib import Path
import re


def check():
    repo = Path(__file__).resolve().parents[1]
    root = repo / "build_module" / "b50_matmul"
    metadata_path = root / "disasm" / ".ze_info"
    metadata = metadata_path.read_text()
    for pattern in (
        r"name:\s+b50_fused_matmul\b", r"version:\s+'1.62'",
        r"grf_count:\s+128\b", r"simd_size:\s+32\b",
        r"required_work_group_size:\s+\[\s*1,\s*1,\s*1\s*\]",
        r"inline_data_payload_size:\s+32\b",
    ):
        if not re.search(pattern, metadata):
            raise ValueError(f"Changed compiled metadata: {pattern}")
    pointers, values = {}, {}
    for block in re.split(r"^\s*-\s+arg_type:\s*", metadata, flags=re.M)[1:]:
        kind = block.splitlines()[0].strip()
        fields = dict(re.findall(r"^\s+(\w+):\s*(.*?)\s*$", block, re.M))
        if kind == "arg_bypointer":
            if fields.get("addrmode") != "stateless":
                raise ValueError("Non-stateless pointer")
            pointers[fields["arg_index"]] = (fields["offset"], fields["size"])
        elif kind == "arg_byvalue":
            values[fields["arg_index"]] = (fields["offset"], fields["size"])
    expected = {"1": ("32", "4"), "3": ("36", "4")}
    expected.update({str(i): (str(40 + (i - 4) * 4), "4") for i in range(4, 16)})
    if pointers != {"0": ("16", "8"), "2": ("24", "8")} or values != expected:
        raise ValueError("Changed dense matmul cross-thread payload")
    asm = root / "disasm" / ".text.b50_fused_matmul.asm"
    isa = asm.read_text()
    if not isa.strip() or re.search(r"\bdpasw?\b", isa, re.I):
        raise ValueError("Missing ISA or unexpected XMX lowering")
    files = [repo / "modules" / "b50_fused_matmul.cl",
             repo / "examples" / "fused_ops.pc",
             repo / "tests" / "test_b50_fused_matmul.c",
             repo / "build_shader_b50_matmul.bat", Path(__file__),
             root / "b50_fused_matmul.bin", asm, metadata_path]
    report = {
        "device": "0xe212 / bmg-g21", "compiler": "Intel oneAPI ocloc 2026.0",
        "options": "-64 -cl-std=CL1.2",
        "semantic_contract": "fused_matmul v1 dense biased-u16 big-endian Q8.8",
        "gpu_executed": False, "dispatch_ready": False, "xmx": False,
        "warning": "ocloc disasm: unexpected padding at end of kernel",
        "pointer_payload": pointers, "scalar_payload": values,
        "metadata": metadata,
        "sha256": {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in files},
    }
    (root / "proof.json").write_text(json.dumps(report, indent=2) + "\n")
    print("B50 dense matmul ISA/metadata/provenance captured (offline only)")


if __name__ == "__main__":
    check()
