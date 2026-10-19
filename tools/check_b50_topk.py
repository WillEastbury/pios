"""Capture exact offline top-k compiler contract; never authorize dispatch."""
import hashlib
import json
from pathlib import Path
import re


def check():
    repo = Path(__file__).resolve().parents[1]
    root = repo / "build_module" / "b50_topk"
    metadata_path = root / "disasm" / ".ze_info"
    metadata = metadata_path.read_text()
    for pattern in (
        r"name:\s+b50_fused_topk\b",
        r"version:\s+'1.62'",
        r"grf_count:\s+128\b",
        r"simd_size:\s+32\b",
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
    expected.update({str(i): (str(40 + (i - 4) * 4), "4") for i in range(4, 11)})
    if pointers != {"0": ("16", "8"), "2": ("24", "8")} or values != expected:
        raise ValueError("Changed top-k cross-thread payload")
    asm = root / "disasm" / ".text.b50_fused_topk.asm"
    if not asm.read_text().strip():
        raise ValueError("Missing actual disassembly")
    files = [repo / "modules" / "b50_fused_topk.cl",
             repo / "examples" / "fused_ops.pc",
             repo / "tests" / "test_b50_fused_topk.c",
             repo / "build_shader_b50_topk.bat",
             Path(__file__), root / "b50_fused_topk.bin", asm, metadata_path]
    report = {
        "device": "0xe212 / bmg-g21",
        "compiler": "Intel oneAPI ocloc 2026.0",
        "options": "-64 -cl-std=CL1.2",
        "semantic_contract": "fused_topk v1 biased-u16 big-endian Q8.8",
        "gpu_executed": False, "dispatch_ready": False, "xmx": False,
        "warning": "ocloc disasm: unexpected padding at end of kernel",
        "pointer_payload": pointers, "scalar_payload": values,
        "metadata": metadata,
        "sha256": {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in files},
    }
    (root / "proof.json").write_text(json.dumps(report, indent=2) + "\n")
    print("B50 top-k ISA/metadata/provenance captured (offline only)")


if __name__ == "__main__":
    check()
