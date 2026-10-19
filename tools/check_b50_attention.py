"""Fail-closed offline compiler ABI/provenance capture, not dispatch authority."""
import hashlib
import json
from pathlib import Path
import re


def check():
    repo = Path(__file__).resolve().parents[1]
    root = repo / "build_module" / "b50_attention"
    meta = root / "disasm" / ".ze_info"
    text = meta.read_text()
    for pattern in (
        r"name:\s+b50_fused_attention\b", r"version:\s+'1.62'",
        r"grf_count:\s+128\b",
        r"simd_size:\s+32\b",
        r"required_work_group_size:\s+\[\s*1,\s*1,\s*1\s*\]",
        r"inline_data_payload_size:\s+32\b",
    ):
        if not re.search(pattern, text):
            raise ValueError(f"Compiler contract changed: {pattern}")
    pointers, values = {}, {}
    for block in re.split(r"^\s*-\s+arg_type:\s*", text, flags=re.M)[1:]:
        kind = block.splitlines()[0].strip()
        fields = dict(re.findall(r"^\s+(\w+):\s*(.*?)\s*$",block,re.M))
        if kind == "arg_bypointer":
            if fields.get("addrmode") != "stateless":
                raise ValueError("Pointer not stateless")
            pointers[fields["arg_index"]] = (fields["offset"],fields["size"])
        elif kind == "arg_byvalue":
            values[fields["arg_index"]] = (fields["offset"],fields["size"])
    expected={"1":("32","4"),"3":("36","4")}
    expected.update({str(i):(str(40+4*(i-4)),"4") for i in range(4,21)})
    if pointers != {"0":("16","8"),"2":("24","8")} or values != expected:
        raise ValueError("Changed attention cross-thread payload")
    asm=root/"disasm"/".text.b50_fused_attention.asm"
    if not asm.read_text().strip():
        raise ValueError("Missing disassembly")
    files=[repo/"modules"/"b50_fused_attention.cl",
           repo/"examples"/"fused_ops.pc",repo/"src"/"picovm.c",
           repo/"tests"/"test_b50_fused_attention.py",
           repo/"tests"/"b50_attention_host.c",
           repo/"tests"/"bitnet_fixture_vm.c",
           repo/"build_shader_b50_attention.bat",Path(__file__),
           root/"b50_fused_attention.bin",meta,asm]
    report={"device":"0xe212 / bmg-g21","compiler":"oneAPI ocloc 2026.0",
            "options":"-64 -cl-std=CL1.2","semantic_contract":
            "v1 causal Q8.8 QK/VM integer Q16.16 exp/V, exact zero tolerance",
            "gpu_executed":False,"dispatch_ready":False,"automatic_lowering":False,
            "xmx":False,"metadata":text,"pointer_payload":pointers,
            "scalar_payload":values,
            "sha256":{str(p.relative_to(repo)):hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in files}}
    (root/"proof.json").write_text(json.dumps(report,indent=2)+"\n")
    print("Offline B50 attention ISA/ABI/provenance captured; not execution proof")


if __name__ == "__main__":
    check()
