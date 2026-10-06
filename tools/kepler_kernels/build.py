#!/usr/bin/env python3
"""Assemble fixed GK107/SM30 kernels; never substitute PTX or a newer ISA."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess

ENVY_REVISION = "f102b82381f3f11cee113d16374c87091db039d9"
HERE = Path(__file__).resolve().parent
PROFILES = {
    "store_const": {"registers": 3, "threads": 1, "params_bytes": 8},
    "vector_add_i32": {"registers": 10, "threads": 64, "params_bytes": 28},
    "matvec_i8": {"registers": 13, "threads": 64, "params_bytes": 32},
}
CODE_BASE = 0x200000
CODE_SLOT = 4096
SCHEDULE = "sched 0 0 0 0 0 0 0"
SCHEDULE_WORD = 0x2000000000000007
EXPECTED_SHA256 = {
    "store_const": "09e0488a71ddec8dc80c8db2379a47971bb8430604f540a987e48e77a1d2e270",
    "vector_add_i32": "d4336152bbdddfff40aeac8a339c96f930dfec4dfb7304ac28cf7975693a4904",
    "matvec_i8": "9a87685caecc369675976361b9eb1747c7f35f042d879b2572bc37f1239c08b5",
}
PARAMS = 0x210000
INPUT_A = 0x220000
INPUT_B = 0x230000
OUTPUT = 0x231040


def validate_code(name, code):
    if name not in EXPECTED_SHA256 or not code or len(code) > CODE_SLOT or len(code) % 64:
        raise ValueError("invalid fixed kernel or code size")
    if hashlib.sha256(code).hexdigest() != EXPECTED_SHA256[name]:
        raise ValueError("kernel differs from reviewed instruction image")
    for offset in range(0, len(code), 8):
        word = struct.unpack_from("<Q", code, offset)[0]
        if offset % 64 == 0 and word != SCHEDULE_WORD:
            raise ValueError("missing conservative GK104 schedule word")
        if word & 8:
            raise ValueError("short instruction in GK104 schedule block")


def make_case(name, a=b"", b=b"", rows=1, cols=1):
    if name not in (*PROFILES, "dot_i8"):
        raise ValueError("unsupported kernel")
    if not 1 <= rows <= 64 or not 1 <= cols <= 1024:
        raise ValueError("dimensions outside reviewed kernel budget")
    kernel = "matvec_i8" if name == "dot_i8" else name
    if name == "store_const":
        if a or b or rows != 1 or cols != 1:
            raise ValueError("store kernel takes only one output word")
        params = struct.pack("<Q", OUTPUT)
        expected = [0x13579BDF]
    elif name == "vector_add_i32":
        if rows != 1 or cols > 64 or len(a) != cols * 4 or len(b) != cols * 4:
            raise ValueError("vector spans must match the bounded count")
        params = struct.pack("<QQQI", INPUT_A, INPUT_B, OUTPUT, cols)
        expected = [((x + y) & 0xFFFFFFFF) for x, y in
                    zip(struct.unpack("<" + "I" * cols, a),
                        struct.unpack("<" + "I" * cols, b))]
    else:
        if name == "dot_i8" and rows != 1:
            raise ValueError("dot product has exactly one output")
        if len(a) != rows * cols or len(b) != cols:
            raise ValueError("INT8 spans must match rows and columns")
        params = struct.pack("<QQQII", INPUT_A, INPUT_B, OUTPUT, rows, cols)
        signed = lambda x: x if x < 128 else x - 256
        expected = [sum(signed(a[r * cols + c]) * signed(b[c])
                        for c in range(cols)) & 0xFFFFFFFF for r in range(rows)]
    result = b"".join(struct.pack("<I", word) for word in expected)
    return {"name": name, "kernel": kernel, "rows": rows, "cols": cols,
            "params_hex": params.hex(), "input_a_hex": a.hex(), "input_b_hex": b.hex(),
            "expected_le_hex": result.hex(),
            "expected_picoscript_be_hex": b"".join(
                struct.pack(">I", word) for word in expected).hex()}


def bitnet_case():
    header = (HERE.parent.parent / "include" / "bitnet_pi5_fixture.h").read_text()
    def array(name):
        match = re.search(r"\b" + name + r"\[\d+\]\s*=\s*\{([^}]+)\}", header)
        if not match:
            raise ValueError("missing BitNet fixture")
        return [int(n) for n in re.findall(r"-?\d+", match[1])]
    packed, activation = array("bitnet_wq_bitmap"), array("bitnet_act_i8")
    if len(packed) != 1024 or len(activation) != 64:
        raise ValueError("BitNet fixture dimensions changed")
    matrix = bytearray()
    for row in range(64):
        for col in range(64):
            bit = 1 << (col & 7)
            zero = packed[row * 16 + col // 8] & bit
            minus = packed[row * 16 + 8 + col // 8] & bit
            matrix.append(0 if zero else (255 if minus else 1))
    case = make_case("matvec_i8", matrix, bytes(x & 255 for x in activation), 64, 64)
    output = struct.unpack("<64i", bytes.fromhex(case["expected_le_hex"]))
    argmax = max(range(64), key=lambda i: output[i])
    checksum = sum((i + 1) * x for i, x in enumerate(output))
    if (argmax, checksum) != (57, 170896):
        raise ValueError("BitNet CPU reference disagrees with the existing Pi5 fixture")
    case.update(name="bitnet_wq64", argmax=argmax, checksum=checksum)
    return case


def schedule(source):
    output = []
    count = 0
    labels = []
    for line in source.splitlines():
        line = line.split("//", 1)[0].strip()
        if not line:
            continue
        if line.endswith(":"):
            labels.append(line)
            continue
        if count % 7 == 0:
            output.append(SCHEDULE)
        output.extend(labels)
        labels.clear()
        output.append("long " + line)
        count += 1
    if labels or not count:
        raise ValueError("empty kernel or trailing label")
    while count % 7:
        output.append("long nop")
        count += 1
    return "\n".join(output) + "\n"


def build(assembler, disassembler, out, header=None):
    out.mkdir(parents=True, exist_ok=True)
    manifest = {"chipset": 0xE7, "isa": "gf100:gk104", "sm": 30,
                "envytools_revision": ENVY_REVISION, "hardware_verified": False,
                "kernels": []}
    for index, (name, profile) in enumerate(PROFILES.items()):
        source = (HERE / (name + ".asm.in")).read_text(encoding="ascii")
        assembly = schedule(source)
        binary = out / (name + ".bin")
        subprocess.run([str(assembler), "-m", "gf100", "-V", "gk104", "-i",
                        "-o", str(binary)], input=assembly.encode("ascii"), check=True)
        code = binary.read_bytes()
        validate_code(name, code)
        dump = subprocess.run([str(disassembler), "-m", "gf100", "-V", "gk104",
                               "-i", "-n"], input=code, capture_output=True, check=True)
        text = dump.stdout.decode("ascii").replace("\r\n", "\n")
        if "unknown" in text.lower() or "???" in text:
            raise ValueError("assembler produced undecodable instructions")
        (out / (name + ".asm")).write_text(assembly, encoding="ascii")
        (out / (name + ".dis")).write_text(text, encoding="ascii")
        manifest["kernels"].append({
            "name": name, "file": binary.name, "bytes": len(code),
            "sha256": hashlib.sha256(code).hexdigest(),
            "source_sha256": hashlib.sha256(source.encode("ascii")).hexdigest(),
            "vram_offset": CODE_BASE + index * CODE_SLOT, "entry_offset": 0,
            "shared_bytes": 0, "local_bytes": 0, **profile,
        })
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n",
                                     encoding="ascii")
    cases = [make_case("store_const"),
             make_case("vector_add_i32", struct.pack("<4I", 0, 0xFFFFFFFF, 0x7FFFFFFF, 5),
                       struct.pack("<4I", 1, 1, 1, 0xFFFFFFF9), cols=4),
             make_case("dot_i8", bytes([128, 127, 255, 1]), bytes([128, 127, 1, 255]),
                       cols=4), bitnet_case()]
    (out / "known_answers.json").write_text(json.dumps(cases, indent=2) + "\n",
                                          encoding="ascii")
    if header:
        lines = ["#pragma once", '#include "types.h"', "",
                 "/* Generated by tools/kepler_kernels/build.py; GK107 SM30 only.",
                 " * Code upload verified separately; GR execution is not yet proven. */"]
        for name in PROFILES:
            code = (out / (name + ".bin")).read_bytes()
            values = struct.unpack("<" + "Q" * (len(code) // 8), code)
            lines.append(f"static const u64 kepler_{name}_code[] ALIGNED(64) = {{")
            for i in range(0, len(values), 2):
                lines.append("    " + ", ".join(f"0x{x:016X}ULL" for x in values[i:i + 2]) + ",")
            lines.extend(["};", ""])
        header.write_text("\n".join(lines), encoding="ascii", newline="\n")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assembler", type=Path, default=Path(
        "build_kepler") / "assembler" / "envydis" / "envyas.exe")
    parser.add_argument("--disassembler", type=Path, default=Path(
        "build_kepler") / "assembler" / "envydis" / "envydis.exe")
    parser.add_argument("--out", type=Path, default=Path("build_kepler") / "kernels")
    parser.add_argument("--header", type=Path, default=HERE.parent.parent / "include" / "kepler_kernels.h")
    args = parser.parse_args()
    result = build(args.assembler.resolve(), args.disassembler.resolve(), args.out, args.header)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
