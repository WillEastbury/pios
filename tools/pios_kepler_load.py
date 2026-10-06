#!/usr/bin/env python3
"""Load only the fixed SM30 code and store fixture; do not dispatch GR."""
import argparse
import hashlib
import json
from pathlib import Path

from kepler_kernels.build import (CODE_BASE, CODE_SLOT, ENVY_REVISION, PARAMS,
                                  OUTPUT, PROFILES, make_case, validate_code)
from pios_kepler_channel import Channel


def bundle(directory):
    with (directory / "manifest.json").open("rb") as source:
        raw = source.read(16385)
    if len(raw) > 16384:
        raise ValueError("manifest exceeds budget")
    manifest = json.loads(raw)
    if (manifest.get("chipset"), manifest.get("isa"), manifest.get("sm"),
        manifest.get("envytools_revision"), manifest.get("hardware_verified")) != \
            (0xE7, "gf100:gk104", 30, ENVY_REVISION, False):
        raise ValueError("wrong ISA or unaccepted kernel manifest")
    entries = manifest.get("kernels")
    if not isinstance(entries, list) or len(entries) != len(PROFILES):
        raise ValueError("wrong kernel set")
    result = []
    for index, (name, profile) in enumerate(PROFILES.items()):
        entry = entries[index]
        expected = {"name": name, "file": name + ".bin",
                    "vram_offset": CODE_BASE + index * CODE_SLOT, "entry_offset": 0,
                    "shared_bytes": 0, "local_bytes": 0, **profile}
        if not isinstance(entry, dict) or any(entry.get(k) != v for k, v in expected.items()):
            raise ValueError("kernel launch metadata changed")
        with (directory / expected["file"]).open("rb") as source:
            code = source.read(CODE_SLOT + 1)
        validate_code(name, code)
        if entry.get("bytes") != len(code) or entry.get("sha256") != hashlib.sha256(code).hexdigest():
            raise ValueError("kernel manifest does not match code")
        result.append((entry, code))
    return result


def load(board, kernels):
    board.begin_posted(for_copy=False)
    for entry, code in kernels:
        address = entry["vram_offset"]
        board.zero(address, CODE_SLOT)
        board.memwrite(address, code)
        actual = b"".join(board.memread(address + i) for i in range(0, len(code), 128))
        if actual[:len(code)] != code:
            raise RuntimeError("GPU code readback mismatch")
        board.record(loaded=entry["name"], bytes=len(code), vram_offset=address,
                     sha256=hashlib.sha256(actual[:len(code)]).hexdigest())
    case = make_case("store_const")
    board.zero(PARAMS, 256)
    board.memwrite(PARAMS, bytes.fromhex(case["params_hex"]))
    board.memwrite(OUTPUT - 64, b"\xCC" * 64 + b"\xCD" * 4 + b"\xCC" * 64)
    board.flush_bar()
    board.check_health()
    if board.read32(0x200) & 0x1000 or board.read32(0x204):
        raise RuntimeError("engine activated during code-only upload")
    result = {"loaded": [entry["name"] for entry, _ in kernels],
              "generation": board.generation, "readback_verified": True,
              "store_params": PARAMS, "store_output": OUTPUT,
              "dispatched": False, "compute_ready": False}
    board.record(result=result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=Path("build_kepler") / "kernels")
    parser.add_argument("--host", default="192.168.0.201")
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    kernels = bundle(args.bundle)
    if not args.execute:
        print(json.dumps({"validated": [e["name"] for e, _ in kernels], "execute": False}))
        return
    with args.log.open("x", encoding="ascii") as log:
        print(json.dumps(load(Channel(args.host, log), kernels), indent=2))


if __name__ == "__main__":
    main()
