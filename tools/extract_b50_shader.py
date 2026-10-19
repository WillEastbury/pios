#!/usr/bin/env python3
"""Extract reviewed B50 Zebin sections; preserve unresolved relocation authority."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import re


def validate_tensor_contract(metadata):
    text = metadata.decode("ascii")
    required = {
        "version": "'1.62'", "grf_count": "128", "simd_size": "32",
        "inline_data_payload_size": "32",
        "required_work_group_size": "[ 16, 1, 1 ]",
    }
    for key, value in required.items():
        matches = re.findall(rf"^\s*{key}:\s*(.*?)\s*$", text, re.MULTILINE)
        if matches != [value]:
            raise ValueError(f"compiled tensor contract changed: {key}")
    blocks = re.split(r"^\s*-\s+arg_type:\s*", text, flags=re.MULTILINE)[1:]
    pointers, values = {}, {}
    local_id = False
    for block in blocks:
        kind = block.splitlines()[0].strip()
        fields = dict(re.findall(r"^\s+(\w+):\s*(.*?)\s*$", block, re.MULTILINE))
        if kind == "arg_bypointer":
            if fields.get("addrmode") != "stateless":
                raise ValueError("non-stateless tensor pointer")
            pointers[fields.get("arg_index")] = (fields.get("offset"), fields.get("size"))
        elif kind == "arg_byvalue":
            values[fields.get("arg_index")] = (fields.get("offset"), fields.get("size"))
        elif kind == "local_id":
            local_id = fields.get("size") == "192" and fields.get("offset") == "0"
    if pointers != {"0": ("16", "8"), "1": ("24", "8"), "2": ("32", "8")} or \
            values != {"3": ("40", "4"), "4": ("44", "4")} or not local_id:
        raise ValueError("compiled tensor argument/local-ID layout changed")


def extract(data):
    header = struct.Struct("<16sHHIQQQIHHHHHH")
    section = struct.Struct("<IIQQQQIIQQ")
    if len(data) < header.size:
        raise ValueError("short ELF header")
    fields = header.unpack_from(data)
    if fields[0][:7] != b"\x7fELF\x02\x01\x01":
        raise ValueError("ELF64 little-endian version 1 required")
    table, stride, count, names_index = fields[6], fields[11], fields[12], fields[13]
    if stride != section.size or not 1 <= count <= 64 or names_index >= count:
        raise ValueError("unsupported ELF section table")
    if table > len(data) or count * stride > len(data) - table:
        raise ValueError("section table outside image")
    sections = [section.unpack_from(data, table + i * stride) for i in range(count)]

    def contents(record):
        offset, size = record[4:6]
        if offset > len(data) or size > len(data) - offset:
            raise ValueError("section outside image")
        return data[offset:offset + size]

    def string(strings, offset):
        if offset >= len(strings):
            raise ValueError("string offset outside table")
        end = strings.find(b"\0", offset)
        if end < 0:
            raise ValueError("unterminated ELF string")
        return strings[offset:end].decode("ascii")

    names = contents(sections[names_index])
    by_name = {}
    for index, record in enumerate(sections):
        name = string(names, record[0])
        if name in by_name:
            raise ValueError("duplicate section name")
        by_name[name] = (index, record)
    text_index, text_record = by_name[".text.b50_w2a8"]
    text = contents(text_record)
    if not text or len(text) > 65536:
        raise ValueError("invalid kernel code size")
    _, rel = by_name[".rel.text.b50_w2a8"]
    if rel[7] != text_index or rel[6] >= count or rel[9] != 16 or rel[5] % 16:
        raise ValueError("invalid kernel relocation table")
    symbols = sections[rel[6]]
    if symbols[1] != 2 or symbols[9] != 24 or symbols[5] % 24 or symbols[6] >= count:
        raise ValueError("invalid symbol table")
    symbol_data = contents(symbols)
    strings = contents(sections[symbols[6]])
    relocations = []
    for offset, info in struct.iter_unpack("<QQ", contents(rel)):
        symbol_index, kind = info >> 32, info & 0xFFFFFFFF
        if symbol_index >= len(symbol_data) // 24 or offset > len(text) - 4:
            raise ValueError("relocation outside code/symbol table")
        name_offset = struct.unpack_from("<I", symbol_data, symbol_index * 24)[0]
        name = string(strings, name_offset)
        if kind != 2 or name != "__INTEL_PATCH_CROSS_THREAD_OFFSET_OFF_R0":
            raise ValueError("unreviewed kernel relocation")
        relocations.append({"offset": offset, "type": kind, "symbol": name})
    return text, contents(by_name[".ze_info"][1]), {
        "zebin_sha256": hashlib.sha256(data).hexdigest(),
        "code_sha256": hashlib.sha256(text).hexdigest(),
        "code_bytes": len(text),
        "relocations": relocations,
        "dispatch_ready": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--check-tensor-contract", action="store_true")
    args = parser.parse_args()
    try:
        text, metadata, report = extract(args.image.read_bytes())
        if args.check_tensor_contract:
            validate_tensor_contract(metadata)
        args.out_dir.mkdir(parents=True, exist_ok=True)
        (args.out_dir / "b50_w2a8.isa").write_bytes(text)
        (args.out_dir / "b50_w2a8.ze_info").write_bytes(metadata)
        (args.out_dir / "b50_w2a8.relocations.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="ascii")
        print(json.dumps(report, indent=2))
    except (OSError, ValueError, KeyError, struct.error) as error:
        parser.exit(1, f"B50 shader extraction failed: {error}\n")


if __name__ == "__main__":
    main()
