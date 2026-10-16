"""Offline, header-only GGUF/safetensors inventory; never reads tensor payloads."""
import argparse
from collections import Counter
import json
from pathlib import Path
import struct


MAX_HEADER = 16 * 1024 * 1024
MAX_ITEMS = 200_000
MAX_TOKENS = 2_000_000
MAX_DEPTH = 16
MAX_FILES = 4096
MAX_FILE_SIZE = 1 << 50
MAX_U64 = (1 << 64) - 1
# GGML storage types: (name, elements per block, bytes per block).
GGML = {
    0: ("F32", 1, 4), 1: ("F16", 1, 2),
    2: ("Q4_0", 32, 18), 3: ("Q4_1", 32, 20),
    6: ("Q5_0", 32, 22), 7: ("Q5_1", 32, 24),
    8: ("Q8_0", 32, 34), 9: ("Q8_1", 32, 40),
    10: ("Q2_K", 256, 84), 11: ("Q3_K", 256, 110),
    12: ("Q4_K", 256, 144), 13: ("Q5_K", 256, 176),
    14: ("Q6_K", 256, 210), 15: ("Q8_K", 256, 292),
    16: ("IQ2_XXS", 256, 66), 17: ("IQ2_XS", 256, 74),
    18: ("IQ3_XXS", 256, 98), 19: ("IQ1_S", 256, 50),
    20: ("IQ4_NL", 32, 18), 21: ("IQ3_S", 256, 110),
    22: ("IQ2_S", 256, 82), 23: ("IQ4_XS", 256, 136),
    24: ("I8", 1, 1), 25: ("I16", 1, 2), 26: ("I32", 1, 4),
    27: ("I64", 1, 8), 28: ("F64", 1, 8), 30: ("BF16", 1, 2),
    29: ("IQ1_M", 256, 56), 39: ("MXFP4", 32, 17),
}
DTYPES = {"F64": 8, "F32": 4, "F16": 2, "BF16": 2, "I64": 8,
          "I32": 4, "I16": 2, "I8": 1, "U8": 1, "BOOL": 1,
          "U16": 2, "U32": 4, "U64": 8,
          "F8_E4M3": 1, "F8_E5M2": 1, "F8_E8M0": 1}


class Invalid(ValueError):
    pass


def require(ok, reason):
    if not ok:
        raise Invalid(reason)


def product(shape):
    require(isinstance(shape, list) and len(shape) <= MAX_DEPTH, "shape budget")
    n = 1
    for d in shape:
        require(type(d) is int and 0 <= d <= MAX_U64, "invalid dimension")
        require(not d or n <= MAX_U64 // d, "dimension overflow")
        n *= d
    return n


def ranges_check(ranges, available):
    end = 0
    for start, stop in sorted(ranges):
        require(0 <= start <= stop <= available, "tensor outside file")
        if start != stop:
            require(start >= end, "overlapping tensors")
            end = stop


class Reader:
    def __init__(self, f, size):
        self.f, self.size, self.pos, self.steps = f, size, 0, 0

    def read(self, n):
        require(0 <= n <= min(MAX_HEADER, self.size) - self.pos, "header byte budget/truncation")
        b = self.f.read(n)
        require(len(b) == n, "truncated header")
        self.pos += n
        return b

    def number(self, fmt):
        return struct.unpack("<" + fmt, self.read(struct.calcsize("<" + fmt)))[0]

    def string(self):
        return self.read(self.number("Q")).decode("utf-8", errors="strict")

    def value(self, kind, depth=0):
        self.steps += 1
        require(self.steps <= MAX_TOKENS and depth <= MAX_DEPTH, "metadata token/depth budget")
        fmts = {0: "B", 1: "b", 2: "H", 3: "h", 4: "I", 5: "i",
                6: "f", 7: "B", 10: "Q", 11: "q", 12: "d"}
        if kind in fmts:
            v = self.number(fmts[kind])
            require(kind != 7 or v in (0, 1), "invalid bool")
            return v
        if kind == 8:
            return self.string()
        require(kind == 9, "unknown metadata type")
        child, count = self.number("I"), self.number("Q")
        require(count <= MAX_TOKENS - self.steps, "array token budget")
        for _ in range(count):
            self.value(child, depth + 1)
        return None


def gguf(r):
    require(r.read(4) == b"GGUF", "GGUF magic")
    version = r.number("I")
    require(version in (2, 3), "unsupported GGUF version")
    nt, nm = r.number("Q"), r.number("Q")
    require(nt <= MAX_ITEMS and nm <= MAX_ITEMS and nt + nm <= MAX_ITEMS, "descriptor budget")
    alignment, architecture, keys = 32, None, set()
    for _ in range(nm):
        key = r.string()
        require(key not in keys, "duplicate metadata")
        keys.add(key)
        kind = r.number("I")
        value = r.value(kind)
        if key == "general.alignment":
            require(kind == 4, "alignment type")
            alignment = value
        if key == "general.architecture":
            require(kind == 8, "architecture type")
            architecture = value
    require(type(alignment) is int and 0 < alignment <= MAX_HEADER
            and alignment & (alignment - 1) == 0, "invalid alignment")
    ranges, types, names, unknown, offsets = [], Counter(), set(), set(), []
    for _ in range(nt):
        name = r.string()
        require(name not in names, "duplicate tensor")
        names.add(name)
        nd = r.number("I")
        require(1 <= nd <= 4, "GGUF dimension count")
        shape = [r.number("Q") for _ in range(nd)]
        require(all(shape), "zero GGUF dimension")
        n = product(shape)
        kind, offset = r.number("I"), r.number("Q")
        offsets.append(offset)
        require(offset % alignment == 0, "unaligned tensor")
        if kind not in GGML:
            types[f"UNKNOWN_GGML_{kind}"] += 1
            unknown.add(kind)
            require(offset <= r.size, "tensor offset outside file")
            continue
        label, block, width = GGML[kind]
        require(shape[0] % block == 0, "quant block/row mismatch")
        length = n // block * width
        require(length <= MAX_U64 and offset <= MAX_U64 - length, "tensor byte overflow")
        ranges.append((offset, offset + length))
        types[label] += 1
    base = (r.pos + alignment - 1) // alignment * alignment
    require(base <= r.size, "missing aligned data region")
    require(all(offset <= r.size - base for offset in offsets), "tensor offset outside data region")
    ranges_check(ranges, r.size - base)
    return {"format": "GGUF", "version": version, "architecture": architecture,
            "tensor_types": dict(types), "tensor_count": nt,
            "range_validation": "incomplete: unknown GGML types" if unknown else "complete",
            "header_bytes_read": r.pos}


def json_budget(text):
    depth = tokens = 0
    quoted = escaped = False
    for c in text:
        if quoted:
            if escaped:
                escaped = False
            elif c == "\\":
                escaped = True
            elif c == '"':
                quoted = False
            continue
        if c == '"':
            quoted = True
            tokens += 1
        elif c in "[{":
            depth += 1
            tokens += 1
        elif c in "]}":
            depth -= 1
        elif c in ",:":
            tokens += 1
        require(0 <= depth <= MAX_DEPTH and tokens <= MAX_TOKENS, "JSON token/depth budget")
    require(not quoted and depth == 0, "truncated JSON")


def unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "duplicate JSON key")
        result[key] = value
    return result


def safetensors(r):
    length = r.number("Q")
    require(0 < length <= MAX_HEADER - 8, "JSON header byte budget")
    raw = r.read(length)
    require(raw.startswith(b"{"), "JSON header must begin with object")
    text = raw.decode("utf-8")
    json_budget(text)
    header = json.loads(text, object_pairs_hook=unique_pairs,
                        parse_constant=lambda _: (_ for _ in ()).throw(Invalid("JSON constant")))
    require(isinstance(header, dict) and len(header) <= MAX_ITEMS, "descriptor budget")
    types, ranges, unknown = Counter(), [], False
    for name, desc in header.items():
        if name == "__metadata__":
            require(isinstance(desc, dict) and all(isinstance(v, str) for v in desc.values()),
                    "invalid metadata")
            continue
        require(isinstance(desc, dict) and set(desc) == {"dtype", "shape", "data_offsets"},
                "invalid tensor descriptor")
        n = product(desc["shape"])
        dtype, offsets = desc["dtype"], desc["data_offsets"]
        require(isinstance(dtype, str) and isinstance(offsets, list) and len(offsets) == 2
                and all(type(v) is int for v in offsets), "invalid dtype/offsets")
        start, end = offsets
        if dtype in DTYPES:
            require(n <= MAX_U64 // DTYPES[dtype] and end - start == n * DTYPES[dtype],
                    "dtype byte count")
        else:
            unknown = True
        ranges.append((start, end))
        types[dtype] += 1
    ranges_check(ranges, r.size - r.pos)
    return {"format": "safetensors", "architecture": None, "tensor_types": dict(types),
            "tensor_count": sum(types.values()), "header_bytes_read": r.pos,
            "range_validation": "complete; unknown dtype byte sizes" if unknown else "complete"}


def inspect(path):
    size = path.stat().st_size
    require(0 <= size <= MAX_FILE_SIZE, "file size limit")
    with path.open("rb") as f:
        r = Reader(f, size)
        result = gguf(r) if path.suffix.lower() == ".gguf" else safetensors(r)
    result["file_bytes"] = size
    return result


def inventory(root):
    records, visited = [], 0
    # Do not traverse canceled model directories, even to open headers.
    import os
    require(root.is_dir(), "inventory root is not a directory")
    require(not any(word in str(root).lower() for word in ("prism", "bonsai")),
            "excluded inventory root")
    directories = 0
    def walk_error(error):
        raise error
    for directory, dirs, files in os.walk(root, followlinks=False, onerror=walk_error):
        directories += 1
        require(directories <= MAX_FILES and len(dirs) + len(files) <= MAX_FILES,
                "directory entry budget")
        for d in list(dirs):
            if any(word in d.lower() for word in ("prism", "bonsai")):
                records.append({"name": str((Path(directory) / d).relative_to(root)),
                                "status": "excluded"})
                dirs.remove(d)
        for name in sorted(files):
            visited += 1
            require(visited <= MAX_FILES, "inventory file limit")
            path = Path(directory) / name
            relative = str(path.relative_to(root))
            if any(word in relative.lower() for word in ("prism", "bonsai")):
                records.append({"name": relative, "status": "excluded"})
            elif path.suffix.lower() in (".gguf", ".safetensors"):
                try:
                    require(not path.is_symlink(), "symlink excluded")
                    records.append({"name": relative, "status": "inspected", **inspect(path)})
                except (ValueError, OSError, struct.error) as exc:
                    records.append({"name": relative, "status": "rejected", "error": str(exc)})
    totals, formats = Counter(), Counter()
    for record in records:
        if record["status"] == "inspected":
            totals.update(record["tensor_types"])
            formats[record["format"]] += 1
    return {"payload_access": False, "formats": dict(formats),
            "tensor_type_counts": dict(totals), "files": records}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = inventory(args.root)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "files"}, indent=2))
    print(dict(Counter(r["status"] for r in result["files"])))


if __name__ == "__main__":
    main()
