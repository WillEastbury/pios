#!/usr/bin/env python3
"""Convert mesa_v3d_wrap output into a checked-in QPU word header."""

from __future__ import annotations

import argparse
import pathlib
import re


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dump", type=pathlib.Path)
    parser.add_argument("header", type=pathlib.Path)
    parser.add_argument("--symbol", required=True)
    parser.add_argument("--comment", required=True)
    args = parser.parse_args()

    text = args.dump.read_text(encoding="ascii")
    size_match = re.search(r"^qpu_size=(\d+)$", text, re.MULTILINE)
    if not size_match:
        raise SystemExit("missing qpu_size")
    words = re.findall(r"^(0x[0-9a-fA-F]{16})\s", text, re.MULTILINE)
    expected = int(size_match.group(1)) // 8
    if len(words) != expected:
        raise SystemExit(f"expected {expected} words, found {len(words)}")
    uniforms = [
        (int(index), int(kind), int(data, 16))
        for index, kind, data in re.findall(
            r"^uniform\[(\d+)] contents=(\d+) data=0x([0-9a-fA-F]{8})$",
            text,
            re.MULTILINE,
        )
    ]
    if uniforms and [item[0] for item in uniforms] != list(range(len(uniforms))):
        raise SystemExit("uniform metadata is not contiguous")

    macro = args.symbol.upper() + "_WORDS"
    uniform_macro = args.symbol.upper() + "_UNIFORMS"
    lines = [
        "#pragma once",
        "",
        '#include "types.h"',
        "",
        f"/* {args.comment} */",
        f"#define {macro} {len(words)}U",
        f"static const u64 {args.symbol}[{macro}] = {{",
    ]
    for offset in range(0, len(words), 4):
        chunk = ", ".join(f"{word}ULL" for word in words[offset : offset + 4])
        lines.append(f"    {chunk},")
    lines.append("};")
    if uniforms:
        lines.extend(
            [
                "",
                f"#define {uniform_macro} {len(uniforms)}U",
                f"static const u8 {args.symbol}_uniform_kind[{uniform_macro}] = {{",
                "    " + ", ".join(f"{kind}U" for _, kind, _ in uniforms) + ",",
                "};",
                f"static const u32 {args.symbol}_uniform_data[{uniform_macro}] = {{",
                "    "
                + ", ".join(f"0x{data:08X}U" for _, _, data in uniforms)
                + ",",
                "};",
            ]
        )
    lines.append("")
    args.header.write_text("\n".join(lines), encoding="ascii", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
