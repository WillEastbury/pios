"""PIOS-local request-scoped Tensor hooks in the shipped compiler registry."""


def extend(data: bytes) -> bytes:
    anchor = b'0x3E9: "ModelFifo.From",'
    if b'0x3EA: "Tensor.MatVecPinned"' in data:
        return data
    if anchor not in data:
        raise ValueError("ModelFifo registry insertion point missing")
    return data.replace(anchor, anchor +
                        b'\n      0x3EA: "Tensor.MatVecPinned",'
                        b'\n      0x3EB: "Tensor.SelectedBackend",')


if __name__ == "__main__":
    import ast
    from pathlib import Path
    from gen_ide_assets import c_escape_bytes

    path = Path(__file__).resolve().parent.parent / "src" / "ide_assets.c"
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    if not any('0x3EA: \\"Tensor.MatVecPinned\\"' in line for line in lines):
        for i, line in enumerate(lines):
            if '0x3E9: \\"ModelFifo.From\\",' in line:
                raw = ast.literal_eval(line.strip().replace(r"\?", "?")).encode()
                lines[i] = "    " + c_escape_bytes(extend(raw)).replace("\n", "\n    ") + "\n"
        path.write_text("".join(lines), encoding="utf-8", newline="\n")
