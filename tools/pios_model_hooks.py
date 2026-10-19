"""PIOS-local hook-table extension, applied before compiler initialization."""

HOOKS = (
    "Route", "Submit", "SubmitBatch", "Drain", "At", "Ack",
    "Published", "Consumed", "Status", "From",
)


def extend(data: bytes) -> bytes:
    if b'0x3E9: "ModelFifo.From"' in data:
        return data
    if b'0x3E8: "ModelFifo.Status",' in data:
        return data.replace(b'0x3E8: "ModelFifo.Status",',
                            b'0x3E8: "ModelFifo.Status",\r\n      0x3E9: "ModelFifo.From",')
    anchor = b'      0x357: "Decimal.ToInt",'
    if anchor not in data:
        raise ValueError("shipped hook-table insertion point missing")
    newline = b"\r\n" if anchor + b"\r\n" in data else b"\n"
    entries = newline.join(
        f'      0x{0x3E0+i:X}: "ModelFifo.{name}",'.encode()
        for i, name in enumerate(HOOKS)
    )
    return data.replace(anchor, anchor + newline + entries)


if __name__ == "__main__":
    # Update only the two shipped registry copies without requiring upstream
    # checkouts or refreshing unrelated portal content.
    import ast
    from pathlib import Path
    from gen_ide_assets import c_escape_bytes

    path = Path(__file__).resolve().parent.parent / "src" / "ide_assets.c"
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)
    has_base = '0x3E0: \\"ModelFifo.Route\\"' in text
    has_from = '0x3E9: \\"ModelFifo.From\\"' in text
    for i, line in enumerate(lines):
        if '0x357: \\"Decimal.ToInt\\",' in line and not has_base:
            raw = ast.literal_eval(line.strip().replace(r"\?", "?")).encode()
            lines[i] = "    " + c_escape_bytes(extend(raw)).replace("\n", "\n    ") + "\n"
        elif '0x3E8: \\"ModelFifo.Status\\",' in line and not has_from:
            raw = ast.literal_eval(line.strip().replace(r"\?", "?")).encode()
            lines[i] = "    " + c_escape_bytes(extend(raw)).replace("\n", "\n    ") + "\n"
    path.write_text("".join(lines), encoding="utf-8", newline="\n")
