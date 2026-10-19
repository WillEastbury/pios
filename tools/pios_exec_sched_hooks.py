"""Register ExecSched hooks after the shipped Tensor.SelectedBackend entry."""


def extend(data: bytes) -> bytes:
    anchor = b'0x3EB: "Tensor.SelectedBackend",'
    if b'0x3EC: "ExecSched.Submit"' in data:
        return data
    if anchor not in data:
        raise ValueError("Tensor.SelectedBackend registry insertion point missing")
    return data.replace(
        anchor,
        anchor
        + b'\n      0x3EC: "ExecSched.Submit",'
        + b'\n      0x3ED: "ExecSched.Reply",'
        + b'\n      0x3EE: "ExecSched.Where",'
        + b'\n      0x3EF: "ExecSched.Reuse",',
    )


if __name__ == "__main__":
    import ast
    from pathlib import Path
    from gen_ide_assets import c_escape_bytes

    path = Path(__file__).resolve().parent.parent / "src" / "ide_assets.c"
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    changed = False
    for i, line in enumerate(lines):
        if '0x3EB: \\"Tensor.SelectedBackend\\",' not in line:
            continue
        if '0x3EC: \\"ExecSched.Submit\\"' in line:
            continue
        raw = ast.literal_eval(line.strip().replace(r"\?", "?")).encode()
        lines[i] = "    " + c_escape_bytes(extend(raw)).replace("\n", "\n    ") + "\n"
        changed = True
    if changed:
        path.write_text("".join(lines), encoding="utf-8", newline="\n")
