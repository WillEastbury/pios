#!/usr/bin/env python3
"""gen_ide_assets.py -- regenerate src/ide_assets.c, the compiled-in hosted
PicoScript WebIDE that PIOS serves at /picoscript.

This is the PIOS analogue of picoweb's tools/gen_ide_assets.py. It vendors the
REAL upstream PicoScript WebIDE portal (../picoscript/docs/index.html, built by
that repo's own gen_site.py -- Guide & Reference / WebIDE / Showcase nav, full
Monaco editor, dialect tabs, Compile & Run/Step/Reset, disassembly/registers/
watches) verbatim, injects tools/ide_server_bridge.js right before </body>, and
embeds a separate PicoWAL workspace page (tools/ide_picowal_workspace.html) plus
the BareMetal.Binary BSO1 codec. Nothing is hand-edited into the byte array --
run this script to regenerate.

Inputs (resolved relative to this script, not the CWD):
  - ../picoscript/docs/index.html         (sibling picoscript checkout; the
    ACTUAL generated portal -- vendored verbatim, never hand-edited so upstream
    `python ../picoscript/gen_site.py` regenerations always drop in cleanly)
  - tools/ide_server_bridge.js            (this repo; injected before </body>;
    the ONLY place that wires the vendored portal to a running PIOS instance --
    /picoscript/config, the PicoWAL workspace tab, deploy controls mapped to
    PIOS's own /api/capsule + /api/walfs + /api/terminal endpoints)
  - tools/ide_picowal_workspace.html      (this repo; a separate, visually
    isolated PicoWAL workspace served at /picoscript/picowal.html and opened
    from the portal's PicoWAL tab via an iframe so its CSS never collides with
    the vendored portal)
  - ../baremetaljstools/src/BareMetal.Binary.js  (sibling baremetaljstools
    checkout; vendored byte-for-byte -- powers the workspace "Fast Serial
    (BSO1)" panel)
  - ../picoscript/vm/pico_hooks.js        (sibling picoscript checkout; kept as a
    standalone compiled-in asset served at /picoscript/pico_hooks.js for
    backward compatibility and hook-namespace verification -- the vendored
    portal already inlines its own copy)

Output:
  - src/ide_assets.c (generated; committed so a normal build never needs the
    sibling picoscript/baremetaljstools checkouts)

Emits PIOS-native `const u8 NAME[]` / `const u32 NAME_LEN` declarations (see
include/ide_assets.h) so the freestanding kernel can stream them directly.
"""
import re
import sys
import textwrap
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
PICOSCRIPT_DOCS = REPO_ROOT.parent / "picoscript" / "docs"
PICOSCRIPT_VM = REPO_ROOT.parent / "picoscript" / "vm"
BAREMETALJSTOOLS_SRC = REPO_ROOT.parent / "baremetaljstools" / "src"

HEX_DIGITS = set(b"0123456789abcdefABCDEF")


def c_escape_bytes(data: bytes) -> str:
    """Render `data` as adjacent C string literals, one source line per input
    line (readable diffs), safe against trigraphs (every '?' -> \\?) and the
    \\xHH digit-swallowing ambiguity (close/reopen the literal after any \\xHH
    escape if the next byte is itself a hex digit). No trailing ';'."""
    lines = []
    parts = []
    buf = []

    def flush_part():
        parts.append("".join(buf))
        buf.clear()

    def flush_line():
        flush_part()
        non_empty = [p for p in parts if p != ""]
        rendered = " ".join('"%s"' % p for p in non_empty) if non_empty else '""'
        lines.append(rendered)
        parts.clear()

    n = len(data)
    for i in range(n):
        b = data[i]
        if b == 0x0A:
            buf.append("\\n")
            flush_line()
            continue
        if b == 0x22:
            buf.append('\\"')
        elif b == 0x5C:
            buf.append("\\\\")
        elif b == 0x3F:
            buf.append("\\?")
        elif b == 0x09:
            buf.append("\\t")
        elif b == 0x0D:
            buf.append("\\r")
        elif 0x20 <= b <= 0x7E:
            buf.append(chr(b))
        else:
            buf.append("\\x%02x" % b)
            nxt = data[i + 1] if i + 1 < n else None
            if nxt is not None and nxt in HEX_DIGITS:
                flush_part()

    if buf or parts:
        flush_line()
    if not lines:
        lines = ['""']
    return "\n".join(lines)


def emit_asset(name: str, data: bytes) -> str:
    body = c_escape_bytes(data)
    indented = textwrap.indent(body, "    ")
    return (
        "const u8 %s[] =\n%s\n;\n"
        "const u32 %s_LEN = (u32)(sizeof(%s) - 1);\n\n"
        % (name, indented, name, name)
    )


def read_required(path: Path) -> bytes:
    if not path.is_file():
        sys.stderr.write(
            "gen_ide_assets: missing input file '%s'\n"
            "  (see the module docstring for where each input comes from)\n" % path
        )
        sys.exit(1)
    return path.read_bytes()


BODY_CLOSE_RE = re.compile(rb"</body>", re.IGNORECASE)


def inject_before_body_close(html: bytes, snippet: bytes) -> bytes:
    """Insert `snippet` immediately before the LAST </body> in `html`."""
    matches = list(BODY_CLOSE_RE.finditer(html))
    if not matches:
        sys.stderr.write("gen_ide_assets: vendored HTML has no </body> to inject before\n")
        sys.exit(1)
    idx = matches[-1].start()
    return html[:idx] + snippet + html[idx:]


def main() -> int:
    portal_html = read_required(PICOSCRIPT_DOCS / "index.html")
    bridge_js = read_required(SCRIPT_DIR / "ide_server_bridge.js")
    picowal_html = read_required(SCRIPT_DIR / "ide_picowal_workspace.html")
    pico_hooks_js = read_required(PICOSCRIPT_VM / "pico_hooks.js")
    baremetal_binary_js = read_required(BAREMETALJSTOOLS_SRC / "BareMetal.Binary.js")

    ide_html = inject_before_body_close(
        portal_html,
        b"<script>\n" + bridge_js + b"\n</script>\n",
    )

    out_path = REPO_ROOT / "src" / "ide_assets.c"
    with out_path.open("w", encoding="utf-8", newline="\n") as f:
        f.write(
            "/* ide_assets.c -- GENERATED by tools/gen_ide_assets.py. DO NOT EDIT BY HAND.\n"
            " * Regenerate with `python tools/gen_ide_assets.py` (requires ../picoscript\n"
            " * and ../baremetaljstools checked out next to this repo). See\n"
            " * include/ide_assets.h for the declarations and src/kernel.c\n"
            " * (http_build_picoscript_response) for how these are served under /picoscript.\n"
            " *\n"
            " * IDE_HTML is the ACTUAL upstream PicoScript WebIDE portal\n"
            " * (../picoscript/docs/index.html, built by that repo's gen_site.py) with\n"
            " * tools/ide_server_bridge.js injected right before </body> -- never a\n"
            " * hand-rolled substitute. IDE_PICOWAL_HTML is this repo's own PicoWAL\n"
            " * workspace page (tools/ide_picowal_workspace.html), served at\n"
            " * /picoscript/picowal.html and opened from the portal's PicoWAL tab via an\n"
            " * iframe for CSS isolation. */\n\n"
            '#include "ide_assets.h"\n\n'
        )
        f.write(emit_asset("IDE_HTML", ide_html))
        f.write(emit_asset("IDE_PICOWAL_HTML", picowal_html))
        f.write(emit_asset("IDE_PICO_HOOKS_JS", pico_hooks_js))
        f.write(emit_asset("IDE_BAREMETAL_BINARY_JS", baremetal_binary_js))

    sizes = {
        "IDE_HTML (portal + bridge)": len(ide_html),
        "IDE_PICOWAL_HTML": len(picowal_html),
        "IDE_PICO_HOOKS_JS": len(pico_hooks_js),
        "IDE_BAREMETAL_BINARY_JS": len(baremetal_binary_js),
    }
    total = sum(sizes.values())
    print("gen_ide_assets: wrote %s (%d bytes of asset payload)" % (out_path, total))
    for k, v in sizes.items():
        print("  %-28s %8d bytes" % (k, v))
    return 0


if __name__ == "__main__":
    sys.exit(main())
