#!/usr/bin/env python3
"""test_ide_assets.py -- deterministic verification of the compiled-in hosted
PicoScript WebIDE integration.

Checks, with zero network/toolchain requirements:
  1. Reproducibility: re-running tools/gen_ide_assets.py yields a src/ide_assets.c
     byte-identical to the committed one (the generator is deterministic and the
     committed asset is in sync with its sources).
  2. Served IDE_HTML content: the vendored upstream portal
     (../picoscript/docs/index.html) really has tools/ide_server_bridge.js
     injected before </body>, and carries the expected portal + bridge markers
     (Monaco WebIDE portal, PicoWAL tab, deploy controls, /picoscript/config,
     the PIOS capsule/walfs/terminal endpoints).
  3. PicoWAL workspace content: tools/ide_picowal_workspace.html drives the PIOS
     endpoints and hosts the BSO1 panel.
  4. Hook namespaces: the embedded ../picoscript/vm/pico_hooks.js exposes the
     updated host-hook namespaces.
  5. kernel.c serving: the /picoscript route family, the /picoscript/config JSON
     (with the current hook table version 0x0DD8A3B3 and endpoint prefixes), and
     the generated ide_assets symbols are wired in src/kernel.c.

Run: python tools/test_ide_assets.py
Exit code 0 on success, 1 on any failure.
"""
import re
import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
GEN = SCRIPT_DIR / "gen_ide_assets.py"
IDE_ASSETS_C = REPO_ROOT / "src" / "ide_assets.c"
PORTAL = REPO_ROOT.parent / "picoscript" / "docs" / "index.html"
BRIDGE = SCRIPT_DIR / "ide_server_bridge.js"
WORKSPACE = SCRIPT_DIR / "ide_picowal_workspace.html"
PICO_HOOKS_JS = REPO_ROOT.parent / "picoscript" / "vm" / "pico_hooks.js"
KERNEL_C = REPO_ROOT / "src" / "kernel.c"

failures = []


def check(cond, msg):
    if cond:
        print("  ok   " + msg)
    else:
        print("  FAIL " + msg)
        failures.append(msg)


def contains_all(hay: bytes, needles, ctx):
    for n in needles:
        b = n.encode("utf-8") if isinstance(n, str) else n
        check(b in hay, "%s contains %r" % (ctx, n))


def main() -> int:
    print("[1] reproducibility (regenerate + byte-compare committed src/ide_assets.c)")
    if not IDE_ASSETS_C.is_file():
        check(False, "committed src/ide_assets.c exists")
        return 1
    committed = IDE_ASSETS_C.read_bytes()
    r = subprocess.run([sys.executable, str(GEN)], capture_output=True)
    if r.returncode != 0:
        sys.stderr.write(r.stderr.decode("utf-8", "replace"))
        check(False, "generator ran successfully")
        return 1
    regenerated = IDE_ASSETS_C.read_bytes()
    check(regenerated == committed, "regenerated ide_assets.c is byte-identical to committed")

    print("[2] served IDE_HTML = upstream portal + injected bridge")
    portal = PORTAL.read_bytes()
    bridge = BRIDGE.read_bytes()
    idx = portal.lower().rfind(b"</body>")
    check(idx != -1, "portal has </body> injection point")
    served = portal[:idx] + b"<script>\n" + bridge + b"\n</script>\n" + portal[idx:]
    contains_all(served, [
        "monaco-editor@",              # real Monaco WebIDE portal
        "PicoScript",
        "PiosIdeBridge",               # bridge installed
        "/picoscript/config",          # bridge loads PIOS config
        "picowal.html?embedded=1",     # PicoWAL tab iframe
        "Deploy Bytecode",             # deploy controls
        "/api/capsule",                # bridge targets PIOS capsule store
    ], "served IDE_HTML")
    # The committed asset must embed these bytes too.
    contains_all(committed, ["IDE_HTML", "IDE_PICOWAL_HTML",
                             "IDE_PICO_HOOKS_JS", "IDE_BAREMETAL_BINARY_JS",
                             "PiosIdeBridge"], "committed ide_assets.c")

    print("[3] PicoWAL workspace content")
    ws = WORKSPACE.read_bytes()
    contains_all(ws, [
        "PicoWAL Workspace",
        "Fast Serial (BSO1)",
        "/api/walfs", "/api/capsule", "/api/terminal",
        "/picoscript/baremetal-binary.js",  # BSO1 codec script
    ], "workspace")

    print("[4] hook namespaces embedded (pico_hooks.js)")
    hooks = PICO_HOOKS_JS.read_bytes()
    contains_all(hooks, [
        "AUTO-GENERATED",
        "Number.Parse", "String.", "Maths.",
    ], "pico_hooks.js")

    print("[5] kernel.c serving + config wiring")
    kern = KERNEL_C.read_bytes()
    contains_all(kern, [
        '#include "ide_assets.h"',
        "/picoscript/config",
        "/picoscript/picowal.html",
        "/picoscript/pico_hooks.js",
        "/picoscript/baremetal-binary.js",
        "IDE_HTML", "IDE_PICOWAL_HTML",
        "IDE_PICO_HOOKS_JS", "IDE_BAREMETAL_BINARY_JS",
        "\\\"capsule_prefix\\\":\\\"/api/capsule\\\"",
        "PV_HOOK_TABLE_VERSION",
    ], "kernel.c")
    # No stale references to the removed hand-vendored asset.
    check(b"picoscript_playground" not in kern, "kernel.c has no stale picoscript_playground reference")

    print()
    if failures:
        print("FAILED: %d check(s)" % len(failures))
        for f in failures:
            print("  - " + f)
        return 1
    print("ALL IDE ASSET CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
