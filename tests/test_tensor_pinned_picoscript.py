"""Shipped compiler -> real VM -> tensor_service -> truthful CPU provider."""
import ast
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "tests" / "_build" / "tensor_pinned"
OUT.mkdir(parents=True, exist_ok=True)
asset = (ROOT / "src" / "ide_assets.c").read_text(encoding="utf-8")
html = "".join(ast.literal_eval(line.strip().replace(r"\?", "?"))
               for line in asset.splitlines() if line.strip().startswith('"'))
scripts = re.findall(r"<script\b[^>]*>(.*?)</script>", html, re.S)
hooks = next(s for s in scripts if "root.PV_HOOKS = H" in s)
compiler = next(s for s in scripts if "root.PicoCompile = P" in s)
source = (ROOT / "examples" / "tensor_pinned.pc").read_text()
cc, node = shutil.which("clang"), shutil.which("node")
assert cc and node
exe = OUT / "tensor_pinned.exe"
subprocess.run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-ffreestanding", "-Wno-comment", "-Wno-unused-function",
                "-I", str(ROOT / "include"),
                str(ROOT / "tests" / "tensor_pinned_vm.c"),
                str(ROOT / "src" / "tensor_service.c"),
                str(ROOT / "src" / "picovm.c"), "-o", str(exe)], check=True)


def run(src, denied=False):
    js = ("const vm=require('vm'); const c={}; vm.createContext(c);\n"
          f"vm.runInContext({json.dumps(hooks)},c);\n"
          f"vm.runInContext({json.dumps(compiler)},c);\n"
          f"const p=c.PicoCompile.compileC({json.dumps(src)});\n"
          "console.log(p.words.map(w=>(w>>>0).toString(16)).join('\\n'));")
    result = subprocess.run([node, "-"], input=js, text=True,
                            capture_output=True)
    assert result.returncode == 0, result.stderr
    words = result.stdout
    # Verify that the real compiler lowered both new names to host hooks.
    if not denied:
        decoded = [int(w, 16) for w in words.split()]
        assert any(w & 0xfff == 0x3ea for w in decoded)
        assert any(w & 0xfff == 0x3eb for w in decoded)
    execution = subprocess.run([str(exe)] + (["denied"] if denied else []),
                               input=words, text=True, capture_output=True,
                               check=False)
    assert execution.returncode == 0, execution.stderr
    return bytes.fromhex(execution.stdout)


expected = b"\x01TSR1" + struct.pack(">Iii", 1, -7, 39)
assert run(source) == expected + expected + bytes([124, 127])
assert run("Io.WriteByte(Tensor.MatVecPinned(0, 1) + 128);"
           "Io.WriteByte(Tensor.SelectedBackend(0) + 128);", True) == bytes([124, 124])
print("pinned Tensor: shipped lowering, VM, CPU oracle, exact pins, host denial PASS")
