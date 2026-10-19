"""Shipped compiler -> real VM -> exec scheduler -> fabric residency."""
import ast
import json
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "tests" / "_build" / "exec_sched"
OUT.mkdir(parents=True, exist_ok=True)
asset = (ROOT / "src" / "ide_assets.c").read_text(encoding="utf-8")
html = "".join(ast.literal_eval(line.strip().replace(r"\?", "?"))
               for line in asset.splitlines() if line.strip().startswith('"'))
scripts = re.findall(r"<script\b[^>]*>(.*?)</script>", html, re.S)
hooks = next(s for s in scripts if "root.PV_HOOKS = H" in s)
compiler = next(s for s in scripts if "root.PicoCompile = P" in s)
assert "ExecSched.Where" in hooks and "ExecSched.Submit" in hooks
cc = shutil.which("clang")
if not cc:
    bundled = Path(r"C:\Program Files\LLVM\bin\clang.exe")
    cc = str(bundled) if bundled.exists() else None
node = shutil.which("node")
assert cc and node
exe = OUT / "exec_sched_vm.exe"
build = subprocess.run([
    cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-ffreestanding",
    "-Wno-comment", "-Wno-unused-function", "-Wno-unused-variable",
    "-I", str(ROOT / "tests" / "stubinc"), "-I", str(ROOT / "include"),
    str(ROOT / "tests" / "exec_sched_fixture_vm.c"),
    str(ROOT / "src" / "picovm.c"),
    str(ROOT / "src" / "pios_exec_sched.c"),
    str(ROOT / "src" / "pcie_fabric.c"),
    "-o", str(exe),
], capture_output=True, text=True)
assert build.returncode == 0, build.stdout + build.stderr

reuse = (ROOT / "examples" / "exec_sched.pc").read_text()
busy = (
    "int where = ExecSched.Where(3, 7);"
    "int request = Bits.Or(Bits.Shl(3, 16), 7);"
    "int handle = ExecSched.Submit(request, 4);"
    "int reply = ExecSched.Reply();"
    "Io.WriteByte(where);"
    "Io.WriteByte(Bits.And(handle, 255));"
    "Io.WriteByte(reply + 128);"
    "Io.WriteByte(ExecSched.Reuse());"
)
unbound = (
    "Io.WriteByte(ExecSched.Where(3, 7) + 128);"
    "Io.WriteByte(ExecSched.Submit(1, 0) + 128);"
    "Io.WriteByte(ExecSched.Reply() + 128);"
    "Io.WriteByte(ExecSched.Reuse() + 128);"
)


def compile_source(source):
    js = ("const vm=require('vm'); const c={}; vm.createContext(c);\n"
          f"vm.runInContext({json.dumps(hooks)},c);\n"
          f"vm.runInContext({json.dumps(compiler)},c);\n"
          f"const p=c.PicoCompile.compileC({json.dumps(source)});\n"
          "console.log(p.words.map(w=>(w>>>0).toString(16)).join('\\n'));")
    result = subprocess.run([node, "-"], input=js, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    words = [int(w, 16) for w in result.stdout.split()]
    assert words, source
    return words, result.stdout


def run(source, mode):
    words, wire = compile_source(source)
    ids = {w & 0xFFF for w in words}
    assert {0x3EC, 0x3ED, 0x3EE, 0x3EF} <= ids, sorted(ids)
    execution = subprocess.run([str(exe), mode], input=wire, text=True,
                               capture_output=True, check=False)
    assert execution.returncode == 0, execution.stderr + execution.stdout
    return execution.stdout


# Resident B50: device 1, slot 1, reused, no DMA, backend B50, complete.
assert run(reuse, "reuse") == "0101010001020301"
# No resident tensor: same program falls through to the CPU slot. The callback
# is not invoked because the script supplied no matrix.
assert run(reuse, "cpu") == "fc01000000010300"
# Explicit B60 while the tensor lives on a B50. Reply stays busy until DMA.
assert run(busy, "busy") == "01017b00"
# Installed hook, no bound scheduler: every call fails closed.
assert run(unbound, "unbound") == "7c7c7c7c"


def words_of(source, required):
    words, text = compile_source(source)
    assert required <= {w & 0xFFF for w in words}
    return len(words), text


def session(steps):
    chunks = []
    for kind, source, required in steps:
        if kind == "P":
            count, text = words_of(source, required)
            chunks.append(f"P {count}\n{text}")
        else:
            chunks.append(kind + "\n")
    execution = subprocess.run([str(exe), "session"], input="".join(chunks),
                               text=True, capture_output=True, check=False)
    assert execution.returncode == 0, execution.stderr + execution.stdout
    return execution.stdout.splitlines()


fifo = (ROOT / "examples" / "exec_sched_fifo.pc").read_text()
reply_again = (
    "int reply = ExecSched.Reply();"
    "Io.WriteByte(reply + 128);"
    "Io.WriteByte(ExecSched.Reuse());"
)
reply_dma = (
    "int reply = ExecSched.Reply();"
    "Io.WriteByte(Bits.And(Bits.Shr(reply, 25), 1));"
    "Io.WriteByte(Bits.And(Bits.Shr(reply, 24), 1));"
    "Io.WriteByte(Bits.And(Bits.Shr(reply, 16), 255));"
    "Io.WriteByte(Bits.And(Bits.Shr(reply, 8), 255));"
    "Io.WriteByte(Bits.And(reply, 255));"
    "Io.WriteByte(ExecSched.Reuse());"
)
reply_empty = "Io.WriteByte(ExecSched.Reply() + 128);"
# Slots 1 then 2. Reply is busy. Completing the newer job does not unblock
# the head. Completing the head releases B60, then the second B60, in order.
assert session([
    ("P", fifo, {0x3EC, 0x3ED, 0x3EF}),
    ("N", None, set()),
    ("P", reply_again, {0x3ED, 0x3EF}),
    ("H", None, set()),
    ("P", reply_dma, {0x3ED, 0x3EF}),
    ("P", reply_dma, {0x3ED, 0x3EF}),
    ("P", reply_empty, {0x3ED}),
    ("H", None, set()),
]) == [
    "01027b00",
    "80",
    "7b00",
    "80",
    "000104030300",
    "000104030300",
    "7f",
    "7d",
]
print("ExecSched PicoScript: shipped compiler, fabric reuse, ordered DMA FIFO PASS")
