"""Offline shipped-compiler -> real C VMs -> ipc_model -> lease registry."""
import ast
import json
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "tests" / "_build" / "model_fifo"
OUT.mkdir(parents=True, exist_ok=True)
asset = (ROOT / "src" / "ide_assets.c").read_text(encoding="utf-8")
html = "".join(ast.literal_eval(l.strip().replace(r"\?", "?"))
               for l in asset.splitlines() if l.strip().startswith('"'))
scripts = re.findall(r"<script\b[^>]*>(.*?)</script>", html, re.S)
hooks = next(s for s in scripts if "root.PV_HOOKS = H" in s)
compiler = next(s for s in scripts if "root.PicoCompile = P" in s)
cc, node = shutil.which("clang"), shutil.which("node")
assert cc and node
exe = OUT / "model_fifo_vm.exe"
build = subprocess.run([
    cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-ffreestanding",
    "-Wno-comment", "-Wno-unused-function", "-Wno-unused-variable",
    "-include", str(ROOT / "tests" / "stubinc" / "types.h"),
    "-I", str(ROOT / "tests" / "stubinc"), "-I", str(ROOT / "include"),
    str(ROOT / "tests" / "ipc_model_fixture_vm.c"),
    str(ROOT / "src" / "picovm.c"), str(ROOT / "src" / "ipc_model.c"),
    str(ROOT / "src" / "ipc_model_vm.c"), str(ROOT / "src" / "lease.c"),
    "-o", str(exe),
], capture_output=True, text=True)
assert build.returncode == 0, build.stdout + build.stderr

cases = []


def case(vm, source, expected):
    cases.append((vm, source, expected))


# Two sending VMs remain attached simultaneously to one receiving VM.
case(0, (ROOT / "examples" / "model_fifo.pc").read_text() +
     "Io.WriteByte(submitted); Io.WriteByte(status+128); Io.WriteByte(sequence);",
     "018001")
case(1, "int r=ModelFifo.Route(2,7); Io.WriteByte(ModelFifo.Submit(r,1));",
     "01")
# Fill the first SPSC lane using one bounded batch of seven local lease grants.
# It does not touch the second sender's lane or the other shard.
setup = "".join(f"Memory.Set({4096+i*4+3},1);" for i in range(8))
case(0, "int r=ModelFifo.Route(2,7);" + setup +
     "int b=Span.Make(4096,32); Io.WriteByte(ModelFifo.SubmitBatch(r,b));"
     "Io.WriteByte(ModelFifo.Status()+128); Io.WriteByte(ModelFifo.Published(r));",
     "077c08")  # FULL=-4, seven entries published before first-lane fullness
case(0, "int r=ModelFifo.Route(2,8); Io.WriteByte(ModelFifo.Submit(r,1));",
     "01")
case(2, "int r=ModelFifo.Route(2,7); r=ModelFifo.From(r,1);"
     "Io.WriteByte(ModelFifo.Drain(r,8)); Io.WriteByte(ModelFifo.At(0,3));"
     "Io.Write(ModelFifo.At(0,1)); int token=ModelFifo.At(0,0);"
     "Io.WriteByte(ModelFifo.Ack(token)+128);"
     "Io.WriteByte(ModelFifo.Ack(token)+128);"
     "Io.WriteByte(ModelFifo.Consumed(r));",
     "010142010203807701")
case(2, "int r=ModelFifo.Route(2,8); Io.WriteByte(ModelFifo.Drain(r,1));"
     "Io.WriteByte(ModelFifo.At(0,4));"
     "int t=ModelFifo.At(0,0); ModelFifo.Ack(t);",
     "0108")
case(2, "int r=ModelFifo.Route(2,7); r=ModelFifo.From(r,0);"
     "Io.WriteByte(ModelFifo.Drain(r,2));"
     "Io.WriteByte(ModelFifo.Drain(r,2)); Io.WriteByte(ModelFifo.Status()+128);"
     "int t=ModelFifo.At(0,0); ModelFifo.Ack(t); t=ModelFifo.At(1,0); ModelFifo.Ack(t);"
     "Io.WriteByte(ModelFifo.Consumed(r));"
     "int n=ModelFifo.Drain(r,8); Io.WriteByte(n);"
     "int i=0; while(i<n){ int q=ModelFifo.At(i,0); ModelFifo.Ack(q); i=i+1; }"
     "Io.WriteByte(ModelFifo.Drain(r,8)); Io.WriteByte(ModelFifo.Status()+128);",
     "0200750206007b")  # PHASE=-11, EMPTY=-5
case(0, "int r=ModelFifo.Route(2,7); Io.WriteByte(ModelFifo.Submit(r,99)+128);"
     "Io.WriteByte(ModelFifo.Published(r));", "7608")
case(0, "int r=ModelFifo.Route(2,7); int b=Span.Make(4096,33);"
     "Io.WriteByte(ModelFifo.SubmitBatch(r,b)); Io.WriteByte(ModelFifo.Status()+128);"
     "Io.WriteByte(ModelFifo.Published(r));", "007a08")

sources = [source for _, source, _ in cases]
js = ("const vm=require('vm'); const ctx={}; vm.createContext(ctx);\n"
      f"vm.runInContext({json.dumps(hooks)},ctx);\n"
      f"vm.runInContext({json.dumps(compiler)},ctx);\n"
      f"const sources={json.dumps(sources)};\n"
      "console.log(JSON.stringify(sources.map(s=>ctx.PicoCompile.compileC(s).words)));\n")
compilation = subprocess.run([node, "-"], input=js, capture_output=True, text=True)
assert compilation.returncode == 0, compilation.stderr
programs = json.loads(compilation.stdout)
# Prove source calls lower to extended hostcalls, not unknown-op/no-op placeholders.
assert any((w & 0xffff) == 0x63e1 for w in programs[0])
wire = "".join(f"{vm} {len(words)}\n" + "\n".join(f"{w & 0xffffffff:x}" for w in words) + "\n"
               for (vm, _, _), words in zip(cases, programs))
result = subprocess.run([str(exe)], input=wire, capture_output=True, text=True)
assert result.returncode == 0, result.stdout + result.stderr
actual = result.stdout.splitlines()
assert actual == [expected for _, _, expected in cases], (actual, cases)
print("Shipped ModelFifo compiler, three C VMs, selective SPSC routing and leases: PASS")
