"""Offline integer oracle, source execution, and actual shipped VM parity."""
import ast
import json
from pathlib import Path
import random
import re
import shutil
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "tests" / "_build" / "b50_attention"


def trunc(n, d):
    return (-1 if n < 0 else 1) * (abs(n) // d)


def exp_q16(z):
    # Independently expressed range reduction/Taylor series; Python arbitrary
    # precision // is floor, not C's truncating signed divide.
    if z <= -681300:
        return 0
    k = ((z * 94548 // 65536) + 32768) // 65536
    residual = z - k * 45426
    terms = [65536]
    for n in range(1, 8):
        terms.append(trunc(terms[-1] * residual // 65536, n))
    return sum(terms) // (2 ** (-k))


def encode(a, p, values):
    for i, value in enumerate(values):
        a[p + 2*i:p + 2*i + 2] = struct.pack(">H", value + 16384)


def oracle(a, args):
    q, _, k, _, v, _, o, _, x, _, t, s, d, e, off, div, _ = args
    def read(p):
        return struct.unpack_from(">H", a, p)[0] - 16384
    result = bytearray(a)
    for i in range(t):
        scores = [trunc(sum(read(q+2*(i*d+c))*read(k+2*(j*d+c))
                            for c in range(d)), div) for j in range(off+i+1)]
        weights = [exp_q16((score-max(scores))*256)//256 for score in scores]
        encode(result, x, scores)
        encode(result, o+2*i*e,
               [trunc(sum(w*read(v+2*(j*e+c)) for j, w in enumerate(weights)),
                      sum(weights)) for c in range(e)])
    return result


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    cc, node = shutil.which("clang"), shutil.which("node")
    assert cc and node, "clang and node required"
    host = OUT / "attention.exe"
    vmexe = OUT / "vm.exe"
    def build(files, exe, extra=()):
        subprocess.run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        *extra, *map(str, files), "-o", str(exe)], check=True)
    build([ROOT / "tests" / "b50_attention_host.c"], host)
    build([ROOT / "tests" / "bitnet_fixture_vm.c", ROOT / "src" / "picovm.c"],
          vmexe, ["-ffreestanding", "-Wno-comment", "-Wno-unused-function",
                  "-I", str(ROOT / "include")])
    actual_exp = list(map(int, subprocess.check_output([host, "exp"], text=True).split()))
    assert actual_exp == [exp_q16(z*256) for z in range(-16384, 1)]

    asset = (ROOT / "src" / "ide_assets.c").read_text(encoding="utf-8")
    html = "".join(ast.literal_eval(line.strip().replace(r"\?", "?"))
                   for line in asset.splitlines() if line.strip().startswith('"'))
    scripts = re.findall(r"<script\b[^>]*>(.*?)</script>", html, re.S)
    hooks = next(s for s in scripts if "root.PV_HOOKS = H" in s)
    compiler = next(s for s in scripts if "root.PicoCompile = P" in s)
    prefix = (ROOT / "examples" / "fused_ops.pc").read_text()
    def vm(source):
        js = ("const vm=require('vm'); const c={}; vm.createContext(c);"
              f"vm.runInContext({json.dumps(hooks)},c);"
              f"vm.runInContext({json.dumps(compiler)},c);"
              f"const p=c.PicoCompile.compileC({json.dumps(source)});"
              "console.log(p.words.map(w=>(w>>>0).toString(16)).join('\\n'));")
        words = subprocess.check_output([node, "-"], input=js, text=True)
        return bytes.fromhex(subprocess.check_output([vmexe], input=words, text=True).strip())
    # Every reachable quantized difference through the actual Maths.Exp hook,
    # using bounded chunks within the real VM output/step capacities.
    vm_exp = []
    for start in range(-16384, 1, 2048):
        end = min(start+2048, 1)
        source = (f"int z={start}; while(z<{end}) {{"
                  "int a=Maths.Exp(z*256);"
                  "Io.WriteByte(a/65536); Io.WriteByte((a/256)%256);"
                  "Io.WriteByte(a%256); z=z+1; }")
        raw = vm(source)
        assert len(raw) == (end-start)*3
        vm_exp.extend(int.from_bytes(raw[i:i+3], "big")
                      for i in range(0,len(raw),3))
    assert vm_exp == actual_exp
    def run(a, args, expected=0, arena_bytes=65536, status_bytes=4):
        payload = " ".join(map(str, [arena_bytes,status_bytes,*args,*a]))
        result = subprocess.check_output([host], input=payload, text=True).splitlines()
        assert int(result[0]) == expected
        actual = bytes.fromhex(result[1])
        assert actual == (oracle(a,args) if expected == 0 else a)
        return actual
    rng = random.Random(0xE212)
    cases = []
    for case in range(40):
        s = rng.randint(1,8)
        t = rng.randint(1,s)
        off = rng.randint(0,s-t)
        d, e = rng.randint(1,8), rng.randint(1,8)
        if case < 4:
            t=s=d=e=8
            off=0
        elif case < 10:
            s=min(s,3)
            t=min(t,s)
            d=min(d,3)
            e=min(e,3)
            off=s-t
        div = [256,4096,362,1024][case%4]
        args = [4096,t*d*2,8192,s*d*2,10240,s*e*2,
                12288,t*e*2,14336,s*2,t,s,d,e,off,div,t*s*(d+e+1)]
        a = bytearray([0xA7]*65536)
        for p, n in [(4096,t*d),(8192,s*d),(10240,s*e)]:
            values = [rng.randint(-512,512) for _ in range(n)]
            if case == 0: values = [512]*n
            if case == 1: values = [-512 if i%2 else 512 for i in range(n)]
            encode(a,p,values)
        run(a,args)
        cases.append((a,args))
    # Input aliases remain legal; full capacities, not just used bytes, conflict.
    a,args = cases[0]
    alias = args.copy()
    alias[2:6] = [4096,128,4096,128]
    run(a,alias)
    rejects = [(10,0,-1),(11,9,-1),(14,2147483647,-1),(15,255,-1),
               (0,2147483647,-2),(1,2147483647,-2),(9,15,-2),
               (6,4096,-3),(8,8192,-3),(8,12288,-3),(1,9000,-3),
               (16,-1,-4)]
    for index,value,status in rejects:
        bad=args.copy()
        bad[index]=value
        run(a,bad,status)
    run(a,args,-2,arena_bytes=65535)
    run(a,args,99,status_bytes=3)
    for p in [4096+126,8192+126,10240+126]:
        for invalid in [-513,513,49151]:
            bad=bytearray(a)
            encode(bad,p,[invalid])
            run(bad,args,-5)
    # Compare output AND scratch byte-for-byte with real compiler + real
    # Maths.Exp hook. This never injects an exp replacement.
    for a,args in [*cases[:2],*cases[4:10]]:
        setup=[]
        for p,cap in [(args[0],args[1]),(args[2],args[3]),(args[4],args[5])]:
            vals=[w[0]-16384 for w in struct.iter_unpack(">H",a[p:p+cap])]
            if len(set(vals)) == 1:
                setup.append(f"int i{p}=0; while(i{p}<{len(vals)}) {{"
                             f"fused_write({p}+i{p}*2,{vals[0]}); i{p}=i{p}+1; }}")
            elif vals == [512 if i%2 == 0 else -512 for i in range(len(vals))]:
                setup.append(f"int i{p}=0; while(i{p}<{len(vals)}) {{"
                             f"fused_write({p}+i{p}*2,512-(i{p}%2)*1024);"
                             f"i{p}=i{p}+1; }}")
            else:
                for i,val in enumerate(vals):
                    setup.append(f"fused_write({p+2*i},{val});")
        source=prefix+"\n"+"\n".join(setup)
        source += "\nint st=fused_attention("+",".join(map(str,args))+");"
        source += f"\nIo.WriteByte(st+128); Io.Write(Span.Make(12288,{args[7]}));"
        source += f"\nIo.Write(Span.Make(14336,{args[9]}));"
        expected=oracle(a,args)
        # Last row always sees all S entries for these fixtures.
        assert args[14]+args[10] == args[11]
        assert vm(source) == bytes([128])+expected[12288:12288+args[7]]+expected[14336:14336+args[9]]
    print("PASS: 16385 exp oracle + actual VM inputs, 41 whole-arena cases, rejection sentinels, "
          "8 actual VM attention fixtures; exact bytes, no GPU execution")


if __name__ == "__main__":
    main()
