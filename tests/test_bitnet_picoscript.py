"""Compile the shipped fixture with the shipped WebIDE compiler; run real C VM."""
import ast
import json
import math
from pathlib import Path
import re
import shutil
import struct
import subprocess

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "tests" / "_build" / "bitnet_fixture"
OUT.mkdir(parents=True, exist_ok=True)

# Reuse the exact compiler and hook registry served by PIOS, without a sibling
# checkout, browser, network, or a replacement source-to-bytecode compiler.
asset = (ROOT / "src" / "ide_assets.c").read_text(encoding="utf-8")
html = "".join(
    ast.literal_eval(line.strip().replace(r"\?", "?"))
    for line in asset.splitlines()
    if line.strip().startswith('"')
)
scripts = re.findall(r"<script\b[^>]*>(.*?)</script>", html, re.S)
hooks = next(s for s in scripts if "root.PV_HOOKS = H" in s)
compiler = next(s for s in scripts if "root.PicoCompile = P" in s)
node = shutil.which("node")
cc = shutil.which("clang")
assert node and cc, "node and clang are required for the offline fixture test"

exe = OUT / "fixture_vm.exe"
build = subprocess.run([
    cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-ffreestanding",
    "-Wno-comment", "-Wno-unused-function",
    "-I", str(ROOT / "include"),
    str(ROOT / "tests" / "bitnet_fixture_vm.c"),
    str(ROOT / "src" / "picovm.c"), "-o", str(exe),
], capture_output=True, text=True)
assert build.returncode == 0, build.stdout + build.stderr

# Dense ternary oracle intentionally does not decode or reuse bitmap kernels.
WEIGHTS = (
    (1, -1, 0, 1, -1, 0, 1, -1),
    (-1, -1, -1, -1, -1, -1, -1, -1),
    (0, 0, 0, 0, 0, 0, 0, 0),
    (1, 0, 1, -1, 1, -1, 0, 1),
)
ACTIVATION = (-128, 127, -3, 4, -5, 6, 0, -7)
SOURCE = (ROOT / "examples" / "bitnet_w2a8_layer.pc").read_text()


def execute(source):
    js = (
        "const vm=require('vm'); const ctx={}; vm.createContext(ctx);\n"
        f"vm.runInContext({json.dumps(hooks)},ctx);\n"
        f"vm.runInContext({json.dumps(compiler)},ctx);\n"
        f"const p=ctx.PicoCompile.compileC({json.dumps(source)});\n"
        "console.log(p.words.map(w=>(w>>>0).toString(16)).join('\\n'));\n"
    )
    compilation = subprocess.run([node, "-"], input=js,
                                 capture_output=True, text=True)
    assert compilation.returncode == 0, compilation.stderr
    compiled = compilation.stdout
    assert 0 < len(compiled.splitlines()) <= 4096
    result = subprocess.run([str(exe)], input=compiled, check=True,
                            capture_output=True, text=True).stdout
    return bytes.fromhex(result.strip())


FUSED_SOURCE = (ROOT / "examples" / "fused_ops.pc").read_text()


def fused_case(call, inputs, count, status=0, output=12288):
    setup = []
    for pointer, values in inputs:
        if values and len(set(values)) == 1:
            setup.append(f"int setupIndex = 0; while (setupIndex < {len(values)}) {{"
                         f" fused_write({pointer} + setupIndex * 2, {values[0]});"
                         " setupIndex = setupIndex + 1; }")
        else:
            for index, value in enumerate(values):
                setup.append(f"fused_write({pointer + index * 2}, {value});")
    # Sentinel proves failures do not mutate caller-owned output.
    setup.append(f"int sentinelIndex = 0; while (sentinelIndex < {count * 2}) {{"
                 f" Memory.Set({output} + sentinelIndex, 90);"
                 " sentinelIndex = sentinelIndex + 1; }")
    source = FUSED_SOURCE + "\n" + "\n".join(setup)
    source += f"\nint fusedStatus = {call};\nIo.WriteByte(fusedStatus + 128);\n"
    source += f"Io.Write(Span.Make({output}, {count * 2}));\n"
    result = execute(source)
    assert result[0] == status + 128, (call, result.hex())
    if status:
        assert result[1:] == bytes([90] * (count * 2))
        return []
    return [x[0] - 16384 for x in struct.iter_unpack(">H", result[1:])]


def trunc_div(n, d):
    return (abs(n) // d) * (-1 if n < 0 else 1)


FA = [256, -128, 64, 192, -256, 128, 0, 256]
FW = [256, 128, -256, 256, 0, -128]
matrix_expected = [
    trunc_div(sum(FA[r * 2 + c] * FW[n * 2 + c] for c in range(2)), 256)
    for r in range(4) for n in range(3)
]
assert fused_case(
    "fused_matmul(4096,16,8192,12,12288,24,2,2,3,2,256,24)",
    [(4096, FA), (8192, FW)], 12) == matrix_expected
assert fused_case(
    "fused_matvec(4096,4,8192,12,12288,6,3,2,6)",
    [(4096, FA[:2]), (8192, FW)], 3) == matrix_expected[:3]
assert fused_case(
    "fused_lmhead(4096,4,8192,12,12288,6,3,2,128,6)",
    [(4096, FA[:2]), (8192, FW)], 3) == [
        trunc_div(x * 128, 256) for x in matrix_expected[:3]]
assert fused_case(
    "fused_matmul(4096,1024,8192,128,12288,1024,8,8,8,8,256,4096)",
    [(4096, [128] * 512), (8192, [-128] * 64)], 512) == [-512] * 512
for call, status in [
    ("fused_matmul(4096,16,8192,12,12288,24,0,2,3,2,256,24)", -1),
    ("fused_matvec(4096,3,8192,12,12288,6,3,2,6)", -2),
    ("fused_matvec(12288,4,8192,12,12288,6,3,2,6)", -3),
    ("fused_matvec(4096,4,8192,12,12288,6,3,2,5)", -4),
    ("fused_matvec(4096,4,8192,12,12288,6,3,2,6)", -5),
]:
    fused_case(call, [(4096, [513, 0]), (8192, FW)], 3, status)

for scores in ([0, -256, 0, 512, 512, -16384],
               [-20, -10, -10, -16384, -1, -30]):
    for k in (0, 1, 3, 6):
        order = sorted(range(len(scores)), key=lambda i: (-scores[i], i))[:k]
        expected_pairs = [x for i in order for x in (scores[i], i)]
        assert fused_case(
            f"fused_topk(4096,12,12288,24,6,{k},36)",
            [(4096, scores)], k * 2) == expected_pairs
for k in (-1, 7):
    fused_case(f"fused_topk(4096,12,12288,24,6,{k},42)",
               [(4096, [0] * 6)], 12, -1)
fused_case("fused_topk(4096,12,12288,24,6,3,17)",
           [(4096, [0] * 6)], 6, -4)
fused_case("fused_topk(4096,2,12288,4,1,1,1)",
           [(4096, [16384])], 2, -5)
assert fused_case("fused_topk(4096,128,12288,256,64,64,4096)",
                  [(4096, [0] * 64)], 128) == [
                      x for i in range(64) for x in (0, i)]

# Independent floating-point QK / sqrt(D) / causal softmax / V oracle.
# Quantized output tolerates only the documented Q8.8 exp/truncation error.
FQ = [256, 0, 128, -256, -128, 192]
FK = [256, 0, 0, 256, -256, 128]
FV = [256, -128, -256, 64, 128, 256]
for tokens, offset in ((3, 0), (1, 2), (2, 1)):
    query = FQ[:tokens * 2]
    oracle = []
    for row in range(tokens):
        scores = [sum(query[row * 2 + d] * FK[j * 2 + d]
                      for d in range(2)) / (256 * 362)
                  for j in range(offset + row + 1)]
        weights = [math.exp(s - max(scores)) for s in scores]
        oracle.extend(sum(w * FV[j * 2 + e] for j, w in enumerate(weights)) /
                      sum(weights) for e in range(2))
    actual = fused_case(
        f"fused_attention(4096,{tokens*4},8192,12,9216,12,"
        f"12288,{tokens*4},16384,6,{tokens},3,2,2,{offset},362,45)",
        [(4096, query), (8192, FK), (9216, FV)], tokens * 2)
    assert all(abs(a - b) < 3 for a, b in zip(actual, oracle)), (actual, oracle)
fused_case(
    "fused_attention(4096,12,8192,12,9216,12,12288,12,16384,5,3,3,2,2,0,362,45)",
    [(4096, FQ), (8192, FK), (9216, FV)], 6, -2)
fused_case(
    "fused_attention(4096,12,8192,12,9216,12,12288,12,16384,6,3,3,2,2,0,362,44)",
    [(4096, FQ), (8192, FK), (9216, FV)], 6, -4)
fused_case(
    "fused_attention(4096,12,8192,12,9216,12,12288,12,16384,6,3,3,2,2,2147483647,362,45)",
    [(4096, FQ), (8192, FK), (9216, FV)], 6, -1)
print("Fused PicoScript: real compiler/VM matrix, head, stable top-k, causal attention PASS")


def expected(activation):
    projection = [sum(w * a for w, a in zip(row, activation))
                  for row in WEIGHTS]
    relu = [max(0, x) for x in projection]
    winner = max(range(4), key=lambda i: relu[i])
    return struct.pack(">8iB", *projection, *relu, winner)


baseline = expected(ACTIVATION)
assert struct.unpack(">8iB", baseline) == (-239, 6, 0, -153, 0, 6, 0, 0, 1)
assert execute(SOURCE) == baseline
assert execute(SOURCE) == baseline, "fresh runs must be deterministic"

# Pin signed byte endpoints, zero-row behavior, negative accumulators and
# first-index argmax ties; only the fixture's activation bytes change.
for activation in ((0,) * 8, (127, -128, 3, -4, 5, -6, 0, 7)):
    source = SOURCE
    for col, value in enumerate(activation):
        source = re.sub(rf"Memory\.Set\({4128 + col}, \d+\);",
                        f"Memory.Set({4128 + col}, {value & 255});", source)
    assert execute(source) == expected(activation)

print("BitNet PicoScript W2A8: compiled C VM, full projection/ReLU/argmax, 4 runs PASS")

# Independently specified dense vocabulary head and recurrent embedding graph.
# No bitmap decoding, native inference orchestration, or tensor-hook mocks.
EMBEDDINGS = ((1, 0), (0, 2), (2, 1), (1, 3))
LM_HEAD = ((-1, -1), (1, -2), (1, -2), (-2, 1))
TOKEN_SOURCE = (ROOT / "examples" / "bitnet_token_loop.pc").read_text()


def token_oracle(seed, budget, state=(0, 0)):
    records = []
    for _ in range(budget):
        state = tuple((h + e) % 8 for h, e in zip(state, EMBEDDINGS[seed]))
        logits = tuple(2 * sum(w * h for w, h in zip(row, state))
                       for row in LM_HEAD)
        seed = max(range(4), key=lambda i: logits[i])
        records.append((*logits, seed))
    return b"".join(struct.pack(">4iB", *record) for record in records)


tokens = token_oracle(0, 8)
assert [tokens[i] for i in range(16, len(tokens), 17)] == [1, 3, 3, 1, 1, 3, 3, 1]
assert struct.unpack(">4iB", tokens[:17]) == (-2, 2, 2, -4, 1)
assert struct.unpack(">4iB", tokens[4 * 17:5 * 17]) == (-10, -2, -2, -8, 1)
assert execute(TOKEN_SOURCE) == tokens
assert execute(TOKEN_SOURCE) == tokens, "autoregressive fresh runs must match"
for seed, budget, state in (
        (0, 0, (0, 0)), (2, 1, (0, 0)), (3, 8, (0, 0)),
        (0, 8, (7, 0)), (1, 1, (2, 0))):
    source = TOKEN_SOURCE.replace("int token = 0;", f"int token = {seed};")
    source = source.replace("int tokenBudget = 8;", f"int tokenBudget = {budget};")
    source = source.replace("int state0 = 0;", f"int state0 = {state[0]};")
    source = source.replace("int state1 = 0;", f"int state1 = {state[1]};")
    assert execute(source) == token_oracle(seed, budget, state)
# Wraparound produces all-zero ties; an all-negative tie keeps first maximum.
assert token_oracle(0, 1, (7, 0)) == struct.pack(">4iB", 0, 0, 0, 0, 0)
assert token_oracle(1, 1, (2, 0)) == struct.pack(">4iB", -8, -4, -4, -4, 1)
print("PicoScript synthetic LM head: 7 C VM runs PASS; tokens 1 3 3 1 1 3 3 1")

# Independent floating-point decoder, not an emulation of VM fixed-point math
# or its CORDIC/exp approximations. All matrix rows are dense mathematical data.
DECODER_SOURCE = FUSED_SOURCE + "\n" + (
    ROOT / "examples" / "tiny_decoder.pc").read_text()
DECODER_EMBED = ((1, .5), (-.5, 1), (.75, -1), (-1, -.75))
DECODER_HEAD = ((1, 1), (1, -1), (-1, 1), (-1, -1))


def rms(vector):
    scale = math.sqrt(sum(x * x for x in vector) / 2 + 1 / 65536)
    return tuple(x / scale for x in vector)


def rotate(vector, pos):
    c, s = math.cos(pos / 2), math.sin(pos / 2)
    return (vector[0] * c - vector[1] * s,
            vector[0] * s + vector[1] * c)


def decoder_oracle(prompt, budget):
    keys, values, records = [], [], []
    token = prompt[0]
    for pos in range(budget):
        if pos == 1:
            token = prompt[1]
        x = DECODER_EMBED[token]
        a, b = rms(x)
        q = rotate((a, b), pos)
        k = rotate((b, a), pos)
        v = ((a - b) / 2, (a + b) / 2)
        keys.append(k)
        values.append(v)
        scores = [sum(qi * ki for qi, ki in zip(q, key)) / math.sqrt(2)
                  for key in keys]
        weights = [math.exp(score - max(scores)) for score in scores]
        attended = tuple(sum(w * value[d] for w, value in zip(weights, values))
                         / sum(weights) for d in range(2))
        residual = tuple(xi + ai for xi, ai in zip(x, attended))
        a, b = rms(residual)
        up = (max(0, a + b), max(0, a - b))
        final = rms((residual[0] + (up[0] - up[1]) / 2,
                     residual[1] + up[1] / 2))
        logits = tuple(sum(w * h for w, h in zip(row, final))
                       for row in DECODER_HEAD)
        winner = max(range(4), key=lambda i: logits[i])
        records.append((pos, token, (*k, *v, *attended, *logits), winner))
        token = winner
    return records


def decoder_source(prompt=(0, 1), budget=8, sessions=1):
    return (DECODER_SOURCE.replace("int prompt0 = 0;", f"int prompt0 = {prompt[0]};")
            .replace("int prompt1 = 1;", f"int prompt1 = {prompt[1]};")
            .replace("int tokenBudget = 8;", f"int tokenBudget = {budget};")
            .replace("int sessions = 1;", f"int sessions = {sessions};"))


def read_decoder(data):
    assert len(data) % 43 == 0
    return [struct.unpack(">BB10iB", data[i:i + 43])
            for i in range(0, len(data), 43)]


decoder_runs = 0
max_error = 0.0
for prompt in ((0, 1), (0, 2), (0, 3), (1, 0), (2, 3), (3, 2)):
    raw = execute(decoder_source(prompt))
    decoder_runs += 1
    actual = read_decoder(raw)
    reference = decoder_oracle(prompt, 8)
    assert len(actual) == len(reference) == 8
    for observed, (pos, token, intermediates, winner) in zip(actual, reference):
        assert observed[:2] == (pos, token)
        assert observed[-1] == winner, (prompt, pos, observed, winner)
        for fixed, floating in zip(observed[2:-1], intermediates):
            error = abs(fixed / 256 - floating)
            max_error = max(max_error, error)
            assert error < .04, (prompt, pos, observed, intermediates, error)
    # Same VM, stale cache from prior session; every prefix is re-established.
    assert execute(decoder_source(prompt, sessions=2)) == raw + raw
    decoder_runs += 1
    for cap in (0, 1, 2, 4, 7):
        assert execute(decoder_source(prompt, budget=cap)) == raw[:cap * 43]
        decoder_runs += 1

# Future prompt tokens cannot change position zero. At position one, attention
# must actually read the previous KV pair, rather than using only current V.
first = execute(decoder_source((0, 1)))
other = execute(decoder_source((0, 3)))
decoder_runs += 2
assert first[:43] == other[:43]
assert first[43:86] != other[43:86]
self_only = DECODER_SOURCE.replace(
    "8568, 4, 8600, 16, 1, pos + 1, 2, 2, pos, 362, 40",
    "8568, 4, 8600, 16, 1, 1, 2, 2, 0, 362, 40").replace(
    "int j = 0;", "int j = pos;").replace(
    "8500 + j * 4", "8500").replace("8502 + j * 4", "8502").replace(
    "8532 + j * 4", "8532").replace("8534 + j * 4", "8534")
assert execute(self_only) != first, "KV history must influence the result"
decoder_runs += 1
for prompt, budget, sessions in (
        ((-1, 1), 8, 1), ((0, 4), 8, 1), ((0, 1), -1, 1),
        ((0, 1), 9, 1), ((0, 1), 8, 0), ((0, 1), 8, 3)):
    assert execute(decoder_source(prompt, budget, sessions)) == b"\xff"
    decoder_runs += 1
# Public decode guard rejects before cache access or output.
for pos, token in ((-1, 0), (8, 0), (0, -1), (0, 4), (1, 0)):
    guarded = DECODER_SOURCE[:DECODER_SOURCE.index("// A two-token prompt")]
    guarded += f"\nIo.WriteByte(decode({pos}, {token}) + 1);\n"
    assert execute(guarded) == b"\x00"
    decoder_runs += 1

# Cache slots retain their original K/V, not the last token's state. Check
# every physical slot after generation, including the highest allowed slot.
cache_dump = DECODER_SOURCE + """
int slot = 0;
while (slot < 8) {
    emit32(load32(8192 + slot * 16));
    emit32(load32(8196 + slot * 16));
    emit32(load32(8200 + slot * 16));
    emit32(load32(8204 + slot * 16));
    slot = slot + 1;
}
Io.WriteByte(Memory.Get(8480));
Io.WriteByte(decode(7, 0) + 1);
Io.WriteByte(decode(8, 0) + 1);
"""
assert execute(cache_dump) == first + b"".join(
    struct.pack(">4i", *row[2:6]) for row in read_decoder(first)) + b"\x08\x00\x00"
decoder_runs += 1

# Poison all future slots: a one-token decode must ignore them. The first
# slot is overwritten before consumption; no cache clear is necessary.
poisoned = decoder_source(budget=1).replace(
    "int prompt0 = 0;", """
int poison = 8192;
while (poison < 8320) {
    Memory.Set(poison, 255);
    poison = poison + 1;
}
int prompt0 = 0;""")
assert execute(poisoned) == first[:43]
decoder_runs += 1
print(f"PicoScript tiny decoder: {decoder_runs} real C VM runs PASS; "
      f"KV/RoPE/attention/logits max float error={max_error:.6f}; "
      f"tokens={[row[-1] for row in read_decoder(first)]}")
