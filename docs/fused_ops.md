# Offline fused PicoScript semantic reference

`examples/fused_ops.pc` supplies reusable functions, not GPU hooks, compiler
opcodes, a pretrained model, or an activated backend. Concatenate it **once**
before the caller source: the shipped WebIDE C frontend has no source imports.
`tiny_decoder.pc` uses this shared attention function rather than a second
softmax implementation. Runtime orchestration is exclusively PicoScript;
Python is an independent test oracle only.

## Version 1 lowering boundary

All pointers and lengths are **private VM byte spans** `(base, capacity)`,
not native addresses or descriptors. The complete capacity must be inside
`[4096,65536)`; callers must actually supply a 64 KiB arena. Scalars occupy
two bytes, big endian, encoded as `Q8.8 integer + 16384`. Accepted matrix,
query, key and value elements are integers in `[-512,512]` (real `[-2,2]`).
All layouts are dense; strides, padding, broadcasting and quantized packed
weights are unsupported. Read-only inputs may alias each other. Entire
output/scratch capacities must be disjoint from inputs and each other.
Caller retains ownership throughout; no handle, allocation, publication,
DMA, asynchronous completion or ownership transfer occurs.

| Function | Argument order after span pairs | Output / arithmetic |
|---|---|---|
| `fused_matmul(A,Al,W,Wl,O,Ol, B,M,N,K,scale,budget)` | B,M,N,K each 1–8; scale 0–256 | A[B,M,K], W[N,K], O[B,M,N]; `trunc(trunc(sum(A*W)/256)*scale/256)` |
| `fused_matvec(A,Al,W,Wl,O,Ol,N,K,budget)` | N,K 1–8 | Same matmul with B=M=1 and scale=256 |
| `fused_lmhead(A,Al,W,Wl,O,Ol,N,K,scale,budget)` | N,K 1–8; scale 0–256 | Same matmul with B=M=1; scaled vocabulary logits, no argmax |
| `fused_topk(A,Al,O,Ol,N,k,budget)` | N 1–64; k 0–N | k `(score,index)` two-scalar records (four bytes each) |
| `fused_attention(Q,Ql,K,Kl,V,Vl,O,Ol,Scratch,Scratchl,T,S,D,E,offset,divisor,budget)` | T,S,D,E 1–8; offset>=0, offset+T<=S; divisor 256–4096 | Q[T,D], K[S,D], V[S,E], O[T,E]; scratch S scalars |

Top-k accepts scores `[-16384,16383]`, descending by signed score, then
ascending token index. No sampling, temperature or mutation of logits.
`k=0` succeeds with no output writes. This integer ABI cannot represent
NaN/infinity: floating-point spans are **unsupported**, not silently given
a ranking policy. Out-of-range encoded integer scores return -5; this does
not detect arbitrary float bytes mislabelled as integer data. A future float
backend must reject NaN before conversion to this ABI, not reinterpret bits.

Attention is one indivisible *semantic* operation: scaled QK, causal mask,
max-subtracted softmax, and weighted V. Row i reads only `j<=offset+i`.
Score integer is `trunc(dot(Qi,Kj)/divisor)`; for sqrt(D) scaling pass the
quantized `256*sqrt(D)` divisor (e.g. 362 for D=2).
Weights are `trunc(Maths.Exp((score-peak)*256)/256)`, using the real VM
Q16.16 hook, then output is `trunc(sum(weight*V)/sum(weight))`.
The maximum weight is 256, so the denominator is positive. Scores stay
within +/-8192, dots within +/-2097152, and weighted sums within +/-1048576;
matrix outputs stay within +/-8192. Intermediate arithmetic fits int32.
No full score matrix is materialized. The reference recomputes weights per
output component; semantic fusion does **not** imply one hardware dispatch.

## Status and admission

Every function returns 0 on success, or -1 shape/scale, -2 span/capacity,
-3 alias, -4 budget, -5 element domain. Validation is in that order and
precedes any output/scratch mutation. Inputs are scanned only after all
spans are validated. Caller discards output on failure. Unused output
capacity remains untouched. Memory stability/exclusive write authority is
the caller's responsibility; these are private synchronous VM functions.

Budget is an explicit operation count, **not milliseconds or VM steps**:
matmul requires B*M*N*K; top-k requires N*k; attention conservatively
requires T*S*(D+E+1), including masked entries. Fixed dimension caps also
bound validation and loops. The VM still needs its independent instruction
cap; the test arena uses 1000000 steps and at most 4096 compiled words.
Capture user-function return values in distinct locals before combining
them: the shipped frontend's shared return binding makes two function calls
in a single arithmetic expression unsafe. Use the bias 16384, avoiding
the frontend's positive signed-16 immediate boundary at 32768.

## Future Xe2 / XMX

A future lowering recognizes these exact call boundaries, validates the
same domains/lengths/budgets, and produces an independently owned native job.
Matmul maps to batched dense GEMM with a shared transposed weight operand;
matvec is its single-row specialization; LM head adds the specified two-stage
integer scale epilogue. Top-k must preserve signed descending/first-index
ties. Attention maps to QK tiles, causal softmax and V accumulation with
bounded private tile scratch. Multi-head batching, FP16/BF16, packed W2A8,
KV descriptors and asynchronous scheduling require separate contracts.
Offline native primitives need not match this byte layout internally but
must marshal and reproduce its observable arithmetic before activation.
No XMX/Xe2 kernel execution or performance is claimed here. The separate
offline [Xe2 lowering contract](fused_ops_xe2_lowering.md) documents an exact
host-source stable top-k, dense matmul/matvec/LM-head, and integer attention candidates compiled
to B50 ISA (single active work-item semantic proofs, not speed claims), and records
why existing W2A8 SIMD/XMX kernels do not implement this dense Q8.8 ABI.

## Verification

`python tests\test_bitnet_picoscript.py` extracts the actual compiler and hook
registry from `src/ide_assets.c`, compiles the source, and executes the actual
`src/picovm.c` via the existing host harness. Tests cover independent dense
batched products, matvec, scaled LM head, top-k negative/tie/k bounds,
causal multi-token and offset attention against floating-point softmax
(less than 3 integer units error), failure output sentinels, and all existing
BitNet/decoder tests. No board access or deployment is needed.

`python tests\test_b50_fused_attention.py` additionally checks the actual
OpenCL attention source against an independent integer Python oracle,
all 16385 possible score-difference exponential inputs against both Python
and the actual VM hook, whole-arena
sentinels and eight byte-exact actual compiler/VM attention fixtures
(including maximum dimensions/extrema). Unlike ideal floating softmax,
this candidate reproduces the VM approximation with **zero tolerance**.
`cmd.exe /d /c "C:\source\pios\build_shader_b50_attention.bat"` compiles
the offline B50 primitive and captures ISA/ABI provenance, not GPU parity.
