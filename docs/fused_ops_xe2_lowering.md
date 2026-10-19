# Fused Q8.8 reference versus offline Xe2 kernels

No lowering is activated. Runtime orchestration remains PicoScript; OpenCL
here is an offline native GPU primitive source, not an OS dependency. Nothing
in this work accesses or deploys to a board. Emitted B50 ISA plus host-source
parity is **not GPU execution parity**, dispatch validation or a speedup.

## Exact stable top-k candidate

`modules/b50_fused_topk.cl` implements `fused_topk` v1 directly in the private
VM byte layout from `examples/fused_ops.pc`: biased-u16 big-endian scores,
signed descending order, ascending index on equal scores, k score/index
records, untouched spare capacity. N=1..64, k=0..N. No float inputs, heap,
sampling or XMX instructions. A single active work-item performs bounded
N-element validation then N*k selection. This deliberately prioritizes exact
semantics over parallel throughput for the tiny reference bound.

Arguments in order:

| Index | Argument | Authority |
|---|---|---|
| 0,1 | arena pointer, uint arena_bytes | At least 65536 backed bytes |
| 2,3 | int32 status pointer, uint status_bytes | At least four backed bytes |
| 4,5 | int input_base, input_capacity | Exact reference span offsets/capacity |
| 6,7 | int output_base, output_capacity | Exact reference span offsets/capacity |
| 8,9,10 | int N, k, budget | Required budget N*k |

Launch global/local (1,1,1) with zero global offset; any other global IDs
return untouched. One invocation/job, not a batched grid. Caller must prove
arena/status allocations valid, status disjoint from the entire arena, stable
input and exclusive output authority. Four status bytes are the only bytes
modified outside the arena. Short status authority returns without any access
(no safe error destination exists).

Status is 0 success, -1 shape, -2 span/short arena, -3 full-capacity overlap,
-4 budget, -5 score domain, with exactly that precedence. Status is separate
from the semantic output, which remains unchanged on rejection. Full spans
are validated before input reads. Encodings 32768..65535 are rejected even
when k=0; k=0 still scans all N scores and otherwise writes no output bytes.
Offset/capacity arithmetic is overflow-safe after the bounded span checks.
The reference's literal interval overlap rule, including zero-capacity
intervals inside a nonempty input span, is preserved.

Status is an ordinary store, **not** an asynchronous release/acquire protocol.
Before activation a native job must establish generation/ownership, DMA and
memory attributes, immutable admission metadata, completion fences/cache
maintenance, and authorized copy-in/out of the private VM arena. Passing a VM
integer as a GPU address is forbidden. No existing W2A8 payload helper is
reused for this different argument layout.

### Reproduction and evidence

```powershell
cmd.exe /d /c "C:\source\pios\build_shader_b50_topk.bat"
clang -std=c11 -Wall -Wextra -Werror tests\test_b50_fused_topk.c -o tests\_build\test_b50_fused_topk.exe
tests\_build\test_b50_fused_topk.exe
```

The existing host runner also includes this test. It compiles the actual
OpenCL scalar source as host C, checks an independent stable insertion-sort
oracle for every N/k combination, minimum/maximum/all-equal/negative/mixed
scores, tail sentinels, invalid domains, full-capacity alias, integer extremes,
short spans/status, k=0, end-of-arena spans and extra work-items. Null payloads
prove early admission rejection doesn't touch them. It does not emulate GPU
execution.

Verified here: the host-source oracle passes with `-Wall -Wextra -Werror`;
`python tests\test_bitnet_picoscript.py` also passes the actual shipped
compiler/VM fused-op tests and the existing BitNet/LM-head/tiny-decoder cases.

Real ocloc 2026.0 compilation on 2026-10-11 targets device 0xe212 (bmg-g21),
64-bit CL1.2. `build_module\b50_topk` contains Zebin and disassembly, `.ze_info`
and `proof.json` recording SHA-256 of source, reference, tests, build/check
scripts, binary, ISA disassembly and complete metadata. The checker fails if
the reviewed pointer/scalar payload or SIMD32/128-GRF/(1,1,1) launch changes.
The emitted payload has arena/status pointers at 16/24, arena/status lengths
at 32/36, seven signed scalar arguments at 40..64, and compiler private-base
pointer at 72; local-ID payload is 192 bytes. This is evidence for a future
dispatcher, not permission to open-code a new walker. The disassembler's
`unexpected padding at end of kernel` warning is retained and disclosed.

## Exact dense matmul / matvec / LM-head candidate

`modules/b50_fused_matmul.cl` implements the dense reference directly, not
`b50_w2a8_matmul` or its packed tensor ABI. One active work-item reads
A[B,M,K] and shared W[N,K], writes O[B,M,N], and implements exactly
`trunc(trunc(dot/256)*scale/256)` with signed int32 arithmetic. B,M,N,K
are 1..8, scale is 0..256, and each input element is -512..512.
Matvec uses B=M=1, scale=256; LM head uses B=M=1 and the supplied scale.
No narrowing to int8, ternary conversion, merged division or negative
right-shift rounding is permitted. Dot and scaled intermediates have
absolute bounds 2097152; output bound is 8192.

Arguments 0..3 are the same arena/length/status/length authority as top-k.
Signed arguments 4..15 are A,Al,W,Wl,O,Ol,B,M,N,K,scale,budget.
The same single-item launch, separate non-aliasing backed status allocation,
private synchronous ownership and ordinary completion-store restrictions
apply. Rejection precedence is -1 shape/scale, -2 spans/short arena,
-3 full output-capacity overlap with either input, -4 budget B*M*N*K,
-5 input domain. Read-only A/W may alias. Both complete input scans precede
any output write, including scale=0. Spare output bytes remain untouched.

```powershell
cmd.exe /d /c "C:\source\pios\build_shader_b50_matmul.bat"
clang -std=c11 -Wall -Wextra -Werror tests\test_b50_fused_matmul.c -o tests\_build\test_b50_fused_matmul.exe
tests\_build\test_b50_fused_matmul.exe
```

The host runner includes the actual OpenCL-source C oracle test: all 4096
shape combinations, mixed signs, extrema, all 257 scales, single-row
specializations, explicitly nonmergeable negative truncation, input alias,
full-capacity output alias, budget/domain precedence, late invalid elements
at scale zero, integer-extreme span rejection, short arena/status, status
canary, spare-capacity canaries, end-of-arena output and extra work-items.
This is not GPU emulation or execution.

Verified 2026-10-11 with real ocloc 2026.0, device 0xe212/bmg-g21, CL1.2
64-bit: emitted SIMD32/128-GRF ISA with no DPAS, required group (1,1,1),
32-byte inline data, stateless arena/status pointers at payload 16/24,
lengths 32/36 and twelve scalar fields 40..84. The metadata checker pins
these fields and captures complete metadata and SHA-256 source/reference/
test/build/checker/binary/disassembly provenance in
`build_module\b50_matmul\proof.json`. Disassembly reports the same
`unexpected padding at end of kernel` warning as top-k. Host-source parity
and actual compiler/VM reference suites pass. No speedup or board execution
is claimed; native job admission, marshalling and GPU parity remain required.

## Explicit incompatible boundaries

| Reference call | Existing primitive | Gap / conversion required |
|---|---|---|
| matmul: A[B,M,K], W[N,K], B/M/N/K 1..8 | `b50_fused_matmul` dense candidate above | Exact host-source semantics and compiled scalar Xe2 ISA; GPU parity, native job/VM marshaling and wider DPAS decomposition remain missing. `b50_w2a8_matmul` packed int8/ternary operands cannot replace these dense -512..512 values. |
| matvec, scale=256 | Dense candidate with B=M=1 | Exact offline specialization; packed `b50_w2a8` remains incompatible and emits native int32 rather than biased byte output. |
| LM head, scale 0..256 | Dense candidate with B=M=1 | Exact two-stage truncating scale epilogue, not merged division or shifts. Existing packed SIMD/XMX accumulators do not meet this contract. |
| stable top-k | `b50_fused_topk` candidate above | Exact host-source semantics and compiled Xe2 ISA; native job/VM marshalling and real GPU validation remain missing. |
| causal attention | `b50_fused_attention` integer candidate below | Exact offline VM semantics; GPU execution, native authority/marshalling and XMX lowering remain unproven. W2A8 dot alone is not attention. |

`b50_w2a8_xmx.cl` computes one A[8,32] * W[16,32] tile with signed byte DPAS
and int32 output. Packed W codes 1/2/3 map -1/0/+1, reserved 0 rejects the
subgroup; ordinary SIMD W2A8 decodes code 0 to -2 instead. Neither is the
reference dense ABI. Their tile padding, narrower activation domain,
missing two-stage scale, status behavior and native allocation arguments
must not be presented as compatible superinstruction implementations.
`intel_bmg_tensor_plan`, weights/job, state/walker/batch and dispatch-prep
helpers describe the existing packed tensor allocations/payloads and offline
submission contracts, not this top-k arena/status or dense reference ABI.
XMX layout helpers prove operand packing, not VM byte semantics.

## Exact integer attention candidate (offline only)

`modules/b50_fused_attention.cl` implements the entire v1 operation in one
bounded active work-item. Shapes T,S,D,E=1..8, offset>=0 and offset+T<=S,
divisor=256..4096, dense biased-big-endian Q8.8 inputs -512..512 only.
It writes the same S-element caller scratch row and T*E outputs as the
reference, with no materialized T*S score matrix. Masked scratch bytes and
spare capacities remain untouched. Read-only Q/K/V may alias; entire O and
scratch capacities must be disjoint from all inputs and each other.

Arguments 0..3 are arena/arena_bytes/status/status_bytes as above.
Signed arguments 4..20 are Q,Ql,K,Kl,V,Vl,O,Ol,Scratch,Scratchl,
T,S,D,E,offset,divisor,budget. The same separate backed status allocation,
zero-offset global/local (1,1,1) launch, stable inputs and exclusive write
authority apply. Rejection precedence -1 shape, -2 spans/short arena,
-3 full-capacity alias, -4 budget T*S*(D+E+1), -5 element domain is identical.
All three inputs are fully scanned before any output **or scratch** write.
A status allocation shorter than four bytes causes no access. Completion
is an ordinary store, not a synchronization/publication guarantee.

### Real VM exponential, not a substitute hook

`src/picovm.c` implements hook `Maths.Exp` through `pv_q16_exp`.
For negative z above -681300 it computes
`k=floor((floor(z*94548/65536)+32768)/65536)`,
`r=z-k*45426`, then seven Taylor terms starting at 65536:
`term=trunc(floor(term*r/65536)/n)`, summed into the accumulator.
It halves the result -k times. At z<=-681300 it returns zero.
Attention supplies only z=(score-peak)*256 in [-4194304,0], so the
positive saturation branch is unreachable. In the unsaturated interval,
k is -15..0 and r is -22717..22712; signed 64-bit products and int32
accumulators are safe. The source spells floor shifts as explicit integer
division, preserving the VM's arithmetic right shift and negative Taylor
rounding; `/n` truncates toward zero. OpenCL `long` is 64-bit (the Windows
C harness explicitly maps it to `long long`). No floating instructions,
`exp`, `native_exp`, table surrogate, fake hook or hidden fallback is used.
Weight is the resulting positive Q16.16 integer divided by 256. A peak
always contributes 256, denominator is 256..2048, weighted sum magnitude
<=1048576, output magnitude <=512; dots <=2097152, scores <=8192.

**Precision policy: exact bytes, zero tolerance**, only this quantized v1
domain. FP16/BF16/FP32, larger shapes, multiple heads, strides, KV/native
descriptors and a differently rounded exponential are unsupported.
Ideal floating softmax's earlier error below three integer units is not a
candidate tolerance or permission for automatic lowering.

### Reproduction, measured evidence and live gates

```powershell
python tests\test_b50_fused_attention.py
cmd.exe /d /c "C:\source\pios\build_shader_b50_attention.bat"
```

Verified 2026-10-11: clang host-source compilation with strict warnings;
independent Python arbitrary-precision integer oracle and the actual VM
`Maths.Exp` hook both match all 16385 quantized exp differences,
41 complete-arena success cases (maximum shapes,
sign/extrema, scales and legal input aliases), early rejection/late-invalid
element sentinels including scratch, and eight actual shipped compiler/
`src/picovm.c` attention fixtures match output and final scratch exactly.
The runner invokes the real `Maths.Exp` hook via the existing VM harness,
not a replacement implementation. These are measured offline tests, not
an exhaustive whole-operation proof or GPU emulation.

Real oneAPI ocloc 2026.0 device 0xe212/bmg-g21, 64-bit CL1.2 compilation and
disassembly succeeded. Reviewed metadata: SIMD32, 128 GRFs, group (1,1,1),
32-byte inline data, stateless pointers 16/24, lengths 32/36, seventeen
signed scalar fields 40..104. `build_module\b50_attention\proof.json`
pins that ABI and hashes kernel, reference, actual VM, harness/test,
build/check scripts, Zebin and disassembly plus full `.ze_info`.
The standalone runner is intentionally not added to the already-dirty
host-suite registry.

No runtime registration, automatic lowering, native job binding, XMX claim,
performance claim, board/network access or deployment. Live activation still
requires separately reviewed allocation/generation/ownership admission,
attribute-safe VM copy-in/out, immutable GPU payload, watchdog/deadlines,
completion fences/cache maintenance and actual B50 byte-parity evidence
including invalid/no-write paths. Compiled metadata is not dispatch authority.
