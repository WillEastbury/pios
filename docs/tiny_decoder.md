# Offline PicoScript decoder draft

`examples/tiny_decoder.pc` is a complete **synthetic**, one-block,
autoregressive decoder graph, not a pretrained model or a native inference
engine. It has actual causal scaled dot-product softmax attention and an
append-only KV cache, not the recurrence in `bitnet_token_loop.pc`.
No board, network, downloaded weights, accelerator, or sibling checkout is
used by the test or required by the graph.

## Model and execution

- Vocabulary 4 token IDs (not text), hidden width 2, one attention head of
  width 2, FFN width 2, one decoder block.
- Four fixed embeddings are listed in the source. RMSNorm uses unit gain
  and epsilon `1/65536`, before attention, before FFN, and before the head.
- Ternary linear projections: Q is identity, K swaps components, and
  V is `[[1,-1],[1,1]]` with scale 1/2. RoPE rotates Q and K by
  `position/2` radians (one pair). Attention divides dot products by
  approximately sqrt(2); softmax subtracts the maximum score.
- Attention output projection is identity, followed by a residual.
  FFN up projection is `[[1,1],[1,-1]]`, activation is ReLU, down projection
  is `[[1,-1],[0,1]]` with scale 1/2, followed by another residual.
  These are explicit scalar BitLinear ternary multiplies/adds in PicoScript,
  not calls to a native Tensor implementation or packed-weight job provider.
- LM head rows are `[1,1]`, `[1,-1]`, `[-1,1]`, `[-1,-1]`.
  Greedy selection keeps the first index on a tie.
- `prompt0` and `prompt1` default to IDs 0 and 1. Position zero processes
  the first prompt token, position one processes the second; subsequent
  inputs are the preceding greedy result. `tokenBudget` is **total context
  positions**, including prompt positions, not additional generated tokens.
  Its allowed range is 0–8. `sessions` is 1–2 for bounded in-VM replay.
- `decode(pos, token)` requires the next sequential position (control byte
  at 8480), position 0–7, and token 0–3. It rejects invalid calls with `-1`
  before touching cache or producing output. Reset the control byte to zero
  before a new sequence. Each slot is overwritten before it is consumed;
  stale future entries cannot affect the prefix.
- Invalid top-level configuration emits one byte `FF`; an empty valid
  context emits no output.

Each position emits a 43-byte diagnostic record: position byte, input-token
byte, ten big-endian signed int32 Q8.8 values (K0/K1, V0/V1, attended0/1,
logits0–3), and the selected-token byte. The full default context emits
344 bytes. Prompt `[0,1]` selects `[0,0,0,0,0,0,0,0]`; synthetic tokens
are not a statement about language quality.

## Precision, memory, and budgets

Activations, K/V, attention scores, and logits are signed Q8.8 integers.
The existing `Maths.Sqrt` is integer floor sqrt; `Maths.Sin`, `Maths.Cos`,
and `Maths.Exp` use the real VM's Q16.16 implementations. There are no new
hooks. Arithmetic truncates divisions towards zero. Softmax exponentials
are reduced to Q8.8 weights before accumulation; the maximum always has
weight 256, so the denominator cannot be zero.

With these fixed parameters, normalized components remain below 400 in
integer units, cached V components below 400, scores below 1000 in
magnitude, and weight sum at most 2048. Residual/FFN components stay below
2048. RMS square sums, rotated products, scores, and weighted sums remain
well inside signed 32-bit bounds. This proof does **not** authorize arbitrary
weights or increased dimensions/caps.

Prefix the source with `examples/fused_ops.pc` (once); see
[`fused_ops.md`](fused_ops.md) for the reusable lowering contract.
Use a private 64 KiB VM arena. KV uses eight 16-byte slots
at `[8192,8320)`, and byte 8480 is the
next-position counter. Each scalar slot stores a value biased by 16384 in
its first two bytes; the other bytes are unused. These private scratch
records are not a cross-core/DMA ABI. Helper bindings are distinct to avoid
depending on C-frontend binding isolation. The angle uses `16384 * 2` rather
than a positive signed-16-boundary immediate.
The shared attention adapter gathers K/V into `[8500,8564)`, puts Q/output
in `[8564,8572)`, and reserves `[8600,8616)` for bounded score scratch.
It admits at most 40 attention work units per position.

Every loop has a fixed bound: at most 8 attention entries, 8 context
positions, and 2 sessions. The test harness permits 1000000 VM instructions,
at most 4096 bytecode words, and a private 64 KiB arena. It does not provide
an inference callback. Runtime computation remains entirely PicoScript.

## Offline verification

Run `python tests\test_bitnet_picoscript.py` with local Node and Clang.
The test extracts the **shipped WebIDE compiler and registry** from
`src/ide_assets.c`, compiles the actual source, and runs `src/picovm.c`
through `tests/bitnet_fixture_vm.c`. No compiler/VM stand-in is used.

An independent Python floating-point reference specifies dense matrices,
standard sqrt, trig, and exp. It compares all cached K/V, attended vectors,
and four logits at every position with absolute tolerance 0.04 in real
units, and checks exact greedy token IDs. Tests include six prompts,
two sessions in the same VM, context truncation, future-prefix causality,
KV-history ablation, future-slot poisoning, all eight physical cache slots,
and invalid token, position, budget, and session bounds.
Existing W2A8 and recurrent examples remain independently tested.

## Not production inference

No tokenizer, trained weights, model-file loader, packed quantization
format, calibration, multiple blocks/heads, arbitrary-width tensors,
dynamic contexts, EOS policy, sampling, batching, concurrent sessions,
streaming service integration, or production performance claim is included.
ReLU and scalar ternary projections are deliberate draft choices, not a
claim to reproduce a particular BitNet checkpoint or architecture.
The complete small graph is host-verified; offline boards are not deployed
or tested by this change.
