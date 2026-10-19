# Offline PicoScript W2A8 layer fixture

`examples/bitnet_w2a8_layer.pc` is a deterministic, self-contained 4-output,
8-input BitNet-style layer, not a downloaded model or a complete language model.
PicoScript owns byte initialization, shape selection, projection, ReLU, argmax
and output. It uses only existing `Memory.Set`, `Span.Make`,
`BitLinear.SetShape/MatVecBitmap`, `Tensor.ReLUI32/ArgMaxI32` and `Io` hooks.
There is no new runtime interface or native model orchestration.

Weights are ternary {-1, 0, +1}, encoded as two one-bit planes per row: one
zero byte, then one minus byte, with column 0 in the least significant bit.
This is the existing bitmap W2 format, **not** the I2S shader/module format.
Activations are signed int8; accumulators are signed int32, with scale 1,
no bias and no normalization or learned calibration.

Input: `[-128, 127, -3, 4, -5, 6, 0, -7]`.
Dense rows (the independent test oracle):

```text
 1 -1  0  1 -1  0  1 -1
-1 -1 -1 -1 -1 -1 -1 -1
 0  0  0  0  0  0  0  0
 1  0  1 -1  1 -1  0  1
```

Expected projection: `[-239, 6, 0, -153]`.
Expected ReLU: `[0, 6, 0, 0]`; argmax: `1`.
Output is 33 binary bytes: four big-endian int32 projection values, four
big-endian int32 ReLU values, then the one-byte argmax index.

## Verification (boards remain offline)

```powershell
python tests\test_bitnet_picoscript.py
python tests\run_host_tests.py
```

The focused test needs Node.js and clang. It extracts the existing compiler
and hook registry from the shipped WebIDE asset, compiles the actual `.pc`
source, then executes its bytecode in `src/picovm.c` with no accelerator hooks
installed. It compares every output byte against Python dense integer dot
products, repeats the baseline, and checks zero/tie and changed-sign inputs.
Compilation uses the freestanding VM path with `-Wall -Wextra -Werror`
(existing VM comment/unused-function warnings are suppressed); the shared VM harness has a 100,000-step budget,
64 KiB memory, and a 4,096-word program limit. Unused emulator/crypto namespaces
are test-only fallthrough stubs; tensor execution is the real VM implementation.

This proves an offline CPU/reference-runtime fixture only. It does not load
`tensor_module`, exercise `intel_bmg_tensor_plan`, submit GPU work, or establish
EU execution, PPGTT correctness, accelerator speed, or active GPU inference.
The B50 CCS store proof and compiled `modules/b50_w2a8.cl` remain separate
bring-up evidence; neither authorizes GPU inference. No board access or
deployment is performed by this test.

## Synthetic autoregressive LM-head foundation

`examples/bitnet_token_loop.pc` adds an explicitly synthetic recurrent/token
graph, **not a real pretrained BitNet transformer**. All embedding lookup,
state update, vocabulary projection, greedy selection and loop control run in
PicoScript. Its reusable `lmhead(weights, hidden)` function invokes the existing
`Tensor.SetShape`, `Tensor.MatVecI8` and `Tensor.ScaleI32` hooks, followed by
`Tensor.ArgMaxI32` in the caller. No C inference engine was added; both sources
use the same existing C VM harness.

The four token IDs have embeddings `[1,0], [0,2], [2,1], [1,3]`.
Starting with state `[0,0]` and seed token `0`, each step computes
`state = (state + embedding[token]) mod 8` componentwise. The row-major signed
int8 vocabulary weights are `[-1,-1], [1,-2], [1,-2], [-2,1]`.
Logits are twice the dense dot products (integer scale 2, no bias, ReLU,
softmax, randomness, normalization or attention). Greedy argmax chooses the
first index on ties, including when every logit is negative. The generated
token becomes the next embedding index. The explicit budget is eight tokens;
there is no EOS rule. States stay in `[0,7]`, logits in `[-42,14]`, and token
IDs in `[0,3]`. This is one matrix-vector projection per step, not a batched
matmul implementation.

Expected step records, each four big-endian int32 logits plus one token byte
(136 output bytes total):

```text
[-2,   2,   2,  -4] -> 1
[-6,  -6,  -6,   0] -> 3
[-14,-16, -16,   2] -> 3
[-6,   6,   6, -12] -> 1
[-10, -2,  -2,  -8] -> 1
[-14,-10, -10,  -4] -> 3
[-22,-20, -20,  -2] -> 3
[-14,  2,   2, -16] -> 1
```

The focused Python test independently specifies dense weights, embeddings and
recurrence, compares every logit/token byte, repeats fresh runs, and tests zero
budget, one-step budget, changed seeds, state wraparound/all-zero ties and
all-negative ties. Its seven token-fixture executions share the existing
100,000-step/4,096-word limits. This establishes offline model-loop and LM-head
composition only, not transformer fidelity, text generation quality, accelerator
activation or GPU inference.

## Complete synthetic decoder draft

[`tiny_decoder.md`](tiny_decoder.md) documents `examples/tiny_decoder.pc`:
a separate complete small decoder graph with RMSNorm, RoPE, causal softmax
attention, a sequential KV cache, residuals, ternary FFN/ReLU, and a greedy
LM head. It is covered by the same offline real-compiler/C-VM test with an
independent floating-point reference. The earlier recurrent example above
remains unchanged and is not relabeled as a transformer.
