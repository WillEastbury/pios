# Experimental offline B50 W2A8 XMX tile

`modules/b50_w2a8_xmx.cl` is a separate, unbound experimental primitive.
It does **not** replace the ordinary integer SIMD kernels, register a tensor
backend, or authorize GPU inference. No board access, deployment, GPU execution,
performance claim, model weights, or native model orchestration is involved.
Future runtime/model orchestration remains PicoScript-only.

## Arithmetic and contract

One subgroup of 16 lanes computes a complete tile:

- W: 16 rows x 32 columns, 128 bytes in the existing Microsoft16x32 I2S
  permutation. Codes 1/2/3 mean -1/0/+1. Reserved code 0 rejects the entire
  subgroup before any activation read or output write.
- Activation: eight batch rows x 32 signed INT8 columns (256 bytes, row-major).
- Output: eight batches x 16 weight rows (128 signed INT32 elements, row-major).
- `Y[batch,row] = sum(k=0..31, A[batch,k] * W[row,k])`.
  Zero initial accumulator; no bias, scale, saturation, padding or partial tile.
  Absolute sum is bounded by 32*128=4096, so INT32 cannot overflow.

The six kernel arguments are three global pointers, then allocation authority
`packed_bytes`, `activation_bytes`, `output_elements`. Short authority returns
before any payload access. Launch exactly global/local `(16,1,1)`. Extra work
groups return without accesses; required subgroup/work-group sizes are embedded
in metadata. The caller still must provide valid, non-overlapping allocations;
integer lengths alone cannot establish address validity.

The Intel builtin is
`int8 intel_sub_group_i8_i8_matrix_mad_k32(short8 a, int8 b, int8 acc)`.
Here the builtin's left matrix is the eight activation rows, and its right
matrix is W transposed (32 x 16):

- Lane s's `a[m]` packs activation bytes `[m,2*s]`, `[m,2*s+1]`
  into the low/high bytes of a short.
- Lane n's `b[j]` packs weights `[n,4*j..4*j+3]` into the low-to-high
  bytes of an int.
- Lane n receives eight INT32 result components, one per batch m.

Unsigned bit assembly plus `as_short`/`as_int` reinterprets packed signed bytes
without arithmetic conversion of the packed word. The reviewed I2S permutation
is decoded into these signed INT8 registers; this is W2 storage / A8 arithmetic,
not a claim of native two-bit DPAS operands.

## Reproduce

```powershell
cmd.exe /d /c "C:\source\pios\build_shader_b50_xmx.bat"
python tests\run_host_tests.py
```

The build uses the installed oneAPI ocloc 2026.0, device `0xe212`
(auto-detected `bmg-g21`), `-64`, `-cl-std=CL2.0 -I modules`.
Generated artifacts remain under ignored `build_module\b50_xmx`:
the binary, SPIR-V/disassembly sections, `.ze_info`, and `proof.json` with
SHA-256 identities for source, layout header, binary, ISA and metadata.
`tools/check_b50_xmx.py` fails closed unless both actual signed DPAS ISA and
SIMD16/DPAS/required-subgroup/work-group metadata are present.

Verified 2026-10-10: compilation succeeded. Actual disassembly includes:

```text
dpas.8x8 (16|M0) r71:d null:d r63:b r45.0:b {$0}
```

Metadata reports `has_dpas: true`, `simd_size: 16`, required subgroup 16,
work group `[16,1,1]`, and 128 GRFs. Register numbers are not an ABI.
Ocloc's disassembler reports **"unexpected padding at end of kernel"**;
that warning remains disclosed, not treated as execution validation.
The compiler recognizes the builtin and required-subgroup attribute but warns
that the two corresponding extension pragmas are unknown; they are omitted
instead of claiming advertised extension support. No alternative builtin or
tool installation was necessary.

## Independent evidence and remaining blockers

`tests/test_b50_w2a8_xmx_layout.c` runs the actual scalar operand packing
helpers against the independently existing Microsoft I2S packer and original
dense ternary weights. It checks every extracted signed byte, compares a
specification-layout reduction to dense integer dot products, and covers
zero/constant signs, -128/127, batch-distinct mixed inputs, all 32 one-hot K
positions, and reserved codes. It does **not** emulate or execute the builtin.
Host parity plus emitted DPAS proves a compiled candidate, not hardware parity.
Real GPU operand/result parity, dispatch/isolation, lifetime/publication,
reserved-code rejection on GPU and performance remain unverified. No active
runtime path may select this artifact on the strength of these offline tests.

## Public provenance

Builtin signatures, subgroup constraints, operand ownership and byte packing
come from the Intel-authored Khronos OpenCL extension specification,
`cl_intel_subgroup_matrix_multiply_accumulate`, revision 1.1.0,
built-on 2025-01-07, retrieved 2026-10-10:

https://github.com/KhronosGroup/OpenCL-Docs/blob/main/extensions/cl_intel_subgroup_matrix_multiply_accumulate.asciidoc

This implementation is original project code using the documented interface;
no proprietary code or weights were obtained. The existing project I2S packer
provides the storage-format oracle; the specification supplies the independent
matrix-operand interpretation. The generated ISA and metadata, not the kernel
name, establish that this candidate really uses DPAS.
