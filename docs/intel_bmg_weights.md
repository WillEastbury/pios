# Offline resident packed-weight lifetime registry

`intel_bmg_weights` is a kernel-private, core-0-only fixed table (eight
entries), not a GPU upload implementation. A zero-initialized instance is
initialized once by its lifetime owner; resetting it would invalidate the
stale-token contract and is forbidden. The explicit core argument checks
routing, not caller privilege. All APIs run serially on core 0; there are no
shared mutable controls, locks, publication barriers, or cross-core readers.
Control and span records have separate 64-byte aligned/stride cache lines.

Attach transfers an **already owned** backing allocation's lifetime to the
registry without copying its bytes. The caller proves kernel accessibility,
exclusive ownership and immutability (including any aliases); shape and
address arithmetic cannot prove those properties. The span contains data,
used length, capacity and shape. Microsoft BitNet 16x32 tile dimensions and
exact `rows * cols / 4` used length are checked with 64-bit arithmetic.
Overlapping backing capacities are refused to prevent duplicate ownership.
This primitive does not validate individual int2 codes or upload VRAM.

Owner identity must be a nonzero, lifetime-stable authority ID supplied by
the kernel, not an unchecked process request or reusable PID alone.
Handles carry slot and generation. Acquire grants one exclusive in-flight
lease per entry; query returns a kernel-private immutable span only while
that exact lease is pinned. Lease sequences prevent a copied old token from
unpinning later work. Release fails while pinned, returns allocation
ownership, clears the stored span/owner and bumps generation. Generation or
lease-sequence exhaustion retires capacity rather than wrapping. Failed
calls do not transfer ownership. Query copies expire on unpin and must not
be retained or passed to another domain. Inputs/outputs must be caller-local,
not aliased into registry storage.

No heap, device I/O, PicoScript hook, runtime registration, scheduling or GPU
activation is added. Future orchestration stays in PicoScript; a real backend
must first prove upload, mapping attributes, completion and DMA revocation.
It must retain the lease until hardware can no longer reference the span.
B50 full GPU activation remains unproven and gated; this is offline lifetime
logic only.

## Prepared tensor job (offline integration)

`intel_bmg_tensor_job` composes the real registry and tensor planner, with
one zero-initialized, core-0-owned job per caller-owned static record. Control
and prepared metadata occupy separate cache lines. Prepare pins a weight
lease, checks its shape against the requested rows/columns, checks exact int8
activation used length and empty int32 output with sufficient capacity, and
builds the existing GPU-VA layout and 64-byte Zebin argument payload.
Pointer/capacity overflow and overlapping backing capacities are rejected.
Only metadata is copied; weights and activations are not read or copied.

Prepare returns false on invalid, busy, stale, wrong-owner/core or exhausted
inputs, without transferring buffer ownership or leaving a new weight pin.
Query returns a private snapshot valid only until abort. Successful prepare
retains activation immutability and exclusive output lifetime until abort;
the caller proves accessible kernel storage and all alias ownership. The job
alone owns its lease: do not unpin it through the registry or mutate either
record. Abort validates owner/generation, unpins exactly once, clears pointers
and payload, and advances generation. Old tokens cannot cancel later jobs;
generation exhaustion permanently retires the job. Never reset its storage.

This is **preparation, not GPU execution**: virtual addresses in the payload
are planner values, not evidence of installed mappings. Job prepare itself
does no VRAM allocation, upload, VM mapping, hardware submission, completion, output production or
runtime hook is implemented. Abort is only safe because there is no hardware
consumer; a future executing backend must prove DMA revocation before unpin.
Model orchestration remains PicoScript, with C limited to the kernel tensor
primitive. The composition tests exercise payload bytes, invalid spans/shape,
busy weights, stale cancellation and pin rollback without any board access.

## Offline plan-to-PPGTT preparation

`intel_bmg_tensor_plan_vm_prepare()` extends the existing planner, without a
new runtime layer or hook. Pass the prepared job's canonical plan, a separate
zero-initialized VM record, an explicitly owned 20 KiB aligned table buffer
and its DMA address, and a disjoint page-aligned DMA backing allocation with
capacity at least `plan.allocation_bytes`. All calls are serial, kernel-private
core-0 operations. Owner IDs are kernel lifetime authorities, not process IDs;
generation tokens belong to this VM storage and must never be reset.

The caller proves both allocations are exclusively owned Normal-NC memory,
including every CPU alias, and that device DMA is revoked before preparation.
The helper does not infer CPU accessibility from DMA addresses. Backing follows
the plan's offsets from `packed_va`: packed weights, activation, output, and
the argument payload each occupy whole distinct page-rounded regions.
The caller must separately populate these owned regions with the real job
data and payload, initialize padding, and retain the job's weight lease;
this helper neither copies nor pretends to upload anything.

The full canonical plan, capacities, address arithmetic, table/backing DMA
disjointness and argument/table CPU overlaps are checked before table writes.
Packed, activation and payload pages are RO; only output pages are RW,
including output's last partially used page. Entries use PAT2 (system UC,
matching Normal-NC backing). Five real table levels are constructed, every
unused entry remains invalid, excess backing capacity is not mapped, and the
VM is sealed before a root DMA address is returned. Invalid input leaves
storage and root output untouched; callers must check the boolean result.
An unexpected post-prepare failure retires/poisons tables and advances the VM
generation, never returning a partially prepared root.

`intel_bmg_tensor_plan_vm_retire()` checks core, owner, generation and explicit
DMA revocation, poisons tables, clears VM authority, and advances generation.
Before aborting/releasing an associated job, retire its VM; do not retain a
returned root after retirement. No GPU root installation, PAT hardware
programming, submission, completion, output production or live execution proof
exists here. Host tests walk the resulting five-level tables; no board or
network access is needed or performed.

Validation: `python tests\run_host_tests.py` includes stale reuse, wrong
owner/core, busy release, copied leases, table exhaustion and shape/address
bounds tests. No board access is required.

## Offline batched matmul / LM-head foundation

`modules/b50_w2a8_matmul.cl` is a separate, compiled **SIMD integer**
primitive: `Y[b,r] = sum_c W[r,c] * A[b,c]`. It does not change the
existing matvec ABI or planner. W uses Microsoft16x32 packing, A contains
contiguous signed-int8 rows, and Y contains contiguous **batch-major int32**
rows. No bias, scaling, quantization, softmax or token selection is fused.

Arguments, in order, are packed pointer, activation pointer, output pointer,
`rows`, `cols`, `batches`, `packed_bytes`, `activation_bytes`,
`output_elements`. Lengths describe real accessible, disjoint allocations;
the future caller must retain immutable inputs and exclusive output ownership.
Rows must be a multiple of 16 in 16..65536, columns a multiple of 32 in
32..65536, batches in 1..256. Lengths must cover `rows*cols/4`,
`batches*cols`, and `batches*rows` respectively. Invalid jobs touch nothing;
they do not produce an error/status output, so a future dispatcher must reject
them before launch rather than interpret unchanged output as success.
The caps keep unsigned indexing and signed-int32 accumulation safe, including
packed code 0 (weight -2). Normal ternary packing uses weights -1, 0, +1.

Launch geometry is 2D `(round_up(rows,16), batches)` with local size `(16,1)`.
Extra row/batch work-items return without touching memory. Partial weight
tiles are unsupported: pad the model matrix before packing, not at dispatch.

An LM-head projection is this same primitive with rows equal to padded
vocabulary size and columns equal to hidden width. Token selection belongs
in PicoScript: use the existing `Tensor.ArgMaxI32` orchestration on each
completed logits-row span, excluding padded vocabulary entries. Do not apply
ReLU to logits. This is a projection foundation, **not** a new callable
GPU-backed PicoScript hook or a complete model execution pipeline.

`build_shader_b50.bat` compiles/disassembles both shaders using installed
ocloc 2026.0, device `0xe212` (`bmg-g21`). Offline inspection of the batched
Zebin shows metadata version 1.62, SIMD32, 128 GRFs, required group
`[16,1,1]`, 32 inline bytes, and a 192-byte local-ID payload. Pointer offsets
are 16/24/32; six uint arguments occupy offsets 40..60; an implicit private
base pointer occupies offset 64. **The old matvec payload/extractor contract
cannot be reused** for this kernel. Generated disassembly is under
`build_module\b50_w2a8_matmul_disasm`. ocloc disassembly reports
`unexpected padding at end of kernel`; compilation succeeds.

`tests/test_b50_w2a8_matmul.c` compiles the exact shader as host C and compares
multiple activation batches against an unpacked dense integer oracle, covering
signed extremes, zero weights/activations, multiple tiles, excess launch IDs,
invalid shapes and short allocation authority. It is included in the host
runner. This proves source arithmetic/indexing, **not GPU execution parity**.
XMX/DPAS source/compiler contracts, batched ABI preparation, device mappings,
dispatch/completion and live output parity remain missing. No hardware,
network or deployment was used for this foundation.
