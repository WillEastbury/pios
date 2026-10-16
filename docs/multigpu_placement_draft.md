# Offline multi-host GPU placement — DRAFT v1

Approved scope: offline multi-host preparation only. `tools\multigpu_plan.py`
does not connect to boards, load weights, allocate device memory, execute
inference, or implement a distributed executor. Output is an **unactivated
proposal**, not an accepted reservation. No existing wire ABI changes.

Run `python tools\multigpu_plan.py input.json --output proposal.json`.
Focused tests: `python tests\test_multigpu_plan.py`. The test fixture is a
complete input example; its advertised Q6_K capability is synthetic, not a
claim about an implemented kernel.

## Artifact contract

Input/output version is `pios-placement-draft/1`. IDs are unique bounded
strings; generations are positive integer identities, never timestamps.
Hosts carry `id`, `generation`, `available`, and `gpus`. GPUs carry the same
identity/status fields, `capacity_bytes`, `reserved` disjoint half-open byte
intervals (`offset`, `bytes`, `owner`, `owner_generation`), and `backends`.
Reservations bind the GPU generation. Capacity is explicit, never guessed.

Backend capability masks are independently supplied for `Xe2SIMD` and `XMX`.
Bit order is F32, F16, BF16, I16, I8, I4, Q4_0, Q4_1, Q4_K, Q5_K, Q6_K,
Q8_0, Q8_K, IQ4_XS, MXFP4. These are **storage/layout contracts**, not
interchangeable arithmetic types. Masks must be operator/layout-validated
for every layer by the capability supplier; this planner cannot certify them.
Unknown formats/backends fail closed. No implicit conversion, including
BF16→F16 or Q4_K→I4. Existing typed conversion remains separate, with an
explicit converted artifact required before changing a description.

Model identity comprises `model_id` and `model_generation`. Ordered `layers`
each supply `id`, `backend`, `depends_on`, `input_bytes`, `scratch_bytes`,
`kv_bytes`, `state_bytes`, and nonempty `tensors`. Only a linear graph is
supported: first dependencies empty, each subsequent layer depends exactly
on its predecessor. All tensors of a layer share one destination GPU.
Tensor fields: `id`, `generation`, `format`, `shape`, `bytes`, `shards`.
Shard fields: `id`, `generation`, `tensor_offset`, `bytes`, `owner`.
Owner fields: `host`, `host_generation`, `gpu`, `gpu_generation`.
Shards cover the exact tensor byte extent without gaps or overlaps.
Tensor/shard generations match the model; owners must exist, be available,
and match both generations. Byte counts come from independently validated
inventory descriptors, not inferred dtype sizing here. No raw pointers,
file paths, network addresses or executable payloads form this contract.

## Deterministic bounded planning

Limits: 4 MiB CLI input, 16 hosts × 16 GPUs, 256 layers, 4096 tensors,
256 shards/tensor, 256 existing reservations/GPU, rank 8, 1 PiB capacities.
Integers are range checked; sums cannot exceed the capacity ceiling.
Every layer reserves weights + scratch + KV + state simultaneously; scratch
is conservatively not shared across layers. First-fit disjoint intervals
respect pre-existing reservations. Eligible candidates minimize remote weight
bytes plus cross-host predecessor activation bytes, then host/GPU ID and
offset. Host input ordering does not affect output. Same-host cross-GPU
copy/peer-access still needs runtime validation and is not estimated.
Greedy planning can reject a feasible arrangement; no global optimum promised.
Any failure rejects the **whole** proposal without mutating input or writing
a partial result. Concurrent reservation races are outside offline planning.

Transfers name tensor/shard or layer and generation-bound owners, never
remote pointers. `ideal_1gbe_ns = bytes × 8` assumes 1 Gbit/s serialization
only: no latency, contention, protocol overhead, bandwidth or speedup promise.

## Proposed orchestration protocol (not active)

PicoScript should own scheduling; `examples\multigpu_schedule_draft.pc`
illustrates fail-closed scalar gates only. No new PicoScript bindings exist.
A future approved adapter must validate bounded manifests, exact formats,
model graph, plan digest and owner generations; prepare every host's full
reservation atomically; abort all prepared owners on any refusal; and commit
only after every owner acknowledges the same immutable plan. Execution must
wait for predecessor/transfer completion with generation-bound descriptor
authority, length, sequence, deadline and explicit failure status.
Commit/abort recovery, authentication, leases, idempotency, transfer integrity,
cache attributes, local GPU copies and operator capability proofs remain
unimplemented. This draft does not claim network atomicity or runtime safety.
