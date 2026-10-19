# Offline W2A8 dispatch batch assembly

`intel_bmg_batch.{h,c}` now assembles an actual deterministic **352-byte
inspection batch**, including general/IOH state, the existing SIMD32 walker,
an immediate-write fence and a terminator. It never uploads or submits it.
No live backend, board, deployment or hardware state is touched.

## Explicit authority and bounded inputs

The independently supplied nonzero owner/generation/context must match the
request, policy and both immutable references. The caller supplies an exact
size policy record with `validated == 1`, nonzero `review_id`, reviewed EU
count, threads/EU, thread limit and CFE thread count. Missing/unvalidated
policy rejects; there is no topology or product-policy fallback. The limit
must fit the reviewed EU/thread product and CFE must fit that limit and its
16-bit command field. These are caller attestations, not GPU discovery,
cryptographic authority or proof that a B50 supports those values.

Instruction, indirect, tensor, batch and fence allocations are explicit
unsigned 48-bit spans. All five whole allocations must be pairwise disjoint.
Instruction/indirect/tensor allocations are page aligned and page sized;
batch starts at 64-byte alignment; fence starts at eight-byte alignment.
Code/indirect references carry host data, used, capacity, GPU address and
immutable/token metadata. Full reference capacities must fit their heaps;
offsets are 64-byte aligned and walker-representable. Code is exactly 1280
used bytes; indirect is exactly 256 used bytes. The current canonical tensor
plan proves the stateless tensor addresses and bounds. Indirect content must
match its remaining cross-thread bytes, all 192 software-local-ID bytes and
32 zero padding bytes; inline content is derived from that same plan.

The caller retains prepared, immutable code and payload backing for the
result lifetime. Code reference identity/signature/relocation validation is
the earlier `intel_bmg_dispatch_code_prepare`/module layer's obligation;
assembly does **not** authenticate arbitrary code or confer PPGTT authority.
The GPU references describe prospective allocations, not installed mappings.
Do not hand these caller-local raw pointers across an ownership domain.

Request/policy sizes, alignment, pointer arithmetic, full output capacity,
result size/alignment and output/result overlap against each input's full
span are validated. No failed call writes either output. All fallible
encoders run into bounded stack storage before the final byte/result copy.
Success preserves bytes beyond `used`; `execution_gate` is always **zero**.
There is no publication, DMA/cache maintenance, completion API or retirement
claim. The input code, indirect data and policy remain unchanged.

## Exact inspection recipe

| Offset | Bytes | Command |
|---:|---:|---|
| 0 | 24 | PIPE_CONTROL, explicit pre-pipeline flags, CS stall |
| 24 | 4 | PIPELINE_SELECT, GPGPU |
| 28 | 24 | CFE_STATE, reviewed thread count, no scratch |
| 52 | 24 | PIPE_CONTROL, explicit pre-SBA flags, CS stall |
| 76 | 88 | STATE_BASE_ADDRESS, instruction + general/IOH |
| 164 | 160 | COMPUTE_WALKER, current W2A8 ABI, SIMD32 mask `0xffff` |
| 324 | 24 | PIPE_CONTROL, explicit completion flags + immediate fence |
| 348 | 4 | MI_BATCH_BUFFER_END |

The new `intel_bmg_sba_encode` sets instruction/general base and page-size
modify bits, raw even instruction/general/stateless MOCS 0..126 and explicit
L1 policy 0..4. General state is **IOH**, not reserved SBA dwords 8/9.
Its page count is bounded to the described allocation (not the runtime
properties path's `0xfffff` blanket size). Reserved, surface, dynamic and
bindless fields remain generated defaults. This is a representable bounded
offline subset, not a reproduction of every runtime SBA path.

## Pinned ordering review: execution remains gated

Intel compute-runtime commit:
`15675b5e1f56289d26a6ba00c9725ec93a9dcb51`.
Generated Xe2 header SHA-256:
`694afcd0816c977da61fed759513921159044dcfa4d841dac3d1683eb2062e0c`.
The MIT adaptation and full license notice in `intel_bmg_walker.md` apply
to the state bytes used here.

Reviewed source contents were independently re-fetched at that immutable
commit and matched the local copies byte-for-byte:

* `shared/source/command_container/command_encoder_xehp_and_later.inl`
  SHA-256 `50bc42527c3e27cda21da9394cc21e815c7697bcee314c2fb622b02a2472be8d`.
  `EncodeDispatchKernel::encode` conditionally adjusts pipeline selection,
  emits a barrier for dirty heaps/uncached MOCS before SBA, and places SBA
  before the walker. SBA takes engine, heapless state, debugger, compression
  and `doubleSbaWa`; `EncodeWA` derives basic/extended nonpipelined-state
  workarounds from hardware/release state. Walker post-sync setup is itself
  conditional, so a separately encoded immediate fence is not equivalent
  proof of the runtime's completion path.
* `shared/source/helpers/preamble_xehp_and_later.inl`
  SHA-256 `2baa6814aec615c22def2f66b8fa4460d9ddf0c1916435d6b800ff3c1bd750e3`.
  `programVfeState` uses device thread limits, stream state and CCS topology;
  generic `addPipeControlBeforeVfeCmd` is empty.
* `shared/source/helpers/preamble_xe_hpg_and_xe_hpc.inl`
  SHA-256 `68d6b9e79ae02052b01b29adb1f126a2d1258469ea005fbc9217b0121711b635`.
  Shared pipeline programming uses conditional synchronization/transitions,
  not authority for a universal Xe2 prefix.
* `shared/source/helpers/state_base_address_xehp_and_later.inl`
  SHA-256 `05b03e50adbd5360d5feb607d869aa01697baa7593ea543939b6e49766ef3b40`.
  IOH uses general state; MOCS comes from GMM/product cache settings.

**Verified:** the above generic partial ordering and exact command layouts.
**Not verified:** the complete Xe2/B50 engine-specific CFE/pipeline/barrier
ordering, workaround selection, product MOCS/L1 semantics, initial context
state, PPGTT mappings/permissions, publication visibility and fence
retirement guarantees. Therefore this recipe is explicitly an offline
inspection fixture, not a vetted hardware execution sequence. Even a
validated caller policy cannot open `execution_gate`. Hardware-specific
review and a separate authorized submission design remain prerequisites;
no runtime tensor parity or runnable hardware claim is made.

## Repeatable acceptance

```powershell
python tools\check_b50_state_oracle.py tests\_build\walker_oracle\pinned_xe2.inl
python tools\check_b50_batch_oracle.py tests\_build\walker_oracle\pinned_xe2.inl
```

Both extract unmodified command definitions from the digest-checked header,
compile independent C++ `sInit`/setter oracles, and compare every byte.
State acceptance extends the existing six-command oracle with full
instruction/general/stateless/L1 SBA cases. Batch acceptance compares two
whole 352-byte streams, including exact-fit offset and highest-fence-address
boundaries. Native tests also cover output alignment/tail preservation,
absent policy, token drift, thread bounds, malformed policy, heap overlap,
overflow, short capacities, altered indirect IDs, host/output aliasing and
unchanged byte/result output on failure.

Strict ARM acceptance for both changed native modules:

```powershell
cmd.exe /d /c "C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin\aarch64-none-elf-gcc.exe -Wall -Wextra -Werror -ffreestanding -nostdlib -nostartfiles -march=armv8.2-a+simd+crc+crypto -Iinclude -O2 -c src\intel_bmg_batch.c -o tests\_build\batch_oracle\intel_bmg_batch.o"
cmd.exe /d /c "C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin\aarch64-none-elf-gcc.exe -Wall -Wextra -Werror -ffreestanding -nostdlib -nostartfiles -march=armv8.2-a+simd+crc+crypto -Iinclude -O2 -c src\intel_bmg_state.c -o tests\_build\batch_oracle\intel_bmg_state.o"
```
