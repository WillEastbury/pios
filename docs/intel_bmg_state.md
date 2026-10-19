# Offline Xe2 state-command subset

`intel_bmg_state.{h,c}` encodes **individual commands only**. It does not
assemble an executable dispatch, program GPU state, create mappings, or submit.
No runtime fixtures, live GPU path, board or deployment were changed.

The separately added `intel_bmg_sba_encode` extends the individual byte
subset with instruction + general/IOH state, explicit general/stateless MOCS
and L1 policy. [`intel_bmg_batch.md`](intel_bmg_batch.md) documents the new
bounded offline assembly and the still-closed execution gate; the original
instruction-only API and fixtures below remain unchanged.

## Provenance and independent acceptance

Same Intel compute-runtime commit and generated-header SHA-256 as
[`intel_bmg_walker.md`](intel_bmg_walker.md):
`15675b5e1f56289d26a6ba00c9725ec93a9dcb51`,
`694afcd0816c977da61fed759513921159044dcfa4d841dac3d1683eb2062e0c`.
Numeric adaptation is MIT, copyright (C) 2024-2026 Intel Corporation;
the full MIT notice in that document applies to this adaptation too.

`tools/check_b50_state_oracle.py` digest-checks that header, extracts the six
unaltered command definitions, calls `sInit()` and named setters, compiles
the native tests independently, and compares stdout byte-for-byte. No
expected-byte fixture is generated or modified. Each complete command,
including reserved bytes, is compared in two configurations:

| Command | Bytes | Supported fields |
|---|---:|---|
| CFE_STATE | 24 | 1..65535 max threads, zero scratch, NORMAL overdispatch |
| STATE_BASE_ADDRESS | 88 | instruction base, size in pages, modify bits, even raw MOCS 0..126 |
| PIPELINE_SELECT | 4 | GPGPU selection, selection mask 3 only |
| PIPE_CONTROL | 24 | mandatory CS stall; bounded cache flags; optional PPGTT immediate write |
| MI_BATCH_BUFFER_START | 12 | PPGTT, direct first-level chain, no predicate |
| MI_BATCH_BUFFER_END | 4 | generated default terminator |

Thread count and MOCS are representability inputs, **not** discovery of safe
B50 topology or cache policy. CFE large-GRF/single-slice controls are not
selected. SBA leaves all non-instruction fields at generated defaults.
PIPE_CONTROL does not implement a complete product workaround recipe.
An immediate-write command is not proof of retirement or visibility.

Requests are immutable caller-local snapshots with independently supplied
nonzero owner/generation/context tokens. Full source and destination capacity
spans are checked for wrap/overlap, and request alignment is checked before
dereference. GPU allocations must fit unsigned 48-bit ranges. Instruction
base/size are page aligned; batch address/size are dword aligned; a fence
allocation is at least eight bytes with eight-byte-aligned address. Unknown
kinds, unused nonzero fields and truncating values reject. All failures leave
output unchanged; success preserves its tail. This does not confer PPGTT
permissions or validate the contents of a batch allocation.

## Ordering review and precise stop

The pinned sources reviewed under `shared/source/` are:

* `command_container/command_encoder_xehp_and_later.inl`,
  `EncodeDispatchKernel::encode`: pipeline adjustment is conditional on
  tracked systolic state, not an unconditional prefix. Dirty heaps/uncached
  MOCS cause a barrier **before** SBA, which is emitted before the walker.
  SBA encoding also takes `doubleSbaWa`, engine and heapless state.
* Same file, `EncodeWA::addPipeControlPriorToNonPipelinedStateCommand`:
  basic versus extended workarounds depend on hardware/release/engine.
  Extended invalidations and untyped flush are conditional; DC flush is
  explicitly cleared on that path. It is not safe to infer a universal
  PIPE_CONTROL mask from a generated layout.
* `helpers/preamble_xehp_and_later.inl`, `programVfeState`:
  thread count comes from device state and front-end stream properties;
  single-slice dispatch also depends on CCS topology. Its generic
  `addPipeControlBeforeVfeCmd` is empty, not authority to invent a CFE barrier.
* `helpers/preamble_xe_hpg_and_xe_hpc.inl`, `programPipelineSelect`:
  the shared preamble illustrates a conditional pre-selection barrier and
  optional 3D/clean-state transitions. This shared implementation alone is
  **not** a proven Xe2 specialization/workaround selection.
* `helpers/state_base_address_xehp_and_later.inl`: the IOH base is programmed
  through **GeneralStateBaseAddress**, not the reserved SBA dwords 8/9;
  indirect-object properties can set general size to `0xfffff`.
* `command_container/command_encoder.inl`,
  `EncodeBatchBufferStartOrEnd::programBatchBufferStart/End`: start uses
  PPGTT and optional second-level/indirect/predicate semantics; end is the
  generated terminator. Our subset is direct, unpredicated first-level only.

**Original isolated-subset stop (superseded only for offline assembly):**
the original instruction-only encoder has no sequence builder. The new
offline builder/full SBA extension is documented above and does not resolve
the execution gates below. In particular the existing walker
indirect heap cannot be made executable by placing this instruction-only SBA
in front of it. Missing gates are real instruction/IOH PPGTT mappings and
permissions, general/stateless heap programming, engine/context initialization,
product-selected MOCS/L1 policy, Xe2-specialized pipeline/CFE/workaround
ordering, device-derived thread limits, publication/cache visibility and
completion retirement. A future sequence must resolve these together and
independently compare the entire selected stream, not concatenate these
isolated command fixtures. Live dispatch/tensor parity remains unclaimed.

## Repeatable validation

```powershell
python tools\check_b50_state_oracle.py tests\_build\walker_oracle\pinned_xe2.inl
```

This builds/runs focused native tests with `-Wall -Wextra -Werror`: exact
bytes, high addresses, field maxima, token drift, short capacity/request,
misalignment, overflowing GPU/host ranges, overlap, unsupported fields and
unchanged rejection output. Strict freestanding ARM compilation:

```powershell
cmd.exe /d /c "C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin\aarch64-none-elf-gcc.exe -Wall -Wextra -Werror -ffreestanding -nostdlib -nostartfiles -march=armv8.2-a+simd+crc+crypto -Iinclude -O2 -c src\intel_bmg_state.c -o tests\_build\state_oracle\intel_bmg_state.o"
```

Both passed offline. These are acceptance gates for the byte subset only.
