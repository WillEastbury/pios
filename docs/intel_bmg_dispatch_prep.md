# B50 W2A8 dispatch preparation (offline, not executable)

`intel_bmg_dispatch_prep.{h,c}` prepares kernel-private bytes for the existing
`b50_w2a8` **matvec** artifact. It neither constructs GPU commands nor submits,
uploads, publishes, enables a backend, or orchestrates a model. Boards were not
accessed. Batched matmul and XMX have different metadata and are rejected.

## Reviewed artifact and authority

Installed ocloc 2026.0, target `0xe212` / `bmg-g21`, Zebin 1.62:

| Input | Bytes | SHA-256 |
|---|---:|---|
| `.text.b50_w2a8` | 1280 | `b527da0bffa4098106e7d476e052988b2097c922366b2fa2c749842446ab70f0` |
| `.ze_info` | 2794 | `44d6d3599c298cb4132a362befcb1f1245c8a71f0f75ff7445c0f422e9e25c62` |

Original Zebin identity:
`4e860e79691c6b3f28b0c1c6d28e1fbdc24fcce5728f0d098c5451d1a1656362`.
Inputs have explicit lengths; exact digests deliberately reject unknown fields,
additional kernels, whitespace changes and compiler drift without a parser.
These are identity checks, not module signature verification or DMA authority.
The existing signed module, owner/generation, lease and VM gates still apply.
No caller-supplied "verified" flag substitutes for these checks.

Intel compute-runtime definitions are pinned to commit
`15675b5e1f56289d26a6ba00c9725ec93a9dcb51`:

https://github.com/intel/compute-runtime/tree/15675b5e1f56289d26a6ba00c9725ec93a9dcb51

Reviewed paths:

- `shared/source/helpers/local_id_gen.inl`: `generateLocalIDsSimdImpl`,
  SIMD32 uint16 channel stride and XYZ wrap order.
- `shared/source/device_binary_format/zebin/zeinfo_decoder.cpp`:
  `populateKernelPayloadArgument`, 32-byte cross-thread rounding, local-ID
  channel sizing.
- `shared/source/command_container/command_encoder_xehp_and_later.inl`:
  first 32 bytes inline, remaining cross-thread followed by per-thread data.
- `shared/source/compiler_interface/linker.h`, `linker.cpp`:
  `implicitArgsRelocationSymbolName`, `patchAddress`, `resolveImplicitArgs`.
- `shared/source/device_binary_format/zebin/zebin_elf.h`: relocation enums.

This project implements only the bounded contract below, not Intel's driver.

## Exact byte layout

The canonical planner still produces its existing 64-byte logical payload.
Maximum populated argument end is 56, rounded to 64. It contains zero global
offsets at 0..11, padding at 12..15, packed/activation/output 64-bit GPU VAs at
16/24/32, rows/cols uint32 at 40/44, and zero private base at 48..55.
The no-scratch/no-stack subset does **not** authorize private allocation.

`struct intel_bmg_dispatch_payload` has 32 inline bytes and 256 indirect bytes:

| Destination | Bytes | Content |
|---|---:|---|
| Inline 0..31 | 32 | Logical payload 0..31 |
| Indirect 0..31 | 32 | Logical payload 32..63 |
| Indirect 32..95 | 64 | X: 32 little-endian uint16 IDs `lane % 16` |
| Indirect 96..159 | 64 | Y: 32 little-endian uint16 zeros |
| Indirect 160..223 | 64 | Z: 32 little-endian uint16 IDs `lane / 16` |
| Indirect 224..255 | 32 | Zero allocation padding, not payload |

Used indirect bytes are **224**, not 256; allocation is rounded to 64.
One SIMD32 hardware thread covers each required `(16,1,1)` work group.
Lanes 16..31 are inactive but their generated Z values are **1**.
A future walker must mask them with exactly `0x0000ffff`, select software
local IDs, retain the per-thread-load entry point (do not use metadata's
192-byte skip offset), and launch `(rows/16,1,1)` groups. Those are requirements
for subsequent command review, **not encoded commands**.

## Relocation subset — important distinction

The two ELF REL records are offsets **76 and 228**, type **2**
(`R_ZE_SYM_ADDR_32`), zero addend, symbol
`__INTEL_PATCH_CROSS_THREAD_OFFSET_OFF_R0`. Both original four-byte placeholders
are zero. Intel names this symbol the **implicit-args-size** builtin.
It is **not** `R_PER_THREAD_PAYLOAD_OFFSET` (type 4), and its value is **not**
the 32 indirect cross-thread bytes.

Intel `resolveImplicitArgs` leaves the placeholders unchanged unless stack
calls or a debugger require implicit args. For this exact no-stack/no-debugger
artifact, the helper copies the immutable source into distinct destination
storage and explicitly writes the reviewed zeros. This is intentionally a
zero-only resolution, not a generic linker. Nonzero implicit args, another
symbol/type, a nonzero addend, missing/extra/duplicate records or code drift
rejects before writing. Record order may vary. Output cannot overlap code,
metadata, relocation records or symbol spans; inputs must remain stable for
the call. Success means **prepared bytes only**, never GPU success.

## Validation and remaining gates

Recreate the ignored fixture with `build_shader_b50.bat` before running
`tests/test_intel_bmg_dispatch_prep.c`. From the repository root:

```powershell
clang -Wall -Wextra -Werror -std=c11 -Itests\stubinc -Iinclude `
  tests\test_intel_bmg_dispatch_prep.c src\intel_bmg_dispatch_prep.c `
  src\intel_bmg_tensor_plan.c src\intel_bmg_vm.c src\sha256_hkdf.c `
  -o tests\_build\test_intel_bmg_dispatch_prep.exe
tests\_build\test_intel_bmg_dispatch_prep.exe
```

The test uses
the real pinned ISA/metadata, checks every output byte, inactive lanes,
padding, capacity, code and metadata identity, relocation rejection and input/
failure-output immutability. Missing fixtures fail with an explicit build
instruction rather than silently skipping the identity test.

A separate bytes-only [compute-walker encoder](intel_bmg_walker.md) now matches
all 160 bytes against pinned generated-header C++ oracle fixtures. It does not
create an executable sequence: a complete Xe2 dispatch still requires
reviewed generated command fields, state-base programming, instruction heap
mapping, entry alignment, MOCS/cache attributes, pipeline selection,
interface descriptor fields, dispatch mask/geometry, software-ID mode,
scratch/private-base policy, barriers/cache visibility and completion lifetime.
The existing tensor VM maps data, not an instruction heap. The existing CCS
store-only proof is not a compute kernel proof. Do not feed this structure
directly to `b50 submit` or reinterpret store completion as tensor parity.

## Staged hardware acceptance protocol (not run)

All stages require explicit future owner-approved board availability. Use
exclusive core-0 authority, bounded deadlines, unchanged version/monotonic
uptime, management health, and a fresh generation for each experiment.

1. **Artifact gate:** rebuild/extract, record source/Zebin/ISA/metadata digests,
   validate signed module and exact ABI. Failure is `artifact/metadata/reloc`;
   no hardware action is permitted.
2. **Transport baseline:** run existing `tools/pios_b50_submit_probe.py` only
   against an explicitly approved board, with a new exclusive log path.
   It requires topology `b50=1`, native attachment, registered CCS context and
   fresh submission; completion must be `42/FEED0001`, not just GuC acceptance.
   It snapshots before `b50 revoke` in `finally`. Preserve stages as
   `topology`, `attachment`, `context`, `submission`, `completion`, `health`,
   `deadline`, `revocation`; never collapse failure into GPU success.
3. **Mapping/publication gate:** separately prove code/data address authority,
   disjointness, permissions, coherent Normal-NC backing, table/code/data
   visibility, entry bounds and revocation. Failure is `mapping/publication`;
   no walker is admitted. Retain immutable inputs and exclusive outputs.
4. **Encoder gate:** the isolated COMPUTE_WALKER layout now matches an
   independent pinned generated-header oracle, including reserved bits and
   rejection tests. The complete state/pipeline/barrier command sequence
   remains **unsupported**, so stages 5–6 cannot run.
5. **Bounded compute probe:** future distinct command, not current `b50 submit`.
   One `(16,32)` tile, independent dense signed-int oracle, sentinel/red zones,
   one-hot K cases and signed extremes, unique completion token. Log launch,
   accepted/started/completed separately. Timeout, fault, malformed descriptor,
   guard mutation or wrong result quarantines; do not retry blindly or poll
   indefinitely. Snapshot before revoke, then prove DMA stopped before reuse.
6. **Parity/retirement:** only after exact output and unchanged guards match,
   test multiple groups and stale handles, reject short authority before
   submission, and repeat fresh generations. Record timing as unvalidated
   until lifecycle/visibility and independent parity pass. No automatic
   backend activation follows even a passing probe.

If revoke cannot prove DMA cessation, keep allocations/leases quarantined and
escalate through the established recovery protocol; do not free/reuse them.
No upload, acceptance harness network call or board probe was performed for
this offline change.
