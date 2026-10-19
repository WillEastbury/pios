# Offline Xe2 COMPUTE_WALKER encoder

`intel_bmg_walker.{h,c}` is a bounded native OS **byte encoder**, not a
dispatch backend. No board, MMIO, allocation, upload, publication or submission
is involved. Inputs stay immutable and every rejection leaves output unchanged.
Owner/generation/context must match independently supplied caller state; these
checks are not signatures, VM permissions, leases or proof of GPU readiness.
The request is caller-local, not a shared control descriptor.

## Authority and independent oracle

Intel compute-runtime commit
[`15675b5e1f56289d26a6ba00c9725ec93a9dcb51`](https://github.com/intel/compute-runtime/tree/15675b5e1f56289d26a6ba00c9725ec93a9dcb51),
`shared/source/generated/xe2_hpg_core/hw_cmds_generated_xe2_hpg_core.inl`.
Downloaded header SHA-256:
`694afcd0816c977da61fed759513921159044dcfa4d841dac3d1683eb2062e0c`.
Authority is the actual generated `COMPUTE_WALKER`,
`INTERFACE_DESCRIPTOR_DATA` and `POSTSYNC_DATA` definitions and setters, not
an inferred opcode table.

`tools/check_b50_walker_oracle.py` checks that digest, extracts these three
unmodified definitions into a repository-local ignored build directory, and
compiles them with host C++. It invokes their `sInit()` and named setters,
then compares **all 160 bytes** of two commands to
`tests/fixtures/intel_bmg_walker.hex`. The C host test independently compares
its encoder to those same complete fixtures. The generated header is not
vendored; the small fixture contains command bytes only.

The same commit's
`shared/source/command_container/command_encoder_xehp_and_later.inl`,
`encodeThreadData`, establishes SIMD/message SIMD selection, the remainder
execution mask, and the software-ID branch: local maxima, emit-local and
generate-local remain zero when IDs are runtime-generated. Thus `(16,1,1)`
is represented by one SIMD32 IDD thread, mask `0xffff`, and the existing
software XYZ payload, **not** by enabling hardware ID generation. The IDD
TG_SIZE_1 enum is explicitly selected, with no SLM, barriers or samplers.

## Exact supported subset

| Absolute command dword | Encoding |
|---|---|
| 0 | `0x72080026` (40 dwords total) |
| 2 | 224 indirect used bytes |
| 3 | 64-byte-aligned **indirect heap offset**, not absolute VA |
| 4 | `0x82040000`: SIMD32, message SIMD32, inline enabled |
| 5 | `0x0000ffff` |
| 7..9 | `(workgroups,1,1)`; starting IDs zero |
| 19 | 64-byte-aligned **instruction heap offset**, not absolute VA |
| 24 | `0x0c000001`: one thread, TG_SIZE_1 |
| 32..39 | 32 inline bytes |

All other dwords, including postsync, are zero. Kernel offset is the entry
including per-thread loads; do not add metadata's 192-byte skip offset.
This primitive accepts bytes, not arbitrary kernels: its intended input is
the already digest-checked matvec preparation. It does not itself verify ISA,
metadata or inline contents; caller must retain those upstream identity gates.
It neither reads nor copies the indirect allocation.

Both heap bases must be 4-KiB aligned and their full nonempty allocations fit
the conservative unsigned 48-bit address subset. Both offsets must fit the
generated 32-bit fields without truncation. Full 1280-byte ISA and 256-byte
indirect allocation ranges must fit and be mutually disjoint. The latter
includes the existing 32-byte allocation padding; encoded length stays 224.
Full host request/output capacity spans are checked for wrap and overlap before
touch; request alignment is checked before dereference. Workgroups are bounded
to 1..4096 as this offline primitive's explicit admission limit.
Success writes exactly 160 bytes and preserves any output tail.

## Validation

From repository root, after obtaining the exact public pinned header:

```powershell
python tools\check_b50_walker_oracle.py PATH_TO_PINNED_HEADER
clang -Wall -Wextra -Werror -std=c11 -Itests\stubinc -Iinclude `
  tests\test_intel_bmg_walker.c src\intel_bmg_walker.c `
  -o tests\_build\test_intel_bmg_walker.exe
tests\_build\test_intel_bmg_walker.exe
```

The oracle covers normal and high 32-bit offsets, 3 and 4096 groups, distinct
inline patterns, and every reserved byte. Tests cover token drift, short
spans, overflow, null/unaligned/overlapping host spans, heap overlap, field
truncation, alignment and unchanged failure output. Strict ARM compilation
uses the existing freestanding `-Wall -Wextra -Werror` toolchain.

## Explicitly unsupported: executable sequence

Individual CFE_STATE, instruction-only STATE_BASE_ADDRESS, PIPELINE_SELECT,
PIPE_CONTROL and batch start/end byte encoders are now independently verified;
see [`intel_bmg_state.md`](intel_bmg_state.md). A complete sequence remains
**not implemented**. Neither are instruction-heap mappings, product-specific CFE
limits, MOCS/cache policy, required pipeline transitions/barriers, visibility
and completion retirement. Heap bases in the request are prospective bounds,
not programmed SBA. No executable batch can be produced by this API and no
backend is activated. A verified walker layout does not prove a complete
dispatch sequence or hardware tensor parity. These remain separate gates
before any future owner-approved board probe.

## Third-party notice

Numeric layout adaptation and oracle fixtures derive from Intel
compute-runtime, MIT, generated header copyright (C) 2024-2026 Intel
Corporation. Its pinned `LICENSE.md` provides this notice:

MIT License

Copyright (C) 2021 Intel Corporation

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
