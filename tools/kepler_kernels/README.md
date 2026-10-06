# K2000 fixed kernel images

These are **GK107 / SM30 native instructions**, not PTX, CUDA runtime modules,
or GK110 code. They are not an enabled PicoScript backend yet. GPU execution
still requires GR/FECS/GPCCS initialization, a compute context, GPU mappings,
and a bounded QMD/completion path.

| Image | Bytes | Registers | Threads in one CTA | Operation |
|---|---:|---:|---:|---|
| `store_const` | 64 | 3 | 1 | Store `0x13579bdf` to one output word |
| `vector_add_i32` | 192 | 10 | 64 | Up to 64 additions, wrapping modulo 2^32 |
| `matvec_i8` | 256 | 13 | 64 | Up to 64 rows x 1024 columns; signed bytes, i32 sums |

`dot_i8` uses `matvec_i8` with exactly one row. No kernel uses shared memory,
local memory, a stack, atomics, or host-memory addresses. Matvec uses one lane
per row and a bounded serial inner loop: this is a correctness-first kernel,
not a demonstrated speedup over NEON.

## Build

Modern CUDA `ptxas` no longer targets SM30. Use envytools revision
`f102b82381f3f11cee113d16374c87091db039d9` from
<https://github.com/envytools/envytools>. Its assembler target for this card is
**`-m gf100 -V gk104`**, not `-m gk110`.

The minimal CMake project builds only the assembler/disassembler and their
dependencies from an external checkout; it does not install a GPU driver.
On Windows, use LLVM-MinGW, CMake, Ninja and WinFlexBison. Resolve WinGet's
Flex/Bison links to the real executables so Bison can locate its data:

```powershell
$gcc = (Get-Command gcc.exe).Source
$flex = (Get-Item (Get-Command win_flex.exe).Source).Target
$bison = (Get-Item (Get-Command win_bison.exe).Source).Target
cmake -S tools\kepler_kernels -B build_kepler\assembler -G Ninja `
  '-DCMAKE_BUILD_TYPE=Release' '-DCMAKE_POLICY_VERSION_MINIMUM=3.5' `
  "-DCMAKE_C_COMPILER=$gcc" "-DFLEX_EXECUTABLE=$flex" "-DBISON_EXECUTABLE=$bison" `
  '-DENVY_ROOT=C:\path\to\pinned\envytools'
cmake --build build_kepler\assembler --target envyas envydis --parallel 4
python tools\kepler_kernels\build.py
python tests\test_kepler_kernels.py
```

The builder inserts a conservative schedule word at every 64-byte boundary
and forces 64-bit instruction encodings. It rejects code that differs from
the reviewed image hashes. Do not assemble `.asm.in` directly: envytools can
otherwise choose short encodings, breaking the scheduling layout.

Outputs are binaries, scheduled assembly, disassembly, a hash/launch manifest
and known-answer vectors in `build_kepler\kernels`. This separate directory
survives the OS build's cleanup of `build`. The same code bytes are
preserved in generated `include\kepler_kernels.h`. The host regression decodes
the emitted instruction subset, checks signed-byte loads, loop targets,
span bounds and overflow behavior, and reproduces the existing BitNet Wq
fixture's argmax **57** / weighted checksum **170896**. This model does not
validate real GPU scheduling or execution.

## Private bring-up ABI

All parameters and device output words are little-endian. PicoScript i32
spans are big-endian; the eventual adapter must convert, not expose these
device words as PicoScript spans unchanged.

- `store_const`: c0 offset 0 contains the output GPU address (u64).
- `vector_add_i32`: c0 offsets 0/8/16 contain A/B/output GPU addresses (u64);
  offset 24 is the element count (u32, 1..64).
- `matvec_i8`: the same three pointers; offsets 24/28 are rows/columns
  (u32, 1..64 / 1..1024). Input lengths must equal rows*columns and columns.
- Every launch is one CTA. Extra lanes exit before accessing memory.
  Zero columns are invalid and must be rejected before dispatch.
- `make_case()` validates exact input lengths and dimensions before creating
  parameters. It emits CPU reference output in both device and PicoScript
  byte order. Parameters must remain immutable for the job lifetime.

Reserved physical VRAM locations (not host pointers):
code slots `0x200000`, `0x201000`, `0x202000` (4 KiB each);
parameters `0x210000`; input A `0x220000` (64 KiB);
input B `0x230000` (4 KiB); output `0x231040` with 64-byte red zones.
These are disjoint from the copy-channel structures. The future compute
VMM must explicitly map the required pages; uploading does not create those
translations.

## Guarded upload only

```powershell
python tools\pios_kepler_load.py --log C:\temporary\kernel-check.jsonl
python tools\pios_kepler_load.py --execute --log C:\temporary\kernel-load.jsonl
```

Without `--execute`, validation is entirely offline. Execution requires a
posted K2000, Bus Master off, no bound channel, PBDMAs and GR disabled, clean
PCIe/management health, and a new log. Only the three exact reviewed images
and fixed slots are accepted. Upload uses generation-checked PRAMIN transfers
and compares every code byte; the store fixture is staged with poisoned
output and intact red zones. No GR dispatch occurs.

Live upload on `v20261002.231552` verified all three images byte-for-byte.
`dispatched=false` and `compute_ready=false` remain explicit.
