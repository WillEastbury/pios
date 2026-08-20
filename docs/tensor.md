# Tensor / ML Compute

## Overview

PIOS provides NEON-accelerated tensor operations with a V3D/QPU dispatch framework.

By default, operations run on the ARM Cortex-A76 using 128-bit NEON SIMD (4 floats per instruction, dual-issue capable). Tensor ops only attempt V3D dispatch when a kernel descriptor is bound and marked ready; otherwise they stay on deterministic NEON fallback paths.

V3D attempts are also gated by operation-size thresholds (to avoid offload overhead on tiny work units), and a kernel is quarantined after a failed dispatch so repeated calls do not get stuck in timeout-heavy retry loops.

For `tensor_add`, successful V3D dispatch is additionally sample-verified against CPU math; any mismatch immediately quarantines that kernel and the call falls back to NEON.

When backend selection is `AUTO`, MMIO CSD dispatch is attempted first; if it times out/fails while mailbox QPU is available, MMIO is quarantined for subsequent AUTO dispatches and mailbox becomes the stable fast fallback path.

MMIO CSD submission now performs an idle handshake before queue programming and uses memory barriers around queue-kick/completion visibility.

## Available Operations

| Operation | Function | Accelerated path |
|-----------|----------|------------------|
| Element-wise add | `tensor_add(c, a, b)` | Native V3D CSD for up to 1024 floats when 16-aligned; NEON fallback |
| Element-wise multiply | `tensor_mul(c, a, b)` | Native V3D CSD for up to 1024 floats when 16-aligned; NEON fallback |
| Scalar multiply | `tensor_scale(b, a, s)` | `dup`, `fmul`, `st1` |
| Dot product | `tensor_dot(&result, a, b)` | V3D multiply into a temporary for up to 1024 floats, ARM reduction; NEON fallback |
| Matrix-vector | `tensor_matmul(c, a, x)` (x is a column) | Single fused GPU dispatch: broadcast x across rows, one V3D multiply over the m*K matrix (m*K up to 1024), ARM row-reduce; NEON fallback |
| Matrix multiply | `tensor_matmul(c, a, b)` | V3D-assisted dot chunks for small/medium inner dimensions; NEON fallback |
| ReLU | `tensor_relu(b, a)` | Native V3D CSD for up to 1024 floats when 16-aligned; NEON fallback |
| Softmax | `tensor_softmax(b, a)` | Schraudolph exp approximation + NEON lane-sum normalize; optional V3D dispatch when a bound softmax kernel exists |

## Tensor Lifecycle

```c
tensor_t a, b, c;

// Allocate in GPU-coherent memory (accessible by both ARM and VideoCore)
tensor_alloc(&a, rows, cols, 4);    // 4 = sizeof(float32)
tensor_alloc(&b, rows, cols, 4);
tensor_alloc(&c, rows, cols, 4);

// Upload data from ARM memory
tensor_upload(&a, my_data);

// Compute
tensor_matmul(&c, &a, &b);
tensor_relu(&c, &c);

// Download result
tensor_download(&c, output_buffer);

// Free
tensor_free(&a);
tensor_free(&b);
tensor_free(&c);
```

Tensors are allocated via VideoCore mailbox (`gpu_mem_alloc`) in coherent memory, so both ARM and GPU can access them without cache management.

## SIMD Primitives (simd.h)

Lower-level NEON operations used throughout the kernel:

| Function | Description | Throughput |
|----------|-------------|------------|
| `simd_memcpy(dst, src, n)` | NEON `ldp q/stp q` copy | 64 bytes/iteration |
| `simd_zero(dst, n)` | NEON zero fill | 64 bytes/iteration |
| `simd_memset(dst, val, n)` | NEON byte fill | 64 bytes/iteration |
| `simd_checksum(data, len)` | IP checksum via `uaddlp`/`uadalp` | 32 bytes/iteration |
| `hw_crc32c(data, len)` | Hardware CRC32C | 8 bytes/cycle |

## QPU Framework (experimental dispatch path)

The QPU dispatch infrastructure is implemented and tested:

```c
qpu_enable(true);                    // Enable QPU access via mailbox
qpu_load_program(&prog, code, len);  // Upload shader to GPU memory
qpu_dispatch(&prog, jobs, num_qpus); // Submit to 1-12 QPUs
qpu_free_program(&prog);
```

For real bound kernels, the V3D layer now exposes `v3d_kernel_bind_blob(...)` to upload externally generated uniform/shader blobs into GPU-coherent memory and bind them into the dispatch descriptor table. Native CSD bring-up additionally exposes `v3d_kernel_bind_csd(...)`, `v3d_kernel_bind_builtin_qpu_grid(...)`, and an admin-gated user API `tensor_bind_kernel_csd(...)` for binding exact CSD `cfg[0..6]` words.

VideoCore VII has 12 QPUs (3 slices × 4), each a 4-wide float SIMD unit at 800MHz. Peak theoretical: 76.8 GFLOPS. The VC VII ISA is still incomplete publicly, so V3D paths remain verification-gated and quarantine on mismatch.

## Native V3D bring-up checklist

Native V3D work is guarded by default-off switches:

```c
#define PIOS_ENABLE_NATIVE_VIDEOCORE 1
#define PIOS_ENABLE_NATIVE_V3D_COMPUTE 1
#define PIOS_ENABLE_TINY_QPU_KERNELS 1
```

Bring-up order:

1. Boot with `PIOS_ENABLE_NATIVE_VIDEOCORE=1` only and run `qpu status`. Expect `native=yes`, V3D tech version `tv=71`, and sane core/QPU/MMU fields.
2. Add `PIOS_ENABLE_NATIVE_V3D_COMPUTE=1`. Expect `nmmu=yes`, `nself=yes`, idle CSD status, and no MMU fault output.
3. Add `PIOS_ENABLE_TINY_QPU_KERNELS=1` and run one-element `tensor_add` / `tensor_relu` trials. These use Mesa-emitted QPU words from `include/v3d_tiny_qpu.h`, verify output against NEON, and quarantine on mismatch.
4. After one-workgroup verification succeeds, expand with Mesa-generated tensor kernels and keep each new primitive behind output verification and quarantine.

Live Pi5 coverage as of `v20260617.230x`:

- `tensor vector16`, `tensor vector128 direct`, and `tensor vector128` pass on native V3D CSD.
- `tensor matvec128` runs as a single fused GPU dispatch: x is broadcast across rows, one loop-free `vector_mulN` multiplies the whole m*K matrix, then rows are reduced on ARM. `tensor matmul64` (multi-column) composes verified V3D multiplies with ARM reduction.
- Direct multi-workgroup vector-N CSD requires `CFG3.MAX_SG_ID = workgroups_x - 1`; leaving it zero CSD-completed but only produced unique workgroup IDs for the first 32 lanes. CSD supergroup/batch packing (`CFG3` wgs-per-sg, batches-per-sg, `CFG4` num_batches) now mirrors Mesa's `v3d_csd_choose_workgroups_per_supergroup` for any workgroup size.
- Production vector-N tries direct multi-workgroup CSD first with full output verification, then falls back to repeated verified 16-lane dispatches if direct-grid dispatch or verification fails.
- A fused single-kernel matvec (`matvecN`, `V3D_KERNEL_MATVEC`) is generated and wired (host `--builtin matvecN`, device `tensor matvecn` diagnostic) but is NOT in the production path: its in-shader K-loop relies on V3D's in-loop uniform-pointer rewind (`r:unif` branches), and the direct MMIO CSD path does not yet drive that, so loop kernels complete CSD without storing. Loop-free kernels (vector add/mul/relu N) work; loop kernels are an open item.

Generated host-side shader tooling lives in `tools/v3d_shaders/`:

- `vector_add.comp`, `relu.comp` — GLSL reference sources.
- `mesa_v3d_wrap.c` — standalone Mesa V3D compiler wrapper with builtins for store/load, vector16, vector-N, scale/AXPY, matvec16, and matmul4.
- `compile_v3d_shaders.ps1` — generates SPIR-V and, when `build/mesa_v3d_wrap.exe` exists, QPU word dumps for all builtins.
- `qpu_dump_to_header.py` — converts audited wrapper output into checked-in QPU word and uniform-metadata headers.

Media acceleration now includes separate Mesa-generated packed-byte residual and
restore kernels. Each CSD workgroup processes one 64-byte luma tile, so a 16x16
block uses one residual dispatch and one restore dispatch. The guarded
`media selftest` verifies both kernels byte-for-byte before exposing them.
`media bench` profiles a zero-copy 4 KiB in-place residual/restore round trip
on Pi 5, the maximum single CSD dispatch for the current 64-byte/workgroup
kernel, and sets the runtime crossover from measured results. Larger spans are
chunked only if QPU wins at that boundary. The deterministic CPU path remains
the fallback below that threshold. Representative
4,194,304-MAC tensor profiles still select QPU, where dispatch amortisation
produces a real speed-up.

PicoVM QPU execution 1 remains a bounded correctness proof for
`(a + b) * c`: the single block is bit-exact but takes about 4.4 us on QPU
versus roughly 1-2 ns on CPU. Batch experiments using SSBO workgroup grids and
straight-line 16/32/64-operation kernels all CSD-completed without stores despite
verified shader, uniform and buffer addresses. Those failed kernels are not part
of the runtime; the dashboard reports the single block as verified with CPU
selected.
