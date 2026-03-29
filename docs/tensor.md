# Tensor / ML Compute

## Overview

PIOS provides NEON-accelerated tensor operations with a V3D/QPU dispatch framework.

By default, operations run on the ARM Cortex-A76 using 128-bit NEON SIMD (4 floats per instruction, dual-issue capable). Tensor ops only attempt V3D dispatch when a kernel descriptor is bound and marked ready; otherwise they stay on deterministic NEON fallback paths.

V3D attempts are also gated by operation-size thresholds (to avoid offload overhead on tiny work units), and a kernel is quarantined after a failed dispatch so repeated calls do not get stuck in timeout-heavy retry loops.

When backend selection is `AUTO`, MMIO CSD dispatch is attempted first; if it times out/fails while mailbox QPU is available, MMIO is quarantined for subsequent AUTO dispatches and mailbox becomes the stable fast fallback path.

MMIO CSD submission now performs an idle handshake before queue programming and uses memory barriers around queue-kick/completion visibility.

## Available Operations

| Operation | Function | NEON Instructions Used |
|-----------|----------|----------------------|
| Element-wise add | `tensor_add(c, a, b)` | `ld1`, `fadd`, `st1` |
| Element-wise multiply | `tensor_mul(c, a, b)` | `ld1`, `fmul`, `st1` |
| Scalar multiply | `tensor_scale(b, a, s)` | `dup`, `fmul`, `st1` |
| Dot product | `tensor_dot(&result, a, b)` | `fmla` (fused multiply-accumulate), `faddp` (pairwise reduce); forced NEON path pending V3D scalar writeback plumbing |
| Matrix multiply | `tensor_matmul(c, a, b)` | Scalar inner loop (NEON for row operations) |
| ReLU | `tensor_relu(b, a)` | `fmax` with zero vector |
| Softmax | `tensor_softmax(b, a)` | Schraudolph exp approximation, scalar normalize |

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

VideoCore VII has 12 QPUs (3 slices × 4), each a 4-wide float SIMD unit at 800MHz. Peak theoretical: 76.8 GFLOPS. The VC VII ISA is still incomplete publicly, so the production path remains NEON unless valid kernel blobs are explicitly bound in the V3D layer.
