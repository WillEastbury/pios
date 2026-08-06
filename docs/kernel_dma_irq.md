# Kernel DMA, IRQ & High-Performance IPC Transfer

This document describes PIOS's approach to moving bytes between cores and
privilege levels as fast as the BCM2712 / Cortex-A76 allows, and how that
interacts with the DMA engine and the interrupt model. It complements
[ipc.md](ipc.md) (the FIFO message protocol) and [mmu.md](mmu.md) (cache and
page-table setup).

The work here is **bench-driven**: every optimisation is measured live on the
Raspberry Pi 5 board via the `ipc bench` harness before it is adopted. Numbers
in this document are real measurements, not estimates.

---

## 1. Hardware context

| Property | Value | Consequence |
|----------|-------|-------------|
| SoC | BCM2712 | 4× Cortex-A76 @ ~2.4 GHz |
| Cores | 0 = `CORE_NET` (kernel/net/services), 1–3 = user/process cores | core 0 owns I/O + services; cores 1–3 run EL0 processes |
| Cache coherency | All 4 cores are in the **Inner Shareable** domain, hardware-coherent | cross-core data sharing needs **barriers only**, never cache maintenance |
| Cache line | 64 bytes | descriptor/ring structures are 64B aligned to avoid false sharing |
| SIMD | NEON / ASIMD 128-bit, 32 vector regs; DotProd, FP16, crypto, CRC32 | the copy/accel engine; **no SVE** on A76 |
| DMA | BCM2712 `dma32` engine, channels 0/2/4/5; 40-bit addressing | bulk async transfers; **not** I/O-coherent into cacheable RAM |
| Interrupts | GIC-400 (`GICD=0x107FFF9000`, `GICC=0x107FFFA000`); per-core CNTPNS PPI 30; RP1 peripherals via PCIe/MSI-X → GIC SPI | timer + device IRQ wake; RP1 routing is delicate |
| System counter | `CNTPCT_EL0`, fixed frequency (`CNTFRQ_EL0`, tens of MHz on this SoC) | all bench "ticks" are CNTPCT counts; relative comparison is what matters |

> **Coherency rule of thumb.** Between the 4 A76 cores: barriers only. At a
> non-coherent DMA boundary (the BCM2712 DMA masters): explicit cache
> maintenance (`dc cvac` / `dc ivac` / `dc civac`). Never confuse the two.

---

## 2. The transfer-strategy rule

The single most important design rule, applied everywhere data moves:

> **DMA for bulk async only. NEON copy for descriptors and small frames.
> Zero-copy pointer-pass wherever possible.**

Rationale:

- **Zero-copy** (pass a `{addr,len}` span descriptor, let the consumer pull the
  RAM directly) beats every copy. The 4 cores are coherent, so a pointer handed
  across a ring is immediately readable by the peer after a release barrier. Use
  this for anything we don't have to physically move.
- **NEON copy** (`ldp/stp q`, 128-bit, 64 B/iter — see `simd_memcpy`) is the
  right tool for small/medium transfers and for the 32-byte ring descriptors.
  It has zero fixed setup cost.
- **DMA** only earns its keep for **large, asynchronous, fire-and-forget**
  transfers, or when we explicitly want to offload the CPU so it can do other
  work while bytes move. Its fixed cost (MMIO programming of a control block to
  device-nGnRnE space, `dma_wait`, and — into cacheable RAM — cache clean +
  invalidate over the whole buffer) dwarfs a NEON copy below several KB.

### Enforced at the API boundary

`dma_memcpy()` / `dma_zero()` (`src/dma.c`) short-circuit straight to
`simd_memcpy` / `simd_zero` below a size threshold, so a caller can pass any
size and always get the fast path automatically:

```c
#define DMA_MIN_EFFECTIVE_BYTES 4096U   /* tunable; see §6 */

bool dma_memcpy(u32 channel, void *dst, const void *src, u32 len) {
    if (len < DMA_MIN_EFFECTIVE_BYTES) {  /* DMA can't amortise setup here */
        simd_memcpy(dst, src, len);
        return len != 0;
    }
    if (dma_hw_memcpy_enabled && dma_memcpy_hw(channel, dst, src, len))
        return true;
    simd_memcpy(dst, src, len);           /* fallback */
    return len != 0;
}
```

The 4096 B threshold is a conservative initial value; the true cross-over point
is being measured empirically (see §6, open item).

---

## 3. The cross-core span ring (zero-copy descriptor transport)

Inter-core and process↔kernel traffic is dominated by handing **span
descriptors** (pointer + length + tag) across cores, not by copying payloads.
The compact ring carries 32-byte descriptors; the actual 2 KB / 64 B payloads
are referenced by pointer (zero-copy) and only physically copied or DMA'd when a
private copy is genuinely required.

```c
struct fifo_span_msg {     // 32 bytes, 32B aligned  (one ldp/stp q pair)
    u64 addr;              // payload pointer (sender/shared RAM)
    u64 tag;               // correlation / ownership id
    u32 len;               // payload length
    u32 flags;             // PROC_IPC_SPAN_F_READONLY, etc.
    u32 aux;               // completion status / routing
    u32 _pad;
};

struct fifo_span {         // SPSC ring, one per ordered (src,dst) core pair
    volatile u32 head ALIGNED(64);   // producer writes
    volatile u32 tail ALIGNED(64);   // consumer writes
    u8 _pad[128 - 2*64];             // no false sharing
    struct fifo_span_msg msgs[256] ALIGNED(64);
};
```

- One ring per ordered `(src,dst)` pair → 12 active cross-core rings + 4 unused
  self-pairs (the self-pair `(CORE_NET,CORE_NET)` is reused as a scratch ring by
  the benchmark). `get_span_fifo(src,dst)` places these after the 16 generic
  `struct fifo` in `SHARED_FIFO_BASE`.
- **Batch** push/pop (`fifo_span_push_batch` / `fifo_span_pop_batch`) amortise
  the publish barrier + SGI/SEV doorbell across up to 16 descriptors. Receiver
  **batch-drain** is the single biggest cross-core win (see §5).

### Publish / consume ordering (SPSC)

```
Producer:  write msgs[head..]      Consumer:  load head        (acquire)
           DMB ISHST   (data→head)            DMB ISHLD  (head→data reads)
           store head                         read msgs[tail..]
           DSB ISHST   (head→SEV)             DMB ISH    (reads→tail)
           SEV                                store tail
```

Two correctness rules learned the hard way:

1. **`SEV` needs a `DSB` (store) before it**, not just a `DMB` — otherwise a
   lost-wakeup race exists where the consumer is parked in `WFE` and never sees
   the event because the head store hasn't drained.
2. The producer must read `tail` and the consumer must read `head` with the
   matching acquire ordering so neither runs ahead of the other's published
   index.

---

## 4. Memory-ordering scope: SY vs ISH vs acquire/release

PIOS's generic `dmb()` / `dsb()` helpers are **full-system** (`dmb sy` /
`dsb sy`). That is the wrong (over-strong) scope for inter-core RAM coherency:
`SY` needlessly orders against the **Outer Shareable** domain *and* device/MMIO
memory. The 4 A76 cores are **Inner Shareable**, so the correct, minimal scope is
`ISH`:

```c
// include/types.h
static inline void dmb_ish(void)   { __asm__ volatile("dmb ish"   ::: "memory"); }
static inline void dmb_ishst(void) { __asm__ volatile("dmb ishst" ::: "memory"); }
static inline void dmb_ishld(void) { __asm__ volatile("dmb ishld" ::: "memory"); }
static inline void dsb_ish(void)   { __asm__ volatile("dsb ish"   ::: "memory"); }
static inline void dsb_ishst(void) { __asm__ volatile("dsb ishst" ::: "memory"); }
```

> **Keep `SY` for device/DMA paths.** Cache-maintenance ordering against the DMA
> engine's view of memory, and MMIO ordering, genuinely need full-system scope.
> Only the pure inter-core RAM FIFO paths were narrowed to `ISH`.

### A/B benchmark (live, BCM2712, 8000 iters, 0 errors)

A four-way same-core round-trip on the scratch ring isolates raw
copy + barrier cost from scheduler / wakeup noise. Each iteration pushes then
pops one 16-descriptor batch; the figure is **CNTPCT ticks per descriptor
round-trip** (one push-copy + one pop-copy + amortised barriers):

| Variant | Ordering | Copy | ticks/desc |
|---------|----------|------|-----------:|
| `rt_base`   | `DMB SY`     | compiler struct copy | **48** |
| `rt_ish`    | `DMB ISH`    | compiler struct copy | **48** |
| `rt_acqrel` | `LDAR`/`STLR` | compiler struct copy | 54 |
| `rt_asm`    | `LDAR`/`STLR` | hand `ldp/stp q`     | 54 |

Cross-core fanout (core 0 → 3 user cores, echo back) after switching the
production ring to `ISH`:

| Metric | `DMB SY` | `DMB ISH` |
|--------|---------:|----------:|
| `span_ring_per` (single target) | 112 | **109** |
| `span_all_per` (3-core fanout)  | 70  | **68** |

### Findings

1. **The 32-byte descriptor copy is not the bottleneck.** Hand-rolled
   `ldp q0,q1 / stp q0,q1` ties the compiler's struct assignment (54 vs 54) — a
   32 B, 32-aligned copy already lowers to the optimal `ldp/stp` pair. *Do not
   hand-write the descriptor copy.*
2. **One amortised `DMB` per batch beats per-slot `LDAR`/`STLR`** (48 vs 54) for
   batched SPSC. Acquire/release pays its ordering cost on *every* head/tail
   touch (2× `LDAR` + 2× `STLR` per round-trip); a single batched `DMB` is
   cheaper.
3. **`DMB ISH` ties `DMB SY` same-core (48 = 48)** — with nothing outstanding,
   barrier *scope* is free locally. But `ISH` is the correct minimal scope and
   is **never worse** cross-core; the fanout numbers improved slightly
   (70 → 68, 112 → 109).

### Decision

- **Adopt `DMB ISH`** in the production span ring (`fifo_span_push_batch` /
  `fifo_span_pop_batch`). Correct scope, zero same-core cost, small cross-core win.
- **Reject** the hand-asm copy (no win) and per-slot acquire/release (slower).
- Keep the `_ish` / `_acqrel` / `_asm` variants as **benchmark instrumentation**
  so the comparison stays reproducible and regressions are visible.

### The bigger lesson

A full push+pop descriptor round-trip is only ~48 ticks. **Copy and barrier
cost is tiny.** The real cross-core cost — one-at-a-time handoff `cross_fifo_per`
≈ 1100, fanout `span_all_per` ≈ 68 — is dominated by **wakeup / handoff /
scheduler latency**, not by the copy or the barrier. Optimisation effort belongs
in *batching to amortise the wakeup* (already the big win via receiver
batch-drain), not in micro-tuning the copy.

---

## 5. Batching & micro-Nagle

Because the wakeup dominates, throughput comes from amortising it:

- `fifo_*_batch` move up to 16 descriptors per publication doorbell.
- **Receiver batch-drain**: the consumer drains everything available in one wake
  (`fifo_pop_batch` / `fifo_span_pop_batch`) instead of one-per-wake. This is
  applied to the real network/socket service paths (`src/net.c`, `src/socket.c`).
- **Micro-Nagle**: under load, let a batch fill; under low load, flush a partial
  batch on a short timer ceiling — a much shorter window than TCP Nagle.

Representative per-descriptor figures (CNTPCT ticks, lower = better):

| Path | ticks/desc |
|------|-----------:|
| One-at-a-time cross-core FIFO (`cross_fifo_per`) | ~1100 |
| Receiver batch-drain, micro-full 16 (`micro_full_per`) | ~156 |
| Micro-Nagle partial 4 (`micro_part_per`) | ~177 |
| Compact span ring batch (`span_ring_per`) | ~109 |
| All-core span fanout (`span_all_per`) | ~68 |
| 2048 B zero-copy span (`span2048_per`) | ~284 |
| 2048 B NEON memcpy (`memcpy2048_per`) | ~259 |
| Bare `SEV` (`sev_per`) | ~16 |

---

## 6. DMA engine

`src/dma.c` drives the BCM2712 `dma32` engine (channels 0/2/4/5, control-block
chains in physical memory, 40-bit addressing). Public API: `dma_memcpy`,
`dma_zero`, `dma_start`/`dma_start_chain`, `dma_busy`/`dma_wait`/`dma_abort`.

### Coherency

The BCM2712 DMA masters are **not I/O-coherent** with the A76 caches. Therefore
`dma_memcpy_hw` must, around every transfer into cacheable RAM:

- `dcache_clean_range(src)` — flush source so the engine sees current data;
- `dcache_clean_invalidate_range(dst)` — evict dest so dirty lines can't later
  clobber DMA output;
- `dcache_invalidate_range(dst)` after completion — so the CPU reads DMA data,
  not stale cache.

This cache maintenance is `O(len/64)` `dc` ops and is a large part of why small
DMA loses to NEON.

### Size fast-path (§2)

`dma_memcpy` / `dma_zero` skip the engine entirely below
`DMA_MIN_EFFECTIVE_BYTES`. The descriptor's `len` makes this a trivial,
branch-predictable check.

### When DMA wins

- Transfer size ≳ several KB (cache-maintenance + setup amortised);
- The CPU has other useful work to do while the transfer runs (true async —
  `dma_start` + poll `dma_busy`/IRQ completion, **not** `dma_wait`);
- Into a **Normal-NC carve-out** (the `0x049…` / `0x04B…` DMA regions in
  `include/core_env.h`), 64 B-aligned, whole-line buffers that never straddle a
  cache line — which removes the cache-maintenance cost from the hot path.

> **Open item.** Add a DMA-vs-NEON cross-over micro-benchmark to `ipc bench` and
> set `DMA_MIN_EFFECTIVE_BYTES` to the measured break-even size. Until then 4096
> is a safe conservative default (NEON wins below it for certain).

---

## 7. AArch64 acceleration: there is no `SMLAD`

A common question coming from Cortex-M (Armv8-M + DSP, e.g. M33): *where are the
packed-16 DSP MAC instructions (`SMLAD`, `SMUAD`, `SMLALD`, `SADD16`)?*

- Those are **AArch32-only** (packed 2×16-bit in 32-bit GP registers). The A76
  could run them in 32-bit state, but **PIOS is pure AArch64 — they do not
  exist** in A64.
- The A64 replacement is **NEON/ASIMD**, and on the A76 it is strictly more
  capable:
  - **`SDOT` / `UDOT`** (DotProd extension, present on A76) — int8→int32 4-way
    dot product: **16 MACs per instruction**. The modern "`SMLAD` on steroids,"
    ideal for the int8 / ternary (BitNet) tensor path.
  - **`SMLAL` / `SMLAL2`, `MLA`, `SQRDMULH`** — vector multiply-accumulate and
    saturating fixed-point DSP.
  - **`FMLA` / `FMLS`** — fused multiply-add (fp16/fp32/fp64). `src/tensor.c`
    currently uses separate `fmul`+`fadd`; fusing to `fmla` is a free win.
  - **Native fp16** (Armv8.2-FP16) — 2× throughput vs fp32 for ML.
  - **No SVE** on A76 (that is Neoverse V1/N2, A510+). NEON 128-bit only.

For the **IPC path specifically none of this MAC hardware is relevant** — moving
descriptors is pure memory movement. The levers there are NEON `ldp/stp q`,
non-temporal stores (`stnp`), `prfm` prefetch, and DMA for bulk. `SDOT`/`FMLA`
belong to the tensor path (`src/tensor.c`, `src/simd.c`), not the ring.

---

## 8. Interrupt model & the IRQ-vs-poll trade-off

### RP1 Ethernet RX

The NIC is a Cadence MACB/GEM behind the RP1 south bridge (PCIe). Reliable RX
interrupt delivery to core 0 required:

- **`macb_irq_ack_rx()` must write-1-to-clear `ISR.RCOMP`** — merely *reading*
  the ISR does not re-arm repeated delivery.
- RP1 → GIC routing via the MIP/MSI-X path: HOST6 / GIC SPI, with
  `CFGL_HOST = 0` to route to the HOST (GIC) rather than VPU. Do **not**
  auto-rearm by toggling `CFGL_HOST` + IACK — it can storm.

When it works, `core0_eth_irq_handler` sets `CORE0_IO_NET | CORE0_IO_TCP` and
`SEV`s, so an inbound packet wakes core 0 immediately, and
`core0_eth_irq_drain_and_quench()` drains RX (up to 8 `net_poll` passes) and
quiesces the source.

### Current state (measured)

`rp1 irq` reports **`count=0`** — the ETH RX interrupt is **not presently
delivering** to core 0 (`last_mip=0`, `quench_passes=0`). RX is therefore being
serviced by the **31 Hz timer poll**, not the IRQ.

> **Constraint this imposes.** We cannot drop the periodic NET/TCP poll for the
> core-0 idle win (§9) until the ETH IRQ delivers reliably — otherwise inbound
> request latency would balloon to the poll period. Re-arming/validating RP1 ETH
> IRQ delivery is the prerequisite for the biggest idle reduction.

### Timer

Core 0 runs a 1000 Hz generic-timer tick (`timer_init(1000)`); user cores run at
`PROC_PREEMPT_TIMER_HZ` for preemption. The per-core timer is independent, but
the software watchdog currently measures liveness in `timer_ticks()` (tick
count), which couples watchdog timing to the tick rate — so naïvely lowering
core 0's timer rate to cut wakes would break the 5 s watchdog window. Decoupling
the watchdog onto `timer_monotonic_ms` (CNTVCT-derived, rate-independent) is the
clean enabler for a tickless/low-rate core 0.

---

## 9. Core-0 service loop and idle reduction

`core0_main()` (`src/kernel.c`) is a wake-driven loop:

```c
for (;;) {
    u32 flags = core0_io_take_flags();   // IRQ-masked read+clear of pending work
    if (flags == 0) { wfi(); continue; } // sleep until next IRQ
    if (flags & CORE0_IO_NET)  { net_poll(); dns_poll(); ... }
    if (flags & CORE0_IO_TCP)  { echo_tcp_poll(); ksvc_run(debug); }
    if (flags & (UART|USB))    { uart drain ×16; ui_handle_keys(); }
    if (flags & CORE0_IO_MAINT){ arp_tick(); tcp_tick(); }
    if (flags & CORE0_IO_DASH) { hdmi_dashboard_render(); }
    watchdog_hw_pet();
}
```

`core0_io_tick_hook` (the 1000 Hz tick) sets the work flags on a cadence:

| Flag | Cadence | Work |
|------|---------|------|
| `NET`/`TCP`/`UART`/`USB` | every 32 ticks (~31 Hz) | `net_poll`,`dns_poll`,`echo_tcp_poll`, console |
| `MAINT` | every 100 ticks (10 Hz) | `arp_tick`, `tcp_tick` (retransmit/timers) |
| `DASH` | every 2000 ticks (0.5 Hz) | `hdmi_dashboard_render` (≈50 `fb_printf` to 1080p) |

### Idle telemetry (measured, board idle)

- **wake-rate ≈ 40/s** — matches the 31 Hz + 10 Hz cadence (the work wakes).
- **wfi-rate ≈ 978/s** — core 0 is woken ~1000×/s by the **1000 Hz timer IRQ**,
  finds no work flags ~938×/s, and immediately re-sleeps. (`SEV` from other
  cores does *not* wake `WFI`; only interrupts do — so this floor is the timer.)
- **Instantaneous idle busy ≈ 17‰ (1.7%)**, derived from the cumulative
  `sched_busy_permille` delta over a 40 s idle window.

### Cost breakdown (approx)

| Source | Share of idle |
|--------|--------------:|
| 31 Hz NET/TCP poll (`net_poll`+`dns_poll`+`echo_tcp_poll`) | ~10‰ |
| Empty timer-IRQ wakes (938/s × IRQ entry/exit) | ~1.5‰ |
| 0.5 Hz dashboard render | ~2‰ |
| 10 Hz MAINT | ~1‰ |
| 31 Hz UART/USB poll (cheap) | ~1.5‰ |

### Reduction plan (target < 10‰ / 1%)

1. **Make ETH RX IRQ deliver reliably** (§8) — prerequisite.
2. **Unbundle and lower the NET/TCP poll** from 31 Hz to ~8 Hz (a safety net
   behind the IRQ), keeping UART/USB responsive at ~31 Hz. Projected NET/TCP
   cost ~10‰ → ~2‰.
3. **Optimise / rate-limit the dashboard** render (skip when unchanged; it
   already self-gates to 1 Hz internally).
4. **(Stretch) lower the core-0 timer base rate** once the watchdog is
   decoupled from tick count — cuts the empty-wake floor.

Projected result of steps 2–3 alone: ~17‰ → ~7–8‰, under the 1% target, without
touching the timer/watchdog.

---

## 10. Reliability: watchdog & PiSOD

- **Hardware watchdog** — BCM2712 PM block (`PM_WDOG`/`PM_RSTC`), armed to a
  15 s single-shot and petted every core-0 loop (`watchdog_hw_pet`). If core 0
  wedges, the SoC resets itself unconditionally.
- **Software watchdog** — `watchdog_poll()` trips if any core's `last_touch`
  exceeds the 5 s window (`watchdog_init(5000, false)`).
- **A/B boot** — OTA writes a raw slot; a failed boot rolls back to the
  last-good slot. `/api/status` (the health gate) marks the running slot good.
- **PiSOD** — `exception_pisod()` paints a deep-indigo panic screen with the
  fault registers (`esr`/`elr`/`far`), persists the crash record, and halts the
  core; the hardware watchdog then auto-recovers the board. A controllable PiSOD
  for demos/diagnostics is a `peek` of an unmapped address (e.g. `0x140000000`,
  L1 index 5 — outside the RAM/peripheral/RP1 mappings), which faults core 0 and
  self-heals via the watchdog.

This is the safety net that makes aggressive iteration on the IRQ/IPC hot paths
practical: a bad build or a wedge reboots back to a known-good slot.

---

## 11. Benchmark harness

`ipc bench [n]` (`proc_ipc_bench`, `src/proc.c`; HTTP handler in `src/kernel.c`)
runs the full matrix and emits `*_ticks` (totals) and `*_per` (per-descriptor)
metrics. Key fields:

| Field | Measures |
|-------|----------|
| `svc_per` | synthetic in-C SVC decode (lower bound, not full EL0 cost) |
| `span_per` / `span_fast_per` | generic span FIFO round-trip / split fast path |
| `copy64_per` / `copy512_per` | copied FIFO at 64 B / 512 B |
| `memcpy2048_per` / `span2048_per` | 2 KB NEON memcpy vs zero-copy span |
| `cross_fifo_per` | one-at-a-time cross-core handoff |
| `micro_full_per` / `micro_part_per` | receiver batch-drain / micro-Nagle |
| `span_ring_per` / `span_all_per` | compact span ring / all-core fanout |
| `rt_base/ish/acqrel/asm_per` | the §4 copy/barrier A/B |
| `sev_per` | bare `SEV` cost |

Host-side load generation: `tools/pios_http_bench.py` (multithreaded HTTP,
exercises real kernel ops incl. `ipc bench`) is used to confirm the box stays
healthy and the IPC path behaves under concurrent real-world load.

---

## 12. Open items / roadmap

- [ ] **Re-arm + validate RP1 ETH RX IRQ delivery** (currently `count=0`) — the
      gate for the core-0 idle reduction and for fully IRQ-driven networking.
- [ ] **DMA-vs-NEON cross-over micro-bench**; set `DMA_MIN_EFFECTIVE_BYTES` to
      the measured break-even size.
- [ ] **Submission + completion rings** with descriptor IDs for real zero-copy
      ownership transfer (avoid use-after-free); EL0-supplied `{addr,len}` must
      be **validated and pinned** against the owning process address space before
      the kernel dereferences (confused-deputy / TOCTOU).
- [ ] **Async DMA-to-client-RAM** via the Normal-NC carve-out (no cacheable EL0
      DMA in v1).
- [ ] **Decouple the watchdog from tick count** (use `timer_monotonic_ms`) to
      enable a low-rate / tickless core 0.
- [ ] **Tensor path**: fuse `fmul`+`fadd` → `fmla`; adopt `SDOT`/`UDOT` for int8
      / ternary kernels.
- [ ] Per-process ingress queues + per-communication-pair response queues for
      completions (the async kernel-API message model).

---

*All figures in this document were measured live on the PIOS Pi 5 board via
`ipc bench` and `tools/pios_http_bench.py`. Re-measure after any change to the
ring, barrier, DMA, or IRQ paths before trusting them.*
