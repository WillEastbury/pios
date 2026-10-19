# PicoScript examples

These programs are PicoScript C-dialect sources. They are not linked into the
kernel image. Load them as EL0 capsule bytecode (picoweb on tcp/82 or a
`capsvc` port) or run them on the PicoScript reference VM.

| File | Role |
| --- | --- |
| `docwal.pc` | Document WAL HTTP host for BareMetal.DocumentWAL. Pack 1 is document meta, pack 2 is the op log, pack 3 is compacted snapshots. Record 0 of each pack is the next-id watermark. Routes match the picowal app host: `GET /list/{pack}`, `POST /{pack}`, `GET\|PUT\|DELETE /{pack}/{record}`. Bind as process `web`, `io = tcp/82`, `entry = http`, capsule pack 1024. Undo and redo are ops in pack 2 (`kind` `undo` or `set`). Compact is a snapshot card on pack 3. |
| `qpu_async_media.pc` | Async QPU job submit/wait with overlapping CPU work. |
| `bitnet_token_loop.pc` / `bitnet_w2a8_layer.pc` | BitNet token and W2A8 layer fixtures. |
| `fused_ops.pc` | Fused PicoScript tensor ops. |
| `model_fifo.pc` | OS-owned model FIFO client sketch. |
| `tensor_pinned.pc` | Pinned tensor / lease sketch. |
| `tiny_decoder.pc` | Tiny decoder fixture. |
| `exec_sched.pc` / `exec_sched_fifo.pc` | Execution-schedule sketches. |
| `multigpu_schedule_draft.pc` | Multi-GPU schedule draft. |

`docwal.pc` is the same generic PicoWAL card router used by the story prototype.
The image is larger than the 4096-byte EL0 bytecode cap, so the current run path
is the PicoScript reference VM. The PIOS bind above is the host contract.
