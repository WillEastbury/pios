# Feature matrix — dialects x VMs x namespaces, current status

A single reference table for "does X work on Y". Two independent axes:

1. **Language features** (statements/keywords — control flow, declarations,
   exceptions, events) vary **by dialect** (the source-level grammar), because
   each dialect has its own parser. All dialects that share the AST
   (BASIC/Python-style/English/COBOL/Report/Functional) compile to
   byte-identical bytecode once parsed; C-style and v1 have their own
   independent AST/compiler islands (see `docs/DIALECT_PARITY.md`).
2. **Host namespaces** (`String.*`, `Map.*`, `Crypto.*`, …) vary **by VM/
   runtime**, not by dialect — every dialect (including v1) compiles down to
   the same bytecode host-hook codes (`HOST_HOOK_CODES` in
   `picoscript_lang.py`), executed by exactly **three** independent runtime
   implementations: the Python VM (`picoscript_vm.py`), the JS VM
   (`vm/picovm.js`), and the C VM (`vm/picovm.c`). Native-C transpile
   (`lower_to_c`) and native-JS transpile (`lower_to_js`) are **not** a fourth
   and fifth implementation — they call directly into the *same* C (`pv_host2`
   -> `pv_default_host`) and JS (`rt.host` -> the same `picovm.js` dispatch)
   runtime code that the interpretive VMs use, so their namespace coverage is
   structurally identical to "C VM" / "JS VM" by construction (verified:
   `picoscript_il.py`'s `lower_to_c`/`lower_to_js` emit a generic
   `pv_host2(ctx, code, a, b)` / `rt.host(code, a, b)` call for **every**
   `HOST_HOOK_CODES` entry — there is no per-namespace allowlist at the
   transpile layer, only at the runtime that receives the call).

## 1. Control-flow / statement keywords by dialect

| Feature | BASIC | Python | English | COBOL | Report | Functional | C-style | v1 |
|---|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| If / While / For | Y | Y | Y | Y | Y | Y | Y | Y |
| ForEach | Y | Y | Y | Y | Y | Y | Y | N |
| DoLoop | Y | Y | Y (`Repeat`) | N | N | N | Y (`do…while`) | N |
| Skip (continue) | Y | Y | Y | N | Y | Y | Y (`continue`) | N |
| Ternary | Y (`IIF`) | Y | Y | N | N | Y | Y (`?:`) | N |
| Dim / IncDec | Y | N \* | N \* | N \* | Y (IncDec only) | N \* | Y | N |
| Gosub | Y | Y | Y | Y | Y | N \*\* | N/A | Y |
| ServerMain | Y | N | Y | N | N | Y | Y | N |
| ConstDecl / EnumDecl | Y | Y | Y | Y | Y | Y | Y | N |
| Dispatch (`Ns.Method(...)` call) | Y | Y | Y | Y | Y | Y | Y | Y |
| **TryExcept / Raise / OnBlock** | **Y** | **Y** | **Y** | **Y** | **Y** | **Y** | **Y** | **N** |
| `EVENT POST` / `EVENT RAISE` sugar | Y | Y (host-call form) | Y | Y | Y | Y | Y (`on Ns.Method{}`) | N \*\*\* |

\* Not a real gap — `Let`/assignment already covers the same ground; verified
per-frontend that adding a redundant declaration form would add no capability
(see `docs/DIALECT_PARITY.md`).
\*\* Not a real gap — Functional's `f 1 2` / `f(1,2)` application already
covers subroutine calls.
\*\*\* v1 has a raw `THREAD.RAISE <irq>` opcode-level primitive (`OP_RAISE`)
but no `TryExcept`/handler-stack mechanism — it is a frozen, IL-less ISA by
design (`picoscript_lang.py`: "Each statement compiles 1:1 to a single 32-bit
instruction. No optimisation. No reordering."), explicitly excluded from the
language-equivalence pass. All v1 bytecode still executes on the **same**
`PicoVM` as every other dialect (`picoscript_build.py` compiles v1 source with
`picoscript_lang.Compiler`, then runs the resulting words on `picoscript_vm.PicoVM`
— same runtime, only the frontend/grammar differs).

## 2. Exception & eventing mechanism by execution path

| Path | TryExcept/Raise (handler stack) | OnBlock (event dispatch) |
|---|:-:|:-:|
| Python VM (bytecode) | Y — real, nested-safe (`docs/EXCEPTION_ENGINE.md`) | Y (`docs/EVENTING.md`) |
| JS VM (bytecode, `vm/picovm.js`) | Y — byte-identical to Python | Y — byte-identical to Python |
| C VM (bytecode, `vm/picovm.c`) | Y — fixed a prior gap (see below) | Y |
| Native C transpile (`lower_to_c`) | **Y — fixed this pass** (see below) | Y |
| Native JS transpile (`lower_to_js`) | **Y — fixed this pass** (see below) | Y |

**Update — native transpile now supported.** This table previously showed
native C/JS transpile as explicitly rejecting `TryExcept`/`Raise` at compile
time because `laddr` "has no PC-addressable/fault-catching equivalent in
straight-line native C/JS output". That diagnosis was correct, and the fix
follows exactly that insight: `Lowerer.lower_try` no longer flattens
`TryExcept` into `laddr`/`Error.SetHandler`/`label`/`jmp` IL at all — it now
builds a **structured** `trycatch` IL node (`ILBuilder.trycatch`) carrying
nested `try_body`/`except_body`/`finally_body` instruction lists.
`lower_to_bytecode_safe` still expands this into the classic flat form
(`_flatten_trycatch`) for the three bytecode VMs — byte-identical to before.
But `lower_to_c` now compiles it into **plain `goto`/labels** (the handler's
label is known at compile time from the structure itself, so there's no
runtime PC value to load at all — `setjmp`/`longjmp` turned out to be
unnecessary), with a `ctx->raise_active` return-code-propagation flag for a
`Raise` from inside a called subroutine. `lower_to_js` compiles it into a
**real JS `try/catch/finally`** and `throw`, which needs no propagation
bookkeeping at all since JS exceptions natively unwind across function
calls. See `docs/EXCEPTION_ENGINE.md`'s "native C/JS transpile now support
this too" section for the full design, `tests/test_native_toc_trycatch.py`
and `tests/test_native_js_trycatch.py` for verification (including a loop
inside a try body, and a `break` crossing a try boundary into an enclosing
loop — the trickiest case for JS specifically, since it lacks `goto`).

**A real, pre-existing bug found along the way — since fixed**: verifying
native C's cross-function-raise support against the Python VM as ground
truth exposed that all three *bytecode* VMs (Python/JS/C interpreter) had a
genuine bug where `Error.Raise` from inside a called subroutine didn't
unwind `vm.call_stack`, leaving a stale return address that got popped later
and silently re-executed code that should have been skipped. Native C didn't
have this bug (real C function returns unwind correctly); native JS didn't
either (real JS exceptions unwind the actual JS call stack). Fixed in all
three bytecode VMs by recording the call-stack depth at `Error.SetHandler`
time (a parallel array: `picoscript_vm.py`'s `_error_handler_call_depth`,
`vm/picovm.js`'s `_errState.callDepth`, `vm/picovm.c`'s
`ctx->err_call_depth[]`) and truncating the call stack back to that depth
whenever `Error.Raise`/a caught genuine fault redirects to a handler — see
`docs/EXCEPTION_ENGINE.md`'s "Cross-function raise: a call-stack-unwinding
bug, found and fixed" section for the full story, and
`tests/test_c_vm_error_parity.py`, `tests/test_exception_engine.py`,
`tests/test_native_toc_trycatch.py` for regression coverage across all 3
bytecode VMs plus native C/JS.

**Update — fixed this pass.** A prior revision of this file claimed the C VM
interpreter "shares the `laddr`/handler-stack bytecode contract" without
verifying it — that was wrong at the time (there was no `Error.*` dispatch in
`vm/picovm.c` at all). Investigating it properly turned up a key fact: `laddr`
needs **no new C opcode whatsoever** — it's a purely compile-time IL/bytecode-
assembly construct (`picoscript_il.py`'s `_emit_const(..., force_wide=True)`)
that lowers to plain `SUB`/`ADD`/`MUL` words (the same "wide constant load"
form used for any large integer literal), which the C interpreter already
executes correctly. The only real gaps were (1) the `Error.*` host-hook
dispatch and (2) a handler-stack + "redirect PC" mechanism — Python/JS can
mutate `vm.pc`/`this.pc` directly from a host hook, but C's `pv_vm_run`'s `pc`
is a local variable, so a new `ctx->pending_jump`/`pending_jump_set` channel
was added for hooks (and caught VM faults) to request a jump back into the
main loop. Both are now implemented (`vm/picovm.c`'s `pv_set_fault` + the new
`Error.*` dispatch block), verified byte-identical to Python for: try/catch/
finally/raise, nested try/catch, uncaught raise (propagates as a real fault,
same code), and — the architecturally riskiest case — a **genuine VM fault**
(bad computed jump) caught by an active handler, not just a script-level
`Raise`. See `tests/test_c_vm_error_parity.py`.

## 3. Host namespaces by runtime (Python VM / JS VM+native-JS / C VM+native-C)

### Database façade / PicoWAL status

The current milestone has a shared public namespace surface and synchronized `Db.*` hook ABI across the compiler, generated C/JavaScript tables, Python VM, JavaScript VM, and portable C host. PicoWAL is the execution substrate for the C/native path: exact index seeks, legacy query parsing, result iteration, bounded batch/materialization, and deterministic plan status reuse the existing overlay/index buffers. Python and JavaScript retain compatibility routing for CRUD; their full structured planner implementation is not yet complete.


71 namespaces are registered in `HOST_HOOK_CODES` (70 verified in an earlier
pass; `Decimal` added since — see its row below). Status below was verified
directly (grep for each runtime's actual dispatch branches — `if ns == "X"` in
`picoscript_vm.py`, `name.indexOf("X.")` in `vm/picovm.js`, hook-code-range
checks in `vm/picovm.c`), not inferred from documentation claims. **Updated
this pass**: every namespace that was previously a silent "unknown hook"
fallthrough now either has a real, deterministic implementation, or an
explicit, documented default (0 / empty span) on all three runtimes — see
"What changed this pass" below.

| Namespace | Methods | Python VM | JS VM (+native JS) | C VM (+native C) | Notes |
|---|:-:|:-:|:-:|:-:|---|
| Arena | 3 | Y | Y | Y | |
| Assert | 5 | Y | Y | Y | |
| Attention | 4 | Y | Y | Y | |
| Async | 3 | Provider | Provider | Provider | Host scheduler for coarse accelerator jobs; defined 0/NOT_FOUND fallback when unbound. |
| Auth | 10 | Stub | Stub | Stub | Host-injected by design — needs identity provider/trust store + entropy. Every method now returns a defined 0/empty-span default (previously silently fell through). |
| Base64 | 4 | Y | Y | Y | |
| Binary | 6 | Y | Y | Y | PSC1/BSO1 card <-> Map |
| **Block** | **13** | **Y (fixed)** | **Y (fixed)** | **Y (fixed)** | Raw block-device API: readiness, geometry, 64-bit offset/LBA split, bounded read/write, resize, sync, and status; compiler hook ABI and disassembly are covered by `tests/test_block_primitives.py`. |
| BitLinear | 8 | Y | Y | Y | |
| Bits | 7 | Y | Y | Y | |
| Capability | 3 | Y | Y | Y | |
| CatQ | 4 | Provider | Provider | Provider | Coarse calibration/optimization/ternarization/packing superinstructions; backend owns gradients and hardware dispatch. |
| Capsule | 5 | Y | Y | Y | |
| Card | 3 | Stub | Stub | Stub | Reserved/hardware-injected (physical card reader). Defined 0 default on all 3 runtimes. |
| Compress | 8 | Y | Y | Y | Pico/Brotli/Gzip/Deflate |
| Context | 15 | Stub | Stub | Stub | Host-injected by design — live request/connection state (overlaps conceptually with `Req.*`, which IS host-fed). Defined 0/empty-span default on all 3 runtimes. |
| Data | 3 | **Y (fixed)** | Y | **Y (fixed)** | Was a Python/C asymmetry (JS explicit stub, Python/C silent fallthrough) — now an explicit, matching 0/empty-span default on all 3 runtimes. |
| DateTime | 15 | Partial | Partial | Partial | `Now`/`UtcNow` host-injected (wall clock); rest pure and implemented |
| **Decimal** | 8 | **Y (real)** | **Y (real)** | **Y (real)** | New: Q16.16 fixed-point fractional numeric library (`Parse/ToString/Add/Sub/Mul/Div/Compare/ToInt`), same encoding as `Maths.Sin/Cos/Exp/Log`. Unlike `Number.Parse` (32-bit-integer only, truncates any fraction), preserves it — for callers needing exact currency/decimal arithmetic. `ToString` renders the shortest round-trip decimal (no binary-fraction noise on values like "19.99"). No host state — real and deterministic on all 3 runtimes + both native transpiles (`tests/test_native_toc.py`). |
| **Descriptor** | 6 | **Y (real)** | **Y (real)** | **Y (real)** | New this pass: a pure buffer descriptor (ptr/len/flags handle table), no host state — real and deterministic on all 3 runtimes. |
| Encoding | 12 | Y | Y | Y | ASCII/UTF-8/UTF-16/UTF-7/Hex |
| Env | 4 | Y | Y | Y | |
| Environment | 9 | Stub | Stub | Stub | Host-injected by design — OS/host facts. Defined 0/empty-span default on all 3 runtimes. |
| Error | 8 | Y | Y | **Y (fixed)** | Handler stack, `Raise`/`PopHandler`. **Now working on all 3 runtimes.** The C VM gap found this pass turned out to need no new opcode (`laddr` is a pure compile-time bytecode-assembly trick) — just the `Error.*` dispatch + a `pending_jump` PC-redirect channel; see section 2. |
| Event | 10 | Y | Y | Y | FIFO queue + `OnBlock` dispatch |
| **Fifo** | 4 | **Y (real)** | **Y (real)** | **Y (real)** | New this pass: independent named byte-channel FIFOs (distinct from `Queue.*`'s fixed 8-channel int FIFO). No host state — real and deterministic. |
| Gpio | 7 | Y | Y | Y | Reference emulator; PIOS injects real driver |
| Html | 10 | **Y (real)** | **Y (real)** | **Y (real)** | New this pass: a real, pure DOM tree (CreateNode/AddChildNode/RemoveChildNode/SetAttribute/GetAttribute/ParseTree/Serialize/QuerySelector), no host state needed -- see "HTML DOM: from stub to real" below. `Encode`/`Decode` unchanged (already real). |
| Http | 12 | Partial | Partial | Partial | `ParseQuery/ParseForm/ParseJson/EncodeJson` implemented (pure); `ReadHeader/ReadBody/GenerateHeaders/GenerateResponse/Request/RespStatus/RespHeaders/RespBody` host-injected (live connection), now return an explicit default on all 3 runtimes (fixed this pass — same silent-fallthrough bug as `Data.*`; note `Request`/`RespStatus`/`RespHeaders`/`RespBody` were previously undocumented gaps, found during this fix) |
| Io | 2 | Y | Y | Y | |
| Json | 11 | Y | Y | Y | |
| Kernel | 6 | Y | Y | **Y (fixed)** | `WaitIRQ`/`WaitSWIRQ`/`FireSWIRQ` reuse the same halt/ack semantics as the raw `OP_WAIT`/`OP_RAISE` opcodes. `ProfileStart`/`ProfileEnd`/`TracePoint` reuse the `Log.*` table — **now on all 3 runtimes** (C VM's `Log.*` gap closed this pass). |
| Kv | 12 | Y | Y | Y | |
| **Lease** | 6 | **Y (real)** | **Y (real)** | **Y (real)** | New this pass: a generic capability/ownership token over a span + type hint, no host state — real and deterministic. Distinct from `Stream.Next`'s own unrelated internal per-frame lease concept. |
| Locale | 7 | Y | Y | Y | Needs `tzdata` on Windows for non-empty `zoneinfo` — see note below |
| Log | 5 | Y | Y | **Y (fixed)** | Real `Log.*` subsystem — **now on all 3 runtimes**. C VM uses a fixed-size table (`PV_MAX_LOGS=128`), consistent with this embedded runtime's other handle tables (Map/Descriptor/Lease/Fifo) — a bounded vs. Python/JS's unbounded dict, not a behavioral difference at any realistic scale. |
| Map | 27 | Y | Y | Y | |
| Maths | 12 | Partial | Partial | Partial | `Sin/Cos/Tan/Log/Log10/Exp` (Q16.16 CORDIC), `Power/Sqrt/Clamp/Lerp` implemented; `Random`/`RandomRange` host-injected (entropy) |
| Media | 11 | Provider | Provider | **Y (real)** | Host/device media acceleration surface; native C provides deterministic grayscale delta, residual, and XOR fallbacks while HEVC remains provider-backed. |
| Memory | 9 | Y | Y | Y | |
| Model | 12 | Y | Y | Y | |
| MoE | 3 | Provider | Provider | Provider | Coarse Qwen-style top-K expert routing/streaming boundary; host owns packed expert storage. |
| Net | 10 | Provider | Provider | Provider | Raw server/client sockets plus span transport. Python ships `SocketNetworkProvider`; native C ships `vm/picovm_net.c`; JS accepts an injected provider. |
| Number | 11 | **Y (fixed)** | **Y (fixed)** | **Y (fixed)** | `Parse` now tolerates a trailing decimal fraction (e.g. "1000.0" -> 1000, truncating towards zero) instead of silently failing to 0/PARSE_ERROR — fixed on all 3 runtimes. See `Decimal.*` for exact (non-truncating) fractional arithmetic. |
| **Pack** | 1 | **Y (real)** | **Y (real)** | **Y (real)** | New this pass: a lightweight "active pack" selector, no host state — real and deterministic. |
| Principal | 3 | Y | Y | Y | |
| Process | 8 | Y | Y | Y | |
| Quant | 5 | Y | Y | Y | |
| Query | 29 | Partial | Partial | **Y (planner bridge)** | Public structured façade lowers to `Db.*`; PicoWAL C adapter routes `Seek/Query/Next/Batch/Materialize/Plan` into existing exact-index, query-result, and fallback scan paths. Builder IR and full cursor semantics remain in progress. |
| Queue | 5 | Y | Y | Y | `DequeueBatch`/`EnqueueBatch` are docs/CONFORMANCE_LEVELS.md's "L3: Optional" batch-container API ("no correctness impact if omitted") -- explicit 0 default on all 3 runtimes (was a silent fallthrough leaving `rd` untouched; fixed this pass) rather than a full v2 batch-container implementation, which is a separate, deliberately deferred design question. |
| Random | 1 | Y (seeded, non-deterministic by design) | Y | Y | |
| Req | 13 | Y | Y | Y | Host-fed request context; native C has a real HTTP server (`docs/NATIVE_HTTP_SERVER.md`) |
| Resp | 13 | Y | Y | Y | |
| Sampling | 4 | Y | Y | Y | |
| Sandbox | 1 | Y | Y | Y | |
| Scheduler | 1 | Y | Y | Y | |
| Search | 28 | Y | Y | Y | |
| Shard | 2 | Provider | Provider | Provider | Host-backed model-shard load/save using opaque handles. |
| Span | 5 | Y | Y | Y | |
| Status | 1 | Y | Y | Y | |
| Storage | 21 | Y | Y | Y | Card-store; host-injected persistence backend, in-VM logic is pure |
| Update | 8 | **ABI** | **ABI** | **ABI** | Public CRUD façade; compiler emits internal `Db.*` hooks. C/PicoWAL adapter currently bridges query-side operations; full CRUD façade parity is in progress. |
| Definition | 6 | **ABI** | **ABI** | **ABI** | Public pack/index definition façade; hook codes and compiler surface exist. Persistent overlay registration/rebuild behavior is host/PicoWAL work in progress. |
| Stream | 8 | Y | Y | Y | |
| String | 14 | Y | Y | Y | `Split`/`Join` are **new, real implementations this pass** (see below) — Map-backed multi-value result, byte-identical on all 3 runtimes. |
| Template | 2 | Y | Y | Y | |
| Tensor | 17 | Y + Provider | Y + Provider | Y + Provider | Existing deterministic INT8/I32 inference primitives plus host-backed map/view/GEMM/reduce/elementwise operations. |
| TextRender | 9 | Y | Y | Y | |
| **Thread** | 1 | **Y (real)** | **Y (real)** | **Y (real)** | New this pass: `YieldCounted` is a deterministic cooperative-yield counter, no host state — real on all 3 runtimes. Distinct from v1's `THREAD.*` opcode-level compiler sugar (unrelated). |
| Timer | 4 | Y | Y | Y | |
| Tokenizer | 7 | Y | Y | Y | |
| Ui | 12 | Y | Y | Y | |
| Utf8Reader | 8 | Y | Y | Y | |
| Utf8Writer | 7 | Y | Y | Y | |
| X509 | 8 | Stub | Stub | Stub | Host-injected by design — needs a trust store + entropy. Defined 0/empty-span default on all 3 runtimes. |

**"Stub" means**: every method in that namespace is fully callable, from
every dialect, on every VM, and returns a well-defined, documented default (0
for numeric/boolean-shaped results, an empty span for text-shaped results) —
**never** a crash and **never** an undefined/stale register value. This is a
deliberate, explicit design choice (not a gap to be "fixed" further) for
namespaces that need external state this deterministic VM cannot source
itself (identity, physical hardware, live network, PKI, OS facts) — the real
capability must be supplied by the host/PIOS kernel, exactly the same
principle already established for `Req.*`/`DateTime.Now`/`Maths.Random`.

## What changed this pass

Prompted by "start making them work and equal... even if returning null":

1. **`Data.*` Python/C asymmetry fixed** — `picoscript_vm.py` and `vm/picovm.c`
   now explicitly default `Data.Lookup`/`FieldNum`/`FieldStr` to 0/empty span,
   matching `vm/picovm.js`'s always-correct behavior exactly (previously they
   silently fell through, leaving a stale register value).
2. **Four new real, pure, deterministic primitives** (no host state needed,
   so implemented properly rather than stubbed) added identically to all
   three runtimes, byte-identical output verified end-to-end (Python VM ==
   JS VM == C VM):
   - `Descriptor.*` — a buffer descriptor (ptr/len/flags handle table).
   - `Lease.*` — a generic capability/ownership token over a span + type hint.
   - `Fifo.*` — independent named byte-channel FIFOs.
   - `Pack.Use` / `Thread.YieldCounted` — a pack selector and a deterministic
     yield counter.
   - `Kernel.WaitIRQ`/`WaitSWIRQ`/`FireSWIRQ` — real, reusing the same
     halt/ack semantics as the raw `OP_WAIT`/`OP_RAISE` opcodes.
3. **`String.Split`/`String.Join` implemented for real** (previously
   registered but unimplemented on every runtime) — parts are stored in a
   fresh `Map` (int key 0..N-1 → string part), reusing `Map.*`'s already-
   parity-tested storage rather than inventing a new container, and without
   disturbing the caller's active map. Byte-identical on all 3 runtimes.
4. **Nine genuinely host-injected namespaces converted from silent
   fallthrough to explicit, documented stubs**: `Auth`, `Card`, `Context`,
   `Environment`, `Net`, `X509` (plus `Data` above) now return a defined
   0/empty-span default on every runtime instead of silently logging an
   "unknown hook" and leaving the destination register untouched.
5. **A real gap found and honestly documented, then closed**: this pass
   discovered that `Error.*` (the exception-engine handler stack) and `Log.*`
   (the tracing/audit subsystem) — both previously documented elsewhere in
   this repo as working on "all runtimes" — were **not actually implemented
   in the C VM interpreter (`vm/picovm.c`) at all**. Python and JS VMs were
   unaffected; this was a real inaccuracy in prior documentation (this file's
   own earlier revision included). **Both are now fixed** — see "Follow-up:
   C VM `Error.*`/`Log.*` closed" below.
6. **`Kernel.ProfileStart`/`ProfileEnd`/`TracePoint`** are real on all 3
   runtimes now (reusing the `Log.*` table on each).

## Follow-up: C VM `Error.*`/`Log.*` closed

The gap documented in point 5 above (and previously in section 2) is now
fixed:

- **`Log.*`**: added a fixed-size handle table (`log_level`/`log_span`/
  `log_used`, `PV_MAX_LOGS=128`) to `pv_ctx`, consistent with this embedded
  runtime's other handle tables (`Map`/`Descriptor`/`Lease`/`Fifo`) — a
  bounded vs. Python/JS's unbounded dict, not a behavioral difference at any
  realistic scale. `Kernel.ProfileStart/ProfileEnd/TracePoint` now reuse it,
  same as Python/JS.
- **`Error.*`**: the investigation turned up a key fact that changed the
  scope entirely — **`laddr` needs no new C opcode at all**. It's a purely
  compile-time IL/bytecode-assembly construct (`picoscript_il.py`'s
  `_emit_const(..., force_wide=True)`) that lowers to plain `SUB`/`ADD`/`MUL`
  words (the same "wide constant load" form used for any large integer
  literal) — the C interpreter already executes this correctly. The only
  real gaps were (1) the `Error.*` host-hook dispatch itself, and (2) a way
  for a hook (or a caught VM fault) to redirect the interpreter's `pc` —
  Python/JS can mutate `vm.pc`/`this.pc` directly from a host hook, but
  `pv_vm_run`'s `pc` is a local C variable, so a new
  `ctx->pending_jump`/`pending_jump_set` channel was added, consumed once
  per instruction in the main loop. `pv_set_fault` (used by every genuine VM
  fault: bad jump, bad opcode, step budget, call overflow) now checks the
  handler stack first and redirects instead of halting, exactly mirroring
  Python's `except PicoFault` handling around `_step()`.
- Verified byte-identical to Python for: try/catch/finally/raise, nested
  try/catch, uncaught raise (propagates as a real fault with the same code),
  and — the architecturally riskiest case, since it touches the core
  interpreter loop used by every program — a **genuine VM fault** (bad
  computed jump) caught by an active handler, not just a script-level
  `Raise`. See `tests/test_c_vm_error_parity.py`.
- Full regression suite (including `--runslow`, which builds and exercises
  the C interpreter + native C/JS transpile via `ziglang`) shows zero new
  failures after this change — the two pre-existing, unrelated failure
  clusters (`test_aliases.py`, `test_engine_security.py`) were re-verified
  via git-stash bisection to fail identically without any of this session's
  changes.

## Follow-up: C-style's JS mirror (`CParser`/`CLowerer`) closed

Row 1's control-flow table above listed C-style's `try`/`catch`/`finally`/
`raise`/`on Ns.Method{}` as working only via the Python `cfront` compiler
(`picoscript_cfront.py`), not its JS mirror (`vm/picoc.js`'s `CParser`/
`CLowerer`) — a third, independent implementation of the exception/eventing
mechanism, deliberately deferred earlier this session to avoid rushing it.
**Now closed**: `C_KW` gained the five keywords (previously `try` etc. would
tokenize as plain identifiers and fail with a confusing parse error, not a
clear "unsupported" one), `CParser` gained `parseTry`/`parseOnBlock`, and
`CLowerer` gained `lowerTry`/`lowerOnBlock` — mirroring
`picoscript_cfront.py`'s grammar/lowering exactly, just re-expressed against
`CLowerer`'s own conventions (`this.loop`/`this.varOf`) rather than sharing
code with `BLowerer` (the two lowerer families remain deliberately
independent, matching the existing architecture). Verified byte-identical
bytecode to the Python `cfront` compiler and byte-identical runtime output
on the JS VM vs. the Python VM for try/catch/finally/raise, nested
try/catch, and `on Ns.Method{}` event dispatch. See
`tests/test_cstyle_js_exception_eventing.py`. C-style's control-flow row in
section 1 is now **Y** across the board except v1 (by design).

## Follow-up: `Html.*`/`Http.*` silent-fallthrough fix (same class as `Data.*`)

Reviewing the remaining "partial-by-design" namespaces turned up two more
instances of the exact same bug class fixed for `Data.*` earlier: `Html.*`'s
unbuilt DOM tree ops (`CreateNode`/`AddChildNode`/`RemoveChildNode`/
`SetAttribute`/`GetAttribute`/`ParseTree`/`Serialize`/`QuerySelector`) and
`Http.*`'s live-connection ops (`ReadHeader`/`ReadBody`/`GenerateHeaders`/
`GenerateResponse`/`Request`/`RespStatus`/`RespHeaders`/`RespBody`) were
silently falling through to the generic "unknown hook" path on all three
runtimes — never a crash, but `rd` was left untouched (stale) rather than a
defined value. Fixed identically to the `Data.*`/reserved-namespace pattern:
an explicit 0 (numeric/boolean-shaped) or empty span (text-shaped) default,
verified byte-identical across Python VM / JS VM / C VM. `Request`/
`RespStatus`/`RespHeaders`/`RespBody` were previously undocumented gaps
(this file's own earlier revision only mentioned `ReadHeader`/`ReadBody`/
`GenerateHeaders`/`GenerateResponse`) — found and closed together. See
`tests/test_namespace_equalization.py`'s
`test_html_dom_ops_now_do_real_work` (Html.* has since moved from stub to
real -- see the next section; this test now verifies the real values the
exact same call sequence produces) and
`test_http_live_connection_ops_return_defined_defaults` (Http.* remains
genuinely host-injected).

This does **not** build a live HTTP connection — an actual host-injected
socket for Http remains a genuinely separate, larger feature — it only
closes the "silent fallthrough leaves a stale register" bug for `Http.*`'s
live-connection ops, which is the specific thing the namespace-equalization
pass set out to fix everywhere. (`Html.*`'s DOM tree ops, listed above as
part of the same fix, have since been built for real -- see the next
section.)

## Follow-up: `Html.*` DOM tree ops — from stub to real

Unlike `Http.*`'s live-connection ops (genuinely host-injected — no VM can
source a real network socket itself), `Html.*`'s DOM tree ops
(`CreateNode`/`AddChildNode`/`RemoveChildNode`/`SetAttribute`/`GetAttribute`/
`ParseTree`/`Serialize`/`QuerySelector`) need **no host state at all** — a
mutable node table + a parser is entirely implementable in-VM, the same
category as `Descriptor`/`Lease`/`Fifo`/`Pack`/`Thread` earlier this pass.
Reviewing `docs/NAMESPACE_STATUS.md`'s "Scope/effort" section (which had
already flagged this as "doable... need a mutable tree model + parser", not
a hard blocker) turned this from a documented stub into a real
implementation:

- **Node model**: `{tag: span handle, attrs: {key: span handle}, children:
  [handle, ...]}`. A node is a *text* node iff its `attrs` has reserved key
  `"#text"` (its value span is the text content) — `CreateNode`+
  `SetAttribute` alone build one (no separate `CreateTextNode` needed), and
  `ParseTree`'s internal builder uses the exact same convention for text
  runs it parses. An empty tag with no `"#text"` is a transparent fragment/
  wrapper (used for `ParseTree`'s synthetic multi-root wrapper).
- **`SetAttribute`** packs `"key=value"` into a single span (the 2-in/1-out
  host-hook ABI has no 3rd argument register — see
  `docs/NAMESPACE_STATUS.md`'s "3-argument ops" section) rather than adding a
  new host hook or a stateful 2-call pattern.
- **`ParseTree`** is a minimal, permissive HTML parser (not full HTML5
  conformance): tokenizes `<tag k="v" k2='v2'>`, `</tag>` (closes the
  innermost open element regardless of name match), self-closing `<tag/>`,
  and a fixed void-element list (`br`/`img`/`hr`/`input`/`meta`/`link`/
  `area`/`base`/`col`/`embed`/`source`/`track`/`wbr`). Always returns a
  synthetic fragment-root handle so multi-root/bare-text input has a single
  handle to return.
- **`QuerySelector`** supports exactly 3 minimal forms (not a full CSS
  selector engine, documented as an intentional simplification): a bare tag
  name, `#id` (exact `id` attribute match), or `.class` (whitespace-token
  match against the `class` attribute) — first match in pre-order,
  root-included.
- **Bounded tree-walk depth** (`HTML_MAX_DEPTH`/`PV_HTML_MAX_DEPTH` = 32,
  matching the existing `TPL_MAXDEPTH` convention) on `Serialize`/
  `QuerySelector`/`ParseTree` protects against a script-constructed cycle
  (`AddChildNode` has no cycle check — the same simplicity/determinism-over-
  defensiveness tradeoff already accepted for every other handle-table
  namespace); stops descending rather than faulting, identically on all 3
  runtimes.
- **C VM**: fixed-size tables (`PV_MAX_HTML_NODES=64`, `PV_HTML_MAX_ATTRS=8`,
  `PV_HTML_MAX_CHILDREN=16` in `vm/picovm.h`), consistent with this embedded
  runtime's other handle tables (`Map`/`Descriptor`/`Lease`/`Fifo`/`Log`) — a
  bounded, deterministic difference from Python/JS's unbounded dict-backed
  version, not a behavioral divergence at any realistic scale.
- Verified byte-identical on **all five** execution paths: Python VM, JS VM,
  C VM interpreter, native C transpile, and native JS transpile — the last
  two just forward generically to the same runtime dispatch (see this file's
  introduction), so there was no separate implementation to write for them,
  only to verify. See `tests/test_html_dom.py`.

### `String.*` — the earlier `_stringlib` regression (separate from #3 above)

`String.*` was **completely broken in the Python VM** earlier this session
(`AttributeError`, every method) due to a real regression: commit `644acd1`
overwrote the `_stringlib` method body with the unrelated `_parse_hook` while
leaving the dispatcher's call site intact. Fixed (see commit `485d7d4`) by
restoring the original implementation verbatim, verified byte-identical to
`vm/picovm.js`'s always-intact version. Full regression suite at the time:
50 failures -> 2. `String.Split`/`Join` were **not** part of that regression
— they were never implemented on any runtime even before `644acd1` — and are
now implemented for real as described in #3 above.

## 4. Test-environment dependency: `tzdata`

The 2 remaining failures after the `String.*` fix were unrelated to any VM/
dialect bug: this Windows Python install has no `tzdata` package, so the
standard-library `zoneinfo.ZoneInfo` cannot resolve **any** timezone key —
including `"UTC"` — at all (Windows, unlike Linux, does not ship the IANA tz
database as OS files; `zoneinfo` depends on either the OS database or the
`tzdata` PyPI package as a fallback). `Locale.SetLocale` is implemented
correctly; it was failing purely because of this missing environment package.

**Fixed**: installed `tzdata` (`pip install tzdata`, user-level). Full suite is
now **2454 passed, 48 skipped, 0 failed**. If you set up a fresh dev/CI
environment on Windows for this repo, install `tzdata` alongside any other
Python test dependencies — it is a pure IANA-database data package for
Python's own standard library, not a third-party framework.
