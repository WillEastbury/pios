# PIOS device & streaming bindings — architecture spec

**Status:** design (language + compiler surface defined here; OS implementation deferred to
the PIOS build agent). No VM/driver code is changed by this document.

**Layering principle (the contract this doc encodes):**
- **PicoScript owns** the *language features* (GPIO via the existing `Storage.*` card
  model; `Device`/`Stream` namespaces for future high-bandwidth streaming), the *compiler
  components* (hook lowering, capability gating, binding lifecycle, source-span/INV-25),
  and the *declared hook surface* OS code plugs into.
- **PIOS (EL1) owns** the *security surface* (capability enforcement, per-pin allow-list,
  lease validation/revoke, DMA-buffer ownership) and the *actual drivers*.

**No direct hardware-level access.** Raw register/MMIO access is deliberately *not*
exposed (see §2) — GPIO and other low-bandwidth control is a hardware-backed **card in a
pack**, so it inherits the already-governed card security model.

This is the same split as `docs/PIOS_HOST_BINDINGS.md` and `docs/INV25_PIOS_TRACE.md`:
PicoScript stays pure, deterministic and 5-path byte-identical; the device is an injected,
capability-gated binding whose real behaviour lives in the kernel.

---

## 1. The unifying model: a device is a binding over `pooldesc` + lease + FIFO

A streaming hardware device is **structurally identical to the HTTP `Req`/`Resp` binding**
(`docs/PIOS_IO_BINDING.md`): the same `pooldesc` descriptors, the same validated leases,
the same async FIFO ABI, the same I1–I8 invariants — only the *producer/consumer* is
hardware (a DMA ring, a register block) instead of a TCP socket. The building blocks
already exist in the language:

| Need | Existing PicoScript surface |
|------|-----------------------------|
| Zero-copy DMA buffer handle | `Descriptor.*` (0x50–0x55) = `pooldesc` |
| Validated / revocable / lifetime-bound access | `Lease.*` (0x58–0x5D) — I4/I8 |
| "Buffer ready" / "transfer complete" event | `Kernel.WaitIRQ` (+ the RP1-ETH IACK-gated drain pattern) |
| Async producer/consumer, backpressure | `Queue.*`, `Thread.Wait/Raise`, FIFO `LEASE_REVOKE` |
| Streaming lifecycle | binding kinds `stream` / `duplex` (`PIOS_IO_BINDING.md` §5) |
| Register field extract/insert | `Bits.*` (And/Or/Xor/Shl/Shr/Sar) |

So the new surface is small: **GPIO needs nothing new** (a pin is a `Storage.*` card, §2b),
and the future streaming class adds an **enumeration/open** layer (`Device.*`) plus a
**streaming-ring** layer (`Stream.*`, thin sugar over `Lease`+`Descriptor`+`WaitIRQ`).
There is **no register-level layer** — raw MMIO is rejected (§2).

---

## 2. Device classes — streaming (later) and GPIO-via-cards (first)

> **Security decision (rejected: raw MMIO).** An earlier draft proposed an `Mmio.*`
> register-window class. It is **rejected**: raw register access — even windowed — is a
> privilege-escalation / brick-the-SoC footgun and the wrong default for an
> isolation-first runtime. **PicoScript exposes no direct hardware-level access.** All
> low-bandwidth control goes through the card model below; the OS keeps register access
> entirely on its side of the boundary.

### 2a. Streaming / DMA-ring devices — `Device.*` + `Stream.*` (future)
High-bandwidth producer/consumer ring of DMA buffers: **camera frames, HDMI scanout,
PCIe/NVMe blocks, SDIO/QSPI blocks, Ethernet RX/TX.** Maps almost verbatim onto
`Descriptor` + `Lease` + a FIFO of ready-events. Zero-copy. **Not started yet** — designed
here for when high-bandwidth streaming is needed.

### 2b. GPIO / low-bandwidth control — **a pin is a card in a pack** (start here)
A GPIO pin needs **no new primitive**: it is a **card in a kernel-backed pack**, read and
written through the existing `Storage.*` card API. This inherits the whole card stack —
lease lifetime, capability gating, per-pack/per-card schema, and the deterministic-replay
provider — for free, with **zero direct hardware access** from the capsule.

- **Pack** — a virtual, kernel-backed pack (e.g. `"gpio"`, or per-controller `"gpio0"`),
  selected with `Storage.UsePack`. Its cards are pins instead of WALFS blobs.
- **Pin card** — keyed by pin (`"gp17"` / pin index). Its value is a **normalised integer
  in `[0, 1024]`** (10-bit full-scale; the kernel maps to native resolution — Pico PWM is
  16-bit, ADC 12-bit):
  - digital input read → **`0`** (low) or **`1024`** (high);
  - digital output write → `0` drives low, `1024` drives high (intermediate values are
    thresholded by the driver);
  - PWM-capable output → duty cycle = `value / 1024` (0 %..100 %);
  - ADC-capable input → reading scaled to `0..1024`.
- **Capability via schema** — `Storage.GetSchemaForPack("gpio")` returns each pin's
  **direction** (in / out / both) and **kind** (digital / pwm / adc), so a capsule discovers
  a pin's range and capability without touching hardware. Direction/mode is set through the
  schema (board config / kernel) or a mode field on the card — see §10.

Both the streaming ring and the GPIO pack flow through the same `pooldesc`/lease substrate
and the same I1–I8 invariants; only the lifecycle differs. **There is no register-level
class.**

---

## 3. Language surface (proposed)

All host hooks are **2-in/1-out** (`rd`, `rs1`, `rs2`) — a hard ABI constraint. Ops that
need >2 inputs (stream open with mode+config, a `Gpio.Write` with pin+value+flags) take a
**config descriptor span** built by the program and pass its handle, exactly as AES packs
`IV||payload` into one span. This keeps the generic `Ns.Method` lowering unchanged (no
frontend edits).

### `Device.*` — enumeration & lifecycle
```
Device.Open(idSpan, cfgDesc) -> devHandle      // idSpan = "gpio0"/"csi0"/"eth0"/"nvme0"; cfgDesc packs mode/flags
Device.Caps(devHandle)       -> capsBitsInt    // class bits the device exposes (stream/duplex)
Device.Close(devHandle)      -> ()             // releases all leases + the device binding
Device.Status(devHandle)     -> statusInt      // typed status (errno-style; see Status.Last, INV-18)
```

### `Stream.*` — DMA-ring streaming (sugar over Lease+Descriptor+WaitIRQ)
```
Stream.Open(devHandle, ringCfgDesc) -> streamHandle   // ringCfgDesc: depth, buf size, direction, policy(block|drop-oldest)
Stream.Next(streamHandle)           -> leaseHandle     // block (yield CPU) until a filled/free buffer; 0 + Status on EOF/timeout
Stream.Span(leaseHandle)            -> span            // zero-copy view of the buffer (read for RX, write for TX)
Stream.Submit(streamHandle, leaseHandle) -> ()         // TX: hand a filled buffer to the device
Stream.Release(leaseHandle)         -> ()              // RX: return a consumed buffer to the ring (I8)
Stream.Close(streamHandle)          -> ()
```
`Stream.Next` is the canonical **"IRQ → drain ring → re-arm"** loop (the RP1-ETH pattern):
it posts a pull to the kernel and sleeps on the return FIFO, yielding the CPU — **no
syscall/transition** (the async message-ABI rule). Backpressure is the existing
`LEASE_REVOKE`: under pressure the kernel reclaims the oldest lease; `ringCfgDesc.policy`
chooses block-vs-drop.

### `Mmio.*` — REJECTED (no register-level access; see §2)
Use the GPIO card surface below for control devices. There is no `Peek`/`Poke`.

### GPIO — the existing `Storage.*` card API on a hardware-backed pack
No new hooks are required for the minimum GPIO surface — a pin is a card:
```
Storage.UsePack("gpio")                         // select the kernel-backed GPIO pack
Storage.GetSchemaForPack("gpio") -> schemaSpan  // per-pin direction (in/out) + kind (digital/pwm/adc)
Storage.ReadCard("gp17")         -> valueInt     // 0..1024 (digital reads -> 0 or 1024)
Storage.UpdateCard("gp17", v)    -> ()           // write 0..1024 (digital: 0=low/1024=high; pwm: duty=v/1024)
Storage.QueryCard(...)           -> ...          // enumerate available pins
```
Optional thin sugar (a *convenience* lowering onto the same card hooks, only if the
ergonomics are wanted — adds a clean `CAP_GPIO` gate and pin-name resolution):
```
Gpio.Read(pinSpan)            -> valueInt        // = Storage.ReadCard on the gpio pack
Gpio.Write(modeDesc)          -> ()              // modeDesc packs (pin, value) -- 2 inputs fit; >2 use a descriptor
Gpio.Mode(modeDesc)           -> ()              // set direction/kind where the board allows it
```
The sugar is optional; the card API alone is sufficient and is the recommended baseline
(it reuses everything and needs no new hook codes).

### Inbound streams — reading request/RX **without FIFOs or descriptors**
This is the consumer side of the same binding substrate, and the platform **already hides
the FIFO/descriptor plumbing** behind two read facades. A capsule never touches a FIFO, a
`pooldesc`, or a raw pointer — it reads through accessors that return **leased spans**:

- **`Req.*`** (binding primitive) — `Req.Method()`, `Req.Path()`, `Req.Header(nameSpan)`,
  `Req.BodyMode()`, `Req.BodyCount()`, `Req.BodySpan(idx)`. The kernel parses the inbound
  message into a bound context (`ctx_desc`: leased header table + body) and installs it;
  the capsule just looks things up.
- **`Context.*`** (web facade) — `Context.GetVerb/GetPath/GetHeaders/GetQueryString/
  GetBody/GetUser/GetClientCert/GetTraceId`, richer sugar over the same context.

**Zero-copy + safe is the whole point** — every accessor returns a **span** (fat `ptr+len`,
INV-8) that points *into the leased kernel/DMA buffer*, never a copy into the capsule arena.
Safety is the lease (the same model as §6): the span is **bounds-checked** (you physically
cannot read past the body — I1 length-bounding), **validated** (I4), **revocable**
(`LEASE_REVOKE` under pressure), and **auto-released at handler scope exit** (I8);
use-after-release faults. So it is DMA/descriptor zero-copy *without* the usual zero-copy
footguns — the lease is the safety wrapper, and the FIFO/descriptor mechanics live entirely
below the binding.

**Two body modes, both hidden** (`ctx_desc.body_mode`, `PIOS_IO_BINDING.md` §3):
- **small / known-length → materialized**: `Req.BodyCount()` + `Req.BodySpan(i)` return the
  inline leased spans. Fast path, no FIFO at all.
- **large / chunked / unknown-length → pull cursor**: a `Req.BodyPull(max) -> span`
  accessor returns the next chunk as a leased span, **blocking (yielding the CPU) until
  bytes arrive**; loop until EOF. The platform issues the `BODY_PULL`/`BODY_CHUNK` FIFO
  exchange underneath — the capsule only ever sees "next chunk span."

Parse **in place** over the leased span — `Utf8Reader.*`, `Http.ParseJson`,
`Http.ParseQuery`/`ParseForm`, `Json.*` — so there is no copy between "received bytes" and
"parsed value".

> **Gap (specified, not yet realised):** `Req.BodyPull` is in `PIOS_IO_BINDING.md` but is
> **not a reserved hook** (Req currently tops out at `BodySpan` 0x0E) and has no VM
> deterministic provider, so today only the *materialized* body path runs in-language. The
> nicer ergonomic is to expose the streamed body as an **inbound `Reader`** (a `Utf8Reader`
> whose source is the body stream): the capsule does `Reader.Next()`/`Read(n)` and the
> platform pulls + leases chunks transparently and zero-copy. See §11.

### Schema / typed-field layer (picowal/walfs)
The reference Python/JS VM now implements the program-level PicoStore card model:
`Storage.UsePack/AddCard/EditCard/DeleteCard`, `SetField/GetField`, `SetFieldStr/
GetFieldStr`, `QueryCard/QueryResult`, and schema span storage
(`GetSchemaForPack`/`SetSchemaForPack`). C-style source also has active-record
sugar (`Order ord = Storage.GetCard(pack,id)`, `ord.qty = 42`,
`Storage.SaveCard(ord)`) that lowers to those hooks.

Production PIOS still owns the real walfs/picowal backing store and must bind the
same hook contract to persistent storage. The device model can use that schema
layer to declare pin `direction`/`kind`/`range`; data cards can use typed fields,
schema-validated CRUD and field-filtered query.

Large/blob cards use slice hooks (`Storage.SetSlice`, `CardLen`, `ReadSlice`,
`WriteSlice`) so a 400MB card can be scanned via WALFS/SD range I/O without
materializing the card in the VM.

---

## 4. Hook codes & capability classes (AS BUILT)

- **Hook range (as built):** the GPIO baseline + capsule + streaming hooks are now
  allocated in genuinely-free space (the original `0x130–0x16F` proposal was stale —
  `0x130–0x137` is `Http`, `0x140–0x149` is `Html`). Live allocation:
  `Gpio 0x150–0x156`, `Pack/Card/Fifo 0x160–0x167`, **`Device 0x168–0x16B`**,
  **`Stream 0x170–0x175`**; `0x157–0x15F`, `0x16C–0x16F`, `0x176+` reserved for growth.
  Adding hooks bumps `PV_HOOK_TABLE_VERSION` (INV-23), which the module check adapts to.
- **Capability classes (as built;** continue from `CAP_CRYPTO = 1<<9`):
  - `CAP_GPIO   = 1<<10` — the hardware-backed GPIO pack (only control gate; **no
    `CAP_MMIO`** — no register-level access).
  - `CAP_CAPSULE = 1<<11` — `Pack/Card/Fifo` (capsule store + intra-capsule IPC).
  - `CAP_DEVICE = 1<<12` — `Device.*` enumerate/open a streaming device.
  - `CAP_DMA    = 1<<13` — `Stream.*` (DMA-ring buffers).
  - `CAP_EVENT  = 1<<14` — `Event.*` (reactive event queue; UI/async dispatch). See docs/PICO_UI.md.
  - `CAP_UI     = 1<<15` — `Ui.*` (retained scene tree / PicoWire remote windowing). See docs/PICO_UI.md.
  - **Per-instance gating** is a **per-pin / per-`idSpan` allow-list** in the capsule's
    grant table, checked by the kernel on each access against the pin/device key —
    instance-level security with no capability-bit exhaustion and no address windows.
  - `CAP_ALL` is now `0xFFFF` (bits 0–15); default grant stays "all" so existing
    programs are unaffected, and the harness/`PICOVM_CAPS` restricts to gate.
  - For the **GPIO-card baseline**, gating can also reuse the existing storage capability +
    the per-pin allow-list (since pins are cards) — a dedicated `CAP_GPIO` is recommended
    for a clear audit boundary but is not strictly required.

---

## 5. Compiler components (PicoScript side — what we build when implementing)

1. **Namespaces + hook codes** in `picoscript_lang.py` (`HOST_HOOK_CODES` + the namespace
   tables). No new opcodes — everything lowers through the existing generic `Ns.Method`
   host-call path, so **the four frontends and `vm/picoc.js` need no parser/lowerer
   changes** (only the hook table regen via `gen_hooks_js.py`).
2. **Capability classifier** entries in all three VMs (`hook_cap`/`pv_hook_cap`/`hookCap`)
   — identical bit values, so a denied device binding faults `PV_FAULT_CAPABILITY`=8
   byte-identically on every path (INV-17).
3. **Config-descriptor helpers**: a small in-language convention (or `Descriptor.Make` +
   `Memory.Set`) to pack stream-open / multi-arg `Gpio` args into a span, since hooks are
   2-in/1-out.
4. **Typed failures** via the existing `Status.Last` channel (INV-18): `Device.Open`
   failure, lease timeout, an out-of-range or not-permitted pin, etc. set a typed code; the
   primary return is a null handle / 0.
5. **Source-span / structured traps** come for free (INV-25) — a fault in a device hook
   already carries `code/pc/detail`; PIOS adds `capsule_id`/`binding_id`.

---

## 6. Security surface (OS contract — what PIOS implements underneath)

Security is the first priority; this is where it bites. PIOS MUST:

1. **Capability check before dispatch** (INV-17) — honour the `CAP_DEVICE/DMA/GPIO` bits
   *and* the per-`idSpan` / per-pin allow-list in the capsule's grant table. Hook existence
   is not permission.
2. **GPIO pin allow-list (replaces MMIO whitelisting)** — the GPIO pack is kernel-backed;
   `ReadCard`/`UpdateCard` are gated per-pin against the capsule's grant table, writes are
   clamped to `[0, 1024]`, and **direction is honoured** (an input pin cannot be driven).
   **No physical address is ever exposed to the capsule** — the kernel translates a pin
   card op into the pin operation entirely on its side. (There is no register window to
   validate because there is no register-level access — §2.)
3. **DMA-buffer ownership** (I2/I3, the iso-lease model, `PIOS_IO_BINDING.md` D6) — a ring
   buffer has exactly one owner at a time; ownership *moves* capsule↔device at
   `Stream.Next`/`Submit`/`Release`. Use-after-release faults (poison + generation bump,
   like the response descriptors). The compile-time iso-lease (INV-7) extends to
   `Stream.*` handles.
4. **Validated leases + eventual release** (I4/I8) — every `Stream.Span` read goes through
   a validated lease; `LEASE_REVOKE` reclaims under pressure; scope exit auto-releases.
5. **Deterministic providers** (INV-15) — in seeded/replay mode every device hook is a
   deterministic provider (recorded camera frames, a fake GPIO pin holding its value, a
   canned block stream) so a recorded trace replays byte-for-byte. **This is what lets a
   device-using capsule still run on the Python/JS VMs and in tests.**
6. **No hidden allocation in hot device hooks** (INV-5) — `Stream.Next`/`Span`/`Release`
   are arena-free or declare arena use; honour `no_alloc`.
7. **DoS/quotas** — ring depth, GPIO write rate, and per-capsule device/pin count are
   bounded (a streaming capsule cannot starve the kernel).

---

## 7. Determinism & 5-path parity (the core tension, resolved)

Hardware is nondeterministic and non-portable; PicoScript is 5-path byte-identical +
deterministically replayable. The resolution is the existing binding doctrine
(INV-3, "host hooks are the only outside world"; the killer rule "bindings are not
ambient"):

- The capsule **logic stays pure and portable** — it manipulates handles/spans, never
  hardware directly.
- Device hooks exist on **all five paths**, so the **bytecode is byte-identical** and the
  parity gate (INV-24) still covers every program.
- **Observable behaviour** of a device hook is allowed to differ between the PIOS driver
  and the Python/JS deterministic provider — that divergence lives *only inside the
  binding*, which is exactly what "bindings are not ambient/portable" permits.
- Tests + replay use the deterministic provider; production uses the driver. Same bytecode,
  same hooks, same capability gates.

---

## 8. Invariant mapping

| Invariant | How device bindings satisfy it |
|-----------|--------------------------------|
| INV-3 outside world only via hooks | GPIO via `Storage.*` cards; streaming via `Device/Stream` hooks |
| INV-4 every hook has a contract | signatures + ownership/lifetime declared here |
| INV-5 no hidden alloc in hot hooks | `Stream.Next/Span/Release` arena-free / declared |
| INV-7 seal consumes ownership | `Stream` handles use the iso-lease (move) model |
| INV-15 deterministic mode | deterministic providers for replay/test (fake pin / recorded ring) |
| INV-17 capability before dispatch | `CAP_DEVICE/DMA/GPIO` + per-`idSpan`/per-pin allow-list |
| INV-18 typed failures | `Status.Last` for open/lease/pin errors |
| INV-24 parity gate | hooks/cards present on all 5 paths; deterministic providers tested |
| INV-25 structured traps | `code/pc/detail` + PIOS `capsule_id/binding_id` |
| I1–I8 (binding) | reuse `pooldesc`+lease+FIFO; one-owner, validated, eventual release |

No invariant is weakened, and **no register-level surface exists** — GPIO reuses the
already-governed card model, so there is no new dangerous primitive to secure.

---

## 9. Worked bring-up examples (mapping to existing PIOS drivers)

- **GPIO digital out (start here)** — `Storage.UsePack("gpio");
  Storage.UpdateCard("gp17", 1024)` drives gp17 high; `Storage.UpdateCard("gp17", 0)` low.
  On the Python/JS VMs the gpio pack is an in-memory card (a fake pin); on PIOS it drives
  the real pin. Same bytecode.
- **GPIO digital in** — `int v = Storage.ReadCard("gp16");` → `0` or `1024`.
- **PWM out** — on a PWM-capable pin, `Storage.UpdateCard("gp18", 512)` = 50 % duty
  (`512/1024`); the kernel scales to the native 16-bit PWM.
- **ADC in** — on an ADC pin, `Storage.ReadCard("gp26")` returns the reading scaled to
  `0..1024` (kernel maps from native 12-bit).
- **RP1 Ethernet RX (stream, future)** — `Device.Open("eth0")` → `Stream.Open(dir=RX,
  policy=drop-oldest)` → loop `l = Stream.Next; s = Stream.Span(l); …; Stream.Release(l)`.
  This *is* the RP1 "MIP-edge IRQ → drain RX ring → IACK re-arm" flow, wrapped.
- **SD2 / QSPI block (stream, future)** — `Stream.Open(dir=RW)`; read =
  `Next`/`Span`/`Release`, write = lease a free buffer, `Stream.Span` (write),
  `Stream.Submit`. (SD2 wiring/clock + QSPI pinout are kernel facts; the capsule sees only
  the ring.)
- **HDMI scanout (stream, future)** — `Stream.Open(dir=TX)`; the capsule fills framebuffer
  leases and `Submit`s them.

---

## 10. Open decisions for PIOS / future implementation

1. **Value range endpoint** — confirm `[0, 1024]` inclusive with `1024` = full-scale (duty
   `v/1024`, so 0 %..100 % inclusive) and digital = `{0, 1024}`.
2. **Pin direction/mode** — fixed by board config / schema, or settable by the capsule
   (`Gpio.Mode` / a card mode field)? Default: board/kernel fixes it; capsule reads it via
   `GetSchemaForPack`.
3. **GPIO gating** — dedicated `CAP_GPIO` (recommended, clean audit boundary) vs reuse the
   storage capability + per-pin allow-list. Either way the per-pin allow-list is the
   instance control.
4. **Pack/pin naming** — pin keys (`"gp17"` vs index) and pack names per SoC (Pi5 RP1,
   Pico2 RP2350); owned by board config.
5. **Inbound streamed body** — confirm the shape: a low-level `Req.BodyPull(max) -> span`
   accessor vs a higher-level inbound `Reader` (recommended) the platform feeds from the
   chunk pull. Either way the FIFO/descriptor pull stays hidden and the chunk is a leased
   zero-copy span.
6. **Schema scope** — how much of the walfs schema to realise first: minimal (typed
   field get/set + schema-declared GPIO pin kind) vs full (validation + field-filtered
   query). Recommended: minimal first (unblocks GPIO), full query later.
7. **Streaming (future)** — backpressure default (block vs drop-oldest) and hook-range
   confirmation (`0x130–0x16F` vs `EXT_HOST_HOOK_BASE`) when the streaming class is built.

## 11. Next implementation steps (deferred until requested)

Three independent, parity-safe pieces, in dependency order:

**(A) Walfs schema / typed-field layer — realise the reserved-but-NOOP Storage hooks.**
This unblocks both data cards and the GPIO model (a pin card needs a schema for
`direction`/`kind`/`range`). Implement, byte-identical across the three VM hosts + a
parity test: `GetSchemaForPack`/`SetSchemaForPack` (typed field defs, name→field-id),
`GetField`/`SetField`/`GetFieldStr` (typed field read/write), schema-validated
`AddCard`/`UpdateCard`, and `QueryCard`/`QueryResult` (field-filtered). No new hook codes —
they already exist (0x60/0x61/0x69–0x6E).

**(B) GPIO via cards** (depends on A for the pin schema) — small, parity-safe, no new
opcode:
1. A **kernel-backed virtual pack** convention; on the Python/JS/C VMs a **deterministic
   provider** = an in-memory gpio pack whose pin cards hold `0..1024` (digital pins
   quantise to `{0, 1024}`), so `ReadCard`/`UpdateCard` run + replay byte-identically.
   Optionally the thin `Gpio.*` sugar + `CAP_GPIO` + classifier entries (3 VMs) +
   widen `CAP_ALL`.
2. `tests/test_gpio.py` 5-path parity (write/read a pin card; digital quantisation; PWM
   duty scaling) + capability-gating.
3. Hand the real pin driver + per-pin allow-list to the PIOS build agent against §6.

**(C) Streamed inbound reader — hide the body pull FIFO.** Reserve `Req.BodyPull`
(+`Req.BodyEof`) or, preferred, an **inbound `Reader`** whose source is the body stream
(`Reader.Next()`/`Read(n)` pulls + leases chunks transparently, zero-copy). VM
deterministic provider = a canned chunk stream; parity test feeds a multi-chunk body. The
materialized path (`BodySpan`) already works.

**(D, later) Streaming devices** (`Device.*`/`Stream.*`) — the DMA-ring class for
camera/Ethernet/block/HDMI, unchanged from this design.

No driver, security enforcement, or VM device behaviour is built until requested; this
document is the contract both sides implement against.
