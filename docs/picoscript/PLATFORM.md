# The PicoScript platform

What began as a bytecode scripting engine has grown into a small, coherent
full-stack application platform. Every capability funnels through **one
deterministic 16-opcode bytecode ISA**, so anything authored on any surface —
a script, a visual workflow, a report, an API handler — compiles to the same
bytecode and runs bit-identically in the browser, on bare metal (RP2350/PIOS),
and on the C# VM.

Author once → run everywhere → verify by differential oracle.

## Repositories

| Repo | Role |
|------|------|
| **picoscript** (this repo) | The language hub: the 16-opcode VM (Python / JS / C references), PicoIL, the frontends, the hook ABI (`vm/pico_hooks.*`), and the `docs/playground.html` WebIDE. |
| **baremetaljstools** | The browser runtime + UI suite (`BareMetal.*`): reactive binding, auth/tokens/RBAC/crypto, storage (IDB/KV), transport, the workflow designer (`BareMetal.WorkflowPico`), and 60+ zero-dependency modules. |
| **developercli** | The server/app generator: the forge ontology → app pipeline, multi-target backends, and the C# `PicoVm` + `WorkflowHost` that runs compiled workflows inside generated apps. |

## Capabilities → components

| Capability | Where it lives |
|------------|----------------|
| **Scripting engine** | PicoScript: 16-opcode VM + 8 frontends (C, BASIC, Python, English, COBOL, report, functional, **workflow**) → PicoIL → bytecode / C / JS. |
| **Workflow** | `picoscript_workflow.py` (reference) · `BareMetal.WorkflowPico` (JS designer) · the playground designer surface · the C# `PicoVm` target. See [WORKFLOW_DIALECT.md](WORKFLOW_DIALECT.md). |
| **Reports** | the `report` (4GL) frontend (+ a planned visual report designer — see [ROADMAP.md](ROADMAP.md)). |
| **Web server / APIs** | developercli forge (ontology → app) · `BareMetal.Communications`/`Rest` · the `Http` / `Req` / `Resp` / `Html` hook namespaces. |
| **STS / auth & security** | `BareMetal.Auth` (OIDC/PKCE), `BareMetal.Tokens` (JWT), `BareMetal.RBAC`, `BareMetal.Crypto` · the `Auth` / `X509` / `Crypto` hook namespaces. |
| **Storage DB** | `BareMetal.IDB` / `LocalKVStore` · the `Storage` / `Card` namespaces (`ReadCard`/`WriteSlice`/…) · developercli's card store. |
| **Query** | the `Query` namespace (`QueryCard` / `QueryResult` / `BuildLookupFilter`) · `BareMetal.Metadata` FK/lookup selects. |
| **Ontology** | developercli forge (`forge_ontology.c`, `forge_ir.c`) — the schema/entity model that drives app generation. |
| **Memory / data ABI** | `Memory.Get/Set` (0x37/0x36) and `Context.Get/SetScratchValue` (0xeb/0xea) — the integer data plane shared by all runtimes (arrays, fields, scratch). |
| **Events** | `RAISE` opcode `0xE`, the `Event.*` reactive queue (0x0180–0x0186), `ON` handlers, `CAP_EVENT` — first-class RAISE/subscribe with the `Event.*` FIFO across the Python/JS/C# VMs and a `BareMetal.PubSub` browser bridge. See [EVENTS.md](EVENTS.md). |

## The shared-ISA contract

The single most important property: **the bytecode is the contract.**

```
 surfaces          one IL           one ISA              runtimes
 ────────          ──────           ───────              ────────
 C / BASIC  ┐                                     ┌─ picovm.py  (reference)
 Python     │                                     ├─ picovm.js  (browser)
 English    ├──►  PicoIL  ──►  16-opcode words  ──┼─ picovm.c   (RP2350/PIOS)
 report     │                                     └─ PicoVm.cs  (generated apps)
 functional │
 workflow   ┘   (workflow lowers via English)
```

- **Hooks** (`Memory`, `Context`, `Storage`, `Http`, `Event`, `Crypto`, …) carry
  the same numeric codes across every runtime; `vm/pico_hooks.*` is the source of
  truth.
- **Differential oracles** keep the runtimes honest:
  `developercli/workflow/test/oracle.js` compiles through the JS bundle and the
  C# `PicoVm` must reproduce identical registers + output;
  `tests/test_workflow_frontend.py` validates the Python reference. Canonical
  workflow cases (`array_sum → 100`, `array_filter → 32`) agree across all three.

## Multi-target host providers (Win / Linux-WSL / PIOS)

Host-injected bindings (time, entropy, environment) are **not** baked into the
VM. They install through `host/pv_host_provider.h`:

| Provider | Use |
|----------|-----|
| `pv_host_null()` | Deterministic CI / replay |
| `pv_host_platform()` | Windows or Linux/WSL hosted |
| `pv_host_pios()` | PIOS kernel IPC skeleton |

See [HOST_PROVIDER.md](HOST_PROVIDER.md). Pure hooks remain in `picovm.c` and
are identical on every target.

## Where to start

- **Play:** `docs/playground.html` — write in any surface (including the visual
  **Workflow** designer), compile, run, and step through the bytecode.
- **Language:** [LANGUAGE_SPEC.md](../LANGUAGE_SPEC.md), [PRIMITIVES.md](PRIMITIVES.md), [HOOK_REFERENCE.md](HOOK_REFERENCE.md).
- **Host providers:** [HOST_PROVIDER.md](HOST_PROVIDER.md).
- **Workflow dialect:** [WORKFLOW_DIALECT.md](WORKFLOW_DIALECT.md).
- **What's next:** [ROADMAP.md](ROADMAP.md).
