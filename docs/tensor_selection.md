# Explicit PicoScript tensor placement — trusted adapter only

Design authority: [ADR-089](architecture_decision_log.md#adr-089--harmonized-picoscript-tensor-offload)
builds on ADR-088's OS-owned descriptor queues. This page distinguishes that
approved production design from the currently implemented local adapter.

## Approved production execution contract

PicoScript requests a canonical tensor operation on OS-backed arena descriptors.
The request describes operation/version, shape, layout, dtype/quantization,
numerical contract and placement policy. These are semantic requirements, not
permission to execute arbitrary kernel bytes or access a device.

PIOS validates process/arena/descriptor generations, complete buffer ranges,
permissions, aliasing, exclusive output ownership and resource budgets before
pinning allocations and admitting a request to an OS-owned queue. Backend
modules provide reviewed implementations of that operation, including kernel
and dispatch requirements. They do not grant memory or device authority.

PIOS selects only compatible, execution-ready implementations using accelerator
identity/generation, capabilities, location and scheduling state. Enumeration
alone, including a device labelled "Intel VGA", does not certify a usable
Xe2/XMX provider. Automatic placement may choose a compatible verified GPU,
QPU or CPU SIMD implementation; this is not a promise that every backend
implements every operation, layout or precision.

An exact pin rejects unavailable or incompatible targets. A preferred target
may fall back at admission when its policy permits; this is a production
design option, not an additional policy accepted by the local adapter below.
Fallback must preserve the operation's defined numerical contract.

PIOS and the owning driver establish bounded DMA mappings, required copies and
cache visibility, submit work, and track completion against the request and
accelerator generation. Remote-host placement requires explicit authorized
data transfer; remote addresses are never treated as local DMA pointers.
This design does not establish a cluster wire protocol or distributed executor.

After completion and output visibility are proven, PIOS publishes the result
and delivers a sequence-backed owner-routed wake. Queued work retains pins
until safe retirement. Cancellation or timeout does not authorize CPU replay
or buffer reuse while a device could still write: quiescence and successful
access revocation must precede reuse, poisoning and generation advance.
Failed revocation retains authority and reports failure.

Production endpoint admission, EL0 arena grants, cross-core scheduler wake
integration and accelerator-module dispatch remain incomplete. The ABI and
state names above are design requirements, not a claim of an implemented
production request layout.

## Harmonized model and current boundary

The canonical operation remains signed-i8 MatVec with i32 accumulation.
Placement is request/descriptor policy, not a new mathematical operation or
an accelerator-specific semantic opcode. `MatVecPinned` below is a local
binding adapter for that same operation and engine; its policy operand is
associated with that request only. These are host-hook IDs, not additions to
the frozen VM instruction set.

The target lifecycle is PicoScript tensor span/descriptor → defined
acceleration module (for example Xe2/XMX) → request to PIOS for GPU scheduling.
PIOS, not PicoScript or the module, must own location, capability admission,
scheduling/activity, DMA authority, transfers, and hardware access to the
authorized EL0 arena. A module supplies the native kernel implementation,
not authority to map or DMA arbitrary VM memory.

**This implementation is only a trusted same-core adapter stepping stone.**
It does not implement that production OS broker/lifecycle, an Xe2/XMX module
dispatch path, EL0 arena DMA grants, asynchronous device ownership, or GPU
transfers. No B50/B60 readiness follows from these hooks. No harmonized
production descriptor ABI is established by the local TSM1 format below.

`Tensor.MatVecI8(matrix, vector)` and `Tensor.SetShape(rows, cols)` keep
their existing behavior. The new, independent interface is:

```
int result = Tensor.MatVecPinned(request, policy);
int selected = Tensor.SelectedBackend(result);
```

The shipped WebIDE compiler lowers these calls to hooks `0x3EA` and `0x3EB`.
`examples/tensor_pinned.pc` is a complete executable example. Selection is
request-scoped: no VM layout change, ambient selection, or shared policy state.

`request` is a local VM span of exactly 20 bytes: five big-endian u32 words
`0x54534D31` (`TSM1`), rows, columns, matrix span handle, vector span handle.
Inputs are signed i8, row-major. Limits are 16 rows and 256 columns; zero
dimensions, short spans, and invalid handles fail before execution.

Policies:

* `0`: AUTO selects a registered compatible QPU, otherwise CPU.
* `1`: exact CPU pin.
* `2`: exact QPU pin, rejected if unavailable or incompatible.
* Other IDs are invalid. B50/B60 are not executable providers here.

On success the return is a span handle. Its contents are `TSR1`, a
big-endian u32 selected backend, then rows big-endian signed i32 values.
`Tensor.SelectedBackend(result)` validates this envelope and returns 1 or 2.
It is informational VM-owned data, not a capability or hardware readiness proof.
Output bytes begin at offset 8. No result allocation or output publication
occurs on failure. Input buffers must be outside the host's allocation arena.

Negative returns are explicit: `-1` invalid request/policy, `-2` queue full,
`-3` stale backend generation, `-4` unavailable/unsupported host,
`-5` execution failed, `-6` wrong owning core.
An exact pin never changes providers. AUTO also never retries after execution
starts. There is no GPU-success claim from a CPU-only provider.

The kernel native hook uses the actual `tensor_service` engine on trusted
core 0 only. It registers CPU and only verified/ready existing QPU kernels.
Calls complete synchronously through the bounded same-core queue; this does
not wire an EL0 service queue, cross-core pointer transport, or Async lifecycle.
Existing Async hooks and their QPA1 media request format are unchanged.
An unsupported VM host returns `-4` instead of entering the portable CPU fallback.

Run `python tests\test_tensor_pinned_picoscript.py` to compile the shipped
fixture with the shipped compiler, execute it in the real C VM through the
service engine, and compare CPU/AUTO output to the independent integer oracle
`[-7, 39]`. The provider is honestly CPU-only; QPU and unknown pins fail.
