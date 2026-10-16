# Internal model/shard request lanes and PicoScript bindings

`ipc_model.h` / `ipc_model.c` provide fixed-capacity, allocation-free IPC
primitives for a runtime to route requests to a typed model generation and
shard. `ipc_model_vm.h` / `ipc_model_vm.c` now expose callable `ModelFifo.*`
bindings to the real C VM and shipped WebIDE compiler. The integration test
keeps three VMs attached to one engine and resolves payloads through the
existing kernel lease registry. **Live cross-core process attachment remains
inactive**: no automatic all-VM admission, MMU grant/revoke adapter, or scheduler
wake mapping is installed. No hardware, network, GPU dispatch or inference is
performed by these bindings.

## Trusted process identity broker (2026-10-16, offline)

`ipc_model_broker` binds a VM generation to an exact process slot, PID,
process generation and owning core. Its injected authoritative snapshot must
match all four fields and certify quiescence before it starts a VM or publishes
instance hooks. The broker rejects non-core-0 callers, duplicate executors,
reused process slots and stale child/parent identities. Initialize fresh
kernel-private broker storage once; serialize management and stop all affected
endpoints, not merely the process being attached. Snapshot checking does not
replace that stop/ack protocol.

Retirement preserves bindings and hooks on busy failure, including outstanding
descriptors, cleanup pins, batches and undelivered wakes. Only successful
quiescent registry retirement detaches the host and clears its binding.
Retirement must happen **before** process identity recycling; a stale snapshot
is deliberately not authority to destroy the previous binding.

`ipc_model_process_start()` in `proc.c` is the concrete production admission
entry/diagnostic: `OWNER` off core 0, `STALE` for mismatched process identity,
and `PHASE` for unproved NC placement or missing stop acknowledgement. It owns
persistent, 64-byte-isolated kernel `.bss` storage, verifies linker `.bss`
bounds and checks actual active identity/Normal-NC/Outer-Shareable attributes
with `mmu_active_nc_range_valid()`. Existing `map_user_kernel_low()` preserves
EL1-only `.bss` permissions and attributes; no EL0 registry mapping is added.
Its process snapshot reads the actual generation-bearing `proc_owner_record`
token around refreshed process metadata and rejects migration-locked tokens.

**Production admission intentionally still fails closed with PHASE for valid
live identities.** `proc_exit()` and `proc_mark_empty()` execute on owning user
cores, and slot reuse/spawn has no core-0 model stop/ack barrier. Neither
`PROC_BLOCKED` nor `PROC_DEAD` proves an executor cannot wake/recycle. Consequently
this change does not call the broker from spawn/exit, mutate a foreign process,
install EL0 callbacks, or claim automatic detach on live process exit. A future
owner-routed lifecycle protocol must acknowledge stop, retire through core 0,
and only then permit reuse. The production snapshot explicitly reports no
quiescence until that exists. MMU payload grants, process wake adapters,
cross-core VM execution and migration remain outside this identity-only helper.
`test_ipc_model_broker.c` tests the real registry/VM adapter with injected process
snapshots, including busy retirement and generational reuse; it is not hardware
or scheduler-integration proof.

## Why existing named queues cannot be reused as SPSC lanes

`ipc_queue.c` is a lock-serialized named byte queue with slot-index handles,
optional overwrite and shared allocation. Its current API does not expose
generation-checked VM ownership, an immutable request descriptor, independent
source/target SPSC lanes, or publication sequences. Using one named queue for
multiple sending VMs would not satisfy the SPSC contract. This module reuses
its error vocabulary and bounded FIFO design, not its unsafe-for-this-purpose
handle or overwrite semantics. Existing queue/topic/pipe behavior is unchanged.

## Management plane

Initialize fresh storage once. VM start, retire, route binding and lane opening
are **quiescent management operations**: every participating VM must be stopped
and no reader/writer may concurrently touch the registry. The runtime owns
this serialization, not a lock in the scheduler. Reinitialization is not a
restart API and invalidates all authorities.

There are eight VM slots, sixteen model slots and sixteen model/shard routes,
32 lanes, eight entries per lane, and 128 descriptors (sixteen reserved for each
source VM). The runtime supplies authoritative model and VM identities. Model
generations cannot move backwards; an active model slot cannot bind a different
generation. Model generations are retained after retirement. VM/lane/descriptor
generations fail closed on exhaustion rather than wrapping into old authorities.
Retiring either endpoint invalidates its lanes and descriptors, including work
already received. The manager must first stop execution and revoke outstanding
payload grants; this primitive does not revoke actual memory mappings.

Each `(source VM generation, model generation, shard)` route has its own lane
and one target VM generation. Independent source VMs never share a producer
cursor. Reopening an existing lane returns the same authority; it does not grant
a second producer. The runtime must serialize execution of each VM, including
multiple native threads representing that VM.

## Data plane and ownership

`request_create` allocates only within the source's partition, validates routing
and checks `offset <= capacity` and `length <= capacity - offset` before storing
metadata. The descriptor contains span ID/generation, not a pointer. Runtime
span resolution must additionally check grant authority, actual allocation
bounds and cacheability before any read or dispatch. Metadata is immutable from
publication until release.

Ownership is `FREE -> source OWNED -> lane QUEUED -> target RECEIVED -> FREE`.
Successful publish consumes the source descriptor authority. Full and all
other publication failures preserve it and do not modify the publication
sequence output. Repeated publication is refused. Descriptor-pool exhaustion
is separate from FIFO fullness: receiving returns ring credit, but releasing
returns descriptor credit. `release` and `cancel` poison metadata and increment
generation before a release publication of `FREE`; source allocation acquires
that publication. Cancellation is legal before publication or after receipt,
not while queued. Queued per-request cancellation needs a runtime cancellation
command routed to the target; it is **not implemented here**.

Batch receive processes at most its ceiling (1..8), never waits to fill, and
returns immediately when empty. Stale/malformed ring entries count against the
ceiling and return explicit `rejected` telemetry without producing a request.
It does not silently execute bad work. A corrupt descriptor may remain
quarantined until quiescent retirement; no best-effort repair is attempted.
Successful receives preserve FIFO order within a lane. There is no global
ordering across lanes.

Each producer/consumer cursor and every control/payload record has 64-byte
stride and isolation. Publication is payload then release head; consumption
acquires head before metadata; ring credit is release tail/acquire tail.
32-bit sequences never wrap: publication fails with `NOSPC` at exhaustion.
`sequences` supplies generation-checked publication/credit snapshots for a
runtime's sticky wake check. The runtime must issue its existing sequence-backed
wake after successful publication and preserve continuation on downstream full;
this module does not emit a wake, park a VM, poll, allocate, or invoke a service.
Shared storage must have compatible **Normal-NC mappings across every domain**
on Pi hardware; CPU ordering does not repair inconsistent memory attributes.

## Callable PicoScript lowering and host boundary

The shipped C VM's `Queue.Enqueue`/`Dequeue`/`Depth` hooks are local eight-channel
integer queues. Its `Queue.EnqueueBatch`/`DequeueBatch` hooks return zero as
deferred container APIs. `Fifo.*` defaults are also VM-local. They do **not**
lower automatically to these cross-VM primitives. They retain their existing
behavior. `ModelFifo.*` is a separate PIOS extension (extended hostcall IDs
`0x3E0..0x3E9`, encoded as `0x63E0..0x63E9`), protected by `PV_CAP_QUEUE`.
Unattached VMs return `OWNER`, not success. Instance callback/data live in
host-private `pv_ctx`, not a shared global provider or script memory.

Validation: `test_ipc_model.c` exercises separate source lanes, shard routing,
batch ceiling, credit, ownership, stale generations, cancellation and bounds.
`python tests\test_ipc_model_picoscript.py` extracts the actual shipped compiler
and hook registry, compiles source, and executes the real C VM/engine/lease
registry without boards or network. It covers simultaneous senders, selective
source demux, independent shards, bounded/partial full batches, credit sequences,
local span import, explicit ack, duplicate/stale ack, invalid span extent,
ungranted lease handles, stale leases, stale routes, and capability denial.

### Host admission

The manager starts VM generations, binds model/shards and opens lanes while
quiescent. It calls `ipc_model_vm_attach()` once per admitted execution owner
and `ipc_model_vm_admit()` for each permitted lane. It must not attach two
executors for the same VM. Script cannot start VMs, bind models, create lanes,
choose a target VM generation, supply a source identity, or pass raw pointers.
Route lookup only returns already-admitted generation-backed lane tokens.

The mandatory trusted callbacks export/pin a local lease capability, import it
as a **receiver-local** span, finish/revoke its pin, and wake a particular VM
generation/lane at a particular sequence. Export must validate actual lease
generation, owner, target grant, immutable ready state, full extent, and matching
memory attributes. Failure must leave no pin. A successful export stays pinned
until failed publication or receiver ack; queued cancellation/retirement requires
quiescent manager cleanup. Import revalidates the grant and extent; no
sender-local span index or address becomes receiver authority. The host test
broker uses `lease_resolve()`/`lease_ptr()` and a bounded four-byte trust-boundary
copy into `pv_span_from_bytes()`. This is not a production MMU adapter.

Wake is invoked once per successful submission/batch with its final publication
sequence, and once per receive batch with the returned-credit sequence addressed
to the producer. The callback receives an explicit returned-credit flag, also
unambiguous for self lanes. Production must map VM generation to the owning process and existing
sticky wake sequence, and retain continuations on downstream full. These
callbacks are synchronous host/control operations, not IRQ/scheduler work.

### Production attachment audit (2026-10-11, offline; updated 2026-10-16)

Production attachment is blocked, not merely missing an init call:

- `user/capsvc_host.c:cap_process_slot` and
  `user/httpd.c:picoweb_init` call `pv_init()` on each HTTP execution.
  Their scratch `pv_ctx` lives in EL0 process memory. Neither scratch address
  nor a script-provided slot/PID is a trusted persistent VM identity. Attaching
  there would also lose instance state on the next request while queued
  descriptors and payload pins can still exist. Admission needs a trusted
  process-slot/generation/owner link, one executor per VM, and quiescent
  restart/exit cleanup before recycling an identity.
- `picovm.c`'s default `Lease.Acquire` records a VM-local span index and
  type; `Lease.Release` only invalidates that local record. These handles
  are not `lease_handle` authorities. The EL0 kernel API exposes span
  rent/release and FIFO operations, not model admission or kernel lease
  export/import. Reinterpreting a local handle or handing the VM heap address
  to another process is not an adapter.
- `lease_set_mmu_ops()` has no production caller in `src`. No installed
  mapping/revocation implementation proves receiver permissions, matching
  Normal-NC attributes, or completed revocation before release. The model
  registry now has checked kernel-private Normal-NC `.bss` placement through
  the identity broker, but no EL0-accessible registry mapping. A new shared
  mapping or kernel API contract requires architecture approval.
- `proc_post_remote_wake()` uses one posting-owner ring per target core,
  carries a PID, and can return false on full. Calling it directly from all
  VM cores would introduce multiple producers. The callback now retains
  failed delivery, but has no production posting-owner integration. Existing `swake` sticky
  sequences must be reached through an owner-routed, generation-checked
  control path with durable publication/credit continuation, not a bare
  SEV or a timer-generated fake wake.
- The original `void` finish/wake callback defect is fixed as described below.
  The production MMU adapter itself is still missing.

No production hook registration, mapping, wake, or deployment was added by
this audit. The existing three-VM shipped-compiler regression passes offline;
it validates the test broker lifecycle, not these absent production contracts.
The approved model FIFO intent does not supply the above memory/EL0 admission
and lifecycle design. Until those contracts are approved and implemented,
unattached production VMs continue to fail closed with `OWNER`.

### Implemented host lifecycle safety (2026-10-16, offline)

The owner approved fixing production contracts within the existing architecture.
The host adapter now reserves one executor per VM generation. Its persistent
trusted host record owns admission, batches and continuations separately from
request scratch memory. Trusted callers unbind before `pv_init`, then rebind
the reset context; received requests and pins survive, but cached local span
handles do not. Repeat import within one context reuses its local handle.
This API is **not yet installed in the EL0 HTTP hosts**; placing this trusted
record in their writable scratch memory would not establish trusted admission.

Finish and wake callbacks now return errors. Ack retains the descriptor,
generation and outstanding count when revocation fails. Successful revocation
is recorded so a later descriptor-release retry cannot revoke twice. Failed
publication retains exported authority and any owned descriptor until cleanup
succeeds; subsequent submission cannot overwrite a pending cleanup.
`lease_revoke()` now propagates unmap failure instead of falsely counting it
as a completed revoke. Attached endpoints cannot be retired with live
descriptors. An export cleanup with no descriptor also blocks retirement.
Unmanaged primitive-only callers retain their existing quiescent retirement API.

Wake failure retains a per-host, per-lane, per-direction continuation with the
latest sequence. `ipc_model_vm_service()` retries cleanup and wakes in bounded
posting-owner context; it never mutates a remote scheduler. Submit returns 1
if publication succeeded even when wake delivery failed; `Status()` exposes
the wake error. **Do not resubmit that request.** Batch calls likewise retain
their published/received prefix count. Callback failures are retryable only
through the host's owner-side service/ack protocol, not a new timer ingress path.
This service still needs a production owner-routed call site.

The shipped-compiler fixture exercises the actual adapter with the existing
lease registry and injected MMU map/unmap failures, wake transport fullness,
request resets with outstanding work, duplicate attachment, cached imports,
failed-publication cleanup and retirement refusal. These are offline lifecycle
proofs, not real page-table or scheduler integration proofs. Trusted process
generation admission, reserved Normal-NC registry storage, a production
grant/revoke implementation and EL0 command/owner-wake wiring remain absent.

### Source API (two operands maximum)

| Call | Result |
|---|---|
| `ModelFifo.Route(modelSlot, shard)` | admitted current-generation route token |
| `ModelFifo.From(route, sourceVmSlot)` | admitted inbound lane for the same model/shard |
| `ModelFifo.Submit(route, localLease)` | 1 on publication, negative status on failure |
| `ModelFifo.SubmitBatch(route, localLeaseListSpan)` | published prefix count (0..8) |
| `ModelFifo.Drain(route, ceiling)` | received count (0..8), never waits |
| `ModelFifo.At(index, field)` | field of a currently received batch entry |
| `ModelFifo.Ack(requestToken)` | release/poison and drop payload pin; 0 or negative status |
| `ModelFifo.Published(route)` / `Consumed(route)` | sequence snapshot |
| `ModelFifo.Status()` | exact status of previous ModelFifo call |

`SubmitBatch` takes a bounded VM-local span containing 1..8 big-endian u32
**local lease capability handles**, not payload pointers or shared descriptors.
Full extent is checked before decoding. It publishes a prefix, not an atomic
transaction: after `FULL` retry only the unsubmitted suffix when credit arrives.
Read `Status()` immediately after submit/drain; subsequent calls overwrite it.
`FULL=-4`, `EMPTY=-5`, `NOSPC=-3`, `STALE=-9`, `OWNER=-10`, `PHASE=-11`.
Other statuses follow `ipc_queue.h`. Batch rejection telemetry remains available
to the trusted host as `host.rejected`.

`At` fields: 0=request token, 1=imported receiver-local span, 2=tag from trusted
export, 3=source VM slot, 4=shard. Imports may consume the receiver's bounded
arena; cache/use the returned local span instead of repeatedly importing.
Drain is refused while any prior batch entry remains unacked. Ack tokens are
generation-backed and cannot ack another VM's request; duplicate ack is stale.
Positive script lane/request tokens fail closed before their bounded generation
encoding exhausts; manager retirement/re-admission is required, never wrapping.
Ring sequence snapshots use the VM's i32 bit pattern for u32 sequences.

`tools/pios_model_hooks.py` extends both embedded hook-registry copies without
refreshing unrelated portal assets; it is idempotent.
`tools/gen_ide_assets.py` applies the same extension before the upstream
compiler initializes its lookup tables. Base upstream hook fingerprint is
unchanged: this is a PIOS host extension, not a claim of upstream runtime or
module compatibility. Example: `examples/model_fifo.pc`.
