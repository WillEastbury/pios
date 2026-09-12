# Network Service Capsules

`capsvc` is the generic kernel-to-EL0 capsule-service dispatcher. It keeps TCP
control blocks and network protocol ownership in the kernel while handing
bounded request/response spans to an EL0 PicoScript host.

## Current implementation

```text
NIC IRQ
  -> IRQ top half publishes bounded AIRQ/FIFO ingress
  -> scheduled MAC, IP and TCP FIFO stages
  -> capsvc or uhttp bridge request span
  -> parked EL0 PicoScript host
  -> bounded response span and kernel-owned TCP output
```

Hardware IRQ handlers are trigger-only. They do not parse packets, run a
capsule, or poll a socket. Each network stage consumes a bounded FIFO record
and publishes the next one; a missing event is fixed at that handoff rather
than by a polling fallback.

`uhttp_bridge` hosts the EL0 workers on ports 82 and 83. `capsvc_register()`
registers a generic capsule service; the admin capsule is registered on port
8090. The kernel owns connection identity, TCP state, scheduling authority,
and privileged operations such as raw-slot writes. Capsules receive opaque,
generation-checked request/response handoffs and have no raw MMIO, socket, or
flash-write authority.

## Contracts

- FIFO/control ownership is single-writer and cache-line isolated.
- Request and response lengths are explicit and validated before access.
- A process parks when its request queue is empty and wakes through the
  existing sequence-backed remote-wake path.
- Cross-core notification uses the target FIFO source and SGI with an SEV
  correctness backstop; no capsule polls a socket for work.
- A capsule failure is contained to that service. It cannot mutate a TCP
  control block, driver state, or raw boot slot.

The authoritative network execution and backpressure rules are in
[`network_stack.md`](network_stack.md); scheduling and AIRQ ownership rules are
in [`architecture_system.md`](architecture_system.md). Historical design
proposals formerly kept in this file are superseded by those implemented
contracts.
