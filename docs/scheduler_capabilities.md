# Scheduler capabilities

User processes receive bounded scheduler primitives through `struct kernel_api`:

| Primitive | Meaning |
|---|---|
| `sched_yield(reason)` | Deschedule voluntarily with an explicit reason |
| `sched_get(out)` | Read the caller's PID, state, core, priority, quantum, runtime, and preemption count |
| `sched_set_priority(pid, class)` | Change only the caller's priority class |
| `sched_set_affinity(pid, core)` | Change only the caller's affinity |
| `sched_preemptions()` | Read the caller's preemption counter |

The kernel validates ownership and bounds every request. EL0 park, yield, and
exit intent is published through the per-process Normal-NC `pctl` line and the
trapped `WFE` doorbell; diagnostic and lifecycle payloads use the bounded
per-process SPSC ring. Wake publication flows through `swake`, and its single
inbound sequence preserves the sticky-wake rule when a reply races a park.
`qbank` grants credit only on otherwise-idle cores and keeps the normal timer
preemption active.

The live scheduler uses 1 kHz accounting and 5 ms quanta. Validate with
`schedquanta`; a healthy idle system reports `starving=no` for user cores.
