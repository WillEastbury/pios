# Scheduler capabilities

User processes receive bounded scheduler primitives through `struct kernel_api`:

| Primitive | Meaning |
|---|---|
| `sched_yield(reason)` | Deschedule voluntarily with an explicit reason |
| `sched_get(out)` | Read the caller's PID, state, core, priority, quantum, runtime, and preemption count |
| `sched_set_priority(pid, class)` | Change only the caller's priority class |
| `sched_set_affinity(pid, core)` | Change only the caller's affinity |
| `sched_preemptions()` | Read the caller's preemption counter |

The kernel validates ownership and bounds every request. EL0 scheduler
commands use a per-process Normal-NC SPSC ring and the trapped `WFE` doorbell;
the kernel consumes only commands carrying the current process generation.
Wake publication advances the inbound sequence, so a park claim that races a
reply remains sticky and cannot sleep past completed work.

The live scheduler uses 1 kHz accounting and 5 ms quanta. Validate with
`schedquanta`; a healthy idle system reports `starving=no` for user cores.
