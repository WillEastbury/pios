#!/usr/bin/env python3
"""Static regression checks for the EL0 event-driven idle contract."""

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
el0_sched = (ROOT / "include" / "el0_scheduler.h").read_text(encoding="utf-8")
pctl = (ROOT / "include" / "pctl.h").read_text(encoding="utf-8")
start = (ROOT / "src" / "start.S").read_text(encoding="utf-8")
proc = (ROOT / "src" / "proc.c").read_text(encoding="utf-8")
fifo = (ROOT / "src" / "fifo.c").read_text(encoding="utf-8")
kernel = (ROOT / "src" / "kernel.c").read_text(encoding="utf-8")
ipc_proc = (ROOT / "src" / "ipc_proc.c").read_text(encoding="utf-8")
ksvc = (ROOT / "src" / "ksvc.c").read_text(encoding="utf-8")
workq = (ROOT / "src" / "workq.c").read_text(encoding="utf-8")
httpd = (ROOT / "user" / "httpd.c").read_text(encoding="utf-8")
capsvc = (ROOT / "user" / "capsvc_host.c").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start_at = source.rindex(signature)
    opening = source.index("{", start_at)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:index]
    raise AssertionError(f"unterminated {signature}")


park = function_body(el0_sched, "static inline void el0_sched_park(void)")
assert "el0_sched_publish_control(PCTL_STATE_AWAITING" in park
assert "el0_sched_wait_for_scheduler();" in park

wait_for_scheduler = function_body(
    el0_sched, "static inline void el0_sched_wait_for_scheduler(void)"
)
assert '__asm__ volatile("wfi"' in wait_for_scheduler
assert '"wfe"' not in wait_for_scheduler

yield_ = function_body(el0_sched, "static inline void el0_sched_yield(void)")
assert "el0_sched_wait_for_scheduler();" in yield_

exit_ = function_body(el0_sched, "static inline NORETURN void el0_sched_exit(u64 code)")
assert exit_.count("el0_sched_wait_for_scheduler();") == 2

assert "SCTLR_EL1.nTWI is clear" in pctl
assert start.count("bic     x0, x0, #(1 << 18)") == 1
assert start.count("bic     x1, x1, #(1 << 18)") == 2
preempt_init = function_body(proc, "void proc_preempt_init(u32 timer_hz, u32 quantum_ms)")
assert "(1ULL << 16) | (1ULL << 18)" in preempt_init
assert "msr sctlr_el1" in preempt_init

wfx = function_body(proc, "bool proc_handle_wfx(struct irq_frame *frame, u64 esr)")
assert "proc_el0_control_verdict_trace(current_proc, p, frame->x[21])" in wfx
assert "PCTL_DESCHEDULE_AWAIT" in wfx
assert "proc_park();" in wfx

wfx_trace = function_body(
    proc, "static enum pctl_verdict proc_el0_control_verdict_trace"
)
assert "el0_sched_invalidate" not in wfx_trace
assert "wfx_last_state = line->state" in wfx_trace
assert "wfx_last_generation = identity->state" in wfx_trace
assert "wfx_last_slot_va = line->publish_seq" in wfx_trace
assert "mmu_user_pte_snapshot(core_id(), slot, el0_slot_va" in wfx_trace

verdict = function_body(
    proc, "static enum pctl_verdict proc_el0_control_verdict"
)
assert "el0_sched_invalidate" not in verdict
assert "pctl_evaluate" in verdict

scheduler = function_body(proc, "void proc_schedule(void)")
assert "u64 inbound_seq = swake_seq(si);" in scheduler
assert "sp->el0_inbound_seq != inbound_seq" in scheduler
assert "sp->state == PROC_READY &&" in scheduler
assert "verdict == PCTL_DESCHEDULE_AWAIT" in scheduler
assert "sp->state = PROC_BLOCKED;" in scheduler
assert "procs[chosen].el0_inbound_seq = swake_seq(chosen);" in scheduler
assert "proc_publish_control(chosen);" in scheduler

notify = function_body(fifo, "static inline void fifo_notify(u32 src, u32 dst)")
assert "gic_send_sgi((u8)(1U << dst), GIC_SGI_WAKE)" in notify
assert "sev();" in notify
remote_wake = function_body(proc, "bool proc_post_remote_wake(u32 target_core, u32 pid)")
assert "gic_send_sgi((u8)(1U << target_core), GIC_SGI_WAKE)" in remote_wake
assert "sev();" in remote_wake
assert "proc_fifo_doorbell_init();" in kernel

eth_irq = function_body(kernel, "static void core0_eth_irq_handler(void)")
assert "airq_post_from(CORE_NET, AIRQ_SRC_ETH_RX" in eth_irq
assert "sev();" not in eth_irq

quench = function_body(
    kernel, "static bool core0_eth_irq_drain_and_quench(bool host_route)"
)
assert "net_dispatch_publish_transport" not in quench
assert "core0_eth_irq_stall_streak++" in quench

assert "sevl();" in ipc_proc
assert "sev();" not in function_body(
    ipc_proc,
    "i32 ipc_proc_fifo_send(u32 principal, i32 channel_id, const void *data, u32 len)",
)
assert "sevl();" in function_body(
    ksvc, "bool ksvc_send(i32 dst_id, const struct ksvc_msg *msg)"
)
assert "sevl();" in function_body(
    workq, "bool workq_enqueue(u32 target_core, work_fn_t fn, void *ctx)"
)

assert "#define el0_wait_event() el0_sched_park()" in httpd
assert "#define el0_wait_event() el0_sched_park()" in capsvc

print("EL0 idle: trapped WFI reaches pctl; SGI doorbells retain a SEV backstop")
