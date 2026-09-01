#!/usr/bin/env python3
"""Regression gate for issue #115: safe cross-core socket receive buffers."""

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
socket = (ROOT / "src" / "socket.c").read_text(encoding="utf-8")
proc = (ROOT / "src" / "proc.c").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:index]
    raise AssertionError(f"unterminated {signature}")


handle = function_body(
    socket,
    "void socket_handle_message(u32 from_core, const struct fifo_msg *req)",
)
recv_case = handle[handle.index("case MSG_SOCK_RECV:"):
                   handle.index("case MSG_SOCK_CLOSE:")]
assert "proc_buffer_ref_acquire(from_core, msg.buffer, msg.length" in recv_case
assert "reply.status = 1;" in recv_case
assert "fifo_push(CORE_NET, from_core, &reply);" in recv_case
assert "socket_pending_start(from_core, &msg, conn, &buffer_ref)" in recv_case

service = function_body(socket, "void socket_service_step(void)")
validate_at = service.index("proc_buffer_ref_validate")
invalidate_at = service.index("dcache_invalidate_range", validate_at)
read_at = service.index("tcp_read(", invalidate_at)
clean_at = service.index("dcache_clean_range", read_at)
reply_at = service.index("socket_reply(", clean_at)
assert validate_at < invalidate_at < read_at < clean_at < reply_at

caller = function_body(socket, "i32 sock_recv(i32 fd, void *buf, u32 len)")
assert caller.index("dcache_clean_range") < caller.index("send_to_net")
assert caller.index("wait_reply") < caller.index("dcache_invalidate_range")

acquire = function_body(
    proc,
    "bool proc_buffer_ref_acquire(u32 core, u64 ptr, u32 len,",
)
assert "current_proc_arr[core].v" in acquire
assert "proc_buffer_range_in_slot(slot, ptr, len)" in acquire
assert "p->owner_core != core" in acquire
assert "p->state != PROC_RUNNING" in acquire

revalidate = function_body(
    proc,
    "bool proc_buffer_ref_validate(u32 core, u64 ptr, u32 len,",
)
assert "p->generation == ref->generation" in revalidate
assert "p->owner_core == core" in revalidate

print("issue #115: recv authorizes slot+generation, maintains WB cache, and replies on rejection")
