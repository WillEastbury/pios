#!/usr/bin/env python3
"""Regression gate for issue #111: bounded coherent WALFS FIFO buffers."""

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
walfs = (ROOT / "src" / "walfs.c").read_text(encoding="utf-8")
proc = (ROOT / "src" / "proc.c").read_text(encoding="utf-8")
picowal = (ROOT / "src" / "picowal_db.c").read_text(encoding="utf-8")


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


handler = function_body(
    walfs,
    "void walfs_handle_message(u32 from_core, const struct fifo_msg *request)",
)
create = handler[handler.index("case MSG_FS_CREATE:"):
                 handler.index("case MSG_FS_WRITE:")]
write = handler[handler.index("case MSG_FS_WRITE:"):
                handler.index("case MSG_FS_READ:")]
read = handler[handler.index("case MSG_FS_READ:"):
               handler.index("case MSG_FS_DELETE:")]
delete = handler[handler.index("case MSG_FS_DELETE:"):
                 handler.index("case MSG_FS_STAT:")]
stat = handler[handler.index("case MSG_FS_STAT:"):
              handler.index("case MSG_FS_FIND:")]
find = handler[handler.index("case MSG_FS_FIND:"):
              handler.index("case MSG_FS_SYNC:")]
readdir = handler[handler.index("case MSG_FS_READDIR:"):
                 handler.index("default:")]

assert "proc_buffer_ref_acquire(from_core, msg.buffer, msg.length" in create
assert "msg.length > WALFS_FIFO_PATH_MAX" in create
assert "dcache_invalidate_range(msg.buffer, msg.length);" in create
assert "walfs_fifo_cstr_valid" in create

for case in (write, read):
    assert "msg.length > WALFS_DATA_MAX" in case
    assert "proc_buffer_ref_acquire(from_core, msg.buffer, msg.length" in case
    assert "dcache_invalidate_range(msg.buffer, msg.length);" in case
    assert "reply.type = MSG_FS_ERROR;" in case

assert read.index("walfs_read(") < read.index("dcache_clean_range(msg.buffer, n)")
assert "proc_buffer_ref_acquire(from_core, msg.buffer, msg.length" in delete
assert "walfs_fifo_cstr_valid" in delete
assert "dcache_invalidate_range(msg.buffer, msg.length);" in delete

assert "proc_buffer_ref_acquire(from_core, msg.buffer" in stat
assert "proc_buffer_ref_acquire(from_core, msg.tag" in stat
assert "dcache_invalidate_range(msg.tag" in stat
assert "dcache_clean_range(msg.tag" in stat

assert "proc_buffer_ref_acquire(from_core, msg.buffer, msg.length" in find
assert "walfs_fifo_cstr_valid" in find
assert "dcache_invalidate_range(msg.buffer, msg.length);" in find

assert "msg.length != max_entries *" in readdir
assert "proc_buffer_ref_acquire(from_core, msg.buffer, msg.length" in readdir
assert "dcache_invalidate_range(msg.buffer, msg.length);" in readdir
assert "dcache_clean_range(msg.buffer" in readdir

assert "fifo_push(CORE_DISK, from_core, &reply);" in handler
wrapper = function_body(walfs, "void walfs_handle_fifo(u32 from_core)")
assert "walfs_handle_message(from_core, &msg);" in wrapper

for source, signature in (
    (proc, "static void fs_request(struct fifo_msg *msg, struct fifo_msg *reply)"),
    (picowal, "static bool fs_request(struct fifo_msg *msg, struct fifo_msg *reply)"),
):
    request = function_body(source, signature)
    assert "MSG_FS_CREATE" in request
    assert "MSG_FS_FIND" in request
    assert "MSG_FS_WRITE" in request
    assert "MSG_FS_DELETE" in request
    assert "MSG_FS_READDIR" in request
    assert "dcache_clean_range(msg->buffer, msg->length);" in request
    assert request.index("dcache_clean_range") < request.index("fifo_push")
    assert request.index("fifo_pop") < request.index("dcache_invalidate_range")

print("issue #111: all WALFS FIFO pointer paths are bounded, authorized, coherent, and replied")
