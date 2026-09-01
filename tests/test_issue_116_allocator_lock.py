#!/usr/bin/env python3
"""Regression gate for issue #116: allocator lock cache-line isolation."""

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
proc = (ROOT / "src" / "proc.c").read_text(encoding="utf-8")
kspin = (ROOT / "include" / "kspin.h").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.rindex(signature)
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


assert "struct kspinlock" in kspin
assert "u8 _pad[63];" in kspin
assert '_Static_assert(sizeof(struct kspinlock) == 64' in kspin
assert "#define PROC_SLOT_ALLOC_LOCK_SLOT 4U" in proc

lock_accessor = function_body(
    proc, "static inline struct kspinlock *proc_slot_alloc_lock(void)"
)
assert "kspin_shared(PROC_SLOT_ALLOC_LOCK_SLOT)" in lock_accessor

find_slot = function_body(proc, "static i32 find_empty_slot(void)")
assert "kspin_lock(slot_lock);" in find_slot
assert find_slot.count("kspin_unlock(slot_lock);") == 2

mem_exec = function_body(
    proc,
    "i32 proc_exec_from_mem(const char *name, const u8 *blob, u32 blob_len,",
)
assert "kspin_lock(slot_lock);" in mem_exec
assert "kspin_unlock(slot_lock);" in mem_exec

init = function_body(proc, "void proc_init_shared(void)")
assert "kspin_init(proc_slot_alloc_lock());" in init
assert "g_slot_alloc_lock" not in proc
assert "__atomic_test_and_set(&g_slot_alloc" not in proc

print("issue #116: process slot allocator lock owns WB-IS kspin cache line 4")
