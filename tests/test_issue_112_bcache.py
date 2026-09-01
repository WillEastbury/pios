#!/usr/bin/env python3
"""Regression gate for issue #112: dirty bcache entries fail closed."""

from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "bcache.c").read_text(encoding="utf-8")


def function_body(signature: str) -> str:
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


evict = function_body("static bcache_entry_t *evict(void)")
dirty = evict[evict.index("if (victim->flags & FLAG_DIRTY)"):
              evict.index("hash_remove(victim->lba)")]
assert "if (!writeback(victim))" in dirty
assert "return NULL;" in dirty
assert evict.index("writeback(victim)") < evict.index("hash_remove(victim->lba)")
assert "if (stats.dirty_count > 0U)" in dirty

invalidate = function_body("void bcache_invalidate(u32 lba)")
assert "if ((e->flags & FLAG_DIRTY) && !writeback(e))" in invalidate
assert "return;" in invalidate

invalidate_all = function_body("void bcache_invalidate_all(void)")
assert "if ((e->flags & FLAG_DIRTY) && !writeback(e))" in invalidate_all
assert "continue;" in invalidate_all

print("issue #112: dirty cache lines are retained when writeback fails")
