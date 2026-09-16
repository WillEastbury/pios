#!/usr/bin/env python3
"""ADR-064 gate: partition enumeration stays a pure read-only parser."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / "src" / "partition_table.c").read_text(encoding="utf-8")

if not source.startswith("/*"):
    print("FAIL partition parser lost source prologue")
    sys.exit(1)
if source.find('#include "types.h"') > source.find('#include "partition_table.h"'):
    print("FAIL types.h must precede partition_table.h")
    sys.exit(1)

for token in ("partition_table_crc32", "input->read",
              "PARTITION_TABLE_GPT_ENTRY_LIMIT", "GPT_PARSE_READ_FAILED"):
    if token not in source:
        print(f"FAIL required bounded-parser token missing: {token}")
        sys.exit(1)

for token in ("mmio_", "sd_", "walfs", "mount", "write(",
              "malloc", "free(", "memcpy", "memset", "strlen", "strcpy",
              "strcat", "strcmp", "strncpy", "sprintf"):
    if token in source:
        print(f"FAIL forbidden storage/heap/string token: {token}")
        sys.exit(1)
print("partition table source gate: passed")
