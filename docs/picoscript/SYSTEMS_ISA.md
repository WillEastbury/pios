# PicoScript systems super-ISA

PicoScript keeps its original sixteen 32-bit opcodes. PicoScript-C adds a
fixed, two-word systems instruction when portable C needs native-width values,
pointers, typed memory, or frames. Classic PicoScript continues to lower to the
base ISA.

## Wire format

The extension uses the previously unassigned `OP_NOOP` immediate range
`0x5xxx`. Host hooks remain in `0x6xxx/0x7xxx`; HTTP control remains in
`0x8xxx..0xAxxx`.

```text
header word
31       28 27    24 23    20 19    16 15    12 11       4 3       0
+-----------+--------+--------+--------+--------+-----------+---------+
| OP_NOOP   | Xdst   | Xsrc   | family | 0x5    | opcode    | type    |
+-----------+--------+--------+--------+--------+-----------+---------+

payload word
31                                                               0
+-----------------------------------------------------------------+
| immediate, displacement, target, length, or X-register operand  |
+-----------------------------------------------------------------+
```

Every systems instruction is exactly two words. Payload values
`0x80000000..0x8000000f` name `X0..X15`; every other bit pattern is a 32-bit
immediate. Decoders skip one payload word after a systems header, so payload
data is never interpreted as base bytecode.

## Register classes and types

- `R0..R15`: existing 32-bit values, handles, and compact VM state.
- `X0..X15`: 64-bit/native-width `I64`, `U64`, `PTR`, `SIZE`, `OFFSET`, and
  typed capability handles (`WAL`, `INDEX`, `POSTINGS`, `GRAPH`, `NODE`).
- `FROM_R32` and `TO_R32` are explicit register-class conversions.
- `X14` is the conceptual frame pointer and `X15` the conceptual stack pointer
  in the reference interpreter. Native backends map frame intent onto their
  normal call stack and ABI.

## Families

| Family | Current contract |
|---|---|
| `INT64` | move, 32-bit immediate materialisation, add/sub/mul/div, bitwise, shifts, signed/unsigned comparisons, R/X conversion |
| `MEMORY` | typed 8/16/32/64-bit loads and stores, field forms, `MEMCPY`, `MEMSET` |
| `POINTER` | `LEA`, pointer add/difference/index |
| `FRAME` | enter/leave, local address, 32/64-bit local load/store, systems call/return |
| `ATOMIC` | reserved (`ATOMIC_CAS64`) |
| `BLOCK` | provider-backed block read/write and mapped ranges |
| `GRAPH` | weighted edge CRUD, incoming/outgoing expansion, result cursors, bounded shortest path |
| `STORAGE` | WAL CRUD/recovery, exact/reverse indexes, token FTS and result cursors |
| `SIMD`, `TENSOR` | intent-preserving fused-operation extension spaces |

The reference definitions and executor live in `picoscript_systems.py`.
PicoIL represents a systems operation as one typed `system` node and emits the
two-word form without decomposing it.

## Native syntax in every dialect

Systems values are language types rather than C-only compiler hints:

```basic
DIM offset AS U64 = Block.Size()
DIM wal AS WAL = Wal.Open()
DIM cards AS WAL = Wal.Pack(wal, 5)
DIM text AS INDEX = FTS.Open(cards, 2)
DIM graph AS GRAPH = Graph.Open(cards, 7)
```

Python-style source uses annotations (`offset: u64 = Block.Size()`). English
source uses `Set offset as unsigned 64-bit to Block.Size().` The same shared
lowerer assigns register classes and emits identical systems semantics.

Native casts (`u64(x)`, `i64(x)`, `ptr(x)`, `size(x)`, `offset(x)`) and the
explicit `U64.*` family are available when source needs to state conversion
or arithmetic intent directly.

## PicoScript-C lowering

The C frontend now supports:

- `struct`, GCC `__attribute__((packed))`, and `#pragma pack(push, 1)` layout;
- pointers, address/dereference, pointer arithmetic, arrays and indexing;
- typedefs, function pointers, callback fields, and indirect callback calls;
- deterministic includes, object macros, and conditional preprocessing;
- `malloc`, zeroing `calloc`, and safe arena-lifetime `free`;
- one-X-register `uint64_t` values and offsets;
- frame-relative local arrays, structs, wide values, and address-taken locals;
- recursive/re-entrant calls with caller R/X state restoration;
- native C bitwise syntax.

The allocator is a deterministic bounded bump arena. `free(ptr)` is currently a
safe no-op and all heap storage is reclaimed when the VM/program arena resets.
Frame-local objects are reclaimed by `FRAME_LEAVE`.

## Backends

- `SystemsPicoVM`: executable reference semantics for base plus systems words.
- bytecode: PicoIL emits the fixed two-word encoding.
- native C: systems arithmetic becomes `uint64_t`/`int64_t`; typed memory uses
  bounds-checked little-endian helpers; native calls provide recursion.
- JavaScript/browser: systems values use `BigInt`; typed memory uses the shared
  `Uint8Array` arena.
- portable C interpreter: verifies, decodes and executes the same fixed two-word
  systems bytecode directly, including X registers and real frame state.

WAL, FTS and graph instructions are part of the executable ISA. Their durable
storage is supplied through the existing `Storage.*` provider ABI. The Python
and browser VMs include deterministic in-memory implementations; native hosts
install `pv_storage_hook` (for example a file, block-device, PIOS, or portable
PicoWAL provider). `GRAPH_SHORTEST_PATH` maps to the coarse `Storage.GraphPath`
provider operation so the provider can use its own adjacency/index layout and
leave a path in the normal graph result cursor.

See [PICOWAL_ENGINE.md](PICOWAL_ENGINE.md) for the storage and index contract.
