# PicoWAL inside PicoScript

PicoWAL is now represented in PicoScript at two separate, deliberately narrow
layers: typed language intrinsics express database intent, while a storage
provider owns durability and the physical index layout. Programs do not need
to call string-dispatched host methods or emulate 64-bit handles.

## Capability types

`WAL`, `INDEX`, `POSTINGS`, `GRAPH`, and `NODE` are native systems types held in
X registers. Handles have a stable compact representation:

```text
kind (top 4 bits) | selector (32 bits) | pack (10 bits)
```

They are opaque to ordinary source. `Wal.Pack`, `Index.Open`, `FTS.Open`, and
`Graph.Open` derive restricted handles rather than exposing provider pointers.

## First-class operations

- `Wal.Open/Pack/Put/Create/Get/Delete/Exists/Scan/Sync/Recover`
- `Index.Open/Upsert/Delete/Exact/Reverse/Result`
- `FTS.Open/Upsert/Delete/Search/Find/Result`
- `Graph.Open/Add/Delete/Weight/Out/In/Path/ShortestPath/ResultNode/ResultWeight`

Each lowers to one systems IL operation and one fixed two-word bytecode
instruction. `Graph.Path` is a bounded weighted shortest-path operation;
negative edges are not traversed. Results are deterministic and are read via
the graph result cursor.

## Engine/provider boundary

The PicoScript engine owns:

- syntax, capability typing and R/X register allocation;
- systems bytecode encoding, verification and execution;
- exact/reverse bounded fallback indexes used by the reference and bare VM;
- deterministic in-memory FTS and weighted graph semantics in Python/browser;
- numeric `Storage.*` ABI mapping for native C and generated C/JavaScript.

The installed provider owns:

- append, sync, recovery, CRC/torn-tail policy and media lifetime;
- durable secondary, postings and adjacency structures;
- tokenizer/schema policy and index rebuild/incomplete status;
- target integration (Windows files, POSIX files, PIOS block I/O, flash, RAM).

This keeps PicoWAL a separate reusable database tool while making WAL and index
semantics native to PicoScript. The portable PicoWAL implementation remains the
behavioural oracle for durable providers; the VM does not duplicate its on-disk
format.

## Transaction and recovery contract

One `Wal.Put`, `Wal.Create`, or `Wal.Delete` is one atomic append unit at the
intrinsic boundary. `Wal.Sync` requests durability. `Wal.Recover` asks the
provider to retain the longest valid prefix and rebuild derived state. Provider
status values remain the PicoWAL status contract (`OK`, `INVALID`, `NOT_FOUND`,
`EXISTS`, `IO_ERROR`, `CORRUPT`, `FULL`, `BUFFER_TOO_SMALL`,
`INDEX_INCOMPLETE`).

The bundled native file host supplies append-log CRUD, CRC recovery and sync.
Its FTS and graph hooks now call the vendored `portable/picowal_index.c`
implementation directly: the same tokenizer, posting tables, graph adjacency,
and Dijkstra routine used by PicoWAL’s standalone C tests. Ordinary JSON card
CRUD remains handled by the existing file adapter, while the adapter translates
FTS documents and graph edges into PicoWAL’s canonical binary card shape before
indexing. Derived-index events are written to reserved pack `4094` as
PicoCompress-compressed records. The first FTS/graph access lazily scans that
pack, decompresses events, and repopulates the bounded index arena. The arena is
bounded and reports an incomplete-index condition when its key/posting pools
are exhausted; pack-level LRU eviction for very small targets remains the next
storage-layer step. The Python and JavaScript engines retain executable bounded semantics for tests, the
Playground, and browser embedding.
