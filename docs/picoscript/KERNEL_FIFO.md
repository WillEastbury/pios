# PicoScript kernel-call boundary

User-mode PicoScript is split into two classes:

- Pure VM work (integer/64-bit systems operations, strings, maps, spans,
  codecs, and local capability bookkeeping) executes inside the VM.
- `Kernel.*` capability hooks are requests, not privileged calls. The EL0 host
  sends a fixed 64-byte request through `/kernel/picovm`; core 0 validates and
  dispatches it, then returns a fixed 64-byte reply.

The wire ABI is in `include/picovm_fifo.h`. Requests contain only hook IDs,
register indices, scalar arguments, and a sequence number. No user pointer is
accepted. Bulk data must use an explicitly authorized shared span/IPC
descriptor.

The kernel service currently handles cooperative wait/IRQ requests,
software-IRQ delivery, and bounded profile/trace acknowledgements. Unknown or
malformed requests fail closed.
