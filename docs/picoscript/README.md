# PicoScript in PIOS

This directory mirrors the upstream PicoScript capability and integration
documentation used by the vendored PIOS VM.

The freestanding PIOS build currently consumes:

- `include/picovm.h` and `src/picovm.c` for the VM core;
- `include/pico_hooks.h` for the generated hook ABI;
- `include/picoscript_abi.h` and `include/picoscript_rng.h` for shared ABI
  contracts;
- PIOS-native Tensor, Media, Bitlinear, and Async providers in `src/tensor.c`.
- Kernel capability calls from EL0 are marshalled through the bounded
  `/kernel/picovm` IPC FIFO (`include/picovm_fifo.h`); they do not execute
  kernel code directly inside the VM process.

The hosted emulator, crypto-extension, and PicoWAL index-context sources remain
under `vendor/picoscript/` because they require libc, sockets, or hosted TLS
dependencies that do not belong in the freestanding kernel image.

`FEATURE_MATRIX.md` and `HOOK_REFERENCE.md` are the authoritative upstream
capability inventories; `PIOS_HOST_BINDINGS.md` and
`PIOS_DEVICE_BINDINGS.md` identify which bindings are actually available in
PIOS.
