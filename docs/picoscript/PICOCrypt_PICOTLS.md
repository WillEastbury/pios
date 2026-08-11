# PicoCrypt and PicoTLS in PIOS

PIOS does not use the hosted PicoScript crypto extension as its security
implementation. The freestanding adapter in `src/picovm_pios_optional.c`
dispatches supported PicoScript crypto hooks to PIOS's existing primitives:

| PicoScript capability | PIOS implementation |
|---|---|
| SHA-256, HMAC-SHA256, HKDF-SHA256 | `src/sha256_hkdf.c`, `include/crypto.h` |
| SHA-512 | `src/sha512.c`, `include/sha512.h` |
| AES-GCM | `src/crypto.c`, `include/crypto.h` |
| Ed25519 sign/verify/key generation | `src/ed25519.c`, `include/ed25519.h` |
| P-256 ECDHE and TLS 1.3 record/key schedule | `src/p256.c`, `src/tls13_*.c` |

The upstream PicoTLS source tree is retained under `vendor/picotls/` for
portable/reference interoperability work. It is not linked into the
freestanding kernel automatically: PIOS's audited crypto and TLS code remains
the production path, avoiding duplicate implementations and hosted libc
dependencies.

Unsupported upstream VM crypto hooks fail closed with a provider-unavailable
status rather than silently substituting a weaker algorithm.
