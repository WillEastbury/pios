# PIOS host bindings for PicoScript VM

PIOS is not a syscall host. PicoScript host hooks on PIOS are expected to route
through kernel-owned transports already present in the system:

- time / timers -> kernel clock / scheduler
- random -> kernel entropy / DRBG service
- net -> kernel socket / FIFO reply path
- http ingress -> existing `uhttp_bridge`
- x509 / key material -> kernel `x509.c` store and TLS binding path

Current PicoScript C host-provider contract:

- `PV_HOOK_CRYPTO_RANDOMBYTES`
  - source: kernel RNG / deterministic DRBG under policy
  - hosted status: documented skeleton (`host_status = 1`)
  - PIOS build status: wired to `crypto_random_bytes`

- `PV_HOOK_NET_LISTEN`, `PV_HOOK_NET_ACCEPT`, `PV_HOOK_NET_READ`,
  `PV_HOOK_NET_WRITE`, `PV_HOOK_NET_SHUTDOWN`, `PV_HOOK_NET_POOLSIZE`,
  `PV_HOOK_NET_REGISTER`, `PV_HOOK_NET_RECVSPAN`
  - source: user-core `sock_*` wrappers over the existing `CORE_NET` FIFO path
  - hosted status: documented skeleton (`host_status = 1`)
  - PIOS build status: wired
  - current shape:
    - `Listen(port, backlog) -> fd`
    - `Accept(fd) -> accepted_fd`
    - `Read(fd, max_bytes) -> span`
    - `RecvSpan(fd, max_bytes) -> span`
    - `Write(fd, span) -> bytes_written`
    - `Shutdown(fd) -> 1|0`
    - `PoolSize() -> active_udp_socket_count`
    - `Register(port) -> udp_fd`
  - `Connect` remains reserved until its VM-side argument contract is fixed

- `PV_HOOK_X509_FETCHCERTIFICATE`, `PV_HOOK_X509_GENERATECSR`,
  `PV_HOOK_X509_GETCERTINFO`, `PV_HOOK_X509_ISCERTVALID`,
  `PV_HOOK_X509_GETKEYHANDLE`
  - source: kernel `x509.c` store and TLS binding state
  - hosted status: documented skeleton (`host_status = 1`)
  - PIOS build status: wired
  - current shape:
    - `FetchCertificate(_) -> current cert DER span`
    - `GenerateCSR(common_name) -> CSR DER span`
    - `GetCertInfo(_) -> current subject string span`
    - `IsCertValid(_) -> 1|0`
    - `GetKeyHandle(_) -> key fingerprint`
  - other `X509.*` hooks remain reserved until a stable opaque-handle contract is defined

- `PV_HOOK_HTTP_REQUEST .. PV_HOOK_HTTP_RESPBODY`
  - hosted egress HTTP/TLS should layer on the net + x509 services above
  - inbound HTTP serving remains the kernel-owned `uhttp_bridge` path

Status code convention for the PIOS path:

- `host_status = 0` -> hook handled successfully
- `host_status = 1` -> provider present but service unavailable / operation failed
- `host_status = 2` -> bad arguments
- `host_status = 3` -> reserved / unsupported hook shape

This keeps the ABI stable while wiring only the parts that already map cleanly
onto real PIOS services.
