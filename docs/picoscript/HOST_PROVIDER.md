# Multi-target host provider

## Goal

Run the **same PicoScript VM features** on:

| Target | Provider | Kind |
|--------|----------|------|
| CI / replay | `pv_host_null()` | frozen clock + seeded DRBG |
| Windows | `pv_host_platform()` | Win32 / BCrypt |
| Linux / WSL | `pv_host_platform()` | POSIX / `/dev/urandom` |
| PIOS | `pv_host_pios()` | kernel IPC (skeleton today) |

The **ISA and pure hooks stay in `picovm.c`**. Only host-injected bindings go through the provider.

## Governing rules

1. **Bindings are not ambient** (INV-17) — `PV_CAP_*` still gates in the VM.
2. **Deterministic mode is first-class** — null provider for tests/replay.
3. **PIOS does not use syscalls in the worker** — pios provider is message-shaped.
4. **WSL is POSIX** — no separate WSL backend.

## Install

```c
#include "pv_host_provider.h"

/* Deterministic tests */
pv_host_null_seed(0xC0FFEE);
pv_host_null_set_time(1700000000);
pv_host_install(pv_host_null());

/* Hosted Windows / Linux / WSL */
pv_host_install(pv_host_platform());

/* PIOS freestanding (kernel agent wires real IPC later) */
pv_host_install(pv_host_pios());
```

`pv_host_install` registers `pv_host_provider_dispatch_hook` so
`pv_default_host` routes matching hooks through the provider.

## Hooks covered

### `pv_host_*` (OS provider)

| Namespace | Hooks | Notes |
|-----------|-------|-------|
| `DateTime` | `Now`, `UtcNow`, `UnixTimestamp` | Wall / frozen UTC seconds |
| `Maths` | `Random`, `RandomRange` | Q16.16 fraction / inclusive range |
| `Crypto` | `RandomBytes` | New span of random bytes (cap 512) |
| `Environment` | all `Get*` | OS string + process facts |

### `vm/picovm_emu.c` (always linked with hosted C VM)

| Namespace | Status |
|-----------|--------|
| `DateTime` pure (`Parse`/`Format`/`Add*`/`Year`/…) | **Y** civil UTC math |
| `Gpio` | **Y** pin emulator |
| `Device` / `Stream` | **Y** DMA-ring emulator |
| `Ui` | **Y** retained tree + serialize |
| `Process` / `Env` / `Timer` / `Scheduler` | **Y** in-process tables |
| `Locale` | **Y** UTC-light formatting |
| `Auth` / `X509` | **Y** demo in-memory (admin/admin, demo cert) |
| `Context` | **Y** maps from `Req.*` when pool-fed + scratch |
| `Http` live (`Request`/`Resp*`/`Read*`) | **Y** hosted sockets |
| `Card` | **Y** soft-card placeholder |

### Compose hooks (unchanged pattern)

| Hook | Backend |
|------|---------|
| `Storage.*` | `host/picowal/storage_file.c` (+ slice/ready/edit) |
| `Net.*` | `vm/picovm_net.c` |
| `Req`/`Resp` | `vm/picovm_pool.c` |

## Layout

```text
host/
  pv_host_provider.h/.c   OS Time/Random/Environment
  pv_host_null/win/posix/pios.c
  test_host_provider.c
  test_emu_hooks.c
vm/
  picovm_emu.c/.h         multi-target emulators + pure DateTime
```

## Build smoke test (Windows MSVC)

```powershell
$vs = "C:\Program Files\Microsoft Visual Studio\18\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
cmd /c "`"$vs`" && cl /nologo /W3 /I vm /I host /D_CRT_SECURE_NO_WARNINGS ^
  vm\picovm.c host\pv_host_provider.c host\pv_host_null.c host\pv_host_win.c host\pv_host_pios.c ^
  host\test_host_provider.c /Fe:build\test_host_provider.exe bcrypt.lib advapi32.lib && ^
  build\test_host_provider.exe"
```

## Linux / WSL

```bash
cc -O2 -Ivm -Ihost \
  vm/picovm.c host/pv_host_provider.c host/pv_host_null.c \
  host/pv_host_posix.c host/pv_host_pios.c host/test_host_provider.c \
  -o build/test_host_provider -lpthread
./build/test_host_provider
```

## Shipping-quality hosted crypto / auth (Win + POSIX)

| Component | File | Notes |
|-----------|------|-------|
| SHA-512 / HMAC-SHA-512 / SHA-1 / MD5 / Blake2b-256 | `vm/picovm_crypto_ext.c` | Pure C; Blake3 hook maps to Blake2b-256 until full tree hash |
| Sign / Verify | same | HMAC-SHA256; Sign returns `sig(32)\|\|msg` |
| GenerateKeyPair / DeriveKey | same | 32B secret + SHA256 pub; HKDF-Extract style |
| Auth + X509 store | `host/pv_auth_store.c` | File `picoscript_auth.store`; seeds `admin`/`admin` (SHA-256 password) |
| Locale | `picovm_emu.c` | OS `localtime` / Win TZ name |
| Ui.Serialize | `picovm_emu.c` | PSC1-style preorder dump |
| Storage SetField* | `storage_file.c` | JSON map merge rebuild |

Link hosted binaries with: `picovm_emu.c`, `picovm_crypto_ext.c`, `pv_auth_store.c`,
`pv_host_*.c`, and `ws2_32` (Win) / pthread (POSIX).

## Raw block layer

See [BLOCK_PRIMITIVES.md](BLOCK_PRIMITIVES.md):

- **Windows / POSIX:** `pv_block_ops_mmap()` — file + mmap
- **PIOS:** `pv_block_ops_walfs()` — installed `pv_walfs_backend` (LBA R/W)
- Host WALFS simulator: `pv_walfs_sim_open()` for CI without a kernel

## Remaining gaps (honest)

1. **PIOS** real kernel IPC / real WALFS SVC (block *API* is ready; kernel install pending).
2. **HTTPS** client (HTTP/1.0 cleartext only).
3. **Blake3** true algorithm (Blake2b-256 stand-in).
4. **Asymmetric** RSA/ECDSA (symmetric HMAC Sign/Verify is what ships).
5. **CatQ / MoE / Shard / Tensor accel** still optional provider plugins.
6. Full IANA zoneinfo database (OS local time used instead).
7. Wire `storage_file` / PicoWAL portable onto `pv_block_dev` (next).
