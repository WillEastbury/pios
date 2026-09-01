# DeveloperCLI repository contract

Canonical local validation for the DeveloperCLI issue-to-PR harness
(`tools/task_harness.py` in [developercli](https://github.com/WillEastbury/developercli);
see [ONE_SHOT_TASK_HARNESS.md](https://github.com/WillEastbury/developercli/blob/main/docs/ONE_SHOT_TASK_HARNESS.md)).

PIOS is a bare-metal AArch64 OS (Pi 5 / Pi 4 / Pi 3 / Zero 2 W / QEMU). There is
**no `make` on the verified Windows path**. Isolated clones must not invent a
second test runner or a command that only prints success.

## What the harness should run

**Test (required, no cross-compiler):** `python tests/run_host_tests.py`

Needs Python 3 and **clang** (host-native compile of `tests/test_*.c`). This is
the smallest real check that a logic/kernel-module change did not break the
pure-logic suites.

**Build (Pi 5 stage0 + stage2):** `cmd.exe /d /c build_bootstrap.bat`

Must run from the repository root through `cmd.exe /d /c` because PowerShell
rewrites `-march`. The batch file pins

`C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin`

If that toolchain is missing, the build fails closed — do not substitute another
compiler from `PATH`. Regenerates `include/build_version.h`; that file is not a
source of truth and must not be committed.

**QEMU smoke** (`python tools/qemu_smoke.py --no-load`) needs a prior
`build_qemu_full.bat` image and QEMU. It is not the isolated-clone gate.

**Deploy is operator-only.** Chunked OTA to a live Pi 5 is recorded below so
humans can find it. The harness must never execute `deploy`, never reboot a
board, and never push credentials.

## Agent constraints (also in AGENTS.md)

- Architecture changes need an owner decision and an ADR.
- Do not switch the `gh` account; use `--user WillEastbury` per command.
- Normalise LF before commit (`git ls-files --eol`). A few files are
  legitimately CRLF in the index.
- Do not pet the hardware watchdog in a stall. Do not block core 0.

```json devcli
{
  "version": 1,
  "test": [
    {
      "name": "host-tests",
      "argv": ["python", "tests/run_host_tests.py"],
      "timeout": 900
    }
  ],
  "build": [
    {
      "name": "pi5-bootstrap",
      "argv": ["cmd.exe", "/d", "/c", "build_bootstrap.bat"],
      "timeout": 3600
    }
  ],
  "deploy": [
    {
      "name": "ota-chunked-reboot",
      "argv": ["python", "tools/pios_ota_update.py", "--chunked", "--reboot"],
      "timeout": 900
    }
  ]
}
```
