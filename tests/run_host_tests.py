#!/usr/bin/env python3
"""
PIOS host-native unit test runner.

Compiles and runs the tests/test_*.c host unit tests natively (clang), so pure
-logic kernel modules can be validated in milliseconds without QEMU or a board.

Each test is built with tests/stubinc AHEAD of include/ on the include path, so
the kernel modules' `#include "types.h"` resolves to the host-safe shim
(tests/stubinc/types.h) instead of the bare-metal header.

To add a test: create tests/test_foo.c that exercises a pure-logic module, then
add an entry to TESTS below listing the kernel sources it needs.

Usage: python tests/run_host_tests.py
"""

from __future__ import annotations

import pathlib
import shutil
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
TESTS = REPO / "tests"
OUT = TESTS / "_build"

# test file -> kernel source files it links against
TESTS_MANIFEST = {
    "test_picocompress.c": ["src/picocompress.c"],
    # The Pi5 editor PBRP envelope is made by tools/pios_brotli.py. This test
    # proves that it is accepted by the in-kernel decoder, validates both
    # checksummed headers, and rejects/re-recognizes corrupt and truncated data.
    "test_ide_asset_pack.c": ["src/brotli.c", "src/ide_asset_pack.c"],
    # DHCP option parser: compiled from src/dhcp_options.c (the standalone
    # parser extracted from dhcp.c, pure logic, no MMIO/asm/network deps).
    "test_dhcp.c": ["src/dhcp_options.c"],
    # Capsule manifest/card text parser: compiled from
    # src/capsule_manifest_parse.c (extracted from capsule_store.c, which
    # also does WALFS I/O -- this half is pure logic, no MMIO/asm/WALFS
    # deps). Also has a standalone libFuzzer harness: fuzz_capsule_manifest.c
    "test_capsule_manifest.c": ["src/capsule_manifest_parse.c"],
    # PicoSTS token/KDF/codec core: base64url, PBKDF2-HMAC-SHA256, HS256 JWT
    # sign/verify, claim reader. Pure logic (no MMIO/WALFS/asm); the test
    # supplies an independent SHA-256/HMAC oracle. See src/sts_token.c.
    "test_sts_token.c": ["src/sts_token.c"],
    # Lease registry cross-core locking: compiled from src/lease.c with host
    # shims for mmu.h (cache ops -> no-ops) and kspin.h (single-address-space
    # spinlock). Pins the serialization contract: lock release on every error
    # path, non-recursive nested-copy composition, and the grant callback
    # snapshot/relock/rollback linearization. See tests/test_lease.c.
    "test_lease.c": ["src/lease.c"],
    # MIDR_EL1 board-family decode for the multi-platform stage0 bootstrap
    # (Cortex-A76 -> Pi5, Cortex-A53 -> BCM2837-family). Pure bit-decode
    # logic, no asm/MMIO. See src/board_detect.c, include/board_detect.h.
    "test_board_detect.c": ["src/board_detect.c"],
    # Board-specific CYW blob selection, BCM2712 stepping classification,
    # BCM2711 pull fields, and VideoCore GPIO property-tag layout.
    "test_wifi_platform.c": ["src/wifi_platform.c", "src/mailbox.c",
                             "src/sha256_hkdf.c"],
    "test_sdhost.c": ["src/sdhost_logic.c"],
    # Offline HCI H4 controller receive framing. This is deliberately pure
    # parsing/ownership logic: no hardware or platform dependency.
    "test_bt_h4.c": ["src/bt_h4.c"],
    # First DWC2 milestone: verified BCM2837 DMA/IRQ-route facts plus a pure
    # generation-safe transfer ownership model. No controller hardware is
    # enabled or accessed by this host-tested contract.
    "test_dwc2_contract.c": ["src/dwc2_contract.c"],
    # BCM2837 ARMCTRL source registry and pending demux. DWC2 IRQ41 remains a
    # dormant known route until a future driver explicitly registers it.
    "test_legacy_armctrl.c": ["src/legacy_armctrl.c"],
    # RP1 CFE / PiSP-FE source-resource facts and stream ownership are
    # deliberately offline: this verifies no-hardware V1 validation,
    # generation handling, and permanent activation rejection.
    "test_rp1_cfe_contract.c": ["src/rp1_cfe_contract.c"],
    # ADR-054 media engines: passive resource facts plus generation-safe
    # ownership. This contract has no hardware, DMA, IRQ, or platform deps.
    "test_media_engine_contract.c": ["src/media_engine_contract.c"],
    # ADR-055 PiSP-BE request/configuration/span ownership contract. It
    # validates a private configuration copy and numeric DMA authority only;
    # it never identifies, enables, or accesses PiSP-BE hardware.
    "test_pisp_be_contract.c": ["src/media_engine_contract.c",
                                "src/pisp_be_contract.c"],
    # ADR-056 HEVC request/frame/job contract. Pure numeric metadata and
    # lifecycle validation only: it contains no compressed-input interpretation
    # or hardware.
    "test_hevc_contract.c": ["src/media_engine_contract.c",
                             "src/hevc_contract.c"],
    # ADR-055 PiSP-BE brick-test gate. It accepts only caller-supplied
    # observations and contract capabilities; it has no platform or I/O path.
    "test_pisp_be_gate.c": ["src/media_engine_contract.c",
                            "src/pisp_be_contract.c",
                            "src/pisp_be_gate.c"],
    # ADR-054 HVS V1 handoff lifecycle. Pure numeric ownership, private
    # snapshots/list descriptors, and backend readback attestations only.
    "test_hvs_handoff_contract.c": ["src/hvs_handoff_contract.c"],
    # USB Bulk-Only/SCSI geometry discovery. The test supplies a deterministic
    # transport mock and verifies READ CAPACITY(16) is issued only after the
    # READ CAPACITY(10) sentinel, including malformed-response rejection.
    "test_usb_storage.c": [],
    "test_crypto_soft.c": ["src/crypto.c"],
    # P-256 general-point ECDH multiply (added for TLS 1.3 server-side
    # ECDHE key exchange). Pure math, no MMIO/asm deps beyond simd_memset
    # (stubbed). Vectors cross-checked against Python's `cryptography`
    # library (independent P-256 implementation). See src/p256.c.
    "test_p256_ecdh.c": ["src/p256.c"],
    "test_bitnet_kernel.c": ["src/bitnet_kernel.c"],
    "test_rp1_adc.c": ["src/rp1_adc_math.c"],
    "test_tls_router_match.c": ["src/tls_router_match.c"],
    # TLS 1.3 HKDF-Expand-Label / key schedule (RFC 8446 Section 7.1),
    # validated against the official RFC 8448 byte-level trace. See
    # src/tls13_keysched.c.
    # TLS 1.3 HKDF-Expand-Label / key schedule (RFC 8446 Section 7.1),
    # validated against the official RFC 8448 byte-level trace. See
    # src/tls13_keysched.c. Links against src/sha256_hkdf.c (pure-logic
    # SHA-256/HMAC/HKDF, extracted from crypto.c which has ARM-crypto-
    # extension inline asm that can't host-compile).
    "test_tls13_keysched.c": ["src/tls13_keysched.c", "src/sha256_hkdf.c"],
    # TLS 1.3 ClientHello parser (RFC 8446 Section 4), validated against a
    # real captured ClientHello from the RFC 8448 byte trace plus a hand-
    # built secp256r1 key_share case and truncation/malformed negative
    # tests. Pure logic, no MMIO/asm deps. See src/tls13_handshake.c.
    "test_tls13_clienthello.c": ["src/tls13_handshake.c", "src/sha256_hkdf.c",
                                 "src/p256.c", "src/ecdsa.c"],
    # TLS 1.3 server-side handshake message builders (ServerHello,
    # EncryptedExtensions, Certificate, CertificateVerify, Finished) and
    # the transcript hash helper. Pure wire-format logic plus the P-256
    # ECDSA sign path (also pure logic, no ARM asm). See
    # src/tls13_handshake.c.
    "test_tls13_handshake_builders.c": ["src/tls13_handshake.c", "src/sha256_hkdf.c",
                                        "src/p256.c", "src/ecdsa.c"],
    # ADR-044 event-driven RFC 8446 client/server transport state and
    # generation-safe public TLS handles.
    "test_tls_event.c": ["src/picotlsserver.c", "src/tls13_client.c",
                         "src/tls13_verify.c", "src/tls13_handshake.c",
                         "src/tls13_keysched.c", "src/tls13_record.c",
                         "src/sha256_hkdf.c", "src/p256.c", "src/ecdsa.c",
                         "src/crypto.c"],
    "test_tls_api.c": ["src/tls.c", "src/picotlsserver.c",
                       "src/tls13_client.c", "src/tls13_verify.c",
                       "src/tls13_handshake.c", "src/tls13_keysched.c",
                       "src/tls13_record.c", "src/sha256_hkdf.c",
                       "src/p256.c", "src/ecdsa.c", "src/crypto.c"],
    # Asynchronous driver framework (src/adrv.c). Pure logic: time, watchdog
    # and liveness are injected hooks, so the contracts can be pinned with a
    # deterministic fake clock. These tests exist because the CYW43455
    # bring-up hit the same failures repeatedly -- core-0 starvation, watchdog
    # misuse, and cadence inversion. The critical assertions are that a stalled
    # operation NEVER pets the watchdog, and that the system SCHEDULES rather
    # than overruns: admission control refuses work that does not fit the pass
    # budget, and a step returning late is quarantined on first offence.
    "test_adrv.c": ["src/adrv.c"],
    # Software interrupt queue + prioritized dispatcher (src/airq.c). Hardware
    # IRQ handlers only enqueue bounded records (top half); the reactor
    # dispatches them by software priority under a time budget (bottom half).
    # Pins that an interrupt storm becomes a bounded backlog rather than a
    # takeover of the core, and that strict priority never starves the
    # cross-core FIFO doorbell that user cores are parked on.
    "test_airq.c": ["src/airq.c"],
    # Issue #96: four simultaneous producer cores target one priority/core.
    # Per-origin lanes remain SPSC while sequence allocation and shared
    # diagnostics must not lose or duplicate updates.
    "test_airq_concurrency.c": ["src/airq.c"],
    # EL0 -> EL1 process control line (src/pctl.c). The trap-free channel that
    # replaces the PARK/EXIT syscalls: async/await over queues with the
    # scheduler as executor. Pins the sticky-wake rule -- a reply landing
    # between "decide to await" and "publish AWAITING" must not be lost -- and
    # that untrusted EL0 input fails closed rather than being clamped.
    "test_pctl.c": ["src/pctl.c"],
    # CPU quantum banking (src/qbank.c). Cooperative yield/await earn spendable
    # credit; preemption earns none. Pins the guard that keeps mandatory
    # preemption intact -- credit buys IDLE cpu only and is unspendable while
    # another process is runnable -- plus the anti-gaming property that banking
    # is lossy and a tight yield loop farms nothing.
    "test_qbank.c": ["src/qbank.c"],
    # Per-core scheduler wake FIFO (src/swake.c), the kernel -> process half of
    # the syscall-free scheduling contract. Pins the rule that FIFO replies,
    # empty timer messages and preempt requests all advance ONE per-process
    # sequence, and carries the end-to-end Thread.Sleep(1000) round trip driven
    # through swake + pctl with no syscall anywhere.
    "test_swake.c": ["src/swake.c", "src/pctl.c"],
    # PPOS immutable positional page format and bounded ANY/AND/PHRASE/NEAR
    # queries.  The test uses an in-memory authoritative document callback,
    # so no WALFS or hardware dependencies are pulled into the host build.
    "test_ppos.c": ["src/ppos.c"],
    # Issue #92 PPOS persistence/provider integration. The test supplies a
    # deterministic in-memory WALFS, PicoWAL, and pv_ctx host mock to cover
    # immutable page reload/rebuild and the existing Storage.* hook ABI.
    "test_ppos_provider.c": ["src/ppos.c", "src/ppos_provider.c"],
    # ABI-level dual-NIC contract: distinct backend identities, preserved
    # 64-byte FIFO messages, and interface-scoped firewall rule defaults.
    "test_dual_nic.c": [],
    # tcp_conn_t encodes both slot and generation; stale handles for a reused
    # TCB must remain distinguishable and fail closed.
    "test_tcp_handle.c": [],
    # pcie1 FFC / LevelZero fail-closed classify (no MMIO). Pins B50 id,
    # BAR size, RP1 window non-overlap, and that finding a GPU is not a
    # verified LevelZero proof.
    "test_pcie1.c": [],
    "test_nvme.c": ["src/nvme.c"],
    "test_net_dispatch.c": ["src/net_dispatch.c", "src/airq.c"],
    "test_nic_receive.c": ["src/nic.c"],
    "test_tcp_pending.c": ["src/tcp.c"],
    "test_dma_logic.c": ["src/dma_logic.c"],
    "test_proc_owner.c": [],
    "test_usb_descriptor.c": ["src/usb_descriptor.c"],
}

TEST_CFLAGS = {
    "test_net_dispatch.c": ["-DPIOS_PLATFORM=2"],
    "test_nic_receive.c": ["-DPIOS_PLATFORM=2"],
    "test_crypto_soft.c": ["-DPIOS_PLATFORM=6"],
    "test_airq_concurrency.c": ["-DPIOS_HOST_CORE_ID_FN", "-pthread"],
    "test_tls_event.c": ["-DPIOS_PLATFORM=6"],
    "test_tls_api.c": ["-DPIOS_PLATFORM=6"],
}


def find_clang() -> str:
    for c in ("clang", r"C:\Program Files\LLVM\bin\clang.exe"):
        if shutil.which(c) or pathlib.Path(c).exists():
            return c
    print("ERROR: clang not found (needed for host unit tests)")
    sys.exit(2)


def main() -> int:
    cc = find_clang()
    OUT.mkdir(exist_ok=True)
    inc = ["-I", str(TESTS / "stubinc"), "-I", str(REPO / "include")]
    cflags = ["-std=gnu11", "-O2", "-g", "-Wall", "-Wextra",
              "-Wno-unused-parameter", "-fno-strict-aliasing"]

    total_fail = 0
    asset_pack = subprocess.run(
        [sys.executable, str(REPO / "tools" / "pack_ide_assets.py"), "--brotli"],
        cwd=REPO, capture_output=True, text=True
    )
    if asset_pack.returncode != 0:
        print("[BUILD FAIL] editor Brotli asset pack")
        print(asset_pack.stderr[-2000:])
        total_fail += 1
    for test, srcs in TESTS_MANIFEST.items():
        exe = OUT / (pathlib.Path(test).stem + (".exe" if sys.platform == "win32" else ""))
        cmd = [cc, *cflags, *TEST_CFLAGS.get(test, []), *inc, str(TESTS / test),
               *[str(REPO / s) for s in srcs], "-o", str(exe)]
        build = subprocess.run(cmd, capture_output=True, text=True)
        if build.returncode != 0:
            print(f"[BUILD FAIL] {test}")
            print(build.stderr[-2000:])
            total_fail += 1
            continue
        run = subprocess.run([str(exe)], capture_output=True, text=True)
        sys.stdout.write(run.stdout)
        if run.returncode != 0:
            sys.stdout.write(run.stderr)
            total_fail += 1

    for test in ("test_network_dispatch.py", "test_net_fifo_contract.py",
                 "test_udp_iface_contract.py", "test_tcp_syn_cookie_contract.py",
                 "test_tcp_generation_contract.py",
                 "test_el0_idle_contract.py",                  "test_bootstrap_fat_import.py",
                 "test_issue_116_allocator_lock.py",
                 "test_issue_115_socket_recv.py",
                 "test_issue_111_walfs_fifo.py",
                 "test_issue_112_bcache.py",
                 "test_qemu_blk_probe.py",
                 "test_sdio_io_only_cmd5.py",
                 "test_irq_fifo_intids.py",
                 "test_issue_134_bcm2837_sdio_irq.py",
                 "test_issue_143_pcie1_bus_master.py",
                 "test_issue_146_pcie1_aer.py",
                 "test_issue_145_pcie1_enum_map.py",
                 "test_issue_142_pcie1_dma_cache.py",
                 "test_issue_147_pcie1_msi_mask.py",
                 "test_issue_137_pcie1_host_contract.py",
                 "test_picoscript_datagram_contract.py",
                 "test_dma_boot_contract.py",
                 "test_issue_166_ota_transport_budget.py",
                 "test_pios_ota_update.py",
                 "test_ide_asset_pack.py",
                 "test_stage0_pkg_check.py",
                 "test_wifi_command_watchdog.py",
                 "test_issue_106_zero2w_wifi_activation.py",
                 "test_issue_131_zero2w_wl_on.py",
                 "test_wifi_preload_progress.py",
                 "test_issue_100_sdio_50mhz_strap.py",
                 "test_sdio_clock_contract.py",
                 "test_issue_120_sdio1_high_speed_probe.py",
                 "test_issue_70_usb3_phy.py",
                 "test_tls_source_gate.py"):
        run = subprocess.run([sys.executable, str(TESTS / test)],
                             capture_output=True, text=True)
        sys.stdout.write(run.stdout)
        if run.returncode != 0:
            sys.stdout.write(run.stderr)
            total_fail += 1

    print()
    if total_fail == 0:
        print("HOST TESTS: all suites passed")
        return 0
    print(f"HOST TESTS: {total_fail} suite(s) failed")
    return 1


if __name__ == "__main__":
    sys.exit(main())
