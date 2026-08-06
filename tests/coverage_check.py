#!/usr/bin/env python3
"""
PIOS test-coverage checker.

For a bare-metal OS there is no single gcov number, so this reports coverage in
two complementary ways:

  1. A SUBSYSTEM COVERAGE MATRIX across the major stacks (MAC, NIC, IP/ICMP,
     UDP, TCP, ARP, DNS, FIFO, IPC, arena, proc/sched, WALFS, crypto, TLS, ...).
     For each stack it shows the strongest test tier that exercises it:
        UNIT     - a host-native unit test compiles+exercises its real code
        SELFTEST - an on-target selftest in the QEMU `selftest` battery
        SMOKE    - exercised at runtime by the QEMU smoke harness (boot + HTTP +
                   OTA traffic), so a regression would likely fail the smoke run
        NONE     - no test touches it  <-- the gaps to close
     plus the count of exported (non-static) functions per stack.

  2. REAL LINE COVERAGE (--llvm-cov) for the modules that have host unit tests:
     rebuilds them with clang coverage instrumentation and reports llvm-cov
     line %.

Usage:
  python tests/coverage_check.py            # subsystem matrix
  python tests/coverage_check.py --llvm-cov # + real line coverage of host tests
"""

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
TESTS = REPO / "tests"
LLVM = pathlib.Path(r"C:\Program Files\LLVM\bin")

# Modules with a host unit test (kept in sync with run_host_tests.py manifest).
HOST_TESTED = {"src/picocompress.c", "src/dhcp_options.c"}

# The selftests actually wired into the QEMU `selftest` battery (kernel.c).
def battery_selftests() -> set[str]:
    txt = (REPO / "src" / "kernel.c").read_text(encoding="utf-8", errors="replace")
    m = re.search(r"http_starts_with\(cmd,\s*\"selftest\"\).*?struct\s*\{.*?\}\s*bat\[\]\s*=\s*\{(.*?)\};",
                  txt, re.S)
    if not m:
        return set()
    return set(re.findall(r"(\w+_selftest)\b", m.group(1)))


# Subsystem -> sources + how each tier exercises it.
# 'smoke' is a short reason the QEMU smoke harness exercises the stack at runtime.
SUBSYSTEMS = {
    "FIFO":        {"src": ["src/fifo.c"],
                    "smoke": "inter-core IPC during HTTP/OTA"},
    "MAC (macb)":  {"src": ["src/macb.c"],
                    "smoke": None, "note": "Pi5-only HW; QEMU path is virtio-net"},
    "NIC":         {"src": ["src/nic.c", "src/virtio_net.c"],
                    "smoke": "every HTTP/OTA frame in/out"},
    "IP/ICMP":     {"src": ["src/net.c"],
                    "smoke": "stackdiag + HTTP burst"},
    "UDP":         {"src": ["src/net.c"],
                    "smoke": "DNS path", "note": "UDP lives in net.c"},
    "TCP":         {"src": ["src/tcp.c"],
                    "smoke": "HTTP burst + OTA stream + stackdiag"},
    "ARP":         {"src": ["src/arp.c"],
                    "smoke": "boot announce + gateway resolution"},
    "DNS":         {"src": ["src/dns.c"],
                    "smoke": "DNS resolve assertion in smoke gate"},
    "DHCP":        {"src": ["src/dhcp.c", "src/dhcp_options.c"],
                    "smoke": "option parser unit-tested (19/19); state-machine SMOKE via boot"},
    "socket":      {"src": ["src/socket.c"], "smoke": "userland HTTP bridge"},
    "IPC":         {"src": ["src/ipc_queue.c", "src/ipc_stream.c",
                            "src/ipc_proc.c", "src/pipe.c"],
                    "smoke": "cross-core during boot"},
    "arena":       {"src": ["include/mem_arena.h"], "smoke": None},
    "proc/sched":  {"src": ["src/proc.c"], "selftest": "proc_svc_selftest",
                    "smoke": "multicore boot + schedquanta"},
    "WALFS":       {"src": ["src/walfs.c"], "smoke": "format/verify at boot"},
    "crypto":      {"src": ["src/crypto.c"], "selftest": "crypto_selftest"},
    "TLS":         {"src": ["src/tls.c"], "selftest": "tls_selftest"},
    "lease":       {"src": ["src/lease.c"], "selftest": "lease_selftest"},
    "picocompress":{"src": ["src/picocompress.c"], "selftest": "pc_selftest"},
    "brotli":      {"src": ["src/brotli.c"], "selftest": "brotli_selftest"},
    "dtrace":      {"src": ["src/dtrace.c"], "smoke": "dtrace assertion"},
    "coredump":    {"src": ["src/coredump.c"], "smoke": "cdump assertion"},
}

FN_RE = re.compile(r"^(?!static\b)(?!typedef\b)[A-Za-z_][\w\s\*]*?\b(\w+)\s*\([^;]*\)\s*\{", re.M)
KEYWORDS = {"if", "for", "while", "switch", "return", "else", "sizeof", "do"}


def exported_fn_count(src: str) -> int:
    p = REPO / src
    if not p.exists():
        return 0
    txt = p.read_text(encoding="utf-8", errors="replace")
    names = [n for n in FN_RE.findall(txt) if n not in KEYWORDS]
    return len(set(names))


def tier(name: str, info: dict, battery: set[str]) -> tuple[str, str]:
    srcs = info["src"]
    if any(s in HOST_TESTED for s in srcs):
        return "UNIT", "host line-coverage test"
    st = info.get("selftest")
    if st and st in battery:
        return "SELFTEST", f"{st} in QEMU battery"
    if st:
        return "SELFTEST*", f"{st} exists (not in QEMU battery)"
    if info.get("smoke"):
        return "SMOKE", info["smoke"]
    return "NONE", "no test exercises this"


TIER_ORDER = {"UNIT": 4, "SELFTEST": 3, "SELFTEST*": 2, "SMOKE": 1, "NONE": 0}


def matrix() -> int:
    battery = battery_selftests()
    rows = []
    for name, info in SUBSYSTEMS.items():
        t, why = tier(name, info, battery)
        nf = sum(exported_fn_count(s) for s in info["src"])
        rows.append((name, t, nf, why, info.get("note", "")))

    print("PIOS subsystem test-coverage matrix")
    print("=" * 78)
    print(f"{'SUBSYSTEM':<14} {'TIER':<10} {'FNS':>4}  WHY / NOTE")
    print("-" * 78)
    gaps = []
    for name, t, nf, why, note in rows:
        detail = why + (f"  [{note}]" if note else "")
        print(f"{name:<14} {t:<10} {nf:>4}  {detail}")
        if t in ("NONE", "SELFTEST*"):
            gaps.append((name, t))

    # summary
    counts = {}
    for _, t, *_ in rows:
        counts[t] = counts.get(t, 0) + 1
    print("-" * 78)
    print("tiers:", ", ".join(f"{k}={counts[k]}" for k in sorted(counts, key=lambda x: -TIER_ORDER[x])))
    covered = sum(1 for _, t, *_ in rows if TIER_ORDER[t] >= 1)
    print(f"{covered}/{len(rows)} subsystems have at least SMOKE-level coverage")
    if gaps:
        print("\nGAPS to close:")
        for name, t in gaps:
            if t == "NONE":
                print(f"  - {name}: NO test exercises it")
            else:
                print(f"  - {name}: has a selftest but it is NOT in the QEMU battery")
    return 0


def llvm_cov() -> int:
    out = TESTS / "_cov"
    out.mkdir(exist_ok=True)
    cc = str(LLVM / "clang.exe")

    COV_MODULES = [
        ("picocompress", "test_picocompress.c", ["src/picocompress.c"]),
        ("dhcp_options",  "test_dhcp.c",         ["src/dhcp_options.c"]),
    ]

    print("\nReal line coverage (host unit tests):")
    any_fail = 0
    for mod, test, srcs in COV_MODULES:
        exe  = out / f"cov_{mod}.exe"
        prof = out / f"{mod}.profraw"
        profd = out / f"{mod}.profdata"
        cmd = [cc, "-std=gnu11", "-O2", "-g",
               "-fprofile-instr-generate", "-fcoverage-mapping",
               "-I", str(TESTS / "stubinc"), "-I", str(REPO / "include"),
               str(TESTS / test),
               *[str(REPO / s) for s in srcs],
               "-o", str(exe)]
        if subprocess.run(cmd, capture_output=True, text=True).returncode != 0:
            print(f"  llvm-cov: build failed for {mod}")
            any_fail = 1
            continue
        import os
        e = dict(os.environ)
        e["LLVM_PROFILE_FILE"] = str(prof)
        subprocess.run([str(exe)], capture_output=True, text=True, env=e)
        subprocess.run([str(LLVM / "llvm-profdata.exe"), "merge", "-sparse",
                        str(prof), "-o", str(profd)], capture_output=True, text=True)
        r = subprocess.run([str(LLVM / "llvm-cov.exe"), "report", str(exe),
                            f"-instr-profile={profd}",
                            *[str(REPO / s) for s in srcs]],
                           capture_output=True, text=True)
        print(f"\n  {mod}:")
        for ln in (r.stdout or r.stderr).splitlines():
            print(f"    {ln}")
    return any_fail


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--llvm-cov", action="store_true", help="add real line coverage for host-tested modules")
    args = ap.parse_args()
    rc = matrix()
    if args.llvm_cov:
        rc |= llvm_cov()
    return rc


if __name__ == "__main__":
    sys.exit(main())
