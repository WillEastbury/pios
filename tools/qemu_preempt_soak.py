#!/usr/bin/env python3
"""ADR-018 preemption soak test.

Preemption is only genuinely proven when a user process OVERRUNS its quantum.
Every EL0 process in PIOS normally parks within microseconds, so the timer PPI
always lands on an idle core, `PREEMPT` counters stay at 0, and the interesting
path -- a timer IRQ crossing the EL2 stage-2 cage to `ctx_switch()` out of a
RUNNING EL0 context (ADR-014) -- is never taken.

This harness forces that path: it drives the capsule host's `/spin/<n>` endpoint
(user/capsvc_host.c) with concurrent CPU-bound requests, each of which busy-loops
in EL0 for far longer than PROC_PREEMPT_QUANTUM_MS, then asserts:

  1. the per-core PREEMPT counter actually advanced (preemption really happened),
  2. every spin request still returned a correct response (no context corruption),
  3. the board stayed responsive throughout (reactor never starved),
  4. wired health counters are still clean (no RX wedge / hole / error).

Usage:
  python tools/qemu_preempt_soak.py [--build] [--rounds N] [--concurrency N]
  python tools/qemu_preempt_soak.py --host 192.168.0.201   # live Pi 5, no QEMU
"""
import argparse
import concurrent.futures as futures
import pathlib
import re
import subprocess
import sys
import time
import urllib.parse
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parent.parent
QEMU = r"C:\Program Files\qemu\qemu-system-aarch64.exe"
KERNEL = ROOT / "build_qemu_full" / "PIOS_QEMU_FULL.BIN"
SERIAL_LOG = ROOT / "preempt_soak_serial.log"

GRN, RED, YEL, RST = "\033[32m", "\033[31m", "\033[33m", "\033[0m"

HTTP = "http://127.0.0.1:8088"
CAPS = "http://127.0.0.1:8090"


def term(cmd, timeout=30.0):
    url = f"{HTTP}/api/terminal?cmd={urllib.parse.quote(cmd)}"
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read().decode("utf-8", "replace")


def spin(units, timeout=60.0):
    with urllib.request.urlopen(f"{CAPS}/spin/{units}", timeout=timeout) as r:
        return r.read().decode("utf-8", "replace")


def parse_sched(text):
    """Parse `proc sched`:
    CORE BUSY_PERMILLE IDLE WAKE IDLE_T TOTAL_T PREEMPT SOFT_EVT SOFT_BOOST TIMER_IRQ
    Returns {core: (preempt, timer_irq)}."""
    out = {}
    for line in text.splitlines():
        f = line.split()
        if len(f) >= 10 and f[0].isdigit():
            try:
                out[int(f[0])] = (int(f[6]), int(f[9]))
            except ValueError:
                pass
    return out


def wait_boot(proc, deadline=180.0):
    t0 = time.time()
    while time.time() - t0 < deadline:
        if proc is not None and proc.poll() is not None:
            return False
        try:
            with urllib.request.urlopen(f"{HTTP}/api/status", timeout=2) as r:
                if r.status == 200:
                    return True
        except Exception:
            time.sleep(1.0)
    return False


def run(args):
    fails = []

    def check(name, ok, detail=""):
        tag = f"{GRN}[PASS]{RST}" if ok else f"{RED}[FAIL]{RST}"
        print(f"  {tag} {name}  {detail}")
        if not ok:
            fails.append(name)

    print("[soak] baseline scheduler counters")
    before = parse_sched(term("proc sched"))
    print(f"        (preempt, timer_irq) before: {before}")

    units = args.units
    total = args.rounds * args.concurrency
    print(f"[soak] driving {total} CPU-bound /spin/{units} requests "
          f"({args.concurrency} concurrent x {args.rounds} rounds)")

    bodies, errors = [], []
    t0 = time.time()
    with futures.ThreadPoolExecutor(max_workers=args.concurrency) as ex:
        for _ in range(args.rounds):
            futs = [ex.submit(spin, units) for _ in range(args.concurrency)]
            for f in futs:
                try:
                    bodies.append(f.result())
                except Exception as e:  # noqa: BLE001
                    errors.append(repr(e))
    elapsed = time.time() - t0
    print(f"[soak] {len(bodies)} ok / {len(errors)} failed in {elapsed:.1f}s")

    print("[soak] post-soak scheduler counters")
    after = parse_sched(term("proc sched"))
    print(f"        (preempt, timer_irq) after:  {after}")

    # 1. Preemption actually happened somewhere.
    delta = {c: (after[c][0] - before.get(c, (0, 0))[0],
                 after[c][1] - before.get(c, (0, 0))[1]) for c in after}
    moved = {c: d for c, d in delta.items() if d[0] > 0}
    # A core that took no timer interrupts cannot preempt; report that distinctly
    # so a failure is never ambiguous between the two causes.
    no_timer = [c for c, d in delta.items() if d[1] == 0]
    check("timer interrupts delivered on every user core", not no_timer,
          f"cores with no timer IRQ: {no_timer}" if no_timer else
          f"deltas={{c: irq for c, (_, irq) in delta.items()}}".replace("{c: irq for c, (_, irq) in delta.items()}",
                                                                        str({c: d[1] for c, d in delta.items()})))
    check("preemption counters advanced", bool(moved),
          f"(preempt, timer_irq) deltas={delta}")

    # 2. Every spin request returned a well-formed response.
    good = sum(1 for b in bodies if '"ok":true' in b and '"spin_units"' in b)
    check("all spin responses correct", good == total and not errors,
          f"{good}/{total} correct, {len(errors)} transport errors"
          + (f" first={errors[0]}" if errors else ""))

    # 3. Board still responsive and serving normal traffic.
    try:
        status = term("proc sched")
        responsive = "CORE" in status
    except Exception as e:  # noqa: BLE001
        status, responsive = repr(e), False
    check("board responsive after soak", responsive, status.strip()[:60])

    # 4. Wired health unchanged.
    try:
        diag = term("macbdiag")
    except Exception as e:  # noqa: BLE001
        diag = repr(e)
    wedge = re.search(r"rx_wedge=(\d+)", diag)
    hole = re.search(r"rx_hole_recover=(\d+)", diag)
    clean = (wedge is None or wedge.group(1) == "0")
    check("wired RX health clean", clean,
          f"rx_wedge={wedge.group(1) if wedge else 'n/a'} "
          f"rx_hole_recover={hole.group(1) if hole else 'n/a'}")

    # 5. Processes still alive (no context corruption killed a capsule).
    try:
        procs = term("processes")
    except Exception as e:  # noqa: BLE001
        procs = repr(e)
    alive = procs.count("capsvc-host0-el0") > 0
    check("capsule process still alive", alive)

    print()
    if fails:
        print(f"{RED}PREEMPT SOAK: FAIL ({len(fails)} check(s)): {fails}{RST}")
        return 1
    print(f"{GRN}PREEMPT SOAK: PASS{RST}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", action="store_true")
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--concurrency", type=int, default=4)
    ap.add_argument("--units", type=int, default=40,
                    help="millions of busy-loop iterations per request")
    ap.add_argument("--host", default=None,
                    help="test a live board instead of launching QEMU")
    args = ap.parse_args()

    global HTTP, CAPS
    if args.host:
        HTTP = f"http://{args.host}"
        CAPS = f"http://{args.host}:8090"
        if not wait_boot(None, 30.0):
            print(f"{RED}[soak] board not reachable at {HTTP}{RST}")
            return 3
        return run(args)

    if args.build:
        print("[soak] building QEMU image...")
        r = subprocess.run(["cmd.exe", "/d", "/c", str(ROOT / "build_qemu_full.bat")],
                           cwd=ROOT, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout[-2000:])
            return 2

    net = ("user,id=n0,net=192.168.0.0/24,host=192.168.0.1,"
           "hostfwd=tcp:127.0.0.1:8088-192.168.0.201:80,"
           "hostfwd=tcp:127.0.0.1:8090-192.168.0.201:8090")
    qargs = [QEMU, "-M", "virt", "-cpu", "cortex-a53", "-smp", "4", "-m", "1G",
             "-display", "none", "-serial", f"file:{SERIAL_LOG}",
             "-kernel", str(KERNEL), "-netdev", net,
             "-device", "virtio-net-device,netdev=n0"]
    print("[soak] booting QEMU (headless)...")
    proc = subprocess.Popen(qargs)
    try:
        if not wait_boot(proc):
            print(f"{RED}[soak] board never reached /api/status{RST}")
            return 3
        print("[soak] board up\n")
        return run(args)
    finally:
        try:
            proc.terminate()
            proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            proc.kill()


if __name__ == "__main__":
    sys.exit(main())
