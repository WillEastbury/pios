#!/usr/bin/env python3
"""Multithreaded real-world HTTP benchmark for PIOS.

Exercises actual PIOS HTTP/admin endpoints from several client threads and
summarises request latency, throughput, and errors. This is a host-side tool;
it does not require any changes on the Pi.
"""

from __future__ import annotations

import argparse
import http.client
import queue
import statistics
import threading
import time
import urllib.parse
from dataclasses import dataclass


@dataclass
class Endpoint:
    name: str
    port: int
    path: str


ENDPOINTS = [
    Endpoint("status", 80, "/api/status"),
    Endpoint("admin-status", 8080, "/?confirm=1"),
    Endpoint("netstat", 80, "/api/netstat"),
    Endpoint("proc-sched", 80, "/api/terminal?cmd=proc+sched"),
    Endpoint("mem", 80, "/api/terminal?cmd=mem+analyze"),
    Endpoint("dma", 80, "/api/terminal?cmd=dma+status"),
    Endpoint("ipc-bench-1k", 80, "/api/terminal?cmd=ipc+bench+1000"),
    Endpoint("update-status", 8082, "/?action=status&confirm=1"),
]


def request(host: str, endpoint: Endpoint, timeout: float) -> tuple[bool, float, int, int, str]:
    start = time.perf_counter()
    status = 0
    body_len = 0
    err = ""
    try:
        conn = http.client.HTTPConnection(host, endpoint.port, timeout=timeout)
        conn.request("GET", endpoint.path, headers={"Connection": "close"})
        resp = conn.getresponse()
        status = resp.status
        body = resp.read()
        body_len = len(body)
        conn.close()
        ok = status == 200 and body_len > 0
    except Exception as exc:  # host-side diagnostics: report exception text
        ok = False
        err = type(exc).__name__
    elapsed = time.perf_counter() - start
    return ok, elapsed, status, body_len, err


def worker(host: str, endpoints: list[Endpoint], stop_at: float, timeout: float,
           out: queue.Queue[tuple[str, bool, float, int, int, str]]) -> None:
    idx = 0
    while time.perf_counter() < stop_at:
        ep = endpoints[idx % len(endpoints)]
        idx += 1
        ok, elapsed, status, body_len, err = request(host, ep, timeout)
        out.put((ep.name, ok, elapsed, status, body_len, err))


def pct(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    k = int((len(ordered) - 1) * p)
    return ordered[k]


def main() -> int:
    ap = argparse.ArgumentParser(description="Run a multithreaded PIOS HTTP benchmark.")
    ap.add_argument("--host", default="192.168.0.201")
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--timeout", type=float, default=3.0)
    ap.add_argument("--endpoints", default="status,admin-status,netstat,proc-sched,mem,dma,update-status")
    args = ap.parse_args()

    selected = {x.strip() for x in args.endpoints.split(",") if x.strip()}
    endpoints = [ep for ep in ENDPOINTS if ep.name in selected]
    if not endpoints:
        raise SystemExit("no endpoints selected")
    if args.threads <= 0:
        raise SystemExit("--threads must be positive")

    q: queue.Queue[tuple[str, bool, float, int, int, str]] = queue.Queue()
    stop_at = time.perf_counter() + args.seconds
    threads = [
        threading.Thread(target=worker, args=(args.host, endpoints, stop_at, args.timeout, q), daemon=True)
        for _ in range(args.threads)
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    rows: list[tuple[str, bool, float, int, int, str]] = []
    while not q.empty():
        rows.append(q.get())

    total = len(rows)
    ok_rows = [r for r in rows if r[1]]
    elapsed = args.seconds
    lat_all = [r[2] for r in rows]
    print(f"PIOS HTTP bench host={args.host} threads={args.threads} seconds={args.seconds:.1f}")
    print(f"requests={total} ok={len(ok_rows)} errors={total - len(ok_rows)} rps={total / elapsed:.2f}")
    if lat_all:
        print(
            "latency_ms "
            f"avg={statistics.mean(lat_all) * 1000:.2f} "
            f"p50={pct(lat_all, 0.50) * 1000:.2f} "
            f"p95={pct(lat_all, 0.95) * 1000:.2f} "
            f"max={max(lat_all) * 1000:.2f}"
        )

    by_name: dict[str, list[tuple[str, bool, float, int, int, str]]] = {}
    for row in rows:
        by_name.setdefault(row[0], []).append(row)
    print("endpoint ok/total avg_ms p95_ms bytes errors")
    for name in sorted(by_name):
        vals = by_name[name]
        lats = [r[2] for r in vals]
        oks = sum(1 for r in vals if r[1])
        bytes_total = sum(r[4] for r in vals)
        errs: dict[str, int] = {}
        for r in vals:
            if not r[1]:
                key = r[5] or f"HTTP{r[3]}"
                errs[key] = errs.get(key, 0) + 1
        err_s = ",".join(f"{k}:{v}" for k, v in sorted(errs.items())) or "-"
        print(
            f"{name} {oks}/{len(vals)} "
            f"{statistics.mean(lats) * 1000:.2f} {pct(lats, 0.95) * 1000:.2f} "
            f"{bytes_total} {err_s}"
        )
    return 0 if total == len(ok_rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
