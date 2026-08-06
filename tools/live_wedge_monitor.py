#!/usr/bin/env python3
"""Live wedge monitor: repeatedly polls macbdiag + nic counters over HTTP
(short timeout, tight loop) and logs every successful read with a
timestamp, so we can correlate exactly when nic counters freeze against
the low-level MAC/DMA ring state (rx_owned, RSR, ETH_CFG_STAT, rx_wedge,
rx_live_recover) during a live, in-progress NIC RX wedge -- without
requiring a UART connection or the newer unified-console build.
"""
import time
import urllib.request

HOST = "192.168.0.201"


def fetch(cmd, timeout=2.5):
    url = f"http://{HOST}/api/terminal?cmd={cmd.replace(' ', '%20')}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.read().decode("ascii", "replace").strip()
    except Exception as exc:
        return f"ERR:{exc}"


def main():
    last_processed = None
    freeze_start = None
    t0 = time.time()
    print(f"[monitor] starting at {time.strftime('%H:%M:%S')}")
    while time.time() - t0 < 600:  # run for up to 10 minutes
        ts = time.strftime("%H:%M:%S")
        nic = fetch("nic counters")
        mb = fetch("macbdiag")
        ok_nic = not nic.startswith("ERR:")
        ok_mb = not mb.startswith("ERR:")
        if ok_nic:
            # parse processed=NNN
            try:
                proc = int(nic.split("processed=")[1].split()[0])
            except Exception:
                proc = None
            if proc is not None:
                if proc == last_processed:
                    if freeze_start is None:
                        freeze_start = time.time()
                    frozen_s = time.time() - freeze_start
                else:
                    if freeze_start is not None:
                        print(f"[{ts}] UNFROZE after {time.time()-freeze_start:.1f}s (processed {last_processed}->{proc})")
                    freeze_start = None
                    frozen_s = 0.0
                last_processed = proc
                tag = f"FROZEN({frozen_s:.0f}s)" if freeze_start else "moving"
                print(f"[{ts}] nic={tag} {nic}")
        else:
            print(f"[{ts}] nic UNREACHABLE: {nic}")
        if ok_mb:
            print(f"[{ts}] mb: {mb}")
        else:
            print(f"[{ts}] mb UNREACHABLE: {mb}")
        time.sleep(1.0)
    print("[monitor] done")


if __name__ == "__main__":
    main()
