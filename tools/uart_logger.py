#!/usr/bin/env python3
"""Continuous timestamped serial logger for the PIOS board UART (COM16, 115200).
Reads raw bytes, appends decoded text (with timestamps per line) to a log file,
and echoes to stdout. Robust against transient read errors."""
import sys, time, datetime, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM16"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
LOG = sys.argv[3] if len(sys.argv) > 3 else r"C:\source\pios\uart_capture.log"

def ts():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

def main():
    try:
        sp = serial.Serial(PORT, BAUD, timeout=0.2)
    except Exception as e:
        print(f"[logger] open {PORT} failed: {e}", flush=True)
        return 1
    print(f"[logger] {PORT}@{BAUD} -> {LOG}", flush=True)
    buf = b""
    with open(LOG, "a", encoding="utf-8", errors="replace") as f:
        f.write(f"\n==== capture start {ts()} {PORT}@{BAUD} ====\n")
        f.flush()
        while True:
            try:
                data = sp.read(4096)
            except Exception as e:
                line = f"[{ts()}] READ ERR: {e}\n"
                f.write(line); f.flush(); print(line, end="", flush=True)
                time.sleep(0.5); continue
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").rstrip("\r")
                out = f"[{ts()}] {text}\n"
                f.write(out); f.flush()
                print(out, end="", flush=True)

if __name__ == "__main__":
    raise SystemExit(main())
