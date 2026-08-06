#!/usr/bin/env python3
"""One-shot UART probe: send a command, print everything received.
Usage: uart_probe.py [COM16] [115200] [command]"""
import sys, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM16"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
CMD  = sys.argv[3] if len(sys.argv) > 3 else "macbdiag"

try:
    sp = serial.Serial(PORT, BAUD, timeout=0.3)
except Exception as e:
    print(f"open failed: {e}")
    raise SystemExit(1)

sp.dtr = True
sp.rts = True
time.sleep(0.2)
sp.reset_input_buffer()

# wake the line, then send the command
sp.write(b"\r\n")
time.sleep(0.3)
sp.write((CMD + "\r\n").encode())
sp.flush()

deadline = time.time() + 3.0
got = bytearray()
while time.time() < deadline:
    chunk = sp.read(4096)
    if chunk:
        got += chunk
        deadline = time.time() + 1.0  # extend while data flows
sp.close()

print(f"RX {len(got)} bytes:")
if got:
    print(got.decode("utf-8", errors="replace"))
else:
    print("(nothing received — board TXD not reaching adapter RX)")
