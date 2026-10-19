#!/usr/bin/env python3
"""Run one bounded CCS store proof on an already registered native B50 context."""
import argparse
import json
from pathlib import Path
import re
import time
import urllib.parse
import urllib.request


class ProbeError(RuntimeError):
    pass


class Board:
    def __init__(self, host, log):
        self.base = f"http://{host}"
        self.log = log

    def fetch(self, path):
        with urllib.request.urlopen(self.base + path, timeout=5) as response:
            data = response.read(65537)
        if len(data) > 65536:
            raise ProbeError("response exceeds probe budget")
        text = data.decode("utf-8")
        self.log.write(json.dumps({"path": path, "response": text}) + "\n")
        self.log.flush()
        return text

    def status(self):
        return json.loads(self.fetch("/api/status"))

    def command(self, command):
        return self.fetch("/api/terminal?cmd=" + urllib.parse.quote(command))


def healthy(before, current):
    if (current["version"] != before["version"] or
            current["uptime"] < before["uptime"]):
        raise ProbeError("board rebooted during probe")
    if current["diag"]["error"] or current["perf"]["nic_rx_wedge"]:
        raise ProbeError("management health gate failed")


def submit_fields(diag):
    match = re.search(
        r"submit state=(\d+) response=([0-9A-Fa-f]{8}) failure=(\d+) "
        r"result=([0-9A-Fa-f]{8}) completion=([0-9A-Fa-f]{8})", diag)
    if not match:
        raise ProbeError("missing structured submission diagnostics")
    return (int(match[1]), int(match[3]), int(match[4], 16),
            int(match[5], 16))


def probe(board, now=time.monotonic, sleep=time.sleep):
    before = board.status()
    healthy(before, before)
    topology = board.command("pcie1 status")
    if not re.search(r"\bb50=1\b", topology):
        raise ProbeError("B50 absent; external power-domain recovery required")
    diag = board.command("b50 diag")
    if not re.search(r"b50 diag attached=1\b", diag):
        raise ProbeError("native attachment required")
    if not re.search(r"context=4/F0000000/0/\d+", diag):
        raise ProbeError("successful CCS context registration required")
    state, failure, _, _ = submit_fields(diag)
    if state != 0 or failure:
        raise ProbeError("fresh, unsubmitted context required")
    healthy(before, board.status())
    deadline = now() + 5
    try:
        reply = board.command("b50 submit")
        if "bounded CCS0 store-only submission scheduled" not in reply:
            raise ProbeError("submission refused: " + reply.strip())
        while now() < deadline:
            diag = board.command("b50 diag")
            healthy(before, board.status())
            state, failure, result, completion = submit_fields(diag)
            if failure:
                raise ProbeError("CCS submission failed; snapshot:\n" + diag)
            if state == 5:
                if result != 42 or completion != 0xFEED0001:
                    raise ProbeError("completion/result mismatch:\n" + diag)
                return diag
            if "b50 diag attached=0" in diag:
                raise ProbeError("native lifecycle quarantined; snapshot:\n" + diag)
            sleep(0.1)
        raise ProbeError("probe deadline expired; snapshot:\n" + diag)
    finally:
        # Capture above before revocation clears hardware state. Even a passing
        # store proof is not authority to leave an experimental engine running.
        board.command("b50 revoke")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.0.201")
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    try:
        with args.log.open("x", encoding="utf-8") as log:
            snapshot = probe(Board(args.host, log))
        print("CCS store proof PASS (42/FEED0001); authority revoked\n" + snapshot)
    except (ProbeError, OSError, ValueError) as error:
        parser.exit(1, f"CCS store proof FAILED: {error}\n")


if __name__ == "__main__":
    main()
