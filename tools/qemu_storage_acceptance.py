#!/usr/bin/env python3
"""Real QEMU acceptance for isolated FAT32 p3 and integrated WALFS p2.

This is deliberately independent from qemu_smoke.py.  It boots the direct
QEMU kernel with a persistent disk made by ``build_qemu_disk_image.py
--exchange`` and drives real HTTP terminal operations.  No host filesystem
mount or fake block callback is used.

Usage:
  python tools\\qemu_storage_acceptance.py [--build] [--keep-artifacts]
"""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import socket
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request


REPO = pathlib.Path(__file__).resolve().parent.parent
QEMU = pathlib.Path(r"C:\Program Files\qemu\qemu-system-aarch64.exe")
KERNEL = REPO / "build_qemu_full" / "PIOS_QEMU_FULL.BIN"
DISK = REPO / "qemu_storage_acceptance.img"
SERIAL_LOG = REPO / "qemu_storage_acceptance_serial.log"
HTTP = "http://127.0.0.1:8188"
OTA_HOST, OTA_PORT = "127.0.0.1", 8182
MAX_SECONDS = 10.0


class Acceptance:
    def __init__(self) -> None:
        self.latencies: list[float] = []
        self.operations = 0
        self.errors: list[str] = []
        self.lock = threading.Lock()

    def command(self, command: str, expect: str | None = None,
                timeout: float = MAX_SECONDS) -> str:
        started = time.monotonic()
        try:
            url = f"{HTTP}/api/terminal?cmd={urllib.parse.quote(command)}"
            with urllib.request.urlopen(url, timeout=timeout) as response:
                body = response.read().decode("utf-8", "replace")
            elapsed = time.monotonic() - started
            if elapsed > MAX_SECONDS:
                raise RuntimeError(f"exceeded {MAX_SECONDS:.0f}s ({elapsed:.3f}s)")
            if expect is not None and expect not in body:
                raise RuntimeError(f"expected {expect!r}, got {body[:240]!r}")
            with self.lock:
                self.operations += 1
                self.latencies.append(elapsed)
            return body
        except Exception as exc:
            with self.lock:
                self.operations += 1
                self.errors.append(f"{command}: {exc}")
            raise

    def get_status(self) -> None:
        started = time.monotonic()
        try:
            with urllib.request.urlopen(f"{HTTP}/api/status", timeout=MAX_SECONDS) as response:
                body = response.read().decode("utf-8", "replace")
            elapsed = time.monotonic() - started
            if response.status != 200 or '"version"' not in body or elapsed > MAX_SECONDS:
                raise RuntimeError(f"status invalid or slow ({elapsed:.3f}s)")
            with self.lock:
                self.operations += 1
                self.latencies.append(elapsed)
        except Exception as exc:
            with self.lock:
                self.operations += 1
                self.errors.append(f"status: {exc}")


def wait_boot(proc: subprocess.Popen[bytes], timeout: float = 75.0) -> bool:
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if proc.poll() is not None:
            return False
        try:
            with urllib.request.urlopen(f"{HTTP}/api/status", timeout=3) as response:
                if response.status == 200:
                    return True
        except Exception:
            time.sleep(1)
    return False


def launch(kernel: pathlib.Path, disk: pathlib.Path) -> subprocess.Popen[bytes]:
    net = (
        "user,id=n0,net=192.168.0.0/24,host=192.168.0.1,"
        "hostfwd=tcp:127.0.0.1:8188-192.168.0.201:80,"
        "hostfwd=tcp:127.0.0.1:8182-192.168.0.201:8082"
    )
    args = [
        str(QEMU), "-M", "virt", "-cpu", "cortex-a76", "-smp", "4", "-m", "1G",
        "-display", "none", "-serial", f"file:{SERIAL_LOG}", "-kernel", str(kernel),
        "-drive", f"if=none,format=raw,file={disk},id=hd0",
        "-device", "virtio-blk-device,drive=hd0",
        # sd.c intentionally requires two block devices on QEMU. Both refer
        # to this one acceptance disk; only the first discovered device is used.
        "-drive", f"if=none,format=raw,file={disk},id=hd1",
        "-device", "virtio-blk-device,drive=hd1",
        "-netdev", net, "-device", "virtio-net-device,netdev=n0",
    ]
    return subprocess.Popen(args, cwd=REPO)


def stop(proc: subprocess.Popen[bytes] | None) -> None:
    if proc is None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=8)


def ota_stream() -> None:
    image = bytes((index * 29 + 11) & 0xFF for index in range(16 * 1024))
    request = (
        f"POST /api/admin/kernel-stream?confirm=1&total={len(image)} HTTP/1.0\r\n"
        f"Host: {OTA_HOST}\r\nConnection: close\r\n"
        f"Content-Type: application/octet-stream\r\nContent-Length: {len(image)}\r\n\r\n"
    ).encode("ascii")
    with socket.create_connection((OTA_HOST, OTA_PORT), timeout=MAX_SECONDS) as sock:
        sock.settimeout(MAX_SECONDS)
        sock.sendall(request + image)
        response = bytearray()
        while len(response) < 8192:
            chunk = sock.recv(4096)
            if not chunk:
                break
            response.extend(chunk)
    if b'"ok":true' not in response:
        raise RuntimeError(f"OTA response missing ok: {bytes(response[-240:])!r}")


def exercise_fat(a: Acceptance) -> None:
    first = "00112233445566778899AABBCCDDEEFF"
    rewrite = "DEADBEEFCAFEBABE"
    tail = "01020304A0A1A2A3"
    a.command("xfer status", "available=yes")
    a.command(f"xfer write ACC.TXT {first}", "xfer write OK")
    a.command("xfer read ACC.TXT", f"hex={first}")
    a.command(f"xfer write ACC.TXT {rewrite}", "xfer write OK")
    a.command(f"xfer append ACC.TXT {tail}", "xfer append OK")
    a.command("xfer read ACC.TXT", f"hex={rewrite}{tail}")
    a.command("xfer rename ACC.TXT RENAME.TXT", "xfer rename OK")
    a.command("xfer read RENAME.TXT", f"hex={rewrite}{tail}")
    a.command("xfer verify", "xfer verify OK")


def exercise_walfs(a: Acceptance) -> None:
    a.command("db add 620 1 77616c66732d6163636570742d6f6e65", "OK: wrote")
    a.command("db get 620 1", "walfs-accept-one")
    a.command("db update 620 1 77616c667332", "OK: wrote")
    a.command("db get 620 1", "walfs2")
    a.command("db copy 620 1 620 2", "OK: copied")
    a.command("db get 620 2", "walfs2")
    a.command("db rename 620 2 620 3", "OK: moved")
    a.command("db del 620 1", "OK: deleted")
    a.command("walfs verify", "walfs verify ok=yes")


def pressure(a: Acceptance) -> None:
    stop = threading.Event()

    def status_worker() -> None:
        while not stop.is_set():
            a.get_status()
            time.sleep(0.15)

    def ipc_worker() -> None:
        try:
            a.command("ipc bench 64", "ipc bench OK")
        except Exception:
            pass

    def ota_worker() -> None:
        started = time.monotonic()
        try:
            ota_stream()
            elapsed = time.monotonic() - started
            with a.lock:
                a.operations += 1
                a.latencies.append(elapsed)
                if elapsed > MAX_SECONDS:
                    a.errors.append(f"OTA exceeded {MAX_SECONDS:.0f}s ({elapsed:.3f}s)")
        except Exception as exc:
            with a.lock:
                a.operations += 1
                a.errors.append(f"OTA: {exc}")

    threads = [
        threading.Thread(target=status_worker, daemon=True),
        threading.Thread(target=ipc_worker, daemon=True),
        threading.Thread(target=ota_worker, daemon=True),
    ]
    for thread in threads:
        thread.start()
    try:
        for index in range(12):
            name = f"P{index:02d}.BIN"
            payload = bytes((index * 17 + byte) & 0xFF for byte in range(48)).hex().upper()
            tail = bytes((0xE0 + byte) & 0xFF for byte in range(32)).hex().upper()
            a.command(f"xfer write {name} {payload}", "xfer write OK")
            a.command(f"xfer append {name} {tail}", "xfer append OK")
            a.command(f"xfer read {name}", f"hex={payload}{tail}")
            a.command(f"db save 621 {100 + index} {payload}", "OK: wrote")
            a.command(f"db get 621 {100 + index}", "len=48")
    finally:
        stop.set()
        for thread in threads:
            thread.join(timeout=MAX_SECONDS)
    a.command("xfer verify", "xfer verify OK")
    a.command("walfs verify", "walfs verify ok=yes")


def persistence(a: Acceptance) -> None:
    a.command("xfer verify", "xfer verify OK")
    a.command("xfer read RENAME.TXT", "hex=DEADBEEFCAFEBABE01020304A0A1A2A3")
    a.command("db get 620 3", "walfs2")
    a.command("walfs verify", "walfs verify ok=yes")
    # Prove deletion independent of all prior mutations, then re-verify p3.
    a.command("xfer delete RENAME.TXT", "xfer delete OK")
    a.command("xfer read RENAME.TXT", "ERR: xfer read fat32-failed")
    a.command("xfer verify", "xfer verify OK")


def reject_exchange_p3(disk: pathlib.Path) -> None:
    """Leave damaged FAT residue that must not be mistaken for blank raw p3."""
    p3_start = 2048 + 64 * 1024 * 1024 // 512 + 96 * 1024 * 1024 // 512
    with disk.open("r+b") as image:
        image.seek(p3_start * 512 + 44)
        image.write(b"\x00" * 4)  # destroy the FAT32 root-cluster field
        image.seek(p3_start * 512 + 71)
        image.write(b"\x00" * 11)  # remove the volume label
        image.seek(p3_start * 512 + 82)
        image.write(b"\x00" * 8)  # remove FAT32 type marker
        image.seek(p3_start * 512 + 510)
        image.write(b"\x00\x00")  # remove terminal boot signature
        image.flush()

def relabel_exchange_p3(disk: pathlib.Path) -> None:
    """Give an otherwise-valid formatted p3 a non-PIOSXFER label."""
    p3_start = 2048 + 64 * 1024 * 1024 // 512 + 96 * 1024 * 1024 // 512
    with disk.open("r+b") as image:
        image.seek(p3_start * 512 + 71)
        image.write(b"USER VOLUME")
        image.flush()

def digest_range(path: pathlib.Path, first_sector: int, sectors: int) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as image:
        image.seek(first_sector * 512)
        remaining = sectors * 512
        while remaining:
            data = image.read(min(1024 * 1024, remaining))
            if not data:
                raise RuntimeError("short disk image while hashing")
            digest.update(data)
            remaining -= len(data)
    return digest.hexdigest()


def exercise_rejected_exchange(a: Acceptance) -> None:
    a.command("exchange status", "exchange available=no")
    a.command("xfer status", "xfer available=no")
    a.command("xfer verify", "ERR: xfer verify unavailable")
    a.command("walfs verify", "walfs verify ok=yes")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true", help="build direct QEMU kernel")
    parser.add_argument("--keep-artifacts", action="store_true",
                        help="keep disk and serial log after success")
    args = parser.parse_args()
    a = Acceptance()
    proc: subprocess.Popen[bytes] | None = None
    success = False
    try:
        if not QEMU.exists():
            raise RuntimeError(f"QEMU unavailable: {QEMU}")
        if args.build:
            build = subprocess.run(["cmd.exe", "/d", "/c", str(REPO / "build_qemu_full.bat")],
                                   cwd=REPO, capture_output=True, text=True)
            if build.returncode != 0 or "BUILD COMPLETE" not in build.stdout:
                raise RuntimeError(f"QEMU build failed:\n{build.stdout[-3000:]}"
                                   f"\n{build.stderr[-3000:]}")
        if not KERNEL.exists():
            raise RuntimeError(f"kernel missing: {KERNEL}; rerun with --build")
        for artifact in (DISK, SERIAL_LOG):
            artifact.unlink(missing_ok=True)
        builder = subprocess.run(
            [sys.executable, str(REPO / "tools" / "build_qemu_disk_image.py"),
             "--pkg", str(KERNEL), "--out", str(DISK), "--disk-id", "0x71584F52",
             "--exchange-raw"],
            cwd=REPO, capture_output=True, text=True,
        )
        if builder.returncode != 0 or not DISK.exists():
            raise RuntimeError(f"disk build failed:\n{builder.stdout}\n{builder.stderr}")
        print(builder.stdout.strip())

        proc = launch(KERNEL, DISK)
        if not wait_boot(proc):
            raise RuntimeError("QEMU never reached /api/status")
        a.command("exchange status", "exchange available=yes")
        a.command("exchange status", "formatted=yes")
        a.command("walfs format confirm", "WALFS format OK")
        # Formatting is explicitly requested and initialization-dependent
        # services (principals/setup) come up only on the next boot.
        stop(proc)
        proc = launch(KERNEL, DISK)
        if not wait_boot(proc):
            raise RuntimeError("QEMU did not boot after explicit WALFS format")
        exercise_fat(a)
        exercise_walfs(a)
        pressure(a)
        stop(proc)
        proc = None

        # A fresh QEMU process against the same disk is the remount/persistence
        # proof; no host process ever mounts the image.
        proc = launch(KERNEL, DISK)
        if not wait_boot(proc):
            raise RuntimeError("QEMU persistence reboot never reached /api/status")
        persistence(a)
        stop(proc)
        proc = None

        # A valid FAT32 p3 attaches regardless of its descriptive volume
        # label, and boot must not rewrite it.
        relabel_exchange_p3(DISK)
        p3_start = 2048 + 64 * 1024 * 1024 // 512 + 96 * 1024 * 1024 // 512
        p3_before = digest_range(DISK, p3_start, 64 * 1024 * 1024 // 512)
        proc = launch(KERNEL, DISK)
        if not wait_boot(proc):
            raise RuntimeError("QEMU unlabeled-p3 reboot never reached /api/status")
        a.command("exchange status", "exchange available=yes")
        a.command("exchange status", "formatted=no")
        if digest_range(DISK, p3_start, 64 * 1024 * 1024 // 512) != p3_before:
            raise RuntimeError("valid unlabeled p3 was rewritten")
        stop(proc)
        proc = None

        reject_exchange_p3(DISK)
        p3_corrupt = digest_range(DISK, p3_start, 64 * 1024 * 1024 // 512)
        protected_before = digest_range(DISK, 0, p3_start)
        proc = launch(KERNEL, DISK)
        if not wait_boot(proc):
            raise RuntimeError("QEMU corrupt-p3 reboot never reached /api/status")
        exercise_rejected_exchange(a)
        if digest_range(DISK, p3_start, 64 * 1024 * 1024 // 512) != p3_corrupt:
            raise RuntimeError("corrupt p3 was overwritten")
        if digest_range(DISK, 0, p3_start) != protected_before:
            raise RuntimeError("p1/p2 changed during corrupt-p3 boot")
        if a.errors:
            raise RuntimeError("; ".join(a.errors))
        average = sum(a.latencies) / len(a.latencies) if a.latencies else 0.0
        maximum = max(a.latencies, default=0.0)
        print(f"STORAGE ACCEPTANCE PASS operations={a.operations} "
              f"avg_ms={average * 1000:.1f} max_ms={maximum * 1000:.1f}")
        success = True
        return 0
    except Exception as exc:
        print(f"STORAGE ACCEPTANCE FAIL: {exc}", file=sys.stderr)
        if a.errors:
            print("operation errors:", *a.errors, sep="\n  ", file=sys.stderr)
        if SERIAL_LOG.exists():
            print(f"serial log retained: {SERIAL_LOG}", file=sys.stderr)
        if DISK.exists():
            print(f"disk retained: {DISK}", file=sys.stderr)
        return 1
    finally:
        stop(proc)
        if success and not args.keep_artifacts:
            DISK.unlink(missing_ok=True)
            SERIAL_LOG.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
