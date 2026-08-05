#!/usr/bin/env python3
"""Boot the full QEMU kernel with ramfb and capture the shared workbench."""

from __future__ import annotations

import argparse
import binascii
import pathlib
import socket
import struct
import subprocess
import sys
import time
import zlib


REPO = pathlib.Path(__file__).resolve().parents[1]
QEMU = pathlib.Path(r"C:\Program Files\qemu\qemu-system-aarch64.exe")
UEFI = pathlib.Path(r"C:\Program Files\qemu\share\edk2-aarch64-code.fd")
EFI_DIR = REPO / "build_uefi"
EFI_APP = EFI_DIR / "EFI" / "BOOT" / "BOOTAA64.EFI"
DISK = REPO / "qemu_shared_workbench_disk.img"
SERIAL = REPO / "qemu_shared_workbench_serial.log"
PPM = REPO / "qemu_shared_workbench.ppm"
DEFAULT_PNG = REPO / "qemu_shared_workbench.png"
DISK_BYTES = 64 * 1024 * 1024
MONITOR_PORT = 4461
READY_MARKER = "[fb] UEFI GOP framebuffer online"


def run_build() -> None:
    result = subprocess.run(
        ["cmd", "/c", str(REPO / "build_uefi_qemu.bat")],
        cwd=REPO,
        text=True,
    )
    if result.returncode:
        raise SystemExit(result.returncode)


def ensure_disk(fresh: bool) -> None:
    if fresh or not DISK.exists() or DISK.stat().st_size != DISK_BYTES:
        with DISK.open("wb") as disk:
            disk.truncate(DISK_BYTES)


def serial_tail(lines: int = 40) -> str:
    if not SERIAL.exists():
        return "(serial log not created)"
    return "\n".join(SERIAL.read_text(errors="replace").splitlines()[-lines:])


def wait_for_ready(proc: subprocess.Popen[bytes], timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(
                f"QEMU exited before framebuffer initialization ({proc.returncode})\n"
                f"{serial_tail()}"
            )
        if SERIAL.exists() and READY_MARKER in SERIAL.read_text(errors="replace"):
            return
        time.sleep(0.5)
    raise RuntimeError(f"timed out waiting for {READY_MARKER!r}\n{serial_tail()}")


def monitor_command(command: str, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    last_error: OSError | None = None
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", MONITOR_PORT), timeout=2.0) as sock:
                sock.settimeout(2.0)
                try:
                    sock.recv(4096)
                except TimeoutError:
                    pass
                sock.sendall(command.encode("ascii") + b"\r\n")
                response = bytearray()
                response_deadline = time.monotonic() + 5.0
                while time.monotonic() < response_deadline:
                    try:
                        chunk = sock.recv(4096)
                    except TimeoutError:
                        continue
                    if not chunk:
                        break
                    response.extend(chunk)
                    if b"\r\n(qemu)" in response:
                        break
                return
        except OSError as exc:
            last_error = exc
            time.sleep(0.25)
    raise RuntimeError(f"QEMU monitor unavailable: {last_error}")


def ppm_tokens(data: bytes) -> tuple[list[bytes], int]:
    tokens: list[bytes] = []
    pos = 0
    while len(tokens) < 4:
        while pos < len(data) and data[pos] in b" \t\r\n":
            pos += 1
        if pos < len(data) and data[pos] == ord("#"):
            while pos < len(data) and data[pos] not in b"\r\n":
                pos += 1
            continue
        start = pos
        while pos < len(data) and data[pos] not in b" \t\r\n":
            pos += 1
        if start == pos:
            raise ValueError("truncated PPM header")
        tokens.append(data[start:pos])
    while pos < len(data) and data[pos] in b" \t\r\n":
        pos += 1
    return tokens, pos


def png_chunk(kind: bytes, payload: bytes) -> bytes:
    body = kind + payload
    return struct.pack(">I", len(payload)) + body + struct.pack(
        ">I", binascii.crc32(body) & 0xFFFFFFFF
    )


def convert_ppm_to_png(src: pathlib.Path, dst: pathlib.Path) -> None:
    data = src.read_bytes()
    tokens, offset = ppm_tokens(data)
    if tokens[0] != b"P6" or tokens[3] != b"255":
        raise ValueError("only 8-bit binary PPM (P6) is supported")
    width = int(tokens[1])
    height = int(tokens[2])
    row_bytes = width * 3
    pixels = data[offset:]
    if len(pixels) != row_bytes * height:
        raise ValueError(
            f"PPM payload size mismatch: {len(pixels)} != {row_bytes * height}"
        )
    scanlines = b"".join(
        b"\0" + pixels[row * row_bytes : (row + 1) * row_bytes]
        for row in range(height)
    )
    png = (
        b"\x89PNG\r\n\x1a\n"
        + png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + png_chunk(b"IDAT", zlib.compress(scanlines, 9))
        + png_chunk(b"IEND", b"")
    )
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(png)


def stop_qemu(proc: subprocess.Popen[bytes]) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture the full PIOS kernel's shared Pi5/QEMU workbench renderer."
    )
    parser.add_argument("--build", action="store_true", help="rebuild BOOTAA64.EFI first")
    parser.add_argument("--fresh-disk", action="store_true", help="replace the persistent disk")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--out", type=pathlib.Path, default=DEFAULT_PNG)
    args = parser.parse_args()

    if args.build:
        run_build()
    if not QEMU.exists():
        raise SystemExit(f"QEMU not found: {QEMU}")
    if not UEFI.exists():
        raise SystemExit(f"UEFI firmware not found: {UEFI}")
    if not EFI_APP.exists():
        raise SystemExit(f"UEFI app not found: {EFI_APP} (use --build)")

    ensure_disk(args.fresh_disk)
    for path in (SERIAL, PPM):
        path.unlink(missing_ok=True)

    netdev = (
        "user,id=n0,net=192.168.0.0/24,host=192.168.0.1,"
        "hostfwd=tcp:127.0.0.1:8088-192.168.0.201:80"
    )
    qargs = [
        str(QEMU),
        "-M", "virt,gic-version=2",
        "-cpu", "cortex-a53",
        "-smp", "4",
        "-m", "512M",
        "-display", "none",
        "-bios", str(UEFI),
        "-drive", f"if=none,file=fat:rw:{EFI_DIR},format=raw,id=esp",
        "-device", "virtio-blk-device,drive=esp",
        "-drive", f"if=none,file={DISK},format=raw,id=piosdisk",
        "-device", "virtio-blk-device,drive=piosdisk",
        "-device", "ramfb",
        "-netdev", netdev,
        "-device", "virtio-net-device,netdev=n0",
        "-serial", f"file:{SERIAL}",
        "-monitor", f"tcp:127.0.0.1:{MONITOR_PORT},server,nowait",
    ]

    proc = subprocess.Popen(qargs, cwd=REPO)
    try:
        wait_for_ready(proc, args.timeout)
        time.sleep(2.0)
        monitor_path = PPM.resolve().as_posix()
        monitor_command(f"screendump {monitor_path}")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if PPM.exists() and PPM.stat().st_size > 0:
                break
            time.sleep(0.25)
        if not PPM.exists() or PPM.stat().st_size == 0:
            raise RuntimeError("QEMU screendump was not created")
        convert_ppm_to_png(PPM, args.out.resolve())
        print(args.out.resolve())
        return 0
    except Exception as exc:
        print(f"qemu workbench capture failed: {exc}", file=sys.stderr)
        return 1
    finally:
        stop_qemu(proc)
        PPM.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
