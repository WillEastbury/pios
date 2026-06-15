@echo off
set LLVM=C:\Program Files\LLVM\bin
set CC=%LLVM%\clang.exe
set LD=%LLVM%\lld-link.exe
set CFLAGS=-Wall -Wextra -ffreestanding -fshort-wchar -fno-builtin -fno-stack-protector -target aarch64-unknown-windows -O2

if exist build_uefi rmdir /S /Q build_uefi
mkdir build_uefi
mkdir build_uefi\EFI
mkdir build_uefi\EFI\BOOT

echo Building BOOTAA64.EFI...
"%CC%" %CFLAGS% -c uefi\bootaa64.c -o build_uefi\bootaa64.o
if errorlevel 1 exit /b 1
"%LD%" /subsystem:efi_application /entry:efi_main /machine:arm64 /nodefaultlib /out:build_uefi\EFI\BOOT\BOOTAA64.EFI build_uefi\bootaa64.o
if errorlevel 1 exit /b 1

for %%f in (build_uefi\EFI\BOOT\BOOTAA64.EFI) do echo BOOTAA64.EFI size: %%~zf bytes
echo Run with: qemu-system-aarch64 -M virt,gic-version=2 -cpu cortex-a53 -m 512M -nographic -bios "C:\Program Files\qemu\share\edk2-aarch64-code.fd" -drive if=none,file=fat:rw:build_uefi,format=raw,id=esp -device virtio-blk-device,drive=esp
