@echo off
set LLVM=C:\Program Files\LLVM\bin
set CC=%LLVM%\clang.exe
set LD=%LLVM%\lld-link.exe
set CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -fshort-wchar -fno-builtin -fno-stack-protector -target aarch64-unknown-windows -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT -Iinclude -O2

echo Building Pi5 bootstrap and standalone QEMU payload...
call .\build_bootstrap.bat
if errorlevel 1 exit /b 1
set CC=%LLVM%\clang.exe
set LD=%LLVM%\lld-link.exe
set CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -fshort-wchar -fno-builtin -fno-stack-protector -target aarch64-unknown-windows -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT -Iinclude -O2

if exist build_uefi rmdir /S /Q build_uefi
mkdir build_uefi
mkdir build_uefi\EFI
mkdir build_uefi\EFI\BOOT

python tools\build_stage2_package.py --qemu build_qemu_full\PIOS_QEMU_FULL.BIN --compress-qemu --out build_uefi\PIOSSTG2.PKG
if errorlevel 1 exit /b 1

echo Building BOOTAA64.EFI...
"%CC%" %CFLAGS% -c uefi\bootaa64.c -o build_uefi\bootaa64.o
if errorlevel 1 exit /b 1
"%CC%" %CFLAGS% -c uefi\stage2_blob.S -o build_uefi\stage2_blob.o
if errorlevel 1 exit /b 1
"%LD%" /subsystem:efi_application /entry:efi_main /machine:arm64 /nodefaultlib /out:build_uefi\EFI\BOOT\BOOTAA64.EFI build_uefi\bootaa64.o build_uefi\stage2_blob.o
if errorlevel 1 exit /b 1

for %%f in (build_uefi\EFI\BOOT\BOOTAA64.EFI) do echo BOOTAA64.EFI size: %%~zf bytes
for %%f in (build_uefi\PIOSSTG2.PKG) do echo QEMU PIOSSTG2.PKG size: %%~zf bytes
echo Optional persistent disk: fsutil file createnew qemu_pios_disk.img 67108864
echo Run with: qemu-system-aarch64 -M virt,gic-version=2 -cpu cortex-a53 -m 512M -nographic -bios "C:\Program Files\qemu\share\edk2-aarch64-code.fd" -drive if=none,file=fat:rw:build_uefi,format=raw,id=esp -device virtio-blk-device,drive=esp -drive if=none,file=qemu_pios_disk.img,format=raw,id=piosdisk -device virtio-blk-device,drive=piosdisk
