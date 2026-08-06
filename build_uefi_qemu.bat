@echo off
set LLVM=C:\Program Files\LLVM\bin
set CC=%LLVM%\clang.exe
set LD=%LLVM%\lld-link.exe
set CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -fshort-wchar -fno-builtin -fno-stack-protector -target aarch64-unknown-windows -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT -Iinclude -O2

echo Building standalone QEMU payload...
call .\build_qemu_full.bat
if errorlevel 1 exit /b 1
set CC=%LLVM%\clang.exe
set LD=%LLVM%\lld-link.exe
set CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -fshort-wchar -fno-builtin -fno-stack-protector -target aarch64-unknown-windows -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT -Iinclude -O2

if exist build_uefi rmdir /S /Q build_uefi
mkdir build_uefi
mkdir build_uefi\EFI
mkdir build_uefi\EFI\BOOT

echo Packaging QEMU stage2...
python tools\build_stage2_package.py --qemu build_qemu_full\PIOS_QEMU_FULL.BIN --compress-qemu --out build_uefi\PIOSSTG2.PKG
if errorlevel 1 exit /b 1
if not exist build_uefi\PIOSSTG2.PKG (
    echo QEMU stage2 package was not created
    exit /b 1
)

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
echo Capture shared workbench: python tools\qemu_workbench_snapshot.py --build
echo Manual run requires: -device ramfb plus two virtio-blk devices (ESP + PIOS disk)
