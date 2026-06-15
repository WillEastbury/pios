@echo off
set LLVM=C:\Program Files\LLVM\bin
set CC=%LLVM%\clang.exe
set LD=%LLVM%\lld-link.exe
set GCC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin\aarch64-none-elf-gcc.exe
set GLD=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin\aarch64-none-elf-ld.exe
set GOC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin\aarch64-none-elf-objcopy.exe
set CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -fshort-wchar -fno-builtin -fno-stack-protector -target aarch64-unknown-windows -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT -Iinclude -O2
set STAGE2_CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8-a -mgeneral-regs-only -Iinclude -O2 -fno-builtin -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT

if exist build_uefi rmdir /S /Q build_uefi
mkdir build_uefi
mkdir build_uefi\EFI
mkdir build_uefi\EFI\BOOT

echo Building common QEMU stage2 image...
"%GCC%" -march=armv8-a -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT -c src\qemu_stage2_start.S -o build_uefi\qemu_stage2_start.o
if errorlevel 1 exit /b 1
"%GCC%" -march=armv8-a -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT -c src\qemu_stage2_manifest.S -o build_uefi\qemu_stage2_manifest.o
if errorlevel 1 exit /b 1
for %%f in (qemu_stage2_os) do (
    "%GCC%" %STAGE2_CFLAGS% -c uefi\%%f.c -o build_uefi\%%f.o
    if errorlevel 1 exit /b 1
)
"%GCC%" %STAGE2_CFLAGS% -c src\sd.c -o build_uefi\sd.o
if errorlevel 1 exit /b 1
"%GCC%" %STAGE2_CFLAGS% -c src\bcache.c -o build_uefi\bcache.o
if errorlevel 1 exit /b 1
"%GCC%" %STAGE2_CFLAGS% -c src\walfs.c -o build_uefi\walfs.o
if errorlevel 1 exit /b 1
"%GCC%" %STAGE2_CFLAGS% -c src\lru.c -o build_uefi\lru.o
if errorlevel 1 exit /b 1
"%GLD%" -T link_qemu_virt.ld -nostdlib -o build_uefi\PIOSSTG2.ELF build_uefi\qemu_stage2_start.o build_uefi\qemu_stage2_manifest.o build_uefi\qemu_stage2_os.o build_uefi\sd.o build_uefi\bcache.o build_uefi\walfs.o build_uefi\lru.o
if errorlevel 1 exit /b 1
"%GOC%" -O binary build_uefi\PIOSSTG2.ELF build_uefi\PIOSSTG2.BIN
if errorlevel 1 exit /b 1

echo Building BOOTAA64.EFI...
"%CC%" %CFLAGS% -c uefi\bootaa64.c -o build_uefi\bootaa64.o
if errorlevel 1 exit /b 1
"%CC%" %CFLAGS% -c uefi\stage2_blob.S -o build_uefi\stage2_blob.o
if errorlevel 1 exit /b 1
"%LD%" /subsystem:efi_application /entry:efi_main /machine:arm64 /nodefaultlib /out:build_uefi\EFI\BOOT\BOOTAA64.EFI build_uefi\bootaa64.o build_uefi\stage2_blob.o
if errorlevel 1 exit /b 1

for %%f in (build_uefi\EFI\BOOT\BOOTAA64.EFI) do echo BOOTAA64.EFI size: %%~zf bytes
for %%f in (build_uefi\PIOSSTG2.BIN) do echo PIOSSTG2.BIN size: %%~zf bytes
echo Run with: qemu-system-aarch64 -M virt,gic-version=2 -cpu cortex-a53 -m 512M -nographic -bios "C:\Program Files\qemu\share\edk2-aarch64-code.fd" -drive if=none,file=fat:rw:build_uefi,format=raw,id=esp -device virtio-blk-device,drive=esp
