@echo off
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8-a -mgeneral-regs-only -Iinclude -O2 -fno-builtin -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT
set ASFLAGS=-march=armv8-a -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT

if exist build_qemu_walfs rmdir /S /Q build_qemu_walfs
mkdir build_qemu_walfs

echo Building qemu-virt RAM WALFS smoke image...
"%CC%" %ASFLAGS% -c src\qemu_virt_start.S -o build_qemu_walfs\qemu_virt_start.o
if errorlevel 1 exit /b 1
for %%f in (qemu_virt_walfs sd bcache walfs lru) do (
    "%CC%" %CFLAGS% -c src\%%f.c -o build_qemu_walfs\%%f.o
    if errorlevel 1 exit /b 1
)
"%LD%" -T link_qemu_virt.ld -nostdlib -o qemu_walfs.elf build_qemu_walfs\qemu_virt_start.o build_qemu_walfs\qemu_virt_walfs.o build_qemu_walfs\sd.o build_qemu_walfs\bcache.o build_qemu_walfs\walfs.o build_qemu_walfs\lru.o
if errorlevel 1 exit /b 1
"%OC%" -O binary qemu_walfs.elf qemu_walfs.img
if errorlevel 1 exit /b 1

for %%f in (qemu_walfs.elf) do echo qemu_walfs.elf size: %%~zf bytes
for %%f in (qemu_walfs.img) do echo qemu_walfs.img size: %%~zf bytes
echo Run with: qemu-system-aarch64 -M virt,gic-version=2 -cpu cortex-a53 -nographic -kernel qemu_walfs.elf
