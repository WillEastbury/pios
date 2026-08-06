@echo off
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8-a -mgeneral-regs-only -Iinclude -O2 -fno-builtin -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT
set ASFLAGS=-march=armv8-a -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT

if exist build_qemu rmdir /S /Q build_qemu
mkdir build_qemu

echo Building qemu-virt minimal image...
"%CC%" %ASFLAGS% -c src\qemu_virt_start.S -o build_qemu\qemu_virt_start.o
if errorlevel 1 exit /b 1
"%CC%" %CFLAGS% -c src\qemu_virt_min.c -o build_qemu\qemu_virt_min.o
if errorlevel 1 exit /b 1
"%LD%" -T link_qemu_virt.ld -nostdlib -o qemu_virt.elf build_qemu\qemu_virt_start.o build_qemu\qemu_virt_min.o
if errorlevel 1 exit /b 1
"%OC%" -O binary qemu_virt.elf qemu_virt.img
if errorlevel 1 exit /b 1

for %%f in (qemu_virt.elf) do echo qemu_virt.elf size: %%~zf bytes
for %%f in (qemu_virt.img) do echo qemu_virt.img size: %%~zf bytes
echo Run with: qemu-system-aarch64 -M virt,gic-version=2 -cpu cortex-a53 -nographic -kernel qemu_virt.elf
