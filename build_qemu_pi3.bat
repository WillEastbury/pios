@echo off
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set FLAGS=-ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8-a -mgeneral-regs-only -Iinclude -O2 -DPIOS_PLATFORM=PIOS_PLATFORM_PI3
if exist build_qemu_pi3 rmdir /S /Q build_qemu_pi3
mkdir build_qemu_pi3
"%CC%" %FLAGS% -c src\qemu_virt_start.S -o build_qemu_pi3\start.o || exit /b 1
"%CC%" %FLAGS% -c src\pi3_qemu_min.c -o build_qemu_pi3\main.o || exit /b 1
"%LD%" -T link_qemu_pi3.ld -nostdlib -o build_qemu_pi3\pios_qemu_pi3.elf build_qemu_pi3\start.o build_qemu_pi3\main.o || exit /b 1
"%OC%" -O binary build_qemu_pi3\pios_qemu_pi3.elf build_qemu_pi3\pios_qemu_pi3.img || exit /b 1
echo QEMU PI3 BUILD COMPLETE
