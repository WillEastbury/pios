@echo off
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2 -fno-builtin -DPIOS_PLATFORM=PIOS_PLATFORM_FVP_A76_GICV2
set ASFLAGS=-march=armv8.2-a+simd+crc+crypto -DPIOS_PLATFORM=PIOS_PLATFORM_FVP_A76_GICV2

if exist build_fvp_a76 rmdir /S /Q build_fvp_a76
mkdir build_fvp_a76
"%CC%" %ASFLAGS% -c src\qemu_virt_start.S -o build_fvp_a76\start.o || exit /b 1
"%CC%" %CFLAGS% -c src\qemu_virt_min.c -o build_fvp_a76\main.o || exit /b 1
"%LD%" -T link_fvp_a76_gicv2.ld -nostdlib -o build_fvp_a76\pios_fvp_a76_gicv2.elf build_fvp_a76\start.o build_fvp_a76\main.o || exit /b 1
"%OC%" -O binary build_fvp_a76\pios_fvp_a76_gicv2.elf build_fvp_a76\pios_fvp_a76_gicv2.bin || exit /b 1
echo FVP A76/GICv2 BUILD COMPLETE
