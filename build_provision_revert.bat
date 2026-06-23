@echo off
REM Rebuild the SD-swap provisioner that embeds revert_stage2.bin and writes it
REM to A/B slot A, then halts with on-screen SUCCESS. NC-boot via start_head.S.
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set BOOT_CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -mgeneral-regs-only -Iinclude -O2 -DPIOS_FB_NO_DOUBLE_BUFFER
set ASFLAGS=-march=armv8.2-a+simd+crc+crypto

if not exist build_boot mkdir build_boot

echo Compiling provisioner objects...
"%CC%" %ASFLAGS% -c build_boot\start_head.S -o build_boot\start_head.o
if errorlevel 1 exit /b 1
"%CC%" %ASFLAGS% -c src\vectors.S -o build_boot\vectors.o
if errorlevel 1 exit /b 1
"%CC%" %ASFLAGS% -c src\provision_revert_payload.S -o build_boot\provision_revert_payload.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\provision_revert.c -o build_boot\provision_revert.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\sd.c -o build_boot\sd.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\fb.c -o build_boot\fb.o
if errorlevel 1 exit /b 1

echo Linking provisioner kernel8...
"%LD%" -T link.ld -nostdlib -o provision_revert.elf build_boot\start_head.o build_boot\vectors.o build_boot\provision_revert.o build_boot\provision_revert_payload.o build_boot\sd.o build_boot\fb.o
if errorlevel 1 exit /b 1
"%OC%" -O binary provision_revert.elf provision_p0_kernel8.img
if errorlevel 1 exit /b 1
for %%f in (provision_p0_kernel8.img) do echo provisioner provision_p0_kernel8.img size: %%~zf bytes
echo PROVISIONER BUILD COMPLETE
