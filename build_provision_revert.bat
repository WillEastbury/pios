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
REM start_head.S (the NC-from-boot head) was an untracked generated file; it is
REM exactly src/start.S compiled with block-0 mapped Non-Cacheable (so early SD
REM DMA is coherent) and the set/way cache bring-up disabled. Compile src/start.S
REM directly with those two gates off instead of relying on the lost file.
"%CC%" %ASFLAGS% -DPIOS_CACHE_WB_FROM_BOOT=0 -DPIOS_CACHE_BRINGUP_FIX=0 -c src\start.S -o build_boot\start_head.o
if errorlevel 1 exit /b 1
"%CC%" %ASFLAGS% -c src\vectors.S -o build_boot\vectors.o
if errorlevel 1 exit /b 1
"%CC%" %ASFLAGS% -c src\provision_revert_payload.S -o build_boot\provision_revert_payload.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\provision_revert.c -o build_boot\provision_revert.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\sd.c -o build_boot\sd.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\sdhost.c -o build_boot\sdhost.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\fb.c -o build_boot\fb.o
if errorlevel 1 exit /b 1

echo Linking provisioner kernel8...
"%LD%" -T link.ld -nostdlib -o provision_revert.elf build_boot\start_head.o build_boot\vectors.o build_boot\provision_revert.o build_boot\provision_revert_payload.o build_boot\sd.o build_boot\sdhost.o build_boot\fb.o
if errorlevel 1 exit /b 1
"%OC%" -O binary provision_revert.elf provision_p0_kernel8.img
if errorlevel 1 exit /b 1
for %%f in (provision_p0_kernel8.img) do echo provisioner provision_p0_kernel8.img size: %%~zf bytes
echo PROVISIONER BUILD COMPLETE
