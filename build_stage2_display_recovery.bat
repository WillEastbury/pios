@echo off
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2 -DPIOS_FB_NO_DOUBLE_BUFFER
set ASFLAGS=-march=armv8.2-a+simd+crc+crypto

if exist build_stage2_display_recovery rmdir /S /Q build_stage2_display_recovery
mkdir build_stage2_display_recovery

"%CC%" %ASFLAGS% -c src\start.S -o build_stage2_display_recovery\start.o
if errorlevel 1 exit /b 1
"%CC%" %ASFLAGS% -c src\vectors.S -o build_stage2_display_recovery\vectors.o
if errorlevel 1 exit /b 1
"%CC%" %ASFLAGS% -c src\stage2_manifest.S -o build_stage2_display_recovery\manifest.o
if errorlevel 1 exit /b 1
"%CC%" %CFLAGS% -c tools\stage2_display_recovery\stage2_display_recovery.c -o build_stage2_display_recovery\recovery.o
if errorlevel 1 exit /b 1
"%CC%" %CFLAGS% -c src\fb.c -o build_stage2_display_recovery\fb.o
if errorlevel 1 exit /b 1
"%CC%" %CFLAGS% -c src\timer.c -o build_stage2_display_recovery\timer.o
if errorlevel 1 exit /b 1
"%CC%" %CFLAGS% -c src\pcie.c -o build_stage2_display_recovery\pcie.o
if errorlevel 1 exit /b 1
"%CC%" %CFLAGS% -c src\rp1.c -o build_stage2_display_recovery\rp1.o
if errorlevel 1 exit /b 1
"%CC%" %CFLAGS% -c src\rp1_uart.c -o build_stage2_display_recovery\rp1_uart.o
if errorlevel 1 exit /b 1
"%CC%" %CFLAGS% -c src\uart.c -o build_stage2_display_recovery\uart.o
if errorlevel 1 exit /b 1

"%LD%" -T link.ld -nostdlib -o build_stage2_display_recovery\stage2_display_recovery.elf build_stage2_display_recovery\start.o build_stage2_display_recovery\vectors.o build_stage2_display_recovery\manifest.o build_stage2_display_recovery\recovery.o build_stage2_display_recovery\fb.o build_stage2_display_recovery\timer.o build_stage2_display_recovery\pcie.o build_stage2_display_recovery\rp1.o build_stage2_display_recovery\rp1_uart.o build_stage2_display_recovery\uart.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_stage2_display_recovery\stage2_display_recovery.elf stage2_display_recovery.bin
if errorlevel 1 exit /b 1
python tools\build_stage2_package.py --pi stage2_display_recovery.bin --out PIOSSTG2.PKG
if errorlevel 1 exit /b 1

for %%f in (stage2_display_recovery.bin) do echo Stage2 display recovery payload: %%~zf bytes
for %%f in (PIOSSTG2.PKG) do echo Stage2 display recovery package: %%~zf bytes
