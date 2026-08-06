@echo off
REM build_bootstrap_qemu.bat - QEMU-virt counterpart to build_bootstrap.bat.
REM Builds a stage0 (kernel8_qemu.img) + stage2 (PIOS_QEMU_STAGE2.BIN) pair
REM that boot through the SAME stage0->trampoline->stage2 chain as real Pi5
REM hardware (bootstrap.c/bootstrap_start.S/bootstrap_trampoline.S), instead
REM of QEMU's existing direct -kernel PIOS_QEMU_FULL.BIN boot. This lets
REM QEMU exercise the real FAT+raw-slot+WALFS disk path (keystore/x509/TLS
REM certificate provisioning, OTA A/B slot switching) instead of the RAM-
REM disk fallback used by qemu_smoke.py's direct-boot path.
REM
REM Does NOT modify or replace build_qemu_full.bat/PIOS_QEMU_FULL.BIN --
REM that direct-boot path and its qemu_smoke.py 29/29 regression suite are
REM untouched by this script.
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set FULL_CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2 -fstack-protector-strong -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT
set BOOT_CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -mgeneral-regs-only -Iinclude -O2 -DPIOS_FB_NO_DOUBLE_BUFFER -DPIOS_RUNTIME_MMIO_BOOTSTRAP=1 -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT
set ASFLAGS=-march=armv8.2-a+simd+crc+crypto -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT

for /f %%i in ('powershell -NoProfile -Command "Get-Date -Format 'yyyyMMdd.HHmmss'"') do set BUILD_STAMP=%%i
> include\build_version.h echo #pragma once
>> include\build_version.h echo #define PIOS_BUILD_STAMP "%BUILD_STAMP%"
>> include\build_version.h echo #define PIOS_VERSION "v%BUILD_STAMP%"
>> include\build_version.h echo #define PIOS_BUILD_NAME "PIOS Kernel"
>> include\build_version.h echo #define PIOS_BUILD_LABEL "PIOS Kernel Booted and Running -> Version v%BUILD_STAMP%"
echo Build version: PIOS Kernel v%BUILD_STAMP% (QEMU stage0 chain)

if exist build_qemu_stage2_full rmdir /S /Q build_qemu_stage2_full
if exist build_boot_qemu rmdir /S /Q build_boot_qemu
mkdir build_qemu_stage2_full
mkdir build_boot_qemu

echo Building QEMU stage2 payload (via stage0 chain, same sources as build_qemu_full.bat)...
for %%f in (src\*.S) do (
    if /I not "%%~nxf"=="bootstrap_start.S" if /I not "%%~nxf"=="bootstrap_trampoline.S" if /I not "%%~nxf"=="provision_payload.S" if /I not "%%~nxf"=="provision_revert_payload.S" if /I not "%%~nxf"=="qemu_virt_start.S" if /I not "%%~nxf"=="qemu_stage2_start.S" if /I not "%%~nxf"=="qemu_stage2_manifest.S" if /I not "%%~nxf"=="stage2_manifest.S" (
        echo Compiling %%~nf.S...
        "%CC%" %ASFLAGS% -c "%%f" -o "build_qemu_stage2_full\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
for %%f in (src\*.c) do (
    if /I not "%%~nxf"=="bootstrap.c" if /I not "%%~nxf"=="provision.c" if /I not "%%~nxf"=="provision_revert.c" if /I not "%%~nxf"=="qemu_virt_min.c" if /I not "%%~nxf"=="qemu_virt_walfs.c" (
        echo Compiling %%~nf.c...
        "%CC%" %FULL_CFLAGS% -c "%%f" -o "build_qemu_stage2_full\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
(for %%f in (build_qemu_stage2_full\*.o) do @echo build_qemu_stage2_full\\%%~nxf) > build_qemu_stage2_full\objs.rsp
"%LD%" -T link_qemu_full.ld -nostdlib -o build_qemu_stage2_full\PIOS_QEMU_STAGE2.ELF @build_qemu_stage2_full\objs.rsp
if errorlevel 1 exit /b 1
"%OC%" -O binary build_qemu_stage2_full\PIOS_QEMU_STAGE2.ELF build_qemu_stage2_full\PIOS_QEMU_STAGE2.BIN
if errorlevel 1 exit /b 1
for %%f in (build_qemu_stage2_full\PIOS_QEMU_STAGE2.BIN) do echo QEMU stage2 payload size: %%~zf bytes

echo Packaging QEMU stage2 as real_kernel_qemu.img (reuses the existing --qemu packaging flag)...
python tools\build_stage2_package.py --qemu build_qemu_stage2_full\PIOS_QEMU_STAGE2.BIN --out real_kernel_qemu.img
if errorlevel 1 exit /b 1
copy /Y real_kernel_qemu.img PIOSSTG2_QEMU.PKG >nul

echo Compiling QEMU stage0...
"%CC%" %ASFLAGS% -c src\bootstrap_start.S -o build_boot_qemu\bootstrap_start.o
if errorlevel 1 exit /b 1
"%CC%" %ASFLAGS% -c src\bootstrap_trampoline.S -o build_boot_qemu\bootstrap_trampoline.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\bootstrap.c -o build_boot_qemu\bootstrap.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\board_detect.c -o build_boot_qemu\board_detect.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\sd.c -o build_boot_qemu\sd.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\fb.c -o build_boot_qemu\fb.o
if errorlevel 1 exit /b 1

echo Linking QEMU stage0 kernel8_qemu.img...
"%LD%" -T link_bootstrap_qemu.ld -nostdlib -o bootstrap_qemu.elf build_boot_qemu\bootstrap_start.o build_boot_qemu\bootstrap_trampoline.o build_boot_qemu\bootstrap.o build_boot_qemu\board_detect.o build_boot_qemu\sd.o build_boot_qemu\fb.o
if errorlevel 1 exit /b 1
"%OC%" -O binary bootstrap_qemu.elf kernel8_qemu.img
if errorlevel 1 exit /b 1

for %%f in (kernel8_qemu.img) do echo QEMU stage0 kernel8_qemu.img size: %%~zf bytes
for %%f in (real_kernel_qemu.img) do echo QEMU stage2 package real_kernel_qemu.img size: %%~zf bytes
echo BOOTSTRAP QEMU BUILD COMPLETE
