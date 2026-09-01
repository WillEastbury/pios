@echo off
REM build_pizero2w.bat -- Raspberry Pi Zero 2 W (BCM2710A1) kernel image.
REM
REM BCM2710A1 is the same die/peripheral generation as Pi3's BCM2837 (same
REM peripheral base 0x3F000000, same QA7 local-interrupt block at
REM 0x40000000 -- see include/platform.h PIOS_PLATFORM_PIZERO2W). This
REM script is otherwise identical to build_pi3.bat; only the platform
REM define differs, which mainly changes PIOS_PLATFORM_NAME in diagnostics.
REM
REM Status: links a full kernel with zero missing symbols (verified).
REM Secondary cores gracefully SKIP (PIOS_HAS_PSCI_SECONDARIES=0, same
REM spin-table gap as Pi3). WiFi uses the board-specific SDIO1/GPIO41
REM profile and the CYW43436 firmware family; hardware validation remains
REM separate from the common boot proof.
REM
REM Not yet hardware-validated: this build has not been tested on real
REM Pi Zero 2 W hardware. Flash kernel8_pizero2w.img (renamed to
REM kernel8.img) + config.txt + bcm2710-rpi-zero-2-w.dtb (or the matching
REM firmware set) to a FAT32 boot partition and report back what the UART
REM console shows.

set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8-a+simd+crc -mno-outline-atomics -Iinclude -O2 -fstack-protector-strong -DPIOS_FB_MBOX_POLL_LIMIT=1000000U -DPIOS_PLATFORM=PIOS_PLATFORM_PIZERO2W
set ASFLAGS=-march=armv8-a+simd+crc -DPIOS_PLATFORM=PIOS_PLATFORM_PIZERO2W

if exist build_pizero2w rmdir /S /Q build_pizero2w
mkdir build_pizero2w

set ERRORS=0

for %%f in (src\*.S) do (
    if /I not "%%~nxf"=="bootstrap_start.S" if /I not "%%~nxf"=="bootstrap_trampoline.S" if /I not "%%~nxf"=="provision_payload.S" if /I not "%%~nxf"=="provision_revert_payload.S" if /I not "%%~nxf"=="qemu_virt_start.S" if /I not "%%~nxf"=="qemu_stage2_start.S" if /I not "%%~nxf"=="qemu_stage2_manifest.S" if /I not "%%~nxf"=="qemu_boot_stage2_manifest.S" (
        echo Compiling %%~nf.S...
        "%CC%" %ASFLAGS% -c "%%f" -o "build_pizero2w\%%~nf.o"
        if errorlevel 1 (
            echo FAIL: %%~nf.S
            set /a ERRORS+=1
        )
    )
)

for %%f in (src\*.c) do (
    if /I not "%%~nxf"=="bootstrap.c" if /I not "%%~nxf"=="provision.c" if /I not "%%~nxf"=="provision_revert.c" if /I not "%%~nxf"=="qemu_virt_min.c" if /I not "%%~nxf"=="qemu_virt_walfs.c" if /I not "%%~nxf"=="ide_assets.c" (
        echo Compiling %%~nf.c...
        "%CC%" %CFLAGS% -c "%%f" -o "build_pizero2w\%%~nf.o"
        if errorlevel 1 (
            echo FAIL: %%~nf.c
            set /a ERRORS+=1
        )
    )
)

if %ERRORS% GTR 0 (
    echo BUILD FAILED: %ERRORS% file(s^) failed to compile.
    exit /b 1
)

echo.
echo Linking...
(for %%f in (build_pizero2w\*.o) do @echo build_pizero2w/%%~nxf) > build_pizero2w\objs.rsp
"%LD%" -T link.ld -nostdlib -o build_pizero2w\kernel8_pizero2w.elf @build_pizero2w\objs.rsp
if errorlevel 1 (
    echo LINK FAILED
    exit /b 1
)

echo Creating kernel8_pizero2w.img...
"%OC%" -O binary build_pizero2w\kernel8_pizero2w.elf build_pizero2w\kernel8_pizero2w.img
if errorlevel 1 (
    echo OBJCOPY FAILED
    exit /b 1
)

for %%f in (build_pizero2w\kernel8_pizero2w.img) do echo kernel8_pizero2w.img size: %%~zf bytes
echo PIZERO2W BUILD COMPLETE (link-verified only -- NOT yet hardware-tested)
