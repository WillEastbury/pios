@echo off
REM build_pi3.bat -- Raspberry Pi 3 (BCM2837/BCM2837B0) kernel image.
REM
REM Status: links a full kernel with zero missing symbols (verified). Core 0
REM boot path is expected to work (UART/mailbox/EMMC/framebuffer share the
REM same legacy Broadcom protocol family PIOS already speaks on Pi5, just at
REM different base addresses -- see include/platform.h PIOS_PLATFORM_PI3).
REM Secondary cores gracefully SKIP with a console warning
REM ("[skip] multicore secondaries on this platform") rather than hang,
REM because PIOS_HAS_PSCI_SECONDARIES=0 here: stock Pi3 firmware does not
REM support the PSCI HVC mechanism Pi5/QEMU use, it expects a spin-table
REM wakeup instead, which is not yet implemented in src/start.S. This is a
REM real, honest gap, not a silently-assumed feature -- multicore on this
REM platform is tracked as follow-up work in the pi3-bcm2837 todo.
REM
REM Not yet hardware-validated: this build has not been tested on real Pi3
REM hardware. Flash kernel8_pi3.img (renamed to kernel8.img) + config.txt +
REM bcm2837-rpi-3-b.dtb (or the matching firmware set) to a FAT32 boot
REM partition and report back what the UART console shows.

set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2 -fstack-protector-strong -DPIOS_PLATFORM=PIOS_PLATFORM_PI3
set ASFLAGS=-march=armv8.2-a+simd+crc+crypto -DPIOS_PLATFORM=PIOS_PLATFORM_PI3

if exist build_pi3 rmdir /S /Q build_pi3
mkdir build_pi3

set ERRORS=0

for %%f in (src\*.S) do (
    if /I not "%%~nxf"=="bootstrap_start.S" if /I not "%%~nxf"=="bootstrap_trampoline.S" if /I not "%%~nxf"=="provision_payload.S" if /I not "%%~nxf"=="provision_revert_payload.S" if /I not "%%~nxf"=="qemu_virt_start.S" if /I not "%%~nxf"=="qemu_stage2_start.S" if /I not "%%~nxf"=="qemu_stage2_manifest.S" if /I not "%%~nxf"=="qemu_boot_stage2_manifest.S" (
        echo Compiling %%~nf.S...
        "%CC%" %ASFLAGS% -c "%%f" -o "build_pi3\%%~nf.o"
        if errorlevel 1 (
            echo FAIL: %%~nf.S
            set /a ERRORS+=1
        )
    )
)

for %%f in (src\*.c) do (
    if /I not "%%~nxf"=="bootstrap.c" if /I not "%%~nxf"=="provision.c" if /I not "%%~nxf"=="provision_revert.c" if /I not "%%~nxf"=="qemu_virt_min.c" if /I not "%%~nxf"=="qemu_virt_walfs.c" (
        echo Compiling %%~nf.c...
        "%CC%" %CFLAGS% -c "%%f" -o "build_pi3\%%~nf.o"
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
(for %%f in (build_pi3\*.o) do @echo build_pi3/%%~nxf) > build_pi3\objs.rsp
"%LD%" -T link.ld -nostdlib -o build_pi3\kernel8_pi3.elf @build_pi3\objs.rsp
if errorlevel 1 (
    echo LINK FAILED
    exit /b 1
)

echo Creating kernel8_pi3.img...
"%OC%" -O binary build_pi3\kernel8_pi3.elf build_pi3\kernel8_pi3.img
if errorlevel 1 (
    echo OBJCOPY FAILED
    exit /b 1
)

for %%f in (build_pi3\kernel8_pi3.img) do echo kernel8_pi3.img size: %%~zf bytes
echo PI3 BUILD COMPLETE (link-verified only -- NOT yet hardware-tested)
