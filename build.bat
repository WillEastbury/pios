@echo off
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2 -fstack-protector-strong
set ASFLAGS=-march=armv8.2-a+simd+crc+crypto

REM Clean first (build_pi5_only.bat does this too) -- a stale build\ dir
REM previously left over bootstrap_start.o/bootstrap_trampoline.o from a
REM different build script's run, which both define bootstrap_trampoline
REM and fail the link with "multiple definition".
if exist build rmdir /S /Q build
mkdir build

set ERRORS=0

REM Exclude the same files build_pi5_only.bat excludes: bootstrap_start.S/
REM bootstrap_trampoline.S/provision_payload.S/provision_revert_payload.S
REM and bootstrap.c/provision.c/provision_revert.c are for the SEPARATE
REM small bootstrap/provisioner sub-images (build_bootstrap.bat/
REM build_provisioner.bat), not this full kernel image -- compiling them in
REM here caused a real multiple-definition link failure (bootstrap_start.S
REM and bootstrap_trampoline.S both define bootstrap_trampoline).
for %%f in (src\*.S) do (
    if /I not "%%~nxf"=="bootstrap_start.S" if /I not "%%~nxf"=="bootstrap_trampoline.S" if /I not "%%~nxf"=="provision_payload.S" if /I not "%%~nxf"=="provision_revert_payload.S" if /I not "%%~nxf"=="qemu_virt_start.S" if /I not "%%~nxf"=="qemu_stage2_start.S" if /I not "%%~nxf"=="qemu_stage2_manifest.S" (
        echo Compiling %%~nf.S...
        "%CC%" %ASFLAGS% -c "%%f" -o "build\%%~nf.o"
        if errorlevel 1 (
            echo FAIL: %%~nf.S
            set /a ERRORS+=1
        )
    )
)

for %%f in (src\*.c) do (
    if /I not "%%~nxf"=="bootstrap.c" if /I not "%%~nxf"=="provision.c" if /I not "%%~nxf"=="provision_revert.c" if /I not "%%~nxf"=="qemu_virt_min.c" if /I not "%%~nxf"=="qemu_virt_walfs.c" (
        echo Compiling %%~nf.c...
        "%CC%" %CFLAGS% -c "%%f" -o "build\%%~nf.o"
        if errorlevel 1 (
            echo FAIL: %%~nf.c
            set /a ERRORS+=1
        )
    )
)

echo.
echo Linking...
(for %%f in (build\*.o) do @echo build\\%%~nxf) > build\objs.rsp
"%LD%" -T link.ld -nostdlib -o kernel8.elf @build\objs.rsp
if errorlevel 1 (
    echo LINK FAILED
    exit /b 1
)

echo Creating kernel8.img...
"%OC%" -O binary kernel8.elf kernel8.img
if errorlevel 1 (
    echo OBJCOPY FAILED
    exit /b 1
)

for %%f in (kernel8.img) do echo kernel8.img size: %%~zf bytes

REM Hard stage2 payload-size guard: kernel8.img is the direct-boot stage2
REM payload and must fit the raw boot slot zone (PIOS_STAGE2_ZONE_BYTES in
REM include/walfs.h = 0x37FE00). Fail the build if it overflows the zone.
set PIOS_STAGE2_ZONE_BYTES=3669504
for %%f in (kernel8.img) do set KIMG_SIZE=%%~zf
if %KIMG_SIZE% GTR %PIOS_STAGE2_ZONE_BYTES% (
    echo ERROR: kernel8.img %KIMG_SIZE% bytes exceeds PIOS_STAGE2_ZONE_BYTES %PIOS_STAGE2_ZONE_BYTES%
    del kernel8.img
    exit /b 1
)
echo kernel8.img %KIMG_SIZE% bytes within PIOS_STAGE2_ZONE_BYTES %PIOS_STAGE2_ZONE_BYTES%
echo BUILD COMPLETE
