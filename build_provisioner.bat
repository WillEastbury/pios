@echo off
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set FULL_CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2
set BOOT_CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -mgeneral-regs-only -Iinclude -O2
set ASFLAGS=-march=armv8.2-a+simd+crc+crypto

if exist build rmdir /S /Q build
if exist build_boot rmdir /S /Q build_boot
mkdir build
mkdir build_boot

echo Building full kernel payload as real_kernel.img...
for %%f in (src\*.S) do (
    if /I not "%%~nxf"=="bootstrap_start.S" if /I not "%%~nxf"=="provision_payload.S" if /I not "%%~nxf"=="qemu_virt_start.S" if /I not "%%~nxf"=="qemu_stage2_start.S" if /I not "%%~nxf"=="qemu_stage2_manifest.S" (
        echo Compiling %%~nf.S...
        "%CC%" %ASFLAGS% -c "%%f" -o "build\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
for %%f in (src\*.c) do (
    if /I not "%%~nxf"=="bootstrap.c" if /I not "%%~nxf"=="provision.c" if /I not "%%~nxf"=="qemu_virt_min.c" if /I not "%%~nxf"=="qemu_virt_walfs.c" if /I not "%%~nxf"=="provision_revert.c" (
        echo Compiling %%~nf.c...
        "%CC%" %FULL_CFLAGS% -c "%%f" -o "build\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
(for %%f in (build\*.o) do @echo build\\%%~nxf) > build\objs.rsp
"%LD%" -T link.ld -nostdlib -o real_kernel.elf @build\objs.rsp
if errorlevel 1 exit /b 1
"%OC%" -O binary real_kernel.elf real_kernel.img
if errorlevel 1 exit /b 1

echo Compiling one-off provisioner...
REM The provisioner is a MINIMAL binary that does raw SD writes via sd.c before
REM jumping to the embedded kernel. It must boot Non-Cacheable (low 1GB NC) so
REM the SD controller DMA stays coherent. PIOS_CACHE_WB_FROM_BOOT=1 (the kernel
REM default, a perf win) makes the provisioner's early SD DMA incoherent and it
REM crashes mid-write -> blank screen (see checkpoint 082). The embedded
REM real_kernel.img above keeps WB-from-boot for performance; only the
REM provisioner's own start.o is forced NC here.
"%CC%" %ASFLAGS% -DPIOS_CACHE_WB_FROM_BOOT=0 -DPIOS_CACHE_BRINGUP_FIX=0 -c src\start.S -o build_boot\start.o
if errorlevel 1 exit /b 1
"%CC%" %ASFLAGS% -c src\vectors.S -o build_boot\vectors.o
if errorlevel 1 exit /b 1
"%CC%" %ASFLAGS% -c src\bootstrap_trampoline.S -o build_boot\bootstrap_trampoline.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\provision.c -o build_boot\provision.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\sd.c -o build_boot\sd.o
if errorlevel 1 exit /b 1
"%CC%" %ASFLAGS% -c src\provision_payload.S -o build_boot\provision_payload.o
if errorlevel 1 exit /b 1

echo Linking one-off kernel8.img...
"%LD%" -T link.ld -nostdlib -o provisioner.elf build_boot\start.o build_boot\vectors.o build_boot\bootstrap_trampoline.o build_boot\provision.o build_boot\sd.o build_boot\provision_payload.o
if errorlevel 1 exit /b 1
"%OC%" -O binary provisioner.elf provisioner_kernel8.img
if errorlevel 1 exit /b 1

copy /Y provisioner_kernel8.img kernel8.img >nul
for %%f in (kernel8.img) do echo provisioner kernel8.img size: %%~zf bytes
for %%f in (real_kernel.img) do echo embedded payload real_kernel.img size: %%~zf bytes
echo PROVISIONER BUILD COMPLETE
