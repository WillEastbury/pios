@echo off
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2
set ASFLAGS=-march=armv8.2-a+simd+crc+crypto

for /f %%i in ('powershell -NoProfile -Command "Get-Date -Format 'yyyyMMdd.HHmmss'"') do set BUILD_STAMP=%%i
> include\build_version.h echo #pragma once
>> include\build_version.h echo #define PIOS_BUILD_STAMP "%BUILD_STAMP%"
>> include\build_version.h echo #define PIOS_VERSION "v%BUILD_STAMP%"
>> include\build_version.h echo #define PIOS_BUILD_NAME "SECOND STAGE LOADER"
>> include\build_version.h echo #define PIOS_BUILD_LABEL "SECOND STAGE LOADER v%BUILD_STAMP%"
echo Build version: SECOND STAGE LOADER v%BUILD_STAMP%

if exist build rmdir /S /Q build
mkdir build

echo Building second-stage payload as real_kernel.img...
for %%f in (src\*.S) do (
    if /I not "%%~nxf"=="bootstrap_start.S" if /I not "%%~nxf"=="provision_payload.S" if /I not "%%~nxf"=="qemu_virt_start.S" (
        echo Compiling %%~nf.S...
        "%CC%" %ASFLAGS% -c "%%f" -o "build\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
for %%f in (src\*.c) do (
    if /I not "%%~nxf"=="bootstrap.c" if /I not "%%~nxf"=="provision.c" if /I not "%%~nxf"=="qemu_virt_min.c" if /I not "%%~nxf"=="qemu_virt_walfs.c" (
        echo Compiling %%~nf.c...
        "%CC%" %CFLAGS% -c "%%f" -o "build\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
(for %%f in (build\*.o) do @echo build\\%%~nxf) > build\objs.rsp
"%LD%" -T link.ld -nostdlib -o real_kernel.elf @build\objs.rsp
if errorlevel 1 exit /b 1
"%OC%" -O binary real_kernel.elf real_kernel.img
if errorlevel 1 exit /b 1

echo Building normal-boot one-off provisioner kernel8.img...
for %%f in (src\*.S) do (
    if /I not "%%~nxf"=="bootstrap_start.S" if /I not "%%~nxf"=="qemu_virt_start.S" (
        echo Compiling provision %%~nf.S...
        "%CC%" %ASFLAGS% -c "%%f" -o "build\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
for %%f in (src\*.c) do (
    if /I not "%%~nxf"=="bootstrap.c" if /I not "%%~nxf"=="provision.c" if /I not "%%~nxf"=="qemu_virt_min.c" if /I not "%%~nxf"=="qemu_virt_walfs.c" (
        echo Compiling provision %%~nf.c...
        "%CC%" %CFLAGS% -DPIOS_ONEOFF_PROVISION -c "%%f" -o "build\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
(for %%f in (build\*.o) do @echo build\\%%~nxf) > build\objs.rsp
"%LD%" -T link.ld -nostdlib -o kernel8.elf @build\objs.rsp
if errorlevel 1 exit /b 1
"%OC%" -O binary kernel8.elf kernel8.img
if errorlevel 1 exit /b 1
copy /Y kernel8.img full_provisioner_kernel8.img >nul

for %%f in (full_provisioner_kernel8.img) do echo full provisioner kernel8.img size: %%~zf bytes
for %%f in (real_kernel.img) do echo embedded payload real_kernel.img size: %%~zf bytes
echo FULL PROVISIONER BUILD COMPLETE
