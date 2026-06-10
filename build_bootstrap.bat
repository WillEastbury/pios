@echo off
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set FULL_CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2
set BOOT_CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -mgeneral-regs-only -Iinclude -O2 -DPIOS_FB_NO_DOUBLE_BUFFER
set USER_CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -mgeneral-regs-only -Iinclude -O2 -fno-builtin
set ASFLAGS=-march=armv8.2-a+simd+crc+crypto

for /f %%i in ('powershell -NoProfile -Command "Get-Date -Format 'yyyyMMdd.HHmmss'"') do set BUILD_STAMP=%%i
> include\build_version.h echo #pragma once
>> include\build_version.h echo #define PIOS_BUILD_STAMP "%BUILD_STAMP%"
>> include\build_version.h echo #define PIOS_VERSION "v%BUILD_STAMP%"
>> include\build_version.h echo #define PIOS_BUILD_NAME "PIOS Kernel"
>> include\build_version.h echo #define PIOS_BUILD_LABEL "PIOS Kernel Booted and Running -> Version v%BUILD_STAMP%"
echo Build version: PIOS Kernel v%BUILD_STAMP%

if exist build rmdir /S /Q build
if exist build_boot rmdir /S /Q build_boot
if exist build_user rmdir /S /Q build_user
mkdir build
mkdir build_boot
mkdir build_user

echo Building embedded userland binaries...
"%CC%" %ASFLAGS% -c user\ustart.S -o build_user\ustart.o
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -c user\httpd.c -o build_user\httpd.o
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -c src\picovm.c -o build_user\picovm.o
if errorlevel 1 exit /b 1
"%LD%" -T user\user.ld -nostdlib -o build_user\user_httpd.elf build_user\ustart.o build_user\httpd.o build_user\picovm.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_user\user_httpd.elf user_httpd.img
if errorlevel 1 exit /b 1
for %%f in (user_httpd.img) do echo user_httpd.img size: %%~zf bytes
"%CC%" %ASFLAGS% -c user\el0_probe.S -o build_user\el0_probe.o
if errorlevel 1 exit /b 1
"%LD%" -T user\el0_probe.ld -nostdlib -o build_user\user_el0_probe.elf build_user\el0_probe.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_user\user_el0_probe.elf user_el0_probe.img
if errorlevel 1 exit /b 1
for %%f in (user_el0_probe.img) do echo user_el0_probe.img size: %%~zf bytes

echo Building full kernel as real_kernel.img...
if not exist build mkdir build
for %%f in (src\*.S) do (
    if /I not "%%~nxf"=="bootstrap_start.S" if /I not "%%~nxf"=="bootstrap_trampoline.S" if /I not "%%~nxf"=="provision_payload.S" (
        echo Compiling %%~nf.S...
        "%CC%" %ASFLAGS% -c "%%f" -o "build\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
for %%f in (src\*.c) do (
    if /I not "%%~nxf"=="bootstrap.c" if /I not "%%~nxf"=="provision.c" (
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

echo Compiling bootstrap...
"%CC%" %ASFLAGS% -c src\start.S -o build_boot\start.o
if errorlevel 1 exit /b 1
"%CC%" %ASFLAGS% -c src\vectors.S -o build_boot\vectors.o
if errorlevel 1 exit /b 1
"%CC%" %ASFLAGS% -c src\bootstrap_trampoline.S -o build_boot\bootstrap_trampoline.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\bootstrap.c -o build_boot\bootstrap.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\sd.c -o build_boot\sd.o
if errorlevel 1 exit /b 1
"%CC%" %BOOT_CFLAGS% -c src\fb.c -o build_boot\fb.o
if errorlevel 1 exit /b 1

echo Linking bootstrap kernel8.img...
"%LD%" -T link.ld -nostdlib -o bootstrap.elf build_boot\start.o build_boot\vectors.o build_boot\bootstrap_trampoline.o build_boot\bootstrap.o build_boot\sd.o build_boot\fb.o
if errorlevel 1 exit /b 1
"%OC%" -O binary bootstrap.elf kernel8.img
if errorlevel 1 exit /b 1

for %%f in (kernel8.img) do echo bootstrap kernel8.img size: %%~zf bytes
for %%f in (real_kernel.img) do echo real_kernel.img size: %%~zf bytes
echo BOOTSTRAP BUILD COMPLETE
