@echo off
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set FULL_CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2
set BOOT_CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -mgeneral-regs-only -Iinclude -O2 -DPIOS_FB_NO_DOUBLE_BUFFER
set USER_CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -mgeneral-regs-only -Iinclude -O2 -fno-builtin
set QEMU_STAGE2_CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8-a -mgeneral-regs-only -Iinclude -O2 -fno-builtin -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT
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
if exist build_qemu_stage2 rmdir /S /Q build_qemu_stage2
mkdir build
mkdir build_boot
mkdir build_user
mkdir build_qemu_stage2

echo Building embedded userland binaries...
"%CC%" %ASFLAGS% -c user\ustart.S -o build_user\ustart.o
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -c user\httpd.c -o build_user\httpd.o
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -c src\picovm.c -o build_user\picovm.o
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -DPIOS_USER_EL0 -DUHTTP_BRIDGE_INDEX=0 -c user\httpd.c -o build_user\httpd_el0.o
if errorlevel 1 exit /b 1
"%LD%" -T user\httpd_el0.ld -nostdlib -o build_user\user_httpd.elf build_user\ustart.o build_user\httpd_el0.o build_user\picovm.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_user\user_httpd.elf user_httpd_vm.img
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -DPIOS_USER_EL0 -DUHTTP_BRIDGE_INDEX=1 -c user\httpd.c -o build_user\httpd_native.o
if errorlevel 1 exit /b 1
"%LD%" -T user\httpd_el0.ld -nostdlib -o build_user\user_httpd_native.elf build_user\ustart.o build_user\httpd_native.o build_user\picovm.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_user\user_httpd_native.elf user_httpd_native.img
if errorlevel 1 exit /b 1
for %%f in (user_httpd_vm.img) do echo user_httpd_vm.img size: %%~zf bytes
for %%f in (user_httpd_native.img) do echo user_httpd_native.img size: %%~zf bytes
"%CC%" %ASFLAGS% -c user\el0_probe.S -o build_user\el0_probe.o
if errorlevel 1 exit /b 1
"%LD%" -T user\el0_probe.ld -nostdlib -o build_user\user_el0_probe.elf build_user\el0_probe.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_user\user_el0_probe.elf user_el0_probe.img
if errorlevel 1 exit /b 1
for %%f in (user_el0_probe.img) do echo user_el0_probe.img size: %%~zf bytes
"%CC%" %USER_CFLAGS% -c user\el0_pico.c -o build_user\el0_pico.o
if errorlevel 1 exit /b 1
"%LD%" -T user\el0_pico.ld -nostdlib -o build_user\user_el0_pico.elf build_user\ustart.o build_user\el0_pico.o build_user\picovm.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_user\user_el0_pico.elf user_el0_pico.img
if errorlevel 1 exit /b 1
for %%f in (user_el0_pico.img) do echo user_el0_pico.img size: %%~zf bytes

echo Building Pi5 stage2 payload...
if not exist build mkdir build
for %%f in (src\*.S) do (
    if /I not "%%~nxf"=="bootstrap_start.S" if /I not "%%~nxf"=="bootstrap_trampoline.S" if /I not "%%~nxf"=="provision_payload.S" if /I not "%%~nxf"=="qemu_virt_start.S" if /I not "%%~nxf"=="qemu_stage2_start.S" if /I not "%%~nxf"=="qemu_stage2_manifest.S" (
        echo Compiling %%~nf.S...
        "%CC%" %ASFLAGS% -c "%%f" -o "build\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
for %%f in (src\*.c) do (
    if /I not "%%~nxf"=="bootstrap.c" if /I not "%%~nxf"=="provision.c" if /I not "%%~nxf"=="qemu_virt_min.c" if /I not "%%~nxf"=="qemu_virt_walfs.c" (
        echo Compiling %%~nf.c...
        "%CC%" %FULL_CFLAGS% -c "%%f" -o "build\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
(for %%f in (build\*.o) do @echo build\\%%~nxf) > build\objs.rsp
"%LD%" -T link.ld -nostdlib -o real_kernel.elf @build\objs.rsp
if errorlevel 1 exit /b 1
"%OC%" -O binary real_kernel.elf build\PIOS_PI5_STAGE2.BIN
if errorlevel 1 exit /b 1

echo Building QEMU stage2 payload...
"%CC%" -march=armv8-a -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT -c src\qemu_stage2_start.S -o build_qemu_stage2\qemu_stage2_start.o
if errorlevel 1 exit /b 1
"%CC%" -march=armv8-a -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT -c src\qemu_stage2_manifest.S -o build_qemu_stage2\qemu_stage2_manifest.o
if errorlevel 1 exit /b 1
"%CC%" %QEMU_STAGE2_CFLAGS% -c uefi\qemu_stage2_os.c -o build_qemu_stage2\qemu_stage2_os.o
if errorlevel 1 exit /b 1
"%CC%" %QEMU_STAGE2_CFLAGS% -c src\sd.c -o build_qemu_stage2\sd.o
if errorlevel 1 exit /b 1
"%CC%" %QEMU_STAGE2_CFLAGS% -c src\bcache.c -o build_qemu_stage2\bcache.o
if errorlevel 1 exit /b 1
"%CC%" %QEMU_STAGE2_CFLAGS% -c src\walfs.c -o build_qemu_stage2\walfs.o
if errorlevel 1 exit /b 1
"%CC%" %QEMU_STAGE2_CFLAGS% -c src\lru.c -o build_qemu_stage2\lru.o
if errorlevel 1 exit /b 1
"%CC%" %QEMU_STAGE2_CFLAGS% -c src\picocompress.c -o build_qemu_stage2\picocompress.o
if errorlevel 1 exit /b 1
"%CC%" %QEMU_STAGE2_CFLAGS% -c src\picoweb.c -o build_qemu_stage2\picoweb.o
if errorlevel 1 exit /b 1
"%LD%" -T link_qemu_virt.ld -nostdlib -o build_qemu_stage2\PIOSSTG2_QEMU.ELF build_qemu_stage2\qemu_stage2_start.o build_qemu_stage2\qemu_stage2_manifest.o build_qemu_stage2\qemu_stage2_os.o build_qemu_stage2\sd.o build_qemu_stage2\bcache.o build_qemu_stage2\walfs.o build_qemu_stage2\lru.o build_qemu_stage2\picocompress.o build_qemu_stage2\picoweb.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_qemu_stage2\PIOSSTG2_QEMU.ELF build_qemu_stage2\PIOSSTG2_QEMU.BIN
if errorlevel 1 exit /b 1

echo Packaging shared Pi5+QEMU stage2 as real_kernel.img...
python tools\build_stage2_package.py --pi build\PIOS_PI5_STAGE2.BIN --qemu build_qemu_stage2\PIOSSTG2_QEMU.BIN --out real_kernel.img
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
for %%f in (build\PIOS_PI5_STAGE2.BIN) do echo Pi5 payload size: %%~zf bytes
for %%f in (build_qemu_stage2\PIOSSTG2_QEMU.BIN) do echo QEMU payload size: %%~zf bytes
for %%f in (real_kernel.img) do echo shared stage2 real_kernel.img size: %%~zf bytes
echo BOOTSTRAP BUILD COMPLETE
