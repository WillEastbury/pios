@echo off
REM build_pi4.bat -- Raspberry Pi 4 (BCM2711 / Cortex-A72) kernel image.
REM Wired NIC is SoC GENET v5, not RP1 MACB. TCP/IP/UDP are the shared stack.
REM A72 is ARMv8-A: -mno-outline-atomics (no LSE). Crypto extensions are present.

set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8-a+simd+crc+crypto -mno-outline-atomics -Iinclude -O2 -fstack-protector-strong -DPIOS_FB_MBOX_POLL_LIMIT=1000000U -DPIOS_PLATFORM=PIOS_PLATFORM_PI4
set ASFLAGS=-march=armv8-a+simd+crc+crypto -DPIOS_PLATFORM=PIOS_PLATFORM_PI4

if exist build_pi4 rmdir /S /Q build_pi4
mkdir build_pi4
if exist build_user rmdir /S /Q build_user
mkdir build_user

set USER_CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8-a+simd+crc+crypto -mno-outline-atomics -mgeneral-regs-only -Iinclude -O2 -fno-builtin
set USER_ASFLAGS=-march=armv8-a+simd+crc+crypto -mno-outline-atomics -DPIOS_PLATFORM=PIOS_PLATFORM_PI4

echo Building embedded userland binaries...
"%CC%" %USER_ASFLAGS% -c user\ustart.S -o build_user\ustart.o
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -c src\picovm.c -o build_user\picovm.o
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -c src\picovm_pios_optional.c -o build_user\picovm_pios_optional.o
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -c src\simd.c -o build_user\simd.o
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -c src\sha256_hkdf.c -o build_user\sha256_hkdf.o
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -c src\sha512.c -o build_user\sha512.o
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -c src\ed25519.c -o build_user\ed25519.o
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -fno-gcse -DPIOS_USER_EL0 -DUHTTP_BRIDGE_INDEX=0 -c user\httpd.c -o build_user\httpd_el0.o
if errorlevel 1 exit /b 1
"%LD%" -T user\httpd_el0.ld -nostdlib -o build_user\user_httpd.elf build_user\ustart.o build_user\httpd_el0.o build_user\picovm.o build_user\picovm_pios_optional.o build_user\simd.o build_user\sha256_hkdf.o build_user\sha512.o build_user\ed25519.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_user\user_httpd.elf user_httpd_vm.img
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -fno-gcse -DPIOS_USER_EL0 -DUHTTP_BRIDGE_INDEX=1 -c user\httpd.c -o build_user\httpd_native.o
if errorlevel 1 exit /b 1
"%LD%" -T user\httpd_el0.ld -nostdlib -o build_user\user_httpd_native.elf build_user\ustart.o build_user\httpd_native.o build_user\picovm.o build_user\picovm_pios_optional.o build_user\simd.o build_user\sha256_hkdf.o build_user\sha512.o build_user\ed25519.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_user\user_httpd_native.elf user_httpd_native.img
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -c user\el0_pico.c -o build_user\el0_pico.o
if errorlevel 1 exit /b 1
"%LD%" -T user\el0_pico.ld -nostdlib -o build_user\user_el0_pico.elf build_user\ustart.o build_user\el0_pico.o build_user\picovm.o build_user\picovm_pios_optional.o build_user\simd.o build_user\sha256_hkdf.o build_user\sha512.o build_user\ed25519.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_user\user_el0_pico.elf user_el0_pico.img
if errorlevel 1 exit /b 1
"%CC%" %USER_CFLAGS% -fno-gcse -DPIOS_USER_EL0 -DCAPSVC_SVC_IDX=0 -c user\capsvc_host.c -o build_user\capsvc_host0.o
if errorlevel 1 exit /b 1
"%LD%" -T user\httpd_el0.ld -nostdlib -o build_user\user_capsvc_host0.elf build_user\ustart.o build_user\capsvc_host0.o build_user\picovm.o build_user\picovm_pios_optional.o build_user\simd.o build_user\sha256_hkdf.o build_user\sha512.o build_user\ed25519.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_user\user_capsvc_host0.elf user_capsvc_host0.img
if errorlevel 1 exit /b 1
"%CC%" %USER_ASFLAGS% -c user\el0_probe.S -o build_user\el0_probe.o
if errorlevel 1 exit /b 1
"%LD%" -T user\el0_probe.ld -nostdlib -o build_user\user_el0_probe.elf build_user\el0_probe.o
if errorlevel 1 exit /b 1
"%OC%" -O binary build_user\user_el0_probe.elf user_el0_probe.img
if errorlevel 1 exit /b 1

set ERRORS=0

for %%f in (src\*.S) do (
    if /I not "%%~nxf"=="bootstrap_start.S" if /I not "%%~nxf"=="bootstrap_trampoline.S" if /I not "%%~nxf"=="provision_payload.S" if /I not "%%~nxf"=="provision_revert_payload.S" if /I not "%%~nxf"=="qemu_virt_start.S" if /I not "%%~nxf"=="qemu_stage2_start.S" if /I not "%%~nxf"=="qemu_stage2_manifest.S" if /I not "%%~nxf"=="qemu_boot_stage2_manifest.S" (
        echo Compiling %%~nf.S...
        "%CC%" %ASFLAGS% -c "%%f" -o "build_pi4\%%~nf.o"
        if errorlevel 1 (
            echo FAIL: %%~nf.S
            set /a ERRORS+=1
        )
    )
)

for %%f in (src\*.c) do (
    if /I not "%%~nxf"=="bootstrap.c" if /I not "%%~nxf"=="provision.c" if /I not "%%~nxf"=="provision_revert.c" if /I not "%%~nxf"=="qemu_virt_min.c" if /I not "%%~nxf"=="qemu_virt_walfs.c" if /I not "%%~nxf"=="ide_assets.c" (
        echo Compiling %%~nf.c...
        "%CC%" %CFLAGS% -c "%%f" -o "build_pi4\%%~nf.o"
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
(for %%f in (build_pi4\*.o) do @echo build_pi4/%%~nxf) > build_pi4\objs.rsp
"%LD%" -T link.ld -nostdlib -o build_pi4\kernel8_pi4.elf @build_pi4\objs.rsp
if errorlevel 1 (
    echo LINK FAILED
    exit /b 1
)

echo Creating kernel8_pi4.img...
"%OC%" -O binary build_pi4\kernel8_pi4.elf build_pi4\kernel8_pi4.img
if errorlevel 1 (
    echo OBJCOPY FAILED
    exit /b 1
)

for %%f in (build_pi4\kernel8_pi4.img) do echo kernel8_pi4.img size: %%~zf bytes
echo PI4 BUILD COMPLETE
