@echo off
setlocal
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set CFLAGS=-Wall -Wextra -Wno-unused-function -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2 -fno-builtin -fstack-protector-strong -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT
set ASFLAGS=-march=armv8.2-a+simd+crc+crypto -DPIOS_PLATFORM=PIOS_PLATFORM_QEMU_VIRT

if exist build_qemu_full rmdir /S /Q build_qemu_full
mkdir build_qemu_full

echo Building full feature-parity QEMU kernel payload...
for %%f in (src\*.S) do (
    if /I not "%%~nxf"=="bootstrap_start.S" if /I not "%%~nxf"=="bootstrap_trampoline.S" if /I not "%%~nxf"=="provision_payload.S" if /I not "%%~nxf"=="provision_revert_payload.S" if /I not "%%~nxf"=="qemu_virt_start.S" if /I not "%%~nxf"=="qemu_stage2_start.S" if /I not "%%~nxf"=="qemu_stage2_manifest.S" if /I not "%%~nxf"=="qemu_boot_stage2_manifest.S" (
        echo Compiling %%~nf.S...
        "%CC%" %ASFLAGS% -c "%%f" -o "build_qemu_full\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
for %%f in (src\*.c) do (
    if /I not "%%~nxf"=="bootstrap.c" if /I not "%%~nxf"=="provision.c" if /I not "%%~nxf"=="provision_revert.c" if /I not "%%~nxf"=="qemu_virt_min.c" if /I not "%%~nxf"=="qemu_virt_walfs.c" (
        echo Compiling %%~nf.c...
        "%CC%" %CFLAGS% -c "%%f" -o "build_qemu_full\%%~nf.o"
        if errorlevel 1 exit /b 1
    )
)
(for %%f in (build_qemu_full\*.o) do @echo build_qemu_full\\%%~nxf) > build_qemu_full\objs.rsp
"%LD%" -T link_qemu_full.ld -nostdlib -o build_qemu_full\PIOS_QEMU_FULL.ELF @build_qemu_full\objs.rsp
if errorlevel 1 exit /b 1
"%OC%" -O binary build_qemu_full\PIOS_QEMU_FULL.ELF build_qemu_full\PIOS_QEMU_FULL.BIN
if errorlevel 1 exit /b 1

for %%f in (build_qemu_full\PIOS_QEMU_FULL.BIN) do echo QEMU full payload size: %%~zf bytes
echo QEMU FULL PAYLOAD BUILD COMPLETE
endlocal
