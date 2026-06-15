@echo off
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
set CC=%TC%\aarch64-none-elf-gcc.exe
set LD=%TC%\aarch64-none-elf-ld.exe
set OC=%TC%\aarch64-none-elf-objcopy.exe
set CFLAGS=-Wall -Wextra -ffreestanding -nostdlib -nostartfiles -std=gnu11 -march=armv8.2-a+simd+crc+crypto -Iinclude -O2
set ASFLAGS=-march=armv8.2-a+simd+crc+crypto

if not exist build mkdir build

set ERRORS=0

for %%f in (src\*.S) do (
    if /I not "%%~nxf"=="qemu_virt_start.S" (
        echo Compiling %%~nf.S...
        "%CC%" %ASFLAGS% -c "%%f" -o "build\%%~nf.o"
        if errorlevel 1 (
            echo FAIL: %%~nf.S
            set /a ERRORS+=1
        )
    )
)

for %%f in (src\*.c) do (
    if /I not "%%~nxf"=="qemu_virt_min.c" if /I not "%%~nxf"=="qemu_virt_walfs.c" (
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
"%LD%" -T link.ld -nostdlib -o kernel8.elf build\*.o
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
echo BUILD COMPLETE
