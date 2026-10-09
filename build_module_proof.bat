@echo off
setlocal
set TC=C:\aarch64-none-elf\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-aarch64-none-elf\bin
if not exist build_module mkdir build_module
"%TC%\aarch64-none-elf-gcc.exe" -march=armv8.2-a -ffreestanding -nostdlib -c modules\proof_v3.S -o build_module\proof_v3.o
if errorlevel 1 exit /b 1
"%TC%\aarch64-none-elf-objcopy.exe" -O binary -j .text build_module\proof_v3.o build_module\proof_v3.bin
if errorlevel 1 exit /b 1
python tools\build_pmod.py build_module\proof_v3.bin --out build_module\proof_v3.pmod --module-id 7 --generation 3 --arena-schema 1
if errorlevel 1 exit /b 1
echo Module artifact: build_module\proof_v3.pmod
