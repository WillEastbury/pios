@echo off
set LLVM=C:\Program Files\LLVM\bin
set CC=%LLVM%\clang.exe
set LD=%LLVM%\lld-link.exe
set CFLAGS=-Wall -Wextra -ffreestanding -fshort-wchar -fno-builtin -fno-stack-protector -mno-red-zone -target x86_64-unknown-windows -DPIOS_PLATFORM=PIOS_PLATFORM_HYPERV_AMD64 -Iinclude -O2

if exist build_hyperv_amd64 rmdir /S /Q build_hyperv_amd64
if exist hyperv_amd64_esp rmdir /S /Q hyperv_amd64_esp
mkdir build_hyperv_amd64
mkdir build_hyperv_amd64\EFI
mkdir build_hyperv_amd64\EFI\BOOT
mkdir hyperv_amd64_esp
mkdir hyperv_amd64_esp\EFI
mkdir hyperv_amd64_esp\EFI\BOOT

echo Building Hyper-V amd64 BOOTX64.EFI probe...
"%CC%" %CFLAGS% -c uefi\bootx64_hyperv.c -o build_hyperv_amd64\bootx64_hyperv.o
if errorlevel 1 exit /b 1
"%LD%" /subsystem:efi_application /entry:efi_main /machine:x64 /nodefaultlib /out:build_hyperv_amd64\EFI\BOOT\BOOTX64.EFI build_hyperv_amd64\bootx64_hyperv.o
if errorlevel 1 exit /b 1

copy /Y build_hyperv_amd64\EFI\BOOT\BOOTX64.EFI hyperv_amd64_esp\EFI\BOOT\BOOTX64.EFI >nul
if errorlevel 1 exit /b 1
python tools\build_hyperv_esp_image.py --bootx64 build_hyperv_amd64\EFI\BOOT\BOOTX64.EFI --out build_hyperv_amd64\hyperv_amd64_esp.raw
if errorlevel 1 exit /b 1
if exist "C:\Program Files\qemu\qemu-img.exe" (
    "C:\Program Files\qemu\qemu-img.exe" convert -f raw -O vpc -o subformat=fixed build_hyperv_amd64\hyperv_amd64_esp.raw build_hyperv_amd64\hyperv_amd64_esp.vhd
    if errorlevel 1 exit /b 1
    "C:\Program Files\qemu\qemu-img.exe" convert -f raw -O vhdx -o subformat=fixed -S 0 build_hyperv_amd64\hyperv_amd64_esp.raw build_hyperv_amd64\hyperv_amd64_esp.vhdx
    if errorlevel 1 exit /b 1
    fsutil sparse setflag build_hyperv_amd64\hyperv_amd64_esp.vhd 0 >nul
    fsutil sparse setflag build_hyperv_amd64\hyperv_amd64_esp.vhdx 0 >nul
)
python tools\build_uefi_boot_iso.py --bootx64 build_hyperv_amd64\EFI\BOOT\BOOTX64.EFI --out build_hyperv_amd64\hyperv_amd64_boot.iso --label PIOS_HV_AMD64
if errorlevel 1 exit /b 1

for %%f in (build_hyperv_amd64\EFI\BOOT\BOOTX64.EFI) do echo BOOTX64.EFI size: %%~zf bytes
for %%f in (build_hyperv_amd64\hyperv_amd64_esp.raw) do echo ESP raw image size: %%~zf bytes
if exist build_hyperv_amd64\hyperv_amd64_esp.vhd for %%f in (build_hyperv_amd64\hyperv_amd64_esp.vhd) do echo ESP VHD size: %%~zf bytes
if exist build_hyperv_amd64\hyperv_amd64_esp.vhdx for %%f in (build_hyperv_amd64\hyperv_amd64_esp.vhdx) do echo ESP VHDX size: %%~zf bytes
for %%f in (build_hyperv_amd64\hyperv_amd64_boot.iso) do echo Boot ISO size: %%~zf bytes
echo Hyper-V ESP staging dir: hyperv_amd64_esp\EFI\BOOT\BOOTX64.EFI
echo Hyper-V Gen2 boot disk: build_hyperv_amd64\hyperv_amd64_esp.vhdx
echo Hyper-V Gen2 boot ISO:  build_hyperv_amd64\hyperv_amd64_boot.iso
