@echo off
REM Build one stage0 package with Pi 5, Pi 4, Pi 3, Zero 2 W, and shared IDE assets.
REM stage0 selects the payload from MIDR (A76/A72/A53) plus board-revision.

setlocal
call "%~dp0build_bootstrap.bat"
if errorlevel 1 exit /b 1

call "%~dp0build_pi4.bat"
if errorlevel 1 exit /b 1

call "%~dp0build_pi3.bat"
if errorlevel 1 exit /b 1

call "%~dp0build_pizero2w.bat"
if errorlevel 1 exit /b 1

echo Packaging autodetected multi-board stage2...
python tools\build_stage2_package.py ^
    --pi build_pi5_stage2\PIOS_PI5_STAGE2.BIN ^
    --pi4 build_pi4\kernel8_pi4.img ^
    --bcm2837 build_pi3\kernel8_pi3.img ^
    --pizero2w build_pizero2w\kernel8_pizero2w.img ^
    --shared assets\pios_shared_assets.bin ^
    --out real_kernel_multiboard.img
if errorlevel 1 exit /b 1

copy /Y real_kernel_multiboard.img PIOSSTG2.PKG >nul
if errorlevel 1 exit /b 1

echo MULTIBOARD BUILD COMPLETE
