@echo off
setlocal
set "OCLOC=C:\Program Files (x86)\Intel\oneAPI\ocloc\2026.0\bin\ocloc.exe"
if not exist "%OCLOC%" exit /b 1
if not exist build_module\b50_topk mkdir build_module\b50_topk
"%OCLOC%" compile -file modules\b50_fused_topk.cl -device 0xe212 -64 -options "-cl-std=CL1.2" -out_dir build_module\b50_topk -output b50_fused_topk -output_no_suffix
if errorlevel 1 exit /b 1
"%OCLOC%" disasm -file build_module\b50_topk\b50_fused_topk.bin -dump build_module\b50_topk\disasm -device bmg-g21
if errorlevel 1 exit /b 1
python tools\check_b50_topk.py
if errorlevel 1 exit /b 1
echo Offline B50 top-k compiled and metadata captured. No GPU execution or activation.
