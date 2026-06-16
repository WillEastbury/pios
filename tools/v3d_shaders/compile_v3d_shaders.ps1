param(
    [string]$MesaCompiler = "v3d_compiler",
    [string]$Glslang = "",
    [string]$OutDir = "build\v3d_shaders"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$root = Split-Path -Parent $root
$out = Join-Path $root $OutDir
New-Item -ItemType Directory -Force -Path $out | Out-Null

if (-not $Glslang) {
    $cmd = Get-Command glslangValidator -ErrorAction SilentlyContinue
    if ($cmd) {
        $Glslang = $cmd.Source
    } else {
        $sdk = Get-ChildItem "C:\VulkanSDK" -Directory -ErrorAction SilentlyContinue |
            Sort-Object Name -Descending | Select-Object -First 1
        if ($sdk) {
            $candidate = Join-Path $sdk.FullName "Bin\glslangValidator.exe"
            if (Test-Path $candidate) {
                $Glslang = $candidate
            }
        }
    }
}

if (-not $Glslang -or -not (Test-Path $Glslang)) {
    throw "glslangValidator not found. Install Vulkan SDK or pass -Glslang <path>."
}

foreach ($shader in @("vector_add", "relu")) {
    $src = Join-Path $PSScriptRoot "$shader.comp"
    $spv = Join-Path $out "$shader.spv"
    $dump = Join-Path $out "$shader.v3d.txt"

    & $Glslang -V $src -o $spv
    if ($LASTEXITCODE -ne 0) {
        throw "glslangValidator failed for $shader"
    }

    $mesaCmd = Get-Command $MesaCompiler -ErrorAction SilentlyContinue
    if ($mesaCmd) {
        & $MesaCompiler $spv | Set-Content -Encoding ASCII $dump
        if ($LASTEXITCODE -ne 0) {
            throw "v3d_compiler failed for $shader"
        }
    } else {
        "Mesa standalone v3d_compiler not found. SPIR-V generated at $spv." |
            Set-Content -Encoding ASCII $dump
    }
}

if (Test-Path (Join-Path $root "build\mesa_v3d_wrap.exe")) {
    foreach ($shader in @("vector_add", "relu")) {
        $qpu = Join-Path $out "$($shader)_builtin.qpu.txt"
        $log = Join-Path $out "$($shader)_builtin.log"
        & (Join-Path $root "build\mesa_v3d_wrap.exe") --builtin $shader 2> $log |
            Set-Content -Encoding ASCII $qpu
    }
}

Write-Host "V3D shader artifacts written to $out"
