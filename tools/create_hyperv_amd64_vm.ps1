param(
  [string]$Name = "PIOS-HyperV-amd64",
  [string]$SourceVhdx = "C:\source\pios\build_hyperv_amd64\hyperv_amd64_esp.vhdx",
  [string]$Iso = "C:\source\pios\build_hyperv_amd64\hyperv_amd64_boot.iso",
  [string]$VmRoot = "C:\source\pios\build_hyperv_amd64\vm",
  [UInt64]$MemoryStartupBytes = 512MB,
  [switch]$UseIso,
  [switch]$Recreate,
  [switch]$Start
)

$ErrorActionPreference = "Stop"

function Assert-Admin {
  $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
  $principal = [Security.Principal.WindowsPrincipal]::new($identity)
  if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw "Hyper-V VM creation requires an elevated PowerShell session."
  }
}

Assert-Admin

if (-not (Get-Command New-VM -ErrorAction SilentlyContinue)) {
  throw "Hyper-V PowerShell cmdlets are not available."
}

if ($UseIso) {
  if (-not (Test-Path $Iso)) { throw "ISO not found: $Iso. Run .\build_hyperv_amd64.bat first." }
} else {
  if (-not (Test-Path $SourceVhdx)) { throw "VHDX not found: $SourceVhdx. Run .\build_hyperv_amd64.bat first." }
  $attrs = (Get-Item $SourceVhdx).Attributes
  if (($attrs -band [IO.FileAttributes]::SparseFile) -ne 0) {
    throw "VHDX is marked SparseFile and Hyper-V may fail with 0xc03a001a. Re-run .\build_hyperv_amd64.bat or: fsutil sparse setflag `"$SourceVhdx`" 0"
  }
}

$existing = Get-VM -Name $Name -ErrorAction SilentlyContinue
if ($existing) {
  if (-not $Recreate) {
    throw "VM '$Name' already exists. Pass -Recreate to replace it."
  }
  if ($existing.State -ne "Off") {
    Stop-VM -Name $Name -TurnOff -Force
  }
  Remove-VM -Name $Name -Force
}

New-Item -ItemType Directory -Force -Path $VmRoot | Out-Null
$pipePath = "\\.\pipe\pios-hv-amd64-com1"

if ($UseIso) {
  $vm = New-VM -Name $Name -Generation 2 -MemoryStartupBytes $MemoryStartupBytes -Path $VmRoot -NoVHD
  Add-VMDvdDrive -VMName $Name -Path $Iso
  $dvd = Get-VMDvdDrive -VMName $Name
  Set-VMFirmware -VMName $Name -EnableSecureBoot Off -FirstBootDevice $dvd
} else {
  $vmDisk = Join-Path $VmRoot "$Name-boot.vhdx"
  Copy-Item -Force $SourceVhdx $vmDisk
  $vm = New-VM -Name $Name -Generation 2 -MemoryStartupBytes $MemoryStartupBytes -Path $VmRoot -VHDPath $vmDisk
  $disk = Get-VMHardDiskDrive -VMName $Name
  Set-VMFirmware -VMName $Name -EnableSecureBoot Off -FirstBootDevice $disk
}

Set-VMProcessor -VMName $Name -Count 2
Set-VMComPort -VMName $Name -Number 1 -Path $pipePath
Set-VM -Name $Name -AutomaticCheckpointsEnabled $false

Write-Host "Created Hyper-V Gen2 VM: $Name"
Write-Host "Secure Boot: disabled (BOOTX64.EFI is unsigned)"
Write-Host "Console: use VMConnect for UEFI text output"
Write-Host "COM1 pipe: $pipePath"
if ($UseIso) {
  Write-Host "Boot ISO: $Iso"
} else {
  Write-Host "Boot VHDX: $vmDisk"
}

if ($Start) {
  Start-VM -Name $Name
  Write-Host "Started VM: $Name"
}
