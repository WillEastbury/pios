"""Pin the no-sparse-after-copy requirement for the Hyper-V x64 launcher."""
from pathlib import Path

script = (Path(__file__).resolve().parent.parent / "tools" /
          "create_hyperv_amd64_vm.ps1").read_text(encoding="utf-8")
copy = script.index("Copy-Item -Force $SourceVhdx $vmDisk")
clear = script.index("fsutil sparse setflag $vmDisk 0", copy)
uncompress = script.index("compact.exe /U /I $vmDisk", clear)
attrs = script.index("Get-Item $vmDisk", uncompress)
attach = script.index("New-VM -Name $Name", attrs)
assert copy < clear < uncompress < attrs < attach
assert "SparseFile" in script[attrs:attach] and "Compressed" in script[attrs:attach]
assert "not full PIOS OS" in script
print("Hyper-V launcher: copied VHDX sparse/compressed attributes are cleared and checked before attach")
