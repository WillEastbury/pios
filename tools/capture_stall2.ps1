# Elevated packet capture bracketing a PIOS bulk-download stall.
# Run in an ADMIN PowerShell:
#   powershell -ExecutionPolicy Bypass -File C:\source\pios\tools\capture_stall2.ps1
# Produces C:\source\pios\tools\stall_capture.txt  (agent reads this).

$ErrorActionPreference = 'SilentlyContinue'
$board = '192.168.0.201'
$etl   = 'C:\source\pios\tools\stall_capture.etl'
$txt   = 'C:\source\pios\tools\stall_capture.txt'
$ip    = [Net.IPAddress]::Parse($board)

function Get-Small($path) {
  $tot=0
  try {
    $req=[Text.Encoding]::ASCII.GetBytes("GET $path HTTP/1.0`r`nHost: $board`r`nConnection: close`r`n`r`n")
    $t=New-Object Net.Sockets.TcpClient; $t.Connect($ip,80); $t.ReceiveTimeout=4000
    $s=$t.GetStream(); $s.Write($req,0,$req.Length); $b=New-Object byte[] 8192
    while($true){$n=$s.Read($b,0,8192); if($n-le0){break}; $tot+=$n}
  } catch {} finally { if($t){$t.Close()} }
  return $tot
}

pktmon stop | Out-Null
pktmon filter remove | Out-Null
pktmon filter add PIOS -i $board | Out-Null
Remove-Item $etl,$txt -ErrorAction SilentlyContinue

Write-Host "Starting capture..."
pktmon start --capture --pkt-size 160 --file-name $etl --file-size 64 | Out-Null
Start-Sleep -Milliseconds 400

Write-Host "PRE-CHECK (board alive?):"
1..3 | ForEach-Object { $n=Get-Small "/api/status"; Write-Host "  /api/status -> $n bytes"; Start-Sleep -Milliseconds 300 }

Write-Host "BIG DOWNLOAD (/picoscript, expect stall):"
$tot=0;$reads=0
try {
  $req=[Text.Encoding]::ASCII.GetBytes("GET /picoscript HTTP/1.0`r`nHost: $board`r`nConnection: close`r`n`r`n")
  $t=New-Object Net.Sockets.TcpClient; $t.Connect($ip,80); $t.ReceiveTimeout=8000
  $s=$t.GetStream(); $s.Write($req,0,$req.Length); $b=New-Object byte[] 8192
  while($true){$n=$s.Read($b,0,8192); if($n-le0){break}; $tot+=$n; $reads++}
} catch { Write-Host "  (ended: stall/timeout)" } finally { if($t){$t.Close()} }
Write-Host "  got $tot bytes in $reads reads"

Write-Host "POST-CHECK (did big download wedge it?):"
1..3 | ForEach-Object { $n=Get-Small "/api/status"; Write-Host "  /api/status -> $n bytes"; Start-Sleep -Milliseconds 300 }

Start-Sleep -Milliseconds 600
pktmon stop | Out-Null
pktmon etl2txt $etl -o $txt | Out-Null
if (Test-Path $txt) { Write-Host "DONE -> $txt ($((Get-Content $txt|Measure-Object -Line).Lines) lines)" }
else { Write-Host "etl2txt failed" }
