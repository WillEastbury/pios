# Elevated packet capture of a PIOS bulk-download stall.
# Run in an ADMIN PowerShell:  powershell -ExecutionPolicy Bypass -File C:\source\pios\tools\capture_stall.ps1
# Produces C:\source\pios\tools\stall_capture.txt  (agent reads this).

$ErrorActionPreference = 'SilentlyContinue'
$board = '192.168.0.201'
$etl   = 'C:\source\pios\tools\stall_capture.etl'
$txt   = 'C:\source\pios\tools\stall_capture.txt'

Write-Host "Resetting pktmon..."
pktmon stop | Out-Null
pktmon filter remove | Out-Null
pktmon filter add PIOS -i $board | Out-Null

Remove-Item $etl,$txt -ErrorAction SilentlyContinue
Write-Host "Starting capture (full packet headers)..."
pktmon start --capture --pkt-size 160 --file-name $etl --file-size 64 | Out-Null
Start-Sleep -Milliseconds 500

# Trigger the stall: large GET (/picoscript ~200KB) via raw socket, read until timeout.
Write-Host "Triggering /picoscript download (expect stall)..."
$ip = [Net.IPAddress]::Parse($board)
$tot = 0; $reads = 0
try {
  $req = [Text.Encoding]::ASCII.GetBytes("GET /picoscript HTTP/1.0`r`nHost: $board`r`nConnection: close`r`n`r`n")
  $tcp = New-Object Net.Sockets.TcpClient
  $tcp.Connect($ip, 80)
  $tcp.ReceiveTimeout = 8000
  $s = $tcp.GetStream(); $s.Write($req,0,$req.Length)
  $buf = New-Object byte[] 8192
  while ($true) { $n = $s.Read($buf,0,8192); if ($n -le 0) { break }; $tot += $n; $reads++ }
} catch { Write-Host "  (read ended: stall/timeout)" }
finally { if ($tcp) { $tcp.Close() } }
Write-Host "Downloaded $tot bytes in $reads reads."

Start-Sleep -Milliseconds 800
Write-Host "Stopping capture..."
pktmon stop | Out-Null
Write-Host "Converting to text..."
pktmon etl2txt $etl -o $txt | Out-Null

if (Test-Path $txt) {
  $lines = (Get-Content $txt | Measure-Object -Line).Lines
  Write-Host "DONE -> $txt ($lines lines)"
} else {
  Write-Host "etl2txt failed; try: pktmon etl2txt $etl -o $txt"
}
