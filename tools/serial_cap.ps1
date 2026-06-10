param(
  [string]$Port = "COM17",
  [int]$Baud = 115200,
  [int]$Seconds = 200,
  [string]$Out = "C:\source\pios\com17_cap.txt"
)
"=== cap start $(Get-Date -Format o) port=$Port baud=$Baud ===" | Out-File -FilePath $Out -Encoding ascii
try {
  $sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,None,8,one
  $sp.ReadTimeout = 500
  $sp.Open()
  $sw = [System.Diagnostics.Stopwatch]::StartNew()
  $buf = New-Object System.Text.StringBuilder
  while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try {
      $chunk = $sp.ReadExisting()
      if ($chunk -and $chunk.Length -gt 0) {
        [void]$buf.Append($chunk)
        Add-Content -Path $Out -Value $chunk -NoNewline -Encoding ascii
      } else {
        Start-Sleep -Milliseconds 50
      }
    } catch { Start-Sleep -Milliseconds 50 }
  }
  $sp.Close()
  Add-Content -Path $Out -Value "`n=== cap end ===" -Encoding ascii
} catch {
  Add-Content -Path $Out -Value ("OPEN ERR: " + $_.Exception.Message) -Encoding ascii
}
