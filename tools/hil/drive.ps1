# drive.ps1: drive the mower by hand in the HIL sim (ESP32 with HIL firmware + Gazebo in WSL + drive page)
# Usage: powershell -ExecutionPolicy Bypass -File tools\hil\drive.ps1 [-World course|edges|garden|fences|lawn|hills] [-Gui] [-Port COM10]
# Stop with Enter, or from another shell by creating tools\hil\out\drive_stop.
param([ValidateSet('edges', 'fences', 'lawn', 'hills', 'garden', 'course')][string]$World = 'course', [switch]$Gui, [string]$Port = 'COM10')
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\common.ps1"
if (-not ([System.IO.Ports.SerialPort]::GetPortNames() -contains $Port)) { throw "ESP32 not found on ${Port}: plug it in (HIL firmware) and run again" }

Write-Host "Starting the $World sim (Gazebo window: $($Gui.IsPresent)) ..."
$g = if ($Gui) { 'true' } else { 'false' }
Start-Sim $World $g '' 'Minimized' | Out-Null
Write-Host 'Resetting the ESP32, waiting for the EKF ...'
$bridge = Start-Link $Port 'bridge_drive'
$bag = Start-Rec "drive_$(Get-Date -Format yyyyMMdd_HHmmss)"
if ($bag) {
  Start-Process wsl -ArgumentList "-e setsid bash $WD/cpu_log.sh $bag/cpu_wsl.log" -WindowStyle Hidden | Out-Null
  Write-Host "Recording this drive to $bag (inside WSL)"
} else { Write-Host 'WARNING: the recorder did not start, this drive is not recorded' -ForegroundColor Yellow }
Start-Process 'http://127.0.0.1:8080/'
Write-Host ''
Write-Host 'Drive page: http://127.0.0.1:8080/  (phone on the same Wi-Fi: http://<this PC ip>:8080/)'
Write-Host 'Press Take over, then drive with the stick or W A S D (the wall assist is always on).'
Write-Host 'Press Enter (in this window) to stop everything.'
# Watch the link: a bridge that dies (COM port error, crash) is restarted; the board reconnects by itself.
# Windows side CPU every 2 s (with the bag's cpu_wsl.log: what slowed the sim).
$restarts = 0
$cpuLog = "$W\drive_cpu_win.log"; Remove-Item $cpuLog -ErrorAction SilentlyContinue
$prev = @{}; $tick = 0
while ($true) {
  if ((++$tick % 4) -eq 0) {
    $now = @{}; Get-Process | ForEach-Object { if ($_.CPU) { $now["$($_.Id)"] = @($_.ProcessName, $_.CPU) } }
    $top = $now.Keys | Where-Object { $prev[$_] } | ForEach-Object { [pscustomobject]@{ n = $now[$_][0]; c = ($now[$_][1] - $prev[$_][1]) / 2 * 100 } } | Sort-Object c -Descending | Select-Object -First 6
    Add-Content $cpuLog ("== $(Get-Date -Format HH:mm:ss) " + (($top | ForEach-Object { "$($_.n) $([int]$_.c)%" }) -join ', '))
    $prev = $now
  }
  try { if ([Console]::KeyAvailable -and [Console]::ReadKey($true).Key -eq 'Enter') { break } } catch { }
  if (Test-Path "$W\drive_stop") { Remove-Item "$W\drive_stop"; break }
  if ($bridge.HasExited) {
    $restarts++
    Copy-Item "$W\bridge_drive_err.log" "$W\bridge_drive_err_$restarts.log" -ErrorAction SilentlyContinue
    Write-Host "$(Get-Date -Format HH:mm:ss) serial bridge stopped (restart $restarts, its log: out\bridge_drive_err_$restarts.log), restarting it" -ForegroundColor Yellow
    $bridge = Start-Bridge
  }
  Start-Sleep -Milliseconds 500
}
Stop-Sim
Write-Host "Stopped. Score the drive: wsl -e bash $WD/score_run.sh $bag $World"
