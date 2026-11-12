# common.ps1: dot-sourced by drive.ps1 and scen.ps1 (Windows side of a HIL run: ESP32 on a COM port, sim in WSL).
# $Repo = the checkout, $W = out\ (logs), $WD = this folder as WSL sees it. PlatformIO's esptool from
# PLATFORMIO_CORE_DIR (default ~\.platformio).
$Repo = (Resolve-Path "$PSScriptRoot\..\..").Path
$W = "$PSScriptRoot\out"; New-Item -ItemType Directory -Force $W | Out-Null
$WD = (wsl -e wslpath -a ($PSScriptRoot -replace '\\', '/')).Trim()
$WHome = (wsl -e sh -c 'echo $HOME').Trim()
$Pio = if ($env:PLATFORMIO_CORE_DIR) { $env:PLATFORMIO_CORE_DIR } else { "$HOME\.platformio" }
$Py = "$Pio\penv\Scripts\python.exe"; $Esptool = "$Pio\packages\tool-esptoolpy\esptool.py"

function Wsl-Run([string]$Script) { wsl -e bash "$WD/$Script" @args }
# A bridge left from an earlier run holds the COM port (Stop-Process on the launcher's id misses the real python)
function Stop-Bridges { Get-CimInstance Win32_Process -Filter "Name like 'python%'" | Where-Object { $_.CommandLine -match 'serial_tcp_bridge' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue } }
function Stop-Sim { Stop-Bridges; Wsl-Run inst_kill.sh 0 2>$null | Out-Null }

# The sim (drive_sim.sh) in WSL; returns once the visualizer and the Gazebo bridge are up
function Start-Sim([string]$World, [string]$Gui, [string]$Robot = '', [string]$Style = 'Hidden') {
  Stop-Sim
  Wsl-Run dds_clean.sh 2>$null | Out-Null   # orphans of a stopped run + stale Fast DDS shm broke discovery (2026-11-01)
  Remove-Item "$W\inst0.log" -ErrorAction SilentlyContinue
  Start-Process wsl -ArgumentList "-e setsid -w bash $WD/drive_sim.sh $World $Gui $Robot" -WindowStyle $Style | Out-Null
  for ($i = 0; $i -lt 90; $i++) {
    Start-Sleep 1
    $log = Get-Content "$W\inst0.log" -Raw -ErrorAction SilentlyContinue
    if ($log -match 'Path visualizer ready' -and $log -match 'Creating GZ->ROS Bridge') { return $i }
  }
  throw "sim did not start in 90 s, see $W\inst0.log"
}

# Reset the board (it waits for the micro-ROS agent), start the serial-TCP bridge, wait for the EKF and the gyro
function Start-Link([string]$Port, [string]$Log = 'bridge') {
  & $Py $Esptool --port $Port --after hard_reset chip_id 2>&1 | Select-Object -Last 1 | Write-Host
  $ip = (wsl -e hostname -I).Trim().Split(' ')[0]
  $script:BridgeArgs = "-u `"$Repo\ros2_ws\src\sim_env\scripts\serial_tcp_bridge.py`" $Port --host $ip --port 8888"
  $script:BridgeLog = $Log
  $b = Start-Bridge
  Wsl-Run wait_imu.sh 0 | Write-Host
  return $b
}
function Start-Bridge { Start-Process python -ArgumentList $script:BridgeArgs -WindowStyle Hidden -PassThru -RedirectStandardOutput "$W\$($script:BridgeLog).log" -RedirectStandardError "$W\$($script:BridgeLog)_err.log" }

# Record to ~/bags/<name> in WSL (drive_rec.sh); returns the bag path or $null
function Start-Rec([string]$Name) {
  $bag = "$WHome/bags/$Name"
  for ($k = 0; $k -lt 2; $k++) {
    Start-Process wsl -ArgumentList "-e setsid -w bash $WD/drive_rec.sh $bag" -WindowStyle Hidden | Out-Null
    for ($j = 0; $j -lt 8; $j++) { Start-Sleep 1; wsl -e test -d $bag; if ($LASTEXITCODE -eq 0) { return $bag } }
  }
  return $null
}
