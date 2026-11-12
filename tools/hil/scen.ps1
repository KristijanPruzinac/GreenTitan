# scen.ps1 -Name a[,b,...] [-Port COM10]: scripted HIL drive tests (tools/drive_test), one Gazebo start each: sim spawned
# at the scenario's pose, ESP reset, bridge, EKF wait, recording, the drive through the drive page, stop, score.
# Output: the bag (WSL ~/bags/scen_<name>_<time>) holds score.json and score.png, a copy goes to out\scen\.
# With several names each run logs to out\scen_<name>.log and the PASS/FAIL lines to out\scen_batch.log.
param([Parameter(Mandatory)][string[]]$Name, [string]$Port = 'COM10')
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\common.ps1"
$Name = @($Name | ForEach-Object { $_ -split ',' } | Where-Object { $_ })   # -File passes "a,b" as one string

if ($Name.Count -gt 1) {
  $B = "$W\scen_batch.log"
  "batch start $(Get-Date -Format HH:mm:ss): $($Name -join ' ')" | Out-File $B -Encoding utf8
  foreach ($n in $Name) {
    "== $n $(Get-Date -Format HH:mm:ss)" | Out-File $B -Append -Encoding utf8
    powershell -ExecutionPolicy Bypass -File $PSCommandPath -Name $n -Port $Port *> "$W\scen_$n.log"
    Get-Content "$W\scen_$n.log" | Where-Object { $_ -match '^(PASS|FAIL) |^\s+\w+ (PASS|FAIL):|pose |map |continuity |dock |Exception|NOT ' } | Out-File $B -Append -Encoding utf8
  }
  "batch done $(Get-Date -Format HH:mm:ss)" | Out-File $B -Append -Encoding utf8
  exit
}

$n = $Name[0]
$world = Wsl-Run scen_prep.sh $n | Select-Object -Last 1
if ($LASTEXITCODE -or -not $world) { throw "no scenario $n" }
$world = $world.Trim()
$i = Start-Sim $world 'false' "$WD/out/scen_robot.yaml"
Write-Host "sim up $i s ($n)"
Start-Link $Port | Out-Null
$bag = Start-Rec "scen_${n}_$(Get-Date -Format yyyyMMdd_HHmmss)"
if (-not $bag) { Stop-Sim; throw 'recorder did not start' }
Write-Host "recording $bag"
python -u "$Repo\tools\drive_test\drive.py" $n
Stop-Sim
Start-Sleep 2
Write-Host 'scoring'
Wsl-Run scen_score.sh $bag $n
