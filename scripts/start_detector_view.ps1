$ErrorActionPreference = "Stop"

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$SimRoot = Join-Path $RepoRoot "drone_simulation"
$ViewHtml = Join-Path $RepoRoot "artifacts\tello_live_view.html"
$DetectorLog = "/tmp/aruco_detector_wsl.log"
$LiveViewLog = "/tmp/aruco_live_view_wsl.log"

function Invoke-WslRos {
  param([string]$Command)
  $bash = @"
cd /mnt/c/Users/aaron/Desktop/cs106/106a_final_project/drone_simulation
source /opt/ros/humble/setup.bash
source install_wsl/setup.bash
$Command
"@
  wsl.exe bash -lc $bash
}

Invoke-WslRos @"
pkill -f '[a]ruco_detector' || true
pkill -f '[a]ruco_live_view.py' || true
sleep 1
"@

Invoke-WslRos @"
rm -f $DetectorLog $LiveViewLog
nohup ros2 run my_drone_vision aruco_detector > $DetectorLog 2>&1 < /dev/null &
nohup python3 /mnt/c/Users/aaron/Desktop/cs106/106a_final_project/scripts/aruco_live_view.py > $LiveViewLog 2>&1 < /dev/null &
sleep 2
ros2 node list
"@

Start-Process $ViewHtml
Write-Host "Tello camera window opened: $ViewHtml"
Write-Host "Detector log: $DetectorLog"
Write-Host "Live view log: $LiveViewLog"
