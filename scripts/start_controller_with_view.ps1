$ErrorActionPreference = "Stop"

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$ViewScript = Join-Path $PSScriptRoot "start_detector_view.ps1"
$ControllerLog = "/tmp/aruco_controller_wsl.log"

& $ViewScript

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
pkill -f '[a]ruco_controller' || true
sleep 1
"@

Invoke-WslRos @"
rm -f $ControllerLog
nohup ros2 run my_drone_vision aruco_controller --ros-args \
  -p auto_takeoff:=true \
  -p takeoff_height:=0.5 \
  -p follow_after_takeoff_sec:=1.0 \
  -p target_dist:=0.45 \
  -p land_on_tag_loss:=true \
  -p tag_loss_land_sec:=1.0 \
  -p enable_search:=false \
  -p require_tag_before_takeoff:=true \
  > $ControllerLog 2>&1 < /dev/null &
sleep 2
ros2 node list
tail -80 $ControllerLog 2>/dev/null || true
"@

Write-Host "Controller started with guarded takeoff."
Write-Host "Controller log: $ControllerLog"
