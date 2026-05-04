param(
    [int]$MarkerId = 0,
    [double]$MarkerSize = 0.15,
    [double]$TargetDist = 0.45
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$ScriptPath = Join-Path $RepoRoot "scripts\tello_aruco_direct.py"

Write-Host "Stopping WSL ROS nodes that use Tello ports..."
wsl.exe bash -lc "pkill -f '[t]ello_driver' || true; pkill -f '[t]ello_joy_main' || true; pkill -f '[a]ruco_detector' || true; pkill -f '[a]ruco_controller' || true; pkill -f '[a]ruco_live_view.py' || true; pkill -f '[j]oy_node' || true"

Write-Host "Stopping Windows UDP bridge processes on Tello ports..."
$ports = @(8889, 8890, 11111, 18889)
$processIds = Get-NetUDPEndpoint -ErrorAction SilentlyContinue |
    Where-Object { $ports -contains $_.LocalPort } |
    Select-Object -ExpandProperty OwningProcess -Unique

foreach ($processId in $processIds) {
    if ($processId -and $processId -ne $PID) {
        try {
            Stop-Process -Id $processId -Force -ErrorAction Stop
            Write-Host "Stopped PID $processId"
        } catch {
            Write-Warning "Could not stop PID ${processId}: $($_.Exception.Message)"
        }
    }
}

Write-Host "Starting direct Tello ArUco window. Focus the OpenCV window, then press:"
Write-Host "  e = takeoff, then follow after 1s"
Write-Host "  f = toggle follow"
Write-Host "  space = stop follow / hover"
Write-Host "  q = land and quit"

$argsList = @(
    "-3",
    $ScriptPath,
    "--marker-id", "$MarkerId",
    "--marker-size", "$MarkerSize",
    "--target-dist", "$TargetDist"
)

Start-Process -FilePath "py" -ArgumentList $argsList -WorkingDirectory $RepoRoot
