$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$ScriptPath = Join-Path $RepoRoot "scripts\tello_aruco_simple_follow.py"

Write-Host "Stopping old Tello processes..."
wsl.exe bash -lc "pkill -f '[t]ello_driver' || true; pkill -f '[t]ello_joy_main' || true; pkill -f '[a]ruco_detector' || true; pkill -f '[a]ruco_controller' || true; pkill -f '[a]ruco_live_view.py' || true; pkill -f '[j]oy_node' || true"

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

Write-Host "Starting simple ArUco follow."
Write-Host "Keys in OpenCV window:"
Write-Host "  e = takeoff and follow"
Write-Host "  f = toggle follow"
Write-Host "  space = stop / hover"
Write-Host "  q = land and quit"

Start-Process -FilePath "py" -ArgumentList @("-3", $ScriptPath) -WorkingDirectory $RepoRoot
