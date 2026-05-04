param(
  [string]$TelloLocalIp = "192.168.10.2",
  [string]$TelloIp = "192.168.10.1",
  [int]$MaxHeightCm = 55,
  [double]$HoverSeconds = 5.0
)

$ErrorActionPreference = "Stop"

function Parse-State {
  param([string]$Raw)
  $state = @{}
  foreach ($part in $Raw.Trim().Split(';')) {
    if (-not $part.Contains(':')) { continue }
    $kv = $part.Split(':', 2)
    $state[$kv[0]] = $kv[1]
  }
  return $state
}

function Send-TelloCommand {
  param(
    [System.Net.Sockets.UdpClient]$Socket,
    [System.Net.IPEndPoint]$Remote,
    [string]$Command,
    [int]$TimeoutMs = 5000
  )
  $Socket.Client.ReceiveTimeout = $TimeoutMs
  $bytes = [System.Text.Encoding]::ASCII.GetBytes($Command)
  [void]$Socket.Send($bytes, $bytes.Length, $Remote)
  try {
    $from = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
    $reply = $Socket.Receive([ref]$from)
    $text = [System.Text.Encoding]::ASCII.GetString($reply).Trim()
    Write-Host "SDK '$Command' -> '$text'"
    return $text
  } catch [System.Net.Sockets.SocketException] {
    Write-Host "SDK '$Command' -> timeout"
    return "timeout"
  }
}

function Read-State {
  param(
    [System.Net.Sockets.UdpClient]$StateSocket,
    [int]$TimeoutMs = 3000
  )
  $StateSocket.Client.ReceiveTimeout = $TimeoutMs
  try {
    $from = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
    $bytes = $StateSocket.Receive([ref]$from)
    $raw = [System.Text.Encoding]::ASCII.GetString($bytes)
    return Parse-State $raw
  } catch [System.Net.Sockets.SocketException] {
    return $null
  }
}

$cmdLocal = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Parse($TelloLocalIp), 8889)
$cmdRemote = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Parse($TelloIp), 8889)
$cmdSocket = [System.Net.Sockets.UdpClient]::new($cmdLocal)
$stateSocket = [System.Net.Sockets.UdpClient]::new(8890)

try {
  Write-Output "Using Tello local IP $TelloLocalIp"
  $reply = Send-TelloCommand $cmdSocket $cmdRemote "command" 5000
  if ($reply -notmatch "ok") {
    Write-Output "ABORT: command mode not accepted"
    exit 2
  }

  $state = $null
  $deadline = (Get-Date).AddSeconds(6)
  while ((Get-Date) -lt $deadline -and $null -eq $state) {
    $state = Read-State $stateSocket 1000
  }
  if ($null -eq $state) {
    Write-Output "ABORT: no Tello state on UDP 8890"
    exit 3
  }

  $bat = [int]$state["bat"]
  $templ = [int]$state["templ"]
  $temph = [int]$state["temph"]
  $tof = [int]$state["tof"]
  $h = [int]$state["h"]
  Write-Output "PRECHECK bat=$bat temp=$templ-$temph h=$h tof=$tof"
  if ($bat -lt 25) {
    Write-Output "ABORT: battery below 25%"
    exit 4
  }
  if ($temph -ge 90) {
    Write-Output "ABORT: high temperature"
    exit 5
  }

  Write-Output "TAKEOFF_START"
  $reply = Send-TelloCommand $cmdSocket $cmdRemote "takeoff" 12000
  if ($reply -notmatch "ok") {
    Write-Output "ABORT: takeoff not accepted"
    [void](Send-TelloCommand $cmdSocket $cmdRemote "land" 3000)
    exit 6
  }

  $start = Get-Date
  $landed = $false
  while (((Get-Date) - $start).TotalSeconds -lt $HoverSeconds) {
    $state = Read-State $stateSocket 500
    if ($null -ne $state) {
      $tof = [int]$state["tof"]
      $h = [int]$state["h"]
      $bat = [int]$state["bat"]
      $height = [Math]::Max($tof, $h)
      $elapsed = [Math]::Round(((Get-Date) - $start).TotalSeconds, 1)
      Write-Output "HOVER t=${elapsed}s h=$h tof=$tof bat=$bat"
      if ($height -gt $MaxHeightCm) {
        Write-Output "HEIGHT_GUARD: ${height}cm > ${MaxHeightCm}cm, landing now"
        [void](Send-TelloCommand $cmdSocket $cmdRemote "land" 8000)
        $landed = $true
        break
      }
    }
    [void](Send-TelloCommand $cmdSocket $cmdRemote "rc 0 0 0 0" 600)
    Start-Sleep -Milliseconds 250
  }

  if (-not $landed) {
    Write-Output "LAND_START"
    [void](Send-TelloCommand $cmdSocket $cmdRemote "land" 10000)
  }

  $deadline = (Get-Date).AddSeconds(10)
  while ((Get-Date) -lt $deadline) {
    $state = Read-State $stateSocket 1000
    if ($null -ne $state) {
      $tof = [int]$state["tof"]
      $h = [int]$state["h"]
      $bat = [int]$state["bat"]
      Write-Output "POST h=$h tof=$tof bat=$bat"
      if ($h -eq 0 -and $tof -le 20) { break }
    }
  }
} finally {
  try { [void](Send-TelloCommand $cmdSocket $cmdRemote "rc 0 0 0 0" 500) } catch {}
  $cmdSocket.Close()
  $stateSocket.Close()
}
