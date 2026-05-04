param(
  [string]$TelloLocalIp = "192.168.10.2",
  [string]$TelloIp = "192.168.10.1",
  [string]$WslIp,
  [int]$CommandListenPort = 18889,
  [int]$CommandResponsePort = 38065,
  [int]$StatePort = 8890,
  [int]$VideoPort = 11111
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($WslIp)) {
  throw "WslIp is required"
}

function New-UdpSender {
  return [System.Net.Sockets.UdpClient]::new(0)
}

function Start-Forwarder {
  param(
    [string]$Name,
    [string]$BindIp,
    [int]$ListenPort,
    [string]$TargetIp,
    [int]$TargetPort
  )

  Start-Job -ArgumentList @($Name, $BindIp, $ListenPort, $TargetIp, $TargetPort) -ScriptBlock {
    param($Name, $BindIp, $ListenPort, $TargetIp, $TargetPort)
    $receiver = [System.Net.Sockets.UdpClient]::new(
      [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Parse($BindIp), $ListenPort)
    )
    $sender = [System.Net.Sockets.UdpClient]::new(0)
    $target = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Parse($TargetIp), $TargetPort)
    Write-Output "$Name forwarding ${BindIp}:$ListenPort -> ${TargetIp}:$TargetPort"
    try {
      while ($true) {
        $remote = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
        $data = $receiver.Receive([ref]$remote)
        [void]$sender.Send($data, $data.Length, $target)
      }
    } finally {
      $receiver.Close()
      $sender.Close()
    }
  }
}

$jobs = @()
$jobs += Start-Forwarder -Name "state" -BindIp "0.0.0.0" -ListenPort $StatePort -TargetIp $WslIp -TargetPort $StatePort
$jobs += Start-Forwarder -Name "video" -BindIp "0.0.0.0" -ListenPort $VideoPort -TargetIp $WslIp -TargetPort $VideoPort

$relay = [System.Net.Sockets.UdpClient]::new($CommandListenPort)
$telloLocal = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Parse($TelloLocalIp), 8889)
$tello = [System.Net.Sockets.UdpClient]::new($telloLocal)
$tello.Client.ReceiveTimeout = 1500
$telloRemote = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Parse($TelloIp), 8889)
$commandResponseTarget = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Parse($WslIp), $CommandResponsePort)

Write-Output "command forwarding 0.0.0.0:$CommandListenPort -> ${TelloIp}:8889 via ${TelloLocalIp}:8889"
Write-Output "command responses -> ${WslIp}:$CommandResponsePort"

try {
  while ($true) {
    $client = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
    $cmd = $relay.Receive([ref]$client)
    $cmdText = [System.Text.Encoding]::ASCII.GetString($cmd)
    Write-Output "driver -> tello from $($client.Address):$($client.Port): $cmdText"

    [void]$tello.Send($cmd, $cmd.Length, $telloRemote)

    try {
      $fromTello = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
      $response = $tello.Receive([ref]$fromTello)
      $responseText = [System.Text.Encoding]::ASCII.GetString($response)
      Write-Output "tello -> driver from $($fromTello.Address):$($fromTello.Port): $responseText"
      [void]$relay.Send($response, $response.Length, $commandResponseTarget)
    } catch [System.Net.Sockets.SocketException] {
      Write-Output "tello response timeout for: $cmdText"
    }
  }
} finally {
  $relay.Close()
  $tello.Close()
  foreach ($job in $jobs) {
    Stop-Job $job -ErrorAction SilentlyContinue
    Remove-Job $job -Force -ErrorAction SilentlyContinue
  }
}
