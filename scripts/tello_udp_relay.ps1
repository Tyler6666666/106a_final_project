param(
  [string]$TelloLocalIp = "192.168.10.2",
  [int]$TelloCommandPort = 8889,
  [int]$RelayListenPort = 18889,
  [string]$TelloIp = "192.168.10.1",
  [string]$ResponseIp = "",
  [int]$ResponsePort = 8889
)

$ErrorActionPreference = "Stop"

$relay = [System.Net.Sockets.UdpClient]::new($RelayListenPort)
$telloLocal = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Parse($TelloLocalIp), $TelloCommandPort)
$tello = [System.Net.Sockets.UdpClient]::new($telloLocal)
$tello.Client.ReceiveTimeout = 1500
$telloRemote = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Parse($TelloIp), $TelloCommandPort)

Write-Output "Tello UDP relay listening on 0.0.0.0:$RelayListenPort, forwarding via ${TelloLocalIp}:$TelloCommandPort to ${TelloIp}:$TelloCommandPort"
if ($ResponseIp -ne "") {
  Write-Output "Tello UDP relay command responses will be sent to ${ResponseIp}:$ResponsePort"
}

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
      if ($ResponseIp -ne "") {
        $responseTarget = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Parse($ResponseIp), $ResponsePort)
      } else {
        $responseTarget = $client
      }
      Write-Output "tello -> driver from $($fromTello.Address):$($fromTello.Port): $responseText; relay response to $($responseTarget.Address):$($responseTarget.Port)"
      [void]$relay.Send($response, $response.Length, $responseTarget)
    } catch [System.Net.Sockets.SocketException] {
      Write-Output "tello response timeout for: $cmdText"
    }
  }
}
finally {
  $relay.Close()
  $tello.Close()
}
