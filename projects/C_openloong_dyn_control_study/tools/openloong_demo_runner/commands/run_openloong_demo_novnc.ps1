[CmdletBinding(PositionalBinding = $false)]
param(
    [string]$DistroName = "Ubuntu-22.04-ProjectC",
    [string]$RepoPath = "/mnt/d/project/Robot_Dynamics_Control",
    [string]$WslExe = "wsl.exe",
    [switch]$BuildImage,
    [switch]$Foreground,
    [switch]$NoOpenBrowser,
    [string]$Name = "",
    [int]$Port = 6080,
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$DemoName
)

$ErrorActionPreference = "Stop"

# Windows 侧 noVNC 启动器。
# 输入：demo 名称。
# 输出：浏览器访问地址 http://localhost:<Port>/vnc.html?autoconnect=true&resize=remote
# 数学/控制逻辑不变；这里只改变可视化通道，绕开 WSLg COPY MODE 异常。

$launcher = "projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/run_openloong_demo_novnc.sh"
$args = @()
if ($BuildImage) {
    $args += "--build-image"
}
if (-not $Foreground) {
    $args += "--hosted"
}
if ($Name -ne "") {
    $args += "--name"
    $args += $Name
}
$args += "--port"
$args += [string]$Port
$args += $DemoName

$forwardedArgs = ""
foreach ($arg in $args) {
    $escaped = $arg.Replace("'", "'\''")
    $forwardedArgs += " '$escaped'"
}

$bashCommand = "cd ${RepoPath} && exec bash ${launcher}${forwardedArgs}"
$url = "http://localhost:$Port/vnc.html?autoconnect=true&resize=remote"

if ($Foreground) {
    & $WslExe -d $DistroName -- bash -lc $bashCommand
    exit $LASTEXITCODE
}

$logDir = Join-Path $env:TEMP "projectc-openloong-novnc"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$logPath = Join-Path $logDir ("{0}_{1:yyyyMMdd_HHmmss}.launcher.txt" -f $DemoName, (Get-Date))
$hostScriptPath = Join-Path $logDir ("host_{0}_{1:yyyyMMdd_HHmmss}.ps1" -f $DemoName, (Get-Date))
$hostScript = @"
`$ErrorActionPreference = 'Stop'
& '$WslExe' -d '$DistroName' -- bash -lc '$($bashCommand.Replace("'", "''"))'
"@
Set-Content -LiteralPath $hostScriptPath -Encoding UTF8 -Value $hostScript
$process = Start-Process -FilePath "powershell.exe" `
    -ArgumentList @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $hostScriptPath) `
    -WindowStyle Hidden `
    -PassThru

Write-Host "Demo: $DemoName"
Write-Host "WSL host process id: $($process.Id)"
Write-Host "Host script: $hostScriptPath"
Write-Host "noVNC URL:"
Write-Host "  $url"
Write-Host "Stop command:"
if ($Name -ne "") {
    Write-Host "  wsl.exe -d $DistroName -- docker rm -f $Name"
}
else {
    Write-Host "  wsl.exe -d $DistroName -- docker ps --filter label=projectc.openloong.runner=novnc"
    Write-Host "  wsl.exe -d $DistroName -- docker rm -f <container_name>"
}

if (-not $NoOpenBrowser) {
    $openScript = @"
`$url = '$url'
for (`$i = 0; `$i -lt 60; `$i++) {
    Start-Sleep -Seconds 1
    try {
        `$response = Invoke-WebRequest -UseBasicParsing -Uri `$url -TimeoutSec 2
        if (`$response.StatusCode -eq 200) {
            Start-Process `$url
            exit 0
        }
    } catch {}
}
exit 1
"@
    $openScriptPath = Join-Path $logDir ("open_{0}_{1:yyyyMMdd_HHmmss}.ps1" -f $DemoName, (Get-Date))
    Set-Content -LiteralPath $openScriptPath -Encoding UTF8 -Value $openScript
    Start-Process -FilePath "powershell.exe" `
        -ArgumentList @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $openScriptPath) `
        -WindowStyle Hidden | Out-Null
}

exit 0
