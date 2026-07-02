[CmdletBinding(PositionalBinding = $false)]
param(
    [string]$DistroName = "Ubuntu-22.04-ProjectC",
    [string]$WslExe = "wsl.exe",
    [string]$RepoPath = "D:\project\Robot_Dynamics_Control",
    [string]$WorktreePath = "D:\project\Robot_Dynamics_Control\projects\C_openloong_dyn_control_study\outputs\docker_reproduction\openloong_jump_mpc_basepos_exp\20260701_001306\worktree\OpenLoong-Dyn-Control",
    [string]$ImageName = "openloong-ubuntu22-novnc:local",
    [string]$Name = "projectc_jump_mpc_ladder_visual",
    [int]$Port = 6087,
    [double]$JumpZ = 0.5,
    [double]$JumpAccT = 0.085,
    [string]$AnklePitchCompGain = "",
    [string]$LandingXOffset = "",
    [switch]$Foreground,
    [switch]$KeepExisting
)

$ErrorActionPreference = "Stop"
$Invariant = [System.Globalization.CultureInfo]::InvariantCulture

function Convert-ToWslPath {
    param([Parameter(Mandatory = $true)][string]$Path)

    $resolved = (Resolve-Path -LiteralPath $Path).Path
    if ($resolved -match "^([A-Za-z]):\\(.*)$") {
        $drive = $matches[1].ToLowerInvariant()
        $rest = $matches[2] -replace "\\", "/"
        return "/mnt/$drive/$rest"
    }
    return ($resolved -replace "\\", "/")
}

function Get-SafeNumberTag {
    param([Parameter(Mandatory = $true)][string]$Value)
    return ($Value -replace "-", "m" -replace "\.", "p")
}

$projectRoot = Resolve-Path -LiteralPath $RepoPath
$worktreeRoot = Resolve-Path -LiteralPath $WorktreePath
$toolRoot = Join-Path $projectRoot "projects\C_openloong_dyn_control_study\tools\openloong_demo_runner"
$containerScript = Join-Path $toolRoot "commands\container_run_jump_mpc_ladder_visual.sh"

if (-not (Test-Path -LiteralPath (Join-Path $worktreeRoot "build\jump_mpc"))) {
    throw "Missing built jump_mpc executable under $worktreeRoot\build"
}
$jumpMpcSource = Join-Path $worktreeRoot "demo\jump_mpc.cpp"
if (-not (Test-Path -LiteralPath $jumpMpcSource)) {
    throw "Missing jump_mpc.cpp under $worktreeRoot\demo"
}
if ((Get-Content -Raw -LiteralPath $jumpMpcSource) -notmatch "OPENLOONG_JUMP_ACC_T") {
    throw "This worktree does not contain the Project C ladder parameter patch: $jumpMpcSource"
}
if (-not (Test-Path -LiteralPath $containerScript)) {
    throw "Missing container script: $containerScript"
}

$jumpZText = $JumpZ.ToString("0.###", $Invariant)
$jumpAccText = $JumpAccT.ToString("0.###", $Invariant)
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$runTag = "visual_ladder_jumpz$(Get-SafeNumberTag $jumpZText)_acc$(Get-SafeNumberTag $jumpAccText)_$timestamp"
$runRoot = Join-Path $projectRoot "projects\C_openloong_dyn_control_study\outputs\docker_reproduction\openloong_jump_mpc_basepos_exp\$runTag"
$recordRoot = Join-Path $runRoot "runtime_record"
$logRoot = Join-Path $runRoot "logs"
New-Item -ItemType Directory -Force -Path $recordRoot, $logRoot | Out-Null

$worktreeWsl = Convert-ToWslPath $worktreeRoot
$runRootWsl = Convert-ToWslPath $runRoot
$recordRootWsl = Convert-ToWslPath $recordRoot
$toolRootWsl = Convert-ToWslPath $toolRoot

if (-not $KeepExisting) {
    $previousErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        & $WslExe -d $DistroName -- docker rm -f $Name 2>$null | Out-Null
    } catch {
        # It is fine if the previous visual container does not exist.
    } finally {
        $ErrorActionPreference = $previousErrorActionPreference
    }
}

$dockerArgs = @(
    "-d", $DistroName, "--",
    "docker", "run", "--rm",
    "--name", $Name,
    "-p", "127.0.0.1:${Port}:6080",
    "-e", "OPENLOONG_HOLD_WINDOW_AFTER_END=1",
    "-e", "OPENLOONG_HOLD_WINDOW_SECONDS=3600",
    "-e", "OPENLOONG_JUMP_Z=$jumpZText",
    "-e", "OPENLOONG_JUMP_ACC_T=$jumpAccText",
    "-v", "${worktreeWsl}:/run_root/worktree/OpenLoong-Dyn-Control",
    "-v", "${recordRootWsl}:/run_root/worktree/OpenLoong-Dyn-Control/record",
    "-v", "${runRootWsl}:/demo_run",
    "-v", "${toolRootWsl}:/tool_root:ro",
    "-w", "/run_root/worktree/OpenLoong-Dyn-Control/build",
    $ImageName,
    "bash", "/tool_root/commands/container_run_jump_mpc_ladder_visual.sh"
)

if ($AnklePitchCompGain -ne "") {
    $insertAt = [Array]::IndexOf($dockerArgs, "-v")
    $dockerArgs = $dockerArgs[0..($insertAt - 1)] + @("-e", "OPENLOONG_ANKLE_PITCH_COMP_GAIN=$AnklePitchCompGain") + $dockerArgs[$insertAt..($dockerArgs.Count - 1)]
}
if ($LandingXOffset -ne "") {
    $insertAt = [Array]::IndexOf($dockerArgs, "-v")
    $dockerArgs = $dockerArgs[0..($insertAt - 1)] + @("-e", "OPENLOONG_LANDING_X_OFFSET=$LandingXOffset") + $dockerArgs[$insertAt..($dockerArgs.Count - 1)]
}

if ($Foreground) {
    & $WslExe @dockerArgs
    exit $LASTEXITCODE
}

$process = Start-Process -FilePath $WslExe -ArgumentList $dockerArgs -WindowStyle Hidden -PassThru
$url = "http://127.0.0.1:$Port/vnc.html?autoconnect=true&resize=remote"

Write-Host "Run root: $runRoot"
Write-Host "Container: $Name"
Write-Host "Host process id: $($process.Id)"
Write-Host "noVNC URL:"
Write-Host "  $url"
Write-Host "Stop command:"
Write-Host "  wsl.exe -d $DistroName -- docker rm -f $Name"
