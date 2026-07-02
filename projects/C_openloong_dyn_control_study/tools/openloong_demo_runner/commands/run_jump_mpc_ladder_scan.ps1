[CmdletBinding(PositionalBinding = $false)]
param(
    [string]$DistroName = "Ubuntu-22.04-ProjectC",
    [string]$WslExe = "wsl.exe",
    [string]$RepoPath = "D:\project\Robot_Dynamics_Control",
    [string]$WorktreePath = "D:\project\Robot_Dynamics_Control\projects\C_openloong_dyn_control_study\outputs\docker_reproduction\openloong_jump_mpc_basepos_exp\20260701_001306\worktree\OpenLoong-Dyn-Control",
    [string]$ImageName = "openloong-ubuntu22-novnc:local",
    [object]$JumpZ = 0.5,
    [object]$JumpAccT = @(0.075, 0.085, 0.090, 0.095),
    [int]$TimeoutSeconds = 95,
    [double]$MaxStableRpy = 0.6,
    [double]$MinStableZ = 0.75
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

function Convert-ToNumberList {
    param(
        [Parameter(Mandatory = $true)][object]$Value,
        [Parameter(Mandatory = $true)][string]$Name
    )

    $tokens = New-Object System.Collections.Generic.List[string]
    foreach ($item in @($Value)) {
        if ($null -eq $item) {
            continue
        }
        foreach ($part in ([string]$item).Split(",")) {
            $trimmed = $part.Trim()
            if ($trimmed.Length -gt 0) {
                $tokens.Add($trimmed)
            }
        }
    }

    if ($tokens.Count -eq 0) {
        throw "$Name must contain at least one numeric value."
    }

    $values = New-Object double[] $tokens.Count
    for ($i = 0; $i -lt $tokens.Count; $i++) {
        try {
            $values[$i] = [double]::Parse($tokens[$i], [System.Globalization.NumberStyles]::Float, $Invariant)
        } catch {
            throw "Invalid $Name value '$($tokens[$i])'. Use values such as 0.075 or comma-separated values such as 0.075,0.085."
        }
    }

    return $values
}

function Read-JumpMetrics {
    param([Parameter(Mandatory = $true)][string]$LogPath)

    $rows = New-Object System.Collections.Generic.List[object]
    foreach ($line in [System.IO.File]::ReadLines($LogPath)) {
        if ([string]::IsNullOrWhiteSpace($line)) {
            continue
        }
        $parts = $line.Split(",")
        if ($parts.Count -lt 227) {
            continue
        }
        $values = New-Object double[] $parts.Count
        try {
            for ($i = 0; $i -lt $parts.Count; $i++) {
                $values[$i] = [double]::Parse($parts[$i], [System.Globalization.NumberStyles]::Float, $Invariant)
            }
            $rows.Add($values)
        } catch {
            continue
        }
    }

    if ($rows.Count -eq 0) {
        throw "No parseable rows in $LogPath"
    }

    $start = $rows[0]
    foreach ($row in $rows) {
        if ([math]::Abs($row[0] - 8.5) -lt [math]::Abs($start[0] - 8.5)) {
            $start = $row
        }
    }

    $peak = $null
    $minAfterJump = $null
    $maxRpyRow = $null
    foreach ($row in $rows) {
        if ($row[0] -lt 8.5) {
            continue
        }
        if ($row[0] -le 9.2 -and ($null -eq $peak -or $row[161] -gt $peak[161])) {
            $peak = $row
        }
        if ($null -eq $minAfterJump -or $row[161] -lt $minAfterJump[161]) {
            $minAfterJump = $row
        }
        $rpyAbs = [math]::Max([math]::Abs($row[156]), [math]::Max([math]::Abs($row[157]), [math]::Abs($row[158])))
        if ($null -eq $maxRpyRow) {
            $maxRpyRow = $row
        } else {
            $oldRpyAbs = [math]::Max([math]::Abs($maxRpyRow[156]), [math]::Max([math]::Abs($maxRpyRow[157]), [math]::Abs($maxRpyRow[158])))
            if ($rpyAbs -gt $oldRpyAbs) {
                $maxRpyRow = $row
            }
        }
    }

    if ($null -eq $peak -or $null -eq $minAfterJump -or $null -eq $maxRpyRow) {
        $firstT = $rows[0][0]
        $lastT = $rows[$rows.Count - 1][0]
        throw "Could not find jump analysis window in $LogPath; time range is $firstT to $lastT"
    }

    $end = $rows[$rows.Count - 1]
    $maxAbsRpy = [math]::Max([math]::Abs($maxRpyRow[156]), [math]::Max([math]::Abs($maxRpyRow[157]), [math]::Abs($maxRpyRow[158])))
    $stable = ($end[0] -ge 12.5 -and $minAfterJump[161] -ge $MinStableZ -and $maxAbsRpy -le $MaxStableRpy)

    [pscustomobject]@{
        Rows = $rows.Count
        TEnd = $end[0]
        JumpZ = $start[220]
        JumpAccT = $start[222]
        JumpDelta = $peak[161] - $start[161]
        PeakZ = $peak[161]
        PeakT = $peak[0]
        MinZAfterJump = $minAfterJump[161]
        MinZT = $minAfterJump[0]
        MaxAbsRpy = $maxAbsRpy
        MaxRpyT = $maxRpyRow[0]
        EndZ = $end[161]
        EndRoll = $end[156]
        EndPitch = $end[157]
        EndYaw = $end[158]
        Stable = $stable
    }
}

$projectRoot = Resolve-Path -LiteralPath $RepoPath
$worktreeRoot = Resolve-Path -LiteralPath $WorktreePath
$toolRoot = Join-Path $projectRoot "projects\C_openloong_dyn_control_study\tools\openloong_demo_runner"
$containerScript = Join-Path $toolRoot "commands\container_run_jump_mpc_ladder_once.sh"
$jumpMpcSource = Join-Path $worktreeRoot "demo\jump_mpc.cpp"

if (-not (Test-Path -LiteralPath (Join-Path $worktreeRoot "build\jump_mpc"))) {
    throw "Missing built jump_mpc executable under $worktreeRoot\build"
}
if (-not (Test-Path -LiteralPath $containerScript)) {
    throw "Missing container script: $containerScript"
}
if ((Get-Content -Raw -LiteralPath $jumpMpcSource) -notmatch "OPENLOONG_JUMP_ACC_T") {
    throw "This worktree does not contain the Project C ladder parameter patch: $jumpMpcSource"
}

$jumpZValues = Convert-ToNumberList -Value $JumpZ -Name "JumpZ"
$jumpAccTValues = Convert-ToNumberList -Value $JumpAccT -Name "JumpAccT"
$jumpZTag = ($jumpZValues | ForEach-Object { Get-SafeNumberTag ($_.ToString("0.###", $Invariant)) }) -join "_"
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$scanRoot = Join-Path $projectRoot "projects\C_openloong_dyn_control_study\outputs\docker_reproduction\openloong_jump_mpc_basepos_exp\ladder_scan_jumpz${jumpZTag}_$timestamp"
New-Item -ItemType Directory -Force -Path $scanRoot | Out-Null

$worktreeWsl = Convert-ToWslPath $worktreeRoot
$toolRootWsl = Convert-ToWslPath $toolRoot
$results = New-Object System.Collections.Generic.List[object]

foreach ($jumpZValue in $jumpZValues) {
    $jumpZText = $jumpZValue.ToString("0.###", $Invariant)
    $jumpZRunTag = "jumpz$(Get-SafeNumberTag $jumpZText)"

    foreach ($acc in $jumpAccTValues) {
        $accText = $acc.ToString("0.###", $Invariant)
        $runTag = "${jumpZRunTag}_acc$(Get-SafeNumberTag $accText)"
        $runRoot = Join-Path $scanRoot $runTag
        $recordRoot = Join-Path $runRoot "runtime_record"
        New-Item -ItemType Directory -Force -Path $recordRoot, (Join-Path $runRoot "logs") | Out-Null

        $runRootWsl = Convert-ToWslPath $runRoot
        $recordRootWsl = Convert-ToWslPath $recordRoot
        $containerName = "projectc_jump_mpc_ladder_scan_$(Get-SafeNumberTag $jumpZText)_$(Get-SafeNumberTag $accText)_$timestamp"

        Write-Host "Running jump_z=$jumpZText acc_t=$accText ..."
        $dockerArgs = @(
            "-d", $DistroName, "--",
            "docker", "run", "--rm",
            "--name", $containerName,
            "-e", "OPENLOONG_JUMP_Z=$jumpZText",
            "-e", "OPENLOONG_JUMP_ACC_T=$accText",
            "-e", "OPENLOONG_RUN_TIMEOUT_SECONDS=$TimeoutSeconds",
            "-v", "${worktreeWsl}:/run_root/worktree/OpenLoong-Dyn-Control",
            "-v", "${recordRootWsl}:/run_root/worktree/OpenLoong-Dyn-Control/record",
            "-v", "${runRootWsl}:/demo_run",
            "-v", "${toolRootWsl}:/tool_root:ro",
            "-w", "/run_root/worktree/OpenLoong-Dyn-Control/build",
            $ImageName,
            "bash", "/tool_root/commands/container_run_jump_mpc_ladder_once.sh"
        )
        & $WslExe @dockerArgs | Out-Host
        $exitCode = $LASTEXITCODE

        $logPath = Join-Path $recordRoot "datalog.log"
        if (-not (Test-Path -LiteralPath $logPath)) {
            $results.Add([pscustomobject]@{
                JumpZ = $jumpZValue
                JumpAccT = $acc
                Stable = $false
                Error = "missing datalog.log, docker exit code $exitCode"
                RunRoot = $runRoot
            })
            continue
        }

        $metrics = Read-JumpMetrics -LogPath $logPath
        $metrics | Add-Member -NotePropertyName DockerExitCode -NotePropertyValue $exitCode
        $metrics | Add-Member -NotePropertyName RunRoot -NotePropertyValue $runRoot
        $metrics | Add-Member -NotePropertyName Error -NotePropertyValue ""
        $results.Add($metrics)
    }
}

$summaryPath = Join-Path $scanRoot "summary.csv"
$results | Export-Csv -NoTypeInformation -Encoding UTF8 -LiteralPath $summaryPath

$results |
    Sort-Object JumpZ, JumpAccT |
    Format-Table JumpZ, JumpAccT, JumpDelta, PeakT, MinZAfterJump, MaxAbsRpy, EndZ, Stable, DockerExitCode -AutoSize

Write-Host "Scan root: $scanRoot"
Write-Host "Summary: $summaryPath"
