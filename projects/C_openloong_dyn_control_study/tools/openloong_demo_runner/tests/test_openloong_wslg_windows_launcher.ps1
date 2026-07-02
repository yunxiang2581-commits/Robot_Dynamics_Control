$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$launcher = Join-Path $scriptDir "..\commands\run_openloong_demo_wslg.ps1"
$fakeWsl = Join-Path $env:TEMP ("fake-wsl-" + [guid]::NewGuid().ToString() + ".ps1")
$capture = Join-Path $env:TEMP ("fake-wsl-args-" + [guid]::NewGuid().ToString() + ".txt")

try {
    if (-not (Test-Path -LiteralPath $launcher)) {
        throw "Missing Windows launcher: $launcher"
    }

    @'
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ArgsFromLauncher
)
$capturePath = $env:OPENLOONG_FAKE_WSL_CAPTURE
[System.IO.File]::WriteAllLines($capturePath, $ArgsFromLauncher)
exit 0
'@ | Set-Content -LiteralPath $fakeWsl -Encoding utf8

    $env:OPENLOONG_FAKE_WSL_CAPTURE = $capture
    & $launcher -WslExe $fakeWsl --list

    $actual = Get-Content -LiteralPath $capture
    $expected = @(
        "-d",
        "Ubuntu-22.04-ProjectC",
        "--",
        "bash",
        "-lc",
        "cd /mnt/d/project/Robot_Dynamics_Control && exec bash projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/run_openloong_demo_wslg.sh '--list'"
    )

    if ($actual.Count -ne $expected.Count) {
        throw "Argument count mismatch. Expected $($expected.Count), got $($actual.Count)."
    }

    for ($i = 0; $i -lt $expected.Count; $i++) {
        if ($actual[$i] -ne $expected[$i]) {
            throw "Argument $i mismatch. Expected '$($expected[$i])', got '$($actual[$i])'."
        }
    }
}
finally {
    Remove-Item -LiteralPath $fakeWsl -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $capture -ErrorAction SilentlyContinue
    Remove-Item Env:\OPENLOONG_FAKE_WSL_CAPTURE -ErrorAction SilentlyContinue
}
