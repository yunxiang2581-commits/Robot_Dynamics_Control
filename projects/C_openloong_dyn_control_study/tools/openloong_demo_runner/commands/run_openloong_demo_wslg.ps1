[CmdletBinding(PositionalBinding = $false)]
param(
    [string]$DistroName = "Ubuntu-22.04-ProjectC",
    [string]$RepoPath = "/mnt/d/project/Robot_Dynamics_Control",
    [string]$WslExe = "wsl.exe",
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$DemoArgs
)

$ErrorActionPreference = "Stop"

# Windows side wrapper.
# Input: PowerShell arguments. Output: the same arguments forwarded to the WSL shell launcher.
# The math/control logic does not change here; this script only enters the distro and repo path.
$Script:OpenLoongLauncherPath = "projects/C_openloong_dyn_control_study/tools/openloong_demo_runner/commands/run_openloong_demo_wslg.sh"
$forwardedArgs = ""

foreach ($arg in $DemoArgs) {
    $escaped = $arg.Replace("'", "'\''")
    $forwardedArgs += " '$escaped'"
}

$bashCommand = "cd ${RepoPath} && exec bash ${Script:OpenLoongLauncherPath}${forwardedArgs}"
$wslArgs = @("-d", $DistroName, "--", "bash", "-lc", $bashCommand)

& $WslExe @wslArgs
exit $LASTEXITCODE
