param(
    [string]$TaskName = "Robot Motion Control AI Daily Hot Report",
    [string]$OutputDir = "",
    [string]$StartTime = "07:30",
    [string]$PythonPath = ""
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent $ScriptDir
$ReportScript = Join-Path $ScriptDir "daily_hot_report.py"
$DefaultOutputDir = "D:\" + [char]0x684c + [char]0x9762 + "\" + [char]0x6bcf + [char]0x65e5 + [char]0x70ed + [char]0x70b9

if ([string]::IsNullOrWhiteSpace($OutputDir)) {
    $OutputDir = $DefaultOutputDir
}

if (-not (Test-Path -LiteralPath $ReportScript)) {
    throw "Cannot find report script: $ReportScript"
}

if ([string]::IsNullOrWhiteSpace($PythonPath)) {
    $PythonPath = (Get-Command python).Source
}

New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

$Argument = "-B `"$ReportScript`" --output-dir `"$OutputDir`""
$Action = New-ScheduledTaskAction -Execute $PythonPath -Argument $Argument -WorkingDirectory $RepoRoot
$Trigger = New-ScheduledTaskTrigger -Daily -At $StartTime
$Settings = New-ScheduledTaskSettingsSet `
    -AllowStartIfOnBatteries `
    -DontStopIfGoingOnBatteries `
    -StartWhenAvailable `
    -MultipleInstances IgnoreNew

Register-ScheduledTask `
    -TaskName $TaskName `
    -Action $Action `
    -Trigger $Trigger `
    -Settings $Settings `
    -Description "Generate the daily Chinese robotics motion-control and AI hot-news report before noon." `
    -Force | Out-Null

Write-Host "Registered scheduled task: $TaskName"
Write-Host "Daily start time: $StartTime"
Write-Host "Python: $PythonPath"
Write-Host "Script: $ReportScript"
Write-Host "Output: $OutputDir"
