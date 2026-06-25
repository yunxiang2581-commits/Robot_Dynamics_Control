<#
Project C WSL/Docker environment continuation script.

Purpose:
  1. After a Windows reboot, check whether WSL is fully available.
  2. Import Ubuntu 22.04 into D:\wsl instead of the default C: location.
  3. Print the next Docker Engine setup hints.

Input:
  - D:\wsl\downloads\ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz

Output:
  - WSL distro: Ubuntu-22.04-ProjectC
  - Distro data directory: D:\wsl\distros\Ubuntu-22.04-ProjectC

Notes:
  - This script only changes the Windows/WSL environment.
  - It does not change robotics control code.
  - It does not chmod /var/run/docker.sock.

Keep this file ASCII-only so Windows PowerShell 5.1 can run it without
UTF-8 encoding issues.
#>

[CmdletBinding()]
param(
    [string]$DistroName = "Ubuntu-22.04-ProjectC",
    [string]$WslRoot = "D:\wsl",
    [string]$ProjectPath = "D:\project\Robot_Dynamics_Control"
)

$ErrorActionPreference = "Stop"

function Write-Step {
    param([string]$Message)
    Write-Host ""
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Require-File {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Required file not found: $Path`nPlease confirm the Ubuntu rootfs is in D:\wsl\downloads."
    }
}

function Test-WslCommandReady {
    $helpText = (& wsl.exe --help 2>&1) -join "`n"
    return ($helpText -match "--import") -or ($helpText -match "--list") -or ($helpText -match "-l")
}

$downloadDir = Join-Path $WslRoot "downloads"
$distrosDir = Join-Path $WslRoot "distros"
$dockerDataDir = Join-Path $WslRoot "docker-data"
$rootfsPath = Join-Path $downloadDir "ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz"
$installDir = Join-Path $distrosDir $DistroName

Write-Step "Checking D: directories and Ubuntu rootfs"
New-Item -ItemType Directory -Force -Path $downloadDir, $distrosDir, $dockerDataDir | Out-Null
Require-File $rootfsPath
$rootfs = Get-Item -LiteralPath $rootfsPath
Write-Host ("rootfs: {0} ({1:N1} MB)" -f $rootfs.FullName, ($rootfs.Length / 1MB))

Write-Step "Checking whether wsl.exe supports distro import"
if (-not (Test-WslCommandReady)) {
    Write-Host "Current wsl.exe still looks like the install bootstrapper, so --import is not available yet." -ForegroundColor Yellow
    Write-Host "Reboot Windows first, then run this script again."
    Write-Host "If it is still blocked after reboot, install the official Microsoft WSL MSI package."
    exit 2
}

Write-Step "Listing existing WSL distros"
$listOutput = (& wsl.exe --list --verbose 2>&1) -join "`n"
Write-Host $listOutput

if ($listOutput -match [regex]::Escape($DistroName)) {
    Write-Host "Distro $DistroName already exists. Skipping import."
}
else {
    Write-Step "Importing Ubuntu 22.04 into D:"
    if (Test-Path -LiteralPath $installDir) {
        $children = Get-ChildItem -LiteralPath $installDir -Force -ErrorAction SilentlyContinue
        if ($children) {
            throw "Target directory exists and is not empty: $installDir`nCheck it manually before continuing."
        }
    }
    else {
        New-Item -ItemType Directory -Force -Path $installDir | Out-Null
    }

    & wsl.exe --import $DistroName $installDir $rootfsPath --version 2
    if ($LASTEXITCODE -ne 0) {
        throw "wsl --import failed with exit code: $LASTEXITCODE"
    }
}

Write-Step "Setting WSL2 as default and showing distro status"
& wsl.exe --set-default-version 2
& wsl.exe --list --verbose

Write-Step "Running a minimal WSL smoke check"
& wsl.exe -d $DistroName -- bash -lc "set -e; echo WSL_OK; uname -a; printf 'Project path: '; ls -ld /mnt/d/project/Robot_Dynamics_Control"
if ($LASTEXITCODE -ne 0) {
    throw "WSL smoke check failed. Check D: mount or distro import status."
}

Write-Step "Next Docker Engine setup hints"
Write-Host "Enter WSL with:"
Write-Host "  wsl -d $DistroName"
Write-Host ""
Write-Host "Inside WSL, create a normal user first, then install Docker Engine."
Write-Host "Do not chmod 666 /var/run/docker.sock."
Write-Host "Project path inside WSL: /mnt/d/project/Robot_Dynamics_Control"
Write-Host "Suggested Docker data directory: /mnt/d/wsl/docker-data"
