[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $PSCommandPath
$RepoRoot = (Resolve-Path (Join-Path $ScriptDir "..")).Path

$ExportRoot = Join-Path $RepoRoot "exports/word"
$SingleDir = Join-Path $ExportRoot "single"
$CombinedDir = Join-Path $ExportRoot "combined"
$LogDir = Join-Path $ExportRoot "logs"
$LogFile = Join-Path $LogDir "export_md_to_docx.log"
$CombinedDocx = Join-Path $CombinedDir "robot_motion_control_job_project_docs.docx"

New-Item -ItemType Directory -Force -Path $SingleDir, $CombinedDir, $LogDir | Out-Null

function Write-Log {
    param([string]$Message)
    Write-Host $Message
    Add-Content -LiteralPath $LogFile -Value $Message -Encoding UTF8
}

function Get-RelativePath {
    param([string]$Path)

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    $rootPath = [System.IO.Path]::GetFullPath($RepoRoot)
    if (-not $rootPath.EndsWith([System.IO.Path]::DirectorySeparatorChar)) {
        $rootPath = $rootPath + [System.IO.Path]::DirectorySeparatorChar
    }

    $rootUri = New-Object System.Uri($rootPath)
    $pathUri = New-Object System.Uri($fullPath)
    $relative = [System.Uri]::UnescapeDataString($rootUri.MakeRelativeUri($pathUri).ToString())
    return ($relative -replace "\\", "/")
}

function Get-SafeDocxName {
    param([string]$RelativePath)

    $safe = $RelativePath -replace "[/\\]", "__"
    $safe = [regex]::Replace($safe, "\.md$", ".docx", "IgnoreCase")
    return $safe
}

function Add-FileIfExists {
    param(
        [string]$Path,
        [System.Collections.Generic.List[string]]$Target,
        [System.Collections.Generic.List[string]]$Skipped
    )

    if (Test-Path -LiteralPath $Path -PathType Leaf) {
        $Target.Add((Resolve-Path -LiteralPath $Path).Path)
    }
    else {
        $Skipped.Add("$(Get-RelativePath $Path) | missing file")
    }
}

function Add-MarkdownFiles {
    param(
        [string]$Directory,
        [bool]$Recurse,
        [System.Collections.Generic.List[string]]$Target,
        [System.Collections.Generic.List[string]]$Skipped
    )

    if (-not (Test-Path -LiteralPath $Directory -PathType Container)) {
        $Skipped.Add("$(Get-RelativePath $Directory) | missing directory")
        return
    }

    $params = @{
        LiteralPath = $Directory
        File = $true
        Filter = "*.md"
    }
    if ($Recurse) {
        $params.Recurse = $true
    }

    Get-ChildItem @params |
        Sort-Object FullName |
        ForEach-Object { $Target.Add($_.FullName) }
}

function Write-PandocInstallHelp {
    Write-Host ""
    Write-Host "Pandoc is required to export Markdown to Word, but it was not found."
    Write-Host ""
    Write-Host "Install it manually, then rerun this script:"
    Write-Host ""
    Write-Host "  Windows:"
    Write-Host "    winget install --id JohnMacFarlane.Pandoc"
    Write-Host ""
    Write-Host "  Or download installer:"
    Write-Host "    https://pandoc.org/installing.html"
    Write-Host ""
}

function Resolve-PandocPath {
    $pandocCommand = Get-Command pandoc -ErrorAction SilentlyContinue
    if ($pandocCommand) {
        return $pandocCommand.Source
    }

    $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
    if (-not $pythonCommand) {
        return $null
    }

    try {
        $pypandocPath = & $pythonCommand.Source -c "import pypandoc; print(pypandoc.get_pandoc_path())" 2>$null
        if ($LASTEXITCODE -eq 0 -and $pypandocPath) {
            if (Test-Path -LiteralPath $pypandocPath -PathType Leaf) {
                return $pypandocPath
            }

            $windowsExePath = "$pypandocPath.exe"
            if (Test-Path -LiteralPath $windowsExePath -PathType Leaf) {
                return $windowsExePath
            }
        }
    }
    catch {
        return $null
    }

    return $null
}

$MarkdownFiles = [System.Collections.Generic.List[string]]::new()
$CombinedFiles = [System.Collections.Generic.List[string]]::new()
$SkippedFiles = [System.Collections.Generic.List[string]]::new()
$SuccessFiles = [System.Collections.Generic.List[string]]::new()
$FailedFiles = [System.Collections.Generic.List[string]]::new()

Add-FileIfExists -Path (Join-Path $RepoRoot "README.md") -Target $MarkdownFiles -Skipped $SkippedFiles
Add-MarkdownFiles -Directory (Join-Path $RepoRoot "docs") -Recurse $true -Target $MarkdownFiles -Skipped $SkippedFiles
Add-MarkdownFiles -Directory (Join-Path $RepoRoot "external") -Recurse $false -Target $MarkdownFiles -Skipped $SkippedFiles

Add-FileIfExists -Path (Join-Path $RepoRoot "README.md") -Target $CombinedFiles -Skipped $SkippedFiles
Add-MarkdownFiles -Directory (Join-Path $RepoRoot "docs/00_preparation") -Recurse $false -Target $CombinedFiles -Skipped $SkippedFiles
Add-MarkdownFiles -Directory (Join-Path $RepoRoot "docs/01_self_baseline") -Recurse $false -Target $CombinedFiles -Skipped $SkippedFiles
Add-MarkdownFiles -Directory (Join-Path $RepoRoot "docs/02_legged_control") -Recurse $false -Target $CombinedFiles -Skipped $SkippedFiles
Add-MarkdownFiles -Directory (Join-Path $RepoRoot "docs/03_unitree_rl_mjlab") -Recurse $false -Target $CombinedFiles -Skipped $SkippedFiles
Add-MarkdownFiles -Directory (Join-Path $RepoRoot "docs/04_compare") -Recurse $false -Target $CombinedFiles -Skipped $SkippedFiles
Add-MarkdownFiles -Directory (Join-Path $RepoRoot "docs/interview") -Recurse $false -Target $CombinedFiles -Skipped $SkippedFiles
Add-MarkdownFiles -Directory (Join-Path $RepoRoot "external") -Recurse $false -Target $CombinedFiles -Skipped $SkippedFiles

Set-Content -LiteralPath $LogFile -Value "" -Encoding UTF8
Write-Log "Markdown to Word export"
Write-Log "Export time: $((Get-Date).ToString('yyyy-MM-dd HH:mm:ss zzz'))"
Write-Log "Repository root: $RepoRoot"
Write-Log "Single output directory: $SingleDir"
Write-Log "Combined output path: $CombinedDocx"
Write-Log ""

$PandocPath = Resolve-PandocPath
if (-not $PandocPath) {
    Write-Log "Pandoc status: missing"
    Write-Log ""
    Write-Log "Skipped files:"
    if ($MarkdownFiles.Count -eq 0) {
        Write-Log "  - none found"
    }
    else {
        foreach ($src in $MarkdownFiles) {
            Write-Log "  - $(Get-RelativePath $src) | pandoc missing"
        }
    }
    Write-Log ""
    Write-Log "Successful files:"
    Write-Log "  - none"
    Write-Log ""
    Write-Log "Failed files:"
    Write-Log "  - none"
    Write-Log ""
    Write-Log "Combined document:"
    Write-Log "  - not generated | pandoc missing"
    Write-PandocInstallHelp
    exit 1
}

Write-Log "Pandoc status: $PandocPath"
Write-Log ""
Write-Log "Starting single-file exports..."

foreach ($src in $MarkdownFiles) {
    $rel = Get-RelativePath $src
    $out = Join-Path $SingleDir (Get-SafeDocxName $rel)

    Write-Host "Exporting $rel -> $(Get-RelativePath $out)"
    try {
        & $PandocPath $src --from markdown --to docx --output $out
        if ($LASTEXITCODE -eq 0) {
            $SuccessFiles.Add("$rel -> $(Get-RelativePath $out)")
        }
        else {
            $FailedFiles.Add("$rel -> $(Get-RelativePath $out) | exit $LASTEXITCODE")
        }
    }
    catch {
        $FailedFiles.Add("$rel -> $(Get-RelativePath $out) | $($_.Exception.Message)")
    }
}

Write-Log ""
Write-Log "Starting combined export..."
if ($CombinedFiles.Count -eq 0) {
    $SkippedFiles.Add("combined document | no markdown files found")
}
else {
    try {
        & $PandocPath @CombinedFiles --from markdown --to docx --output $CombinedDocx
        if ($LASTEXITCODE -eq 0) {
            $SuccessFiles.Add("combined -> $(Get-RelativePath $CombinedDocx)")
        }
        else {
            $FailedFiles.Add("combined -> $(Get-RelativePath $CombinedDocx) | exit $LASTEXITCODE")
        }
    }
    catch {
        $FailedFiles.Add("combined -> $(Get-RelativePath $CombinedDocx) | $($_.Exception.Message)")
    }
}

Write-Log ""
Write-Log "Successful files:"
if ($SuccessFiles.Count -eq 0) {
    Write-Log "  - none"
}
else {
    foreach ($item in $SuccessFiles) {
        Write-Log "  - $item"
    }
}

Write-Log ""
Write-Log "Skipped files:"
if ($SkippedFiles.Count -eq 0) {
    Write-Log "  - none"
}
else {
    foreach ($item in $SkippedFiles) {
        Write-Log "  - $item"
    }
}

Write-Log ""
Write-Log "Failed files:"
if ($FailedFiles.Count -eq 0) {
    Write-Log "  - none"
}
else {
    foreach ($item in $FailedFiles) {
        Write-Log "  - $item"
    }
}

Write-Log ""
Write-Log "Combined document:"
if (Test-Path -LiteralPath $CombinedDocx -PathType Leaf) {
    Write-Log "  - $(Get-RelativePath $CombinedDocx)"
}
else {
    Write-Log "  - not generated"
}

Write-Log ""
Write-Log "Generated Word paths:"
$GeneratedDocx = Get-ChildItem -LiteralPath $SingleDir, $CombinedDir -Filter "*.docx" -File -ErrorAction SilentlyContinue |
    Sort-Object FullName
if (-not $GeneratedDocx) {
    Write-Log "  - none"
}
else {
    foreach ($docx in $GeneratedDocx) {
        Write-Log "  - $(Get-RelativePath $docx.FullName)"
    }
}

if ($FailedFiles.Count -gt 0) {
    exit 1
}
