param(
    [string]$DelphiBin = 'C:\Program Files (x86)\Embarcadero\Studio\23.0\bin',
    [string]$OutputDirectory = ''
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$dcc = Join-Path $DelphiBin 'dcc32.exe'
$lib = Join-Path (Split-Path $DelphiBin -Parent) 'lib\win32\release'
$bin = Join-Path $projectRoot 'Win32\GateRecordedTests'
if (-not $OutputDirectory) { $OutputDirectory = Join-Path $bin 'results' }
$OutputDirectory = [System.IO.Path]::GetFullPath($OutputDirectory)
New-Item -ItemType Directory -Force $bin | Out-Null
Push-Location (Join-Path $projectRoot 'tests')
try {
    foreach ($name in @('GateRecordedTuning','GateGeometryCheck')) {
        & $dcc -B '-NSSystem;Winapi;Vcl' "-U..;$lib" "-I$lib" "-E$bin" "-N0$bin" "$name.dpr"
        if ($LASTEXITCODE -ne 0) { throw "Build failed: $name" }
        & (Join-Path $bin "$name.exe") $OutputDirectory
        if ($LASTEXITCODE -ne 0) { throw "Run failed: $name" }
    }
} finally { Pop-Location }
