param([switch]$Tune, [string]$DelphiBin='C:\Program Files (x86)\Embarcadero\Studio\23.0\bin')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$bin=Join-Path $root 'Win32\Hovercraft'
$results=Join-Path $root 'profiles\Hovercraft-20261029'
$lib=Join-Path (Split-Path $DelphiBin -Parent) 'lib\win32\release'
New-Item -ItemType Directory -Force -Path $bin | Out-Null
$names=@('HovercraftValidation')
if ($Tune) { $names=@('HovercraftTuning','HovercraftValidation') }
foreach ($name in $names) {
 & (Join-Path $DelphiBin 'dcc32.exe') -B '-NSSystem;Winapi;Vcl' "-U$root;$lib" "-E$bin" "-N0$bin" (Join-Path $root "tests\$name.dpr")
 if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $name" }
 & (Join-Path $bin "$name.exe") $results
 if ($LASTEXITCODE -ne 0) { throw "Validation failed: $name" }
}
& (Join-Path $DelphiBin 'dcc32.exe') -B '-NSSystem;Winapi;Vcl' "-U$root;$lib" "-E$bin" "-N0$bin" (Join-Path $root 'FModel.dpr')
if ($LASTEXITCODE -ne 0) { throw 'FModel compilation failed' }
Write-Output "Ready: $bin\FModel.exe. Preset: $results\FModel.ini."
