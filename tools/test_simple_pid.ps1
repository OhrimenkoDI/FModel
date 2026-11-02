param([switch]$Tune, [string]$DelphiBin='C:\Program Files (x86)\Embarcadero\Studio\23.0\bin')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$bin=Join-Path $root 'Win32\SimplePIDTests'
$results=Join-Path $root 'profiles\SimplePID-20261030'
$lib=Join-Path (Split-Path $DelphiBin -Parent) 'lib\win32\release'
New-Item -ItemType Directory -Force -Path $bin,$results | Out-Null
Push-Location $root
try {
 foreach ($name in @('SimplePIDTest','PIDArrayTest','ParameterDefaultsTest','MotionModelTest','HistoryTest','RecordedIntegrationTest','RegulatorAudit','TuningValidation','SimplePIDTuning')) {
  & (Join-Path $DelphiBin 'dcc32.exe') -B '-NSSystem;Winapi;Vcl' "-U$root;$lib" "-E$bin" "-N0$bin" (Join-Path $root "tests\$name.dpr")
  if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $name" }
  if ($name -eq 'SimplePIDTuning') {
   if ($Tune) { & (Join-Path $bin "$name.exe") $results | Tee-Object (Join-Path $results 'run.txt') }
   else { & (Join-Path $bin "$name.exe") $results '--validate' | Tee-Object (Join-Path $results 'validation_run.txt') }
  } else { & (Join-Path $bin "$name.exe") }
  if ($LASTEXITCODE -ne 0) { throw "Validation failed: $name" }
 }
 & (Join-Path $DelphiBin 'dcc32.exe') -B '-NSSystem;Winapi;Vcl' "-U$root;$lib" "-E$bin" "-N0$bin" (Join-Path $root 'FModel.dpr')
 if ($LASTEXITCODE -ne 0) { throw 'FModel compilation failed' }
 $appIni=Join-Path $bin 'FModel.ini'
 if (-not (Test-Path -LiteralPath $appIni)) {
  $sourceIni=Join-Path $root 'Win32\Debug\FModel.ini'
  if (Test-Path -LiteralPath $sourceIni) { Copy-Item -LiteralPath $sourceIni -Destination $appIni }
  else { Set-Content -LiteralPath $appIni -Value "[Controller]`r`n" -Encoding UTF8 }
 }
 $preset=Get-Content (Join-Path $results 'simple_pid.ini') -Raw
 $appText=Get-Content -LiteralPath $appIni -Raw
 $appText=[regex]::Replace($appText,'(?ms)^\[Controller\]\r?\n.*?(?=^\[|\z)',[System.Text.RegularExpressions.MatchEvaluator]{
  param($match)
  $section=$match.Value
  foreach ($entry in ($preset -split '\r?\n')) {
   if ($entry -match '^([^=]+)=(.*)$') {
    $key=[regex]::Escape($Matches[1])
    if ($section -match "(?m)^$key=") { $section=[regex]::Replace($section,"(?m)^$key=[^\r\n]*",$entry) }
    else { $section=$section.TrimEnd()+"`r`n$entry`r`n" }
   }
  }
  return $section
 })
 Set-Content -LiteralPath $appIni -Value $appText -Encoding UTF8
 Write-Output "Ready: $bin\FModel.exe"
} finally { Pop-Location }
