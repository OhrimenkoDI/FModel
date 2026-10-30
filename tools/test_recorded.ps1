param([string]$DelphiBin = 'C:\Program Files (x86)\Embarcadero\Studio\23.0\bin')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$dcc = Join-Path $DelphiBin 'dcc32.exe'
$lib = Join-Path (Split-Path $DelphiBin -Parent) 'lib\win32\release'
$output = Join-Path $projectRoot 'Win32\RecordedTests'
New-Item -ItemType Directory -Force $output | Out-Null
Push-Location (Join-Path $projectRoot 'tests')
try {
    foreach ($testName in @('RecordedPlantTest','RecordedIntegrationTest','MotionModelTest','SharedMotionTest','GateNavigationTest','GateControllerTest','GatePlusIntegrationTest','ParameterDefaultsTest','HistoryTest','GateSettingsTest','IntegratorTest','AdaptiveGlideTest')) {
        & $dcc -B '-NSSystem;Winapi;Vcl' "-U..;$lib" "-I$lib" "-E$output" "-N0$output" "$testName.dpr"
        if ($LASTEXITCODE -ne 0) { throw "Build failed: $testName" }
        Push-Location $projectRoot
        try {
            if ($testName -eq 'RecordedPlantTest') { & (Join-Path $output "$testName.exe") (Join-Path $projectRoot 'profiles\EDF-20261023') }
            else { & (Join-Path $output "$testName.exe") }
            if ($LASTEXITCODE -ne 0) { throw "Test failed: $testName" }
        } finally { Pop-Location }
    }
} finally { Pop-Location }
