$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'Build.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Build failed.' }
$output = Join-Path $root 'verification'
[void](New-Item -ItemType Directory -Path $output -Force)
& (Join-Path $root 'bin\UartUiTests.exe') --vectors (Join-Path $output 'wire_vectors.txt')
if ($LASTEXITCODE -ne 0) { throw 'Offline tests failed.' }
& (Join-Path $root 'bin\UartUiTests.exe') --preview (Join-Path $output 'preview.png')
if ($LASTEXITCODE -ne 0) { throw 'Preview failed.' }
& (Join-Path $root 'bin\UartUiTests.exe') --preview (Join-Path $output 'preview_compact.png') compact
if ($LASTEXITCODE -ne 0) { throw 'Compact preview failed.' }
Write-Host 'PASS: offline tests and off-screen previews. No TD or serial port used.'
