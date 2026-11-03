param([switch]$Force)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$sources = @(Get-ChildItem -LiteralPath (Join-Path $root 'src') -Filter '*.cs' | ForEach-Object { $_.FullName })
$bin = Join-Path $root 'bin'
$compiler = Join-Path $env:WINDIR 'Microsoft.NET\Framework64\v4.0.30319\csc.exe'
if (!(Test-Path -LiteralPath $compiler)) { $compiler = Join-Path $env:WINDIR 'Microsoft.NET\Framework\v4.0.30319\csc.exe' }
if (!(Test-Path -LiteralPath $compiler)) { throw 'Windows .NET Framework compiler is unavailable. Enable .NET Framework 4.x first.' }
[void](New-Item -ItemType Directory -Path $bin -Force)
$app = Join-Path $bin 'FpgaControl.exe'
$tests = Join-Path $bin 'UartUiTests.exe'
$latestSource = ($sources | ForEach-Object { (Get-Item -LiteralPath $_).LastWriteTimeUtc } | Sort-Object -Descending | Select-Object -First 1)
if (!$Force -and (Test-Path -LiteralPath $app) -and (Test-Path -LiteralPath $tests) -and (Get-Item -LiteralPath $app).LastWriteTimeUtc -ge $latestSource -and (Get-Item -LiteralPath $tests).LastWriteTimeUtc -ge $latestSource) { Write-Host 'Build is up to date.'; exit 0 }
$common = @('/nologo', '/codepage:65001', '/optimize+', '/warn:4', '/r:System.dll', '/r:System.Core.dll', '/r:System.Drawing.dll', '/r:System.Windows.Forms.dll')
& $compiler @common '/target:winexe' '/main:FpgaControl.Program' "/out:$app" @sources
if ($LASTEXITCODE -ne 0) { throw 'UI compilation failed.' }
& $compiler @common '/target:exe' '/main:FpgaControl.Tests' "/out:$tests" @sources
if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed.' }
Write-Host "Build OK: $app"
