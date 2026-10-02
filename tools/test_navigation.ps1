param([string]$BuildDirectory = 'build-navigation')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$candidates = @(
    (Get-Command cmake -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -ErrorAction SilentlyContinue),
    'C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe',
    'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
)
$cmake = $candidates | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -First 1
if (-not $cmake) { throw 'CMake not found. Install CMake and a C++14 compiler (Visual Studio C++ or MinGW).' }
$ctest = Join-Path (Split-Path $cmake) 'ctest.exe'
$buildPath = Join-Path $projectRoot $BuildDirectory
& $cmake -S (Join-Path $projectRoot 'tests/navigation') -B $buildPath
if ($LASTEXITCODE -ne 0) { throw 'Navigation test configuration failed.' }
& $cmake --build $buildPath --config Release --parallel 4
if ($LASTEXITCODE -ne 0) { throw 'Navigation test build failed.' }
& $ctest --test-dir $buildPath -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'Navigation verification failed.' }
Write-Host "Detailed comparison: $buildPath/Testing/Temporary/LastTest.log"
Write-Host 'Host verification only: no device flashing, UART access or modification of sensor logs.'
