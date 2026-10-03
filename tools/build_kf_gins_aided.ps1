$ErrorActionPreference = 'Stop'
$project = Join-Path $PSScriptRoot 'kf_gins_aided'
$build = Join-Path $PSScriptRoot '..\build-navigation\aided'
$candidates = @(
    (Get-Command cmake -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -ErrorAction SilentlyContinue),
    'C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe',
    'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
)
$cmake = $candidates | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -First 1
if (-not $cmake) { throw 'CMake not found. Install Visual Studio Desktop C++ tools or CMake.' }
& $cmake -S $project -B $build '-DCMAKE_POLICY_VERSION_MINIMUM=3.5'
if ($LASTEXITCODE -ne 0) { throw 'Offline aiding CMake configuration failed.' }
& $cmake --build $build --config Release --parallel 8
if ($LASTEXITCODE -ne 0) { throw 'Offline aiding build failed.' }
& (Join-Path (Split-Path $cmake) 'ctest.exe') --test-dir $build -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'Offline aiding tests failed.' }
Write-Host 'Offline aiding built and verified. Original solver and firmware flow remain unchanged.'
