$ErrorActionPreference = 'Stop'
$project = (Resolve-Path (Join-Path $PSScriptRoot '..\KF-GINS')).Path
$candidates = @(
    (Get-Command cmake -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -ErrorAction SilentlyContinue),
    'C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe',
    'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
)
$cmake = $candidates | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -First 1
if (-not $cmake) { throw '未找到 CMake；请安装 Visual Studio C++ 桌面开发工作负载或 CMake。' }
$build = Join-Path $project 'build-msvc'
& $cmake -S $project -B $build '-DCMAKE_POLICY_VERSION_MINIMUM=3.5' '-DYAML_CPP_BUILD_TESTS=OFF' '-DABSL_BUILD_TESTING=OFF'
if ($LASTEXITCODE -ne 0) { throw 'KF-GINS CMake 配置失败。' }
& $cmake --build $build --config Release --parallel 8
if ($LASTEXITCODE -ne 0) { throw 'KF-GINS 编译失败。' }
Write-Host 'KF-GINS 编译完成。'
