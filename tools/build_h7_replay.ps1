param(
    [string]$Log = 'tools/sample/terramind_20261003_160516.txt',
    [string]$OutputDirectory = 'build-navigation/h7-replay',
    [switch]$SkipFirmware
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$logPath = if ([IO.Path]::IsPathRooted($Log)) { $Log } else { Join-Path $projectRoot $Log }
$outputPath = if ([IO.Path]::IsPathRooted($OutputDirectory)) { $OutputDirectory } else { Join-Path $projectRoot $OutputDirectory }
$outputPath = [IO.Path]::GetFullPath($outputPath)
& python -B (Join-Path $PSScriptRoot 'prepare_h7_replay.py') $logPath --output $outputPath
if ($LASTEXITCODE -ne 0) { throw 'Replay export / offline reference failed. Build tools/build_kf_gins_aided.ps1 first if needed.' }
$candidates = @(
    (Get-Command cmake -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -ErrorAction SilentlyContinue),
    'C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe',
    'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
)
$cmake = $candidates | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -First 1
if (-not $cmake) { throw 'CMake / C++ compiler required.' }
$hostBuild = Join-Path $outputPath 'host'
& $cmake -S (Join-Path $projectRoot 'tests/navigation') -B $hostBuild "-DREPLAY_DIR=$outputPath"
if ($LASTEXITCODE -ne 0) { throw 'Replay host configuration failed.' }
& $cmake --build $hostBuild --config Release --target real_capture_replay
if ($LASTEXITCODE -ne 0) { throw 'Replay host build failed.' }
& (Join-Path $hostBuild 'Release/real_capture_replay.exe') $outputPath
if ($LASTEXITCODE -ne 0) { throw 'Real capture replay verification failed.' }
if (-not $SkipFirmware) {
    $keil = 'C:\Keil_v5\UV4\UV4.exe'
    if (-not (Test-Path -LiteralPath $keil)) { throw 'Keil UV4 not found at C:\Keil_v5\UV4\UV4.exe.' }
    $targetProject = Join-Path $outputPath 'navigation_replay.uvprojx'
    $buildLog = Join-Path $outputPath 'keil-build.log'
    $process = Start-Process -FilePath $keil -ArgumentList @('-b',('"'+$targetProject+'"'),'-j0','-o',('"'+$buildLog+'"')) -WindowStyle Hidden -PassThru -Wait
    Get-Content -LiteralPath $buildLog -Tail 12
    if ($process.ExitCode -ne 0) { throw "Keil build returned $($process.ExitCode). See $buildLog" }
}
Write-Host "Verified replay artifacts: $outputPath"
Write-Host 'No device flashed. On H7 inspect g_nav_replay_report after phase reaches 3.'
