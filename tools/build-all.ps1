param(
    [ValidateSet('Debug', 'RelWithDebInfo', 'Release', 'MinSizeRel')]
    [string]$Preset = 'Debug'
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot

foreach ($board in @('board1', 'board2')) {
    $boardRoot = Join-Path $repoRoot $board
    Push-Location $boardRoot
    try {
        cmake --preset $Preset
        if ($LASTEXITCODE -ne 0) { throw "CMake configure failed for $board" }
        cmake --build --preset $Preset
        if ($LASTEXITCODE -ne 0) { throw "CMake build failed for $board" }
    }
    finally {
        Pop-Location
    }
}
