<#
.SYNOPSIS
  Clean rebuild of the DS5 Bridge firmware UF2 and the companion app.

.DESCRIPTION
  Wipes previous firmware build trees, built UF2s, and companion build output,
  then builds a fresh Pico 2 W firmware image (companion interface on) and the
  companion installer.

    powershell -NoProfile -ExecutionPolicy Bypass -File tools\rebuild-all.ps1

  Outputs:
    uf2_builds\ds5-bridge-DDMMYY-HHMM.uf2 and uf2_builds\ds5-bridge-latest.uf2
    companion\artifacts\installer\DS5-Bridge-Companion-Setup-<version>.exe
    (or companion\artifacts\DS5 Bridge-win32-x64-* with -Unpacked)

.PARAMETER PicoSdkPath
  Pico SDK 2.3.0 checkout. Defaults to $env:PICO_SDK_PATH, then
  ~\.pico-sdk\sdk\2.3.0 (where the VS Code extension installs it).

.PARAMETER Diagnostics
  DS5_DIAGNOSTICS_PRESET for the firmware: off, audio, traces, all, custom.

.PARAMETER Waveshare
  Build for the Waveshare RP2350B-Plus-W instead of the Pico 2 W.

.PARAMETER Unpacked
  Produce the unpacked app (package:win) instead of the NSIS installer.
#>
param(
  [string] $PicoSdkPath,
  [ValidateSet('off', 'audio', 'traces', 'all', 'custom')]
  [string] $Diagnostics = 'off',
  [switch] $Waveshare,
  [switch] $Unpacked
)

$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$companionDir = Join-Path $repoRoot 'companion'
$uf2Dir = Join-Path $repoRoot 'uf2_builds'
$firmwareBuild = Join-Path $repoRoot 'build\pico2w'

function Invoke-Checked {
  param([string[]] $Command)
  & $Command[0] @($Command | Select-Object -Skip 1)
  if ($LASTEXITCODE -ne 0) {
    throw "Command failed with exit code ${LASTEXITCODE}: $($Command -join ' ')"
  }
}

function Write-Step {
  param([string] $Message)
  Write-Host ''
  Write-Host "==> $Message" -ForegroundColor Cyan
}

function Remove-IfPresent {
  param([string] $Path)
  if (Test-Path -LiteralPath $Path) {
    Write-Host "  removing $Path"
    Remove-Item -LiteralPath $Path -Recurse -Force
  }
}

# The Pico SDK builds picotool and pioasm as host tools, so a native compiler
# must be on the environment even though the firmware is built by arm-none-eabi.
function Import-MsvcEnvironment {
  if (Get-Command cl.exe -ErrorAction SilentlyContinue) {
    return
  }
  $vcvars = $null
  $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
  if (Test-Path -LiteralPath $vswhere) {
    $installPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($installPath) {
      $vcvars = Join-Path $installPath 'VC\Auxiliary\Build\vcvars64.bat'
    }
  }
  if (-not $vcvars -or -not (Test-Path -LiteralPath $vcvars)) {
    throw 'MSVC C++ build tools not found (vcvars64.bat). Install Visual Studio Build Tools with the C++ workload.'
  }
  Write-Host "  loading $vcvars"
  cmd /c "`"$vcvars`" >nul 2>&1 && set" | ForEach-Object {
    if ($_ -match '^([^=]+)=(.*)$') {
      Set-Item -Path "env:$($matches[1])" -Value $matches[2] -ErrorAction SilentlyContinue
    }
  }
}

# --- Preflight --------------------------------------------------------------

if (-not $PicoSdkPath) { $PicoSdkPath = $env:PICO_SDK_PATH }
if (-not $PicoSdkPath) { $PicoSdkPath = Join-Path $HOME '.pico-sdk\sdk\2.3.0' }
if (-not (Test-Path -LiteralPath (Join-Path $PicoSdkPath 'pico_sdk_init.cmake'))) {
  throw "No Pico SDK at '$PicoSdkPath'. Pass -PicoSdkPath or set PICO_SDK_PATH."
}
$PicoSdkPath = (Resolve-Path $PicoSdkPath).Path
# The flash-nuke helper built by the companion packaging reads it from the environment.
$env:PICO_SDK_PATH = $PicoSdkPath

$firmwareVersion = (Get-Content -Raw (Join-Path $repoRoot 'firmware-version.txt')).Trim()
Write-Host "DS5 Bridge firmware $firmwareVersion"
Write-Host "  Pico SDK:    $PicoSdkPath"
Write-Host "  Diagnostics: $Diagnostics"
Write-Host "  Board:       $(if ($Waveshare) { 'Waveshare RP2350B-Plus-W' } else { 'Pico 2 W' })"

Write-Step 'Loading MSVC environment'
Import-MsvcEnvironment

Write-Step 'Updating submodules'
Invoke-Checked @('git', '-C', $repoRoot, 'submodule', 'update', '--init', '--recursive')

# --- Clean ------------------------------------------------------------------

Write-Step 'Cleaning previous build output'
Remove-IfPresent (Join-Path $repoRoot 'build')
if (Test-Path -LiteralPath $uf2Dir) {
  Get-ChildItem -LiteralPath $uf2Dir -Filter '*.uf2' | ForEach-Object { Remove-IfPresent $_.FullName }
}
foreach ($dir in @('dist', 'dist-main', 'artifacts', 'native\AudioHelper\bin', 'native\AudioHelper\obj')) {
  Remove-IfPresent (Join-Path $companionDir $dir)
}
Get-ChildItem -LiteralPath (Join-Path $companionDir 'firmware') -Include '*.uf2', '*.uf2.sha256' -Recurse -ErrorAction SilentlyContinue |
  ForEach-Object { Remove-IfPresent $_.FullName }

# --- Firmware ---------------------------------------------------------------

Write-Step 'Building firmware'
$cmakeArgs = @(
  'cmake', '-S', $repoRoot, '-B', $firmwareBuild, '-G', 'Ninja',
  '-DCMAKE_BUILD_TYPE=Release',
  "-DPICO_SDK_PATH=$PicoSdkPath",
  '-DENABLE_COMPANION=ON',
  "-DDS5_DIAGNOSTICS_PRESET=$Diagnostics"
)
if ($Waveshare) { $cmakeArgs += '-DWAVESHARE_RP2350B_PLUS_W_BUILD=ON' }
Invoke-Checked $cmakeArgs
Invoke-Checked @('cmake', '--build', $firmwareBuild, '--target', 'ds5-bridge')

$builtUf2 = Join-Path $firmwareBuild 'ds5-bridge.uf2'
if (-not (Test-Path -LiteralPath $builtUf2)) {
  throw "Expected UF2 was not produced: $builtUf2"
}
New-Item -ItemType Directory -Path $uf2Dir -Force | Out-Null
$stampedUf2 = Join-Path $uf2Dir "ds5-bridge-$(Get-Date -Format 'ddMMyy-HHmm').uf2"
$latestUf2 = Join-Path $uf2Dir 'ds5-bridge-latest.uf2'
Copy-Item -LiteralPath $builtUf2 -Destination $stampedUf2
Copy-Item -LiteralPath $builtUf2 -Destination $latestUf2

# --- Companion --------------------------------------------------------------

# Install with pnpm, not `npm ci`: package.json declares pnpm as its
# packageManager, and electron-builder's pnpm dependency scan reinstalls an
# npm-layout node_modules mid-build, deleting app-builder-lib out from under it.
Write-Step 'Installing companion dependencies'
Push-Location $companionDir
try {
  Invoke-Checked @('pnpm.cmd', 'install', '--frozen-lockfile')

  if ($Unpacked) {
    Write-Step 'Packaging companion app (unpacked)'
    Invoke-Checked @('pnpm.cmd', 'run', 'package:win')
  } else {
    Write-Step 'Building companion installer'
    Invoke-Checked @('pnpm.cmd', 'run', 'installer:win')
  }
} finally {
  Pop-Location
}

# --- Summary ----------------------------------------------------------------

Write-Step 'Done'
Write-Host 'Firmware:'
Write-Host "  $stampedUf2"
Write-Host "  $latestUf2"
Write-Host 'Companion:'
$artifacts = Join-Path $companionDir 'artifacts'
if ($Unpacked) {
  Get-ChildItem -LiteralPath $artifacts -Directory | ForEach-Object { Write-Host "  $($_.FullName)" }
} else {
  Get-ChildItem -LiteralPath (Join-Path $artifacts 'installer') -Filter '*.exe' | ForEach-Object { Write-Host "  $($_.FullName)" }
}
