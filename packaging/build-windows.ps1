param(
    [string]$PythonExecutable = ""
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot

if (-not $PythonExecutable) {
    $PythonExecutable = Join-Path $ProjectRoot ".venv\Scripts\python.exe"
}
if (-not (Test-Path -LiteralPath $PythonExecutable -PathType Leaf)) {
    throw "Python 3.12 environment not found at $PythonExecutable. Create .venv first or pass -PythonExecutable."
}
$PythonExecutable = (Resolve-Path -LiteralPath $PythonExecutable).Path

$Version = & $PythonExecutable -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
if ($Version -ne "3.12") {
    throw "Windows release builds require the validated Python 3.12 runtime; found $Version at $PythonExecutable"
}

$OriginalPath = $env:PATH
$PythonRoot = Split-Path -Parent (Split-Path -Parent $PythonExecutable)
$env:PATH = @(
    $PythonRoot,
    (Join-Path $PythonRoot "Scripts"),
    (Join-Path $env:SystemRoot "System32"),
    $env:SystemRoot
) -join ";"

try {
    & $PythonExecutable -m pip install --no-build-isolation --editable . "pyinstaller>=6.12,<7"
    if ($LASTEXITCODE -ne 0) { throw "Dependency installation failed with exit code $LASTEXITCODE" }

    & $PythonExecutable -X faulthandler -u -m PyInstaller --noconfirm --clean packaging/drone_mission_planner.spec
    if ($LASTEXITCODE -ne 0) { throw "PyInstaller failed with exit code $LASTEXITCODE" }
}
finally {
    $env:PATH = $OriginalPath
}

$Exe = Join-Path $ProjectRoot "dist/DroneMissionPlanner.exe"
if (-not (Test-Path $Exe)) { throw "PyInstaller did not create $Exe" }

$SmokeProcess = Start-Process -FilePath $Exe -ArgumentList "--packaged-smoke-test" -PassThru -WindowStyle Hidden
if (-not $SmokeProcess.WaitForExit(15000)) {
    Stop-Process -Id $SmokeProcess.Id -Force -ErrorAction SilentlyContinue
    throw "Packaged application startup smoke timed out; check for a hidden error dialog."
}
if ($SmokeProcess.ExitCode -ne 0) {
    throw "Packaged application startup smoke failed with exit code $($SmokeProcess.ExitCode)"
}

Write-Host "Built $Exe ($((Get-Item $Exe).Length) bytes)"
