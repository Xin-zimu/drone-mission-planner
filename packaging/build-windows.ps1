$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
Set-Location $ProjectRoot

python -m pip install --upgrade pip
python -m pip install . "pyinstaller>=6.12,<7"
python -m PyInstaller --noconfirm --clean packaging/drone_mission_planner.spec

$Exe = Join-Path $ProjectRoot "dist/DroneMissionPlanner.exe"
if (-not (Test-Path $Exe)) { throw "PyInstaller did not create $Exe" }
Write-Host "Built $Exe ($((Get-Item $Exe).Length) bytes)"
