# Windows packaging

Run `packaging/build-windows.ps1` from PowerShell with Python 3.12. It creates the standalone one-file executable at `dist/DroneMissionPlanner.exe`; end users do not need Python.

The GitHub Actions workflow runs the complete test suite on Windows, builds the same spec, checks that the EXE exists and is non-empty, and uploads `DroneMissionPlanner-Windows-x64.zip` as a workflow artifact. Tagged `v*` builds are also suitable for attaching to a GitHub release.
