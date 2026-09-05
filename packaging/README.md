# Windows packaging

Run `packaging/build-windows.ps1` from PowerShell. It uses the validated Python 3.12 `.venv` and creates the standalone one-file executable at `dist/DroneMissionPlanner.exe`; end users do not need Python. Pass `-PythonExecutable C:\path\to\python.exe` only to select another Python 3.12 environment—the script rejects other runtime versions.

The build runs without an isolated dependency-download environment, sanitizes `PATH` while PyInstaller scans DLLs, and replaces Python's older root-level VC++ runtime with the newer backward-compatible runtime bundled by PySide6. These safeguards prevent unrelated DLLs from leaking into the package and prevent `DLL load failed while importing QtCore` on startup. After packaging, the script launches the EXE in a self-closing smoke-test mode and fails if startup hangs or returns a non-zero exit code.

The GitHub Actions workflow runs the complete test suite on Windows, builds the same spec, checks that the EXE exists and is non-empty, and uploads `DroneMissionPlanner-Windows-x64.zip` as a workflow artifact. Tagged `v*` builds are also suitable for attaching to a GitHub release.
