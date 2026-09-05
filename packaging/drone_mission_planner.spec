# -*- mode: python ; coding: utf-8 -*-
from pathlib import Path

import PySide6


ROOT = Path(SPECPATH).parent
PYSIDE6_DIR = Path(PySide6.__file__).parent
MSVC_RUNTIME_NAMES = (
    "concrt140.dll",
    "msvcp140.dll",
    "msvcp140_1.dll",
    "msvcp140_2.dll",
    "msvcp140_codecvt_ids.dll",
    "vcruntime140.dll",
    "vcruntime140_1.dll",
)
AMBIENT_TOOL_RUNTIME_MARKERS = ("\\.cache\\codex-runtimes\\",)

a = Analysis(
    [str(ROOT / "run.py")],
    pathex=[str(ROOT), str(ROOT / "src")],
    binaries=[],
    datas=[
        (str(ROOT / "assets"), "assets"),
        (str(ROOT / "examples"), "examples"),
        (str(ROOT / "docs"), "docs"),
    ],
    hiddenimports=["PySide6.QtSvg"],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=["tkinter", "matplotlib", "IPython"],
    noarchive=False,
    optimize=1,
)

# Python 3.12 bundles an older VC++ runtime than recent PySide6 wheels.  If
# both versions are present, Windows loads Python's root-level copy first and
# QtCore then fails with an unhelpful "specified procedure could not be found"
# error.  Use PySide6's newer, backward-compatible runtime for the whole app.
a.binaries = [
    entry
    for entry in a.binaries
    if not any(marker in entry[1].lower() for marker in AMBIENT_TOOL_RUNTIME_MARKERS)
    and not (
        "\\" not in entry[0]
        and "/" not in entry[0]
        and entry[0].lower() in MSVC_RUNTIME_NAMES
    )
]
for runtime_name in MSVC_RUNTIME_NAMES:
    runtime_path = PYSIDE6_DIR / runtime_name
    if runtime_path.is_file():
        a.binaries.append((runtime_name, str(runtime_path), "BINARY"))

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="DroneMissionPlanner",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=str(ROOT / "assets" / "icons" / "drone-mission-planner.ico"),
    version=str(ROOT / "packaging" / "version_info.txt"),
)
