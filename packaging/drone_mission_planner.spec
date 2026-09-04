# -*- mode: python ; coding: utf-8 -*-
from pathlib import Path


ROOT = Path(SPECPATH).parent

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
