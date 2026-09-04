from __future__ import annotations

import sys
from pathlib import Path


def resource_path(relative_path: str) -> Path:
    bundle_root = getattr(sys, "_MEIPASS", None)
    root = Path(bundle_root) if bundle_root is not None else Path(__file__).resolve().parents[3]
    return root / relative_path
