"""F3-b group-criteria probe: put the group criterion readouts on disk.

The group criteria use ``-k`` subsets, which the convergence record referenced
by path only; this probe runs exactly those commands and prints their counts so
the numbers are independently re-checkable from the ledger.
"""

from __future__ import annotations

import subprocess

PY = "D:/dmp/.venv/Scripts/python.exe"
BASETEMP = "D:/dmp/.pytest-tmp-f3b2"

CRITERIA = {
    "opt04_criterion": [
        "-m", "pytest", "D:/dmp/tests/unit/test_optimization.py",
        f"--basetemp={BASETEMP}",
        "-k", "status_enum or rounding_is_conservative or empty_window",
    ],
    "opt05_criterion": [
        "-m", "pytest", "D:/dmp/tests/unit/test_coverage_advanced.py",
        f"--basetemp={BASETEMP}",
        "-k", "hole_cells or vertical_scan",
    ],
    "coverage_blocks": [
        "-m", "pytest", "D:/dmp/tests/unit/test_coverage_blocks.py",
        f"--basetemp={BASETEMP}",
    ],
}


def main() -> None:
    for name, args in CRITERIA.items():
        proc = subprocess.run(
            [PY, *args], capture_output=True, text=True, encoding="utf-8", errors="replace"
        )
        lines = proc.stdout.strip().splitlines()
        summary = next(
            (line for line in reversed(lines) if "passed" in line or "failed" in line),
            "no summary",
        )
        print(name, "exit", proc.returncode, "|", summary)


if __name__ == "__main__":
    main()
