"""F3-b final verification probe: record the three gates' real readouts.

Runs the full suite, ruff and mypy and prints one line each so the numbers are
on disk rather than in a chat message.
"""

from __future__ import annotations

import subprocess

PY = "D:/dmp/.venv/Scripts/python.exe"
BASETEMP = "D:/dmp/.pytest-tmp-f3b2"


def _run(args: list[str]) -> tuple[int, list[str]]:
    proc = subprocess.run(
        [PY, *args], capture_output=True, text=True, encoding="utf-8", errors="replace"
    )
    return proc.returncode, proc.stdout.strip().splitlines()


def main() -> None:
    code, lines = _run(["-m", "pytest", "D:/dmp/tests", f"--basetemp={BASETEMP}"])
    summary = next(
        (line for line in reversed(lines) if "passed" in line or "failed" in line), "no summary"
    )
    print("pytest_exit", code, "|", summary)

    code, lines = _run(["-m", "ruff", "check", "D:/dmp/src", "D:/dmp/tests", "D:/dmp/scripts"])
    print("ruff_exit", code, "|", lines[-1] if lines else "no output")

    code, lines = _run(
        [
            "-m",
            "mypy",
            "--config-file",
            "D:/dmp/pyproject.toml",
            "D:/dmp/src",
            "D:/dmp/tests",
        ]
    )
    print("mypy_exit", code, "|", lines[-1] if lines else "no output")


if __name__ == "__main__":
    main()
